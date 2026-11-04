/*
 * viva (MT6781): battery, charger and power policy.
 *
 * VivaBattery (/arm-io/i2c3/battery, "ti,bq27z561") is the phone's
 * IOPMPowerSource: powerd reads it like Apple's AppleARMPMUPowerSource
 * (IOPSCopyPowerSourcesInfo -> SpringBoard battery icon and percentage,
 * charging bolt, Settings > Battery, Low Power Mode auto-off at 80 %).
 *
 * Hardware (Xiaomi viva-r-oss + live DT, I2C3 @0x1100f000, 400 kHz):
 *  - TI bq27z561 fuel gauge at 0x55 (drivers/power/supply/mediatek/battery/
 *    bq27z561.c): little-endian 16-bit standard commands, read only here.
 *    0x06 temperature (0.1 K), 0x08 voltage (mV), 0x0a BatteryStatus,
 *    0x0c current now (mA, + = charging), 0x14 average current, 0x16/0x18
 *    time to empty/full (min, 65535 = n/a), 0x10 remaining capacity, 0x12
 *    full charge capacity (mAh), 0x2a cycle count, 0x2c relative state of
 *    charge (%), 0x2e state of health (%), 0x30/0x32 the gauge's charging
 *    voltage/current request (JEITA aware), 0x3c design capacity.
 *  - Silergy SYV690 charger at 0x6a (charger/syv690.c), BQ25890 register map;
 *    REG14 PN 1 = SYV690 (ICHG 64 mA/step), 4 = SC89890H (60 mA/step).
 *    REG0B VBUS_STAT[7:5] CHRG_STAT[4:3] PG_STAT[2]; REG0C faults; REG11
 *    VBUS_GD[7] VBUSV[6:0] (2600 + 100 mV); REG12 ICHGR (50 mA); REG13 IDPM_LIM
 *    (100 + 50 mA). Its INT line is EINT 11 (falling edge): status changes
 *    (plug, unplug, charge done) poll at once.
 *
 * Charger writes (boot-arg vivacharge): 0 = never write the charger (it runs on
 * whatever LK left, and on its own defaults once its I2C watchdog expires:
 * 4.208 V, so the battery stops at ~85 %); 1 (default) = keep it at the gauge's
 * request: VREG = min(gauge ChargingVoltage, ADT charge-voltage-max) and
 * ICHG = min(gauge ChargingCurrent, ADT charge-current-max) (0 = off only if the
 * gauge says full or the battery is outside 0..50 C), watchdog 160 s,
 * kicked at every poll (if this kernel dies the charger falls back to its
 * defaults). Only REG03 (WD_RST, CHG_CONFIG), REG04 ICHG, REG06 VREG and REG07
 * WATCHDOG are ever written, read-modify-write, each one read back. Gauge
 * values outside 3800..4500 mV / 0..6000 mA, or an unknown PN: no writes.
 * Input limit (boot-arg vivachargein, default 2000 mA, 0 = off): D+/D- are on the USB PHY, so BC1.2 says "unknown
 * adapter" (500 mA); with no USB host on the link the cable is a wall charger and REG00 IINLIM goes to that value
 * (VINDPM protects a weak adapter); a host configuring the link brings it back to 500 mA.
 * Charge limit (boot-arg vivachargelimit=N, sysctl kern.viva_chargelimit,
 * default 100 = off): CHG_CONFIG off at N %, back on below N - 5 %.
 *
 * Low Power Mode: powerd posts com.apple.system.lowpowermode; vivad
 * (tools/vivad) forwards its state to sysctl kern.viva_lowpower. On: the GPU
 * DVFS ceiling drops to vivalpmgpu MHz (default 550) and panel copies get at
 * most vivalpmduty % of a CPU (default 10). The battery service carries
 * "LowPowerMode". Boot-arg vivabattery=0 leaves the old static SweetBattery.
 *
 * USB link: "bat" (state, both chips' registers), "bat poll", "bat lpm on|off".
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOCommandGate.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/pwr_mgt/IOPMPowerSource.h>
#include <libkern/c++/OSCollectionIterator.h>
#include <pexpert/pexpert.h>
#include <sys/sysctl.h>

#include "VivaIO.h"

/* bq27z561 standard commands */
#define FG_TEMP         0x06
#define FG_VOLT         0x08
#define FG_STATUS       0x0a
#define FG_CURRENT      0x0c
#define FG_RM           0x10
#define FG_FCC          0x12
#define FG_AVG_CURRENT  0x14
#define FG_TTE          0x16
#define FG_TTF          0x18
#define FG_CYCLES       0x2a
#define FG_SOC          0x2c
#define FG_SOH          0x2e
#define FG_CHG_VOLT     0x30
#define FG_CHG_CURRENT  0x32
#define FG_DESIGN       0x3c
#define FG_STATUS_FD    (1u << 4)       /* fully discharged */
#define FG_STATUS_FC    (1u << 5)       /* fully charged */
#define FG_STATUS_DSG   (1u << 6)       /* discharging */
#define FG_STATUS_INIT  (1u << 7)       /* initialization complete */

/* SYV690 / BQ25890 */
#define CHG_REG00       0x00
#define R00_IINLIM      0x3f            /* 100 + 50 mA per step */
#define CHG_REG03       0x03
#define CHG_REG04       0x04
#define CHG_REG06       0x06
#define CHG_REG07       0x07
#define CHG_REG0B       0x0b
#define CHG_REG0C       0x0c
#define CHG_REG11       0x11
#define CHG_REG12       0x12
#define CHG_REG13       0x13
#define CHG_REG14       0x14
#define R03_WD_RST      (1u << 6)
#define R03_OTG         (1u << 5)
#define R03_CHG_CONFIG  (1u << 4)
#define R04_ICHG        0x7f
#define R06_VREG_SHIFT  2
#define R06_VREG        0xfc
#define R07_WD_MASK     0x30
#define R07_WD_160S     0x30
#define VREG_BASE_MV    3840
#define VREG_STEP_MV    16
#define PN_SYV690       1
#define PN_SC89890H     4

enum { VBUS_NONE = 0, VBUS_SDP, VBUS_CDP, VBUS_DCP, VBUS_HVDCP, VBUS_UNKNOWN, VBUS_NONSTD, VBUS_OTG };
enum { CHRG_NOT = 0, CHRG_PRE, CHRG_FAST, CHRG_DONE };

/* IOPSKeys.h family codes */
#define FAMILY_USB_HOST         0xe0004000u
#define FAMILY_USB_DEDICATED    0xe0004004u
#define FAMILY_USB_DOWNSTREAM   0xe0004005u
#define FAMILY_USB_UNKNOWN      0xe0004007u

#define POLL_MS         15000
#define IRQ_DELAY_MS    300

struct VivaBatterySample {
	bool     fgOK, chgOK;
	uint16_t status;
	int      tempDeciC;     /* 0.1 C */
	uint32_t mv, rm, fcc, design, cycles, soc, soh, chgMv, chgMa, tte, ttf;
	int      ma, avgMa;
	uint8_t  r[0x15];       /* charger registers 0x00..0x14 */
};

class VivaBattery : public IOPMPowerSource {
	OSDeclareDefaultStructors(VivaBattery);
public:
	virtual bool init(OSDictionary *dictionary = NULL) APPLE_KEXT_OVERRIDE;
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOWorkLoop *getWorkLoop(void) const APPLE_KEXT_OVERRIDE { return fWorkLoop; }

	void pollNow(void);
	void link(const char *args, char *out, size_t len);
	void lowPowerChanged(void);

private:
	static void timerFired(OSObject *owner, IOTimerEventSource *sender);
	static void interruptOccurred(OSObject *owner, IOInterruptEventSource *src, int count);
	void poll(void);
	bool readSample(VivaBatterySample *s);
	void maintainCharger(VivaBatterySample *s);
	void publish(const VivaBatterySample *s);
	IOReturn fgWord(uint8_t reg, uint16_t *v);
	IOReturn chgRead(uint8_t reg, uint8_t *v, uint32_t n);
	IOReturn chgUpdate(uint8_t reg, uint8_t mask, uint8_t value);
	void putNumber(const char *key, int64_t value);
	void putBool(const char *key, bool value);
	void putString(const char *key, const char *value);

	VivaI2C              *fBus;
	IOWorkLoop           *fWorkLoop;
	IOCommandGate        *fGate;
	IOTimerEventSource   *fTimer;
	IOInterruptEventSource *fInterrupt;
	uint8_t               fFgAddr, fChgAddr;
	uint32_t              fDesignDefault, fMaxMv, fMaxMa;
	uint32_t              fChargeMode, fInputMa;
	bool                  fLimitHold;
	uint32_t              fPolls, fFgErrors, fChgErrors, fIrqs, fWrites, fWriteFails;
	uint32_t              fLastSoc, fLastVbus, fLastChrg;
	bool                  fLastExternal, fLastLPM, fHaveSample;
	VivaBatterySample     fLast;
	char                  fLastWrite[96];
};

OSDefineMetaClassAndStructors(VivaBattery, IOPMPowerSource);

static VivaBattery *gVivaBattery;
static int gVivaLowPower;               /* kern.viva_lowpower */
static int gVivaChargeLimit = 100;      /* kern.viva_chargelimit */
static uint32_t gVivaLPMGpuMHz = 550, gVivaLPMDuty = 10;

extern "C" int viva_usb_host_configured(void);           /* bsd/dev/viva_usb.c */
extern "C" void viva_gpu_set_dvfs_cap(uint32_t mhz);      /* VivaGPU.cpp: 0 = no cap */
extern "C" void viva_cpufreq_set_lowpower(int on);        /* osfmk/arm64/viva_cpufreq.c: CPU ceilings */
extern "C" void sweet_fb_set_duty_cap(uint32_t pct);      /* SweetFramebuffer.cpp: 0 = no cap */

static uint32_t
dtU32(IOService *provider, const char *key, uint32_t dflt)
{
	OSData *d = OSDynamicCast(OSData, provider->getProperty(key));
	return (d != NULL && d->getLength() >= 4) ? *(const uint32_t *)d->getBytesNoCopy() : dflt;
}

static inline int
tenths(int deci)
{
	return deci < 0 ? (-deci) % 10 : deci % 10;
}

/* ---- power policy (Low Power Mode) ---- */

static void
viva_power_apply(void)
{
	bool on = gVivaLowPower != 0;
	viva_gpu_set_dvfs_cap(on ? gVivaLPMGpuMHz : 0);
	viva_cpufreq_set_lowpower(on);
	sweet_fb_set_duty_cap(on ? gVivaLPMDuty : 0);
	IOLog("VIVA POWER: low power mode %s (GPU ceiling %u MHz, panel copy duty %u %%)\n", on ? "ON" : "off",
	    on ? gVivaLPMGpuMHz : 0, on ? gVivaLPMDuty : 0);
	if (gVivaBattery != NULL) {
		gVivaBattery->lowPowerChanged();
	}
}

static int
sysctl_viva_lowpower(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int value = gVivaLowPower, changed = 0;
	int error = sysctl_io_number(req, value, sizeof(int), &value, &changed);
	if (error == 0 && changed) {
		value = value != 0;
		if (value != gVivaLowPower) {
			gVivaLowPower = value;
			viva_power_apply();
		}
	}
	return error;
}

static int
sysctl_viva_chargelimit(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int value = gVivaChargeLimit, changed = 0;
	int error = sysctl_io_number(req, value, sizeof(int), &value, &changed);
	if (error == 0 && changed) {
		if (value < 50 || value > 100) {
			return EINVAL;
		}
		gVivaChargeLimit = value;
		if (gVivaBattery != NULL) {
			gVivaBattery->pollNow();
		}
	}
	return error;
}

SYSCTL_PROC(_kern, OID_AUTO, viva_lowpower, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sysctl_viva_lowpower, "I", "viva: Low Power Mode (from powerd through vivad)");
SYSCTL_PROC(_kern, OID_AUTO, viva_chargelimit, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    NULL, 0, sysctl_viva_chargelimit, "I", "viva: stop charging at this battery percentage (100 = off)");

/* ---- the power source ---- */

bool
VivaBattery::init(OSDictionary *dictionary)
{
	/* As SweetBattery: IOPMPowerSource only overrides init(void) (its PS dictionary). */
	if (!IOPMPowerSource::init()) {
		return false;
	}
	if (dictionary != NULL) {
		OSCollectionIterator *it = OSCollectionIterator::withCollection(dictionary);
		if (it == NULL) {
			return false;
		}
		const OSSymbol *key;
		while ((key = OSDynamicCast(OSSymbol, it->getNextObject())) != NULL) {
			setProperty(key, dictionary->getObject(key));
		}
		it->release();
	}
	return true;
}

IOReturn
VivaBattery::fgWord(uint8_t reg, uint16_t *v)
{
	uint8_t b[2];
	IOReturn ret = fBus->transfer(fFgAddr, &reg, 1, b, 2);
	if (ret == kIOReturnSuccess) {
		*v = (uint16_t)(b[0] | b[1] << 8);
	}
	return ret;
}

IOReturn
VivaBattery::chgRead(uint8_t reg, uint8_t *v, uint32_t n)
{
	while (n > 0) {
		uint32_t k = n < (uint32_t)VivaI2C::kFIFOSize ? n : (uint32_t)VivaI2C::kFIFOSize;
		IOReturn ret = fBus->transfer(fChgAddr, &reg, 1, v, k);
		if (ret != kIOReturnSuccess) {
			return ret;
		}
		reg += k;
		v += k;
		n -= k;
	}
	return kIOReturnSuccess;
}

/* Read-modify-write of `mask` in one of the writable charger registers, read back. */
IOReturn
VivaBattery::chgUpdate(uint8_t reg, uint8_t mask, uint8_t value)
{
	if (reg != CHG_REG00 && reg != CHG_REG03 && reg != CHG_REG04 && reg != CHG_REG06 && reg != CHG_REG07) {
		return kIOReturnNotPermitted;
	}
	uint8_t old;
	IOReturn ret = chgRead(reg, &old, 1);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	uint8_t w[2] = { reg, (uint8_t)((old & ~mask) | (value & mask)) };
	if (w[1] == old && !(reg == CHG_REG03 && (mask & R03_WD_RST))) {
		return kIOReturnSuccess;
	}
	ret = fBus->transfer(fChgAddr, w, 2, NULL, 0);
	uint8_t check = 0;
	uint8_t verifyMask = (uint8_t)(mask & ~(reg == CHG_REG03 ? R03_WD_RST : 0));   /* WD_RST self-clears */
	if (ret == kIOReturnSuccess) {
		ret = chgRead(reg, &check, 1);
	}
	if (ret == kIOReturnSuccess && (check & verifyMask) != (w[1] & verifyMask)) {
		ret = kIOReturnIOError;
	}
	fWrites++;
	if (ret != kIOReturnSuccess) {
		fWriteFails++;
	}
	if (verifyMask != 0) {
		snprintf(fLastWrite, sizeof(fLastWrite), "REG%02X %02x -> %02x (reads %02x)%s", reg, old, w[1], check,
		    ret == kIOReturnSuccess ? "" : " FAILED");
		IOLog("VIVA BATTERY: charger %s\n", fLastWrite);
	}
	return ret;
}

bool
VivaBattery::readSample(VivaBatterySample *s)
{
	uint16_t v[15];
	static const uint8_t regs[15] = { FG_STATUS, FG_TEMP, FG_VOLT, FG_CURRENT, FG_AVG_CURRENT, FG_RM, FG_FCC,
		                          FG_DESIGN, FG_CYCLES, FG_SOC, FG_SOH, FG_CHG_VOLT, FG_CHG_CURRENT, FG_TTE, FG_TTF };
	bzero(s, sizeof(*s));
	s->fgOK = true;
	for (unsigned i = 0; i < 15 && s->fgOK; i++) {
		s->fgOK = fgWord(regs[i], &v[i]) == kIOReturnSuccess;
	}
	if (s->fgOK) {
		s->status = v[0];
		s->tempDeciC = (int)v[1] - 2731;
		s->mv = v[2];
		s->ma = (int16_t)v[3];
		s->avgMa = (int16_t)v[4];
		s->rm = v[5];
		s->fcc = v[6];
		s->design = v[7];
		s->cycles = v[8];
		s->soc = v[9];
		s->soh = v[10];
		s->chgMv = v[11];
		s->chgMa = v[12];
		s->tte = v[13];
		s->ttf = v[14];
		/* implausible data never becomes 0 % or a hot battery */
		if (s->mv < 2500 || s->mv > 4700 || s->soc > 100 || s->tempDeciC < -400 || s->tempDeciC > 900) {
			s->fgOK = false;
		}
	}
	if (!s->fgOK) {
		fFgErrors++;
	}
	s->chgOK = chgRead(0, s->r, sizeof(s->r)) == kIOReturnSuccess;
	if (!s->chgOK) {
		fChgErrors++;
	}
	return s->fgOK || s->chgOK;
}

void
VivaBattery::maintainCharger(VivaBatterySample *s)
{
	if (fChargeMode == 0 || !s->chgOK) {
		return;
	}
	uint32_t pn = (s->r[CHG_REG14] >> 3) & 7, step = pn == PN_SYV690 ? 64 : pn == PN_SC89890H ? 60 : 0;
	uint32_t vbus = s->r[CHG_REG0B] >> 5;
	if (step == 0 || vbus == VBUS_OTG || (s->r[CHG_REG03] & R03_OTG)) {
		return;
	}
	/* watchdog 160 s, kicked now */
	chgUpdate(CHG_REG07, R07_WD_MASK, R07_WD_160S);
	chgUpdate(CHG_REG03, R03_WD_RST, R03_WD_RST);
	if (!s->fgOK) {
		return;         /* no gauge: leave voltage/current as they are */
	}
	if (s->chgMv >= 3800 && s->chgMv <= 4500 && s->chgMa <= 6000) {
		uint32_t mv = MIN(s->chgMv, fMaxMv), ma = MIN(s->chgMa, fMaxMa);
		uint32_t vreg = mv >= VREG_BASE_MV ? (mv - VREG_BASE_MV) / VREG_STEP_MV : 0;
		uint32_t ichg = MIN(ma / step, (uint32_t)R04_ICHG);
		chgUpdate(CHG_REG06, R06_VREG, (uint8_t)(MIN(vreg, 0x3fu) << R06_VREG_SHIFT));
		/* 0 mA (= charging off) only when the gauge has a reason: full, or outside 0..50 C */
		bool zeroOK = (s->status & FG_STATUS_FC) || s->tempDeciC < 0 || s->tempDeciC > 500;
		if (ichg != 0 || zeroOK) {
			chgUpdate(CHG_REG04, R04_ICHG, (uint8_t)ichg);
		}
	}
	/*
	 * Input limit. D+/D- go to the USB PHY (usb_switch GPIOs left as LK set them), so the charger's BC1.2 detection
	 * sees an "unknown adapter" and stays at 500 mA. Without a USB host on the link it is a wall charger: 2000 mA
	 * (fInputMa); the charger's VINDPM backs off if the adapter sags. With a host (or SDP/CDP detected): untouched.
	 */
	uint32_t vbusStat = s->r[CHG_REG0B] >> 5;
	bool pg = (s->r[CHG_REG0B] & 0x04) != 0;
	if (fInputMa != 0 && pg && !viva_usb_host_configured() &&
	    (vbusStat == VBUS_UNKNOWN || vbusStat == VBUS_NONSTD || vbusStat == VBUS_DCP || vbusStat == VBUS_HVDCP)) {
		uint32_t steps = MIN((fInputMa - 100) / 50, (uint32_t)R00_IINLIM);
		chgUpdate(CHG_REG00, R00_IINLIM, (uint8_t)steps);
	} else if (fInputMa != 0 && pg && viva_usb_host_configured() && (s->r[CHG_REG00] & R00_IINLIM) > 8) {
		chgUpdate(CHG_REG00, R00_IINLIM, 8);            /* a computer after a charger: back to 500 mA */
	}
	/* charge limit with 5 % hysteresis */
	bool hold = fLimitHold;
	if (gVivaChargeLimit >= 100) {
		hold = false;
	} else if (s->soc >= (uint32_t)gVivaChargeLimit) {
		hold = true;
	} else if (s->soc + 5 <= (uint32_t)gVivaChargeLimit) {
		hold = false;
	}
	if (hold != fLimitHold || hold != !(s->r[CHG_REG03] & R03_CHG_CONFIG)) {
		if (chgUpdate(CHG_REG03, R03_CHG_CONFIG, hold ? 0 : R03_CHG_CONFIG) == kIOReturnSuccess) {
			fLimitHold = hold;
			IOLog("VIVA BATTERY: charging %s (limit %d %%, battery %u %%)\n", hold ? "held" : "allowed",
			    gVivaChargeLimit, s->soc);
		}
	}
	chgRead(0, s->r, sizeof(s->r));
}

void
VivaBattery::putNumber(const char *key, int64_t value)
{
	const OSSymbol *sym = OSSymbol::withCString(key);
	OSNumber *num = OSNumber::withNumber((unsigned long long)value, 64);
	if (sym != NULL && num != NULL) {
		setPSProperty(sym, num);
	}
	OSSafeReleaseNULL(num);
	OSSafeReleaseNULL(sym);
}

void
VivaBattery::putBool(const char *key, bool value)
{
	const OSSymbol *sym = OSSymbol::withCString(key);
	if (sym != NULL) {
		setPSProperty(sym, value ? kOSBooleanTrue : kOSBooleanFalse);
		sym->release();
	}
}

void
VivaBattery::putString(const char *key, const char *value)
{
	const OSSymbol *sym = OSSymbol::withCString(key);
	OSString *str = OSString::withCString(value);
	if (sym != NULL && str != NULL) {
		setPSProperty(sym, str);
	}
	OSSafeReleaseNULL(str);
	OSSafeReleaseNULL(sym);
}

void
VivaBattery::publish(const VivaBatterySample *s)
{
	uint32_t vbus = s->chgOK ? s->r[CHG_REG0B] >> 5 : VBUS_NONE;
	uint32_t chrg = s->chgOK ? (s->r[CHG_REG0B] >> 3) & 3 : CHRG_NOT;
	bool pg = s->chgOK && (s->r[CHG_REG0B] & 0x04);
	bool vbusGood = s->chgOK && (s->r[CHG_REG11] & 0x80);
	bool external = (pg || vbusGood) && vbus != VBUS_OTG;
	bool charging = external && (chrg == CHRG_PRE || chrg == CHRG_FAST);
	uint32_t soc = s->fgOK ? s->soc : 100;
	bool full = external && (chrg == CHRG_DONE || (s->fgOK && (s->status & FG_STATUS_FC)) || soc >= 100);
	if (full) {
		soc = MAX(soc, s->fgOK && s->soc >= 95 ? 100u : soc);
	}
	uint32_t fcc = s->fgOK && s->fcc ? s->fcc : fDesignDefault;
	uint32_t design = s->fgOK && s->design ? s->design : fDesignDefault;

	setBatteryInstalled(true);
	/* AppleARMPMUPowerSource convention: percent over 100; raw mAh in the Apple raw keys */
	setCurrentCapacity(soc);
	setMaxCapacity(100);
	setExternalConnected(external);
	setExternalChargeCapable(external);
	setIsCharging(charging);
	setAtWarnLevel(!external && soc <= 10);
	setAtCriticalLevel(false);      /* PSCI SYSTEM_OFF hangs on this phone: never ask iOS to shut down */
	setVoltage(s->fgOK ? s->mv : 4000);
	setAmperage(s->fgOK ? s->avgMa : 0);
	setCycleCount(s->fgOK ? s->cycles : 0);
	int minutes = -1;
	if (s->fgOK && !external && s->tte != 0xffff) {
		minutes = (int)s->tte;
	} else if (s->fgOK && charging && s->ttf != 0xffff) {
		minutes = (int)s->ttf;
	}
	setTimeRemaining(minutes);
	putBool("FullyCharged", full);
	putBool("AppleRawExternalConnected", external);
	putNumber("AppleRawCurrentCapacity", s->fgOK ? s->rm : fcc * soc / 100);
	putNumber("AppleRawMaxCapacity", fcc);
	putNumber("AppleRawBatteryVoltage", s->fgOK ? s->mv : 4000);
	putNumber("NominalChargeCapacity", fcc);
	putNumber("DesignCapacity", design);
	putNumber("MaximumCapacityPercent", s->fgOK && s->soh && s->soh <= 100 ? s->soh : 100);
	putNumber("InstantAmperage", s->fgOK ? s->ma : 0);
	putNumber("Temperature", s->fgOK ? s->tempDeciC * 10 : 2500);    /* centi-degrees C */
	putNumber("AvgTimeToEmpty", s->fgOK ? s->tte : 0xffff);
	putNumber("AvgTimeToFull", s->fgOK ? s->ttf : 0xffff);
	putNumber("StateOfCharge", soc);
	putNumber("ChargingVoltage", s->fgOK ? s->chgMv : 0);
	putBool("LowPowerMode", gVivaLowPower != 0);
	putBool("VivaGaugeValid", s->fgOK);
	putBool("VivaChargerValid", s->chgOK);
	putBool("VivaChargeLimitHold", fLimitHold);

	/* the adapter: from the charger's input detection and its live input limit */
	OSDictionary *adapter = OSDictionary::withCapacity(8);
	if (adapter != NULL) {
		uint32_t vbusMv = s->chgOK && (s->r[CHG_REG11] & 0x7f) ? 2600 + 100 * (s->r[CHG_REG11] & 0x7f) : 5000;
		uint32_t limitMa = s->chgOK ? 100 + 50 * (s->r[CHG_REG13] & 0x3f) : 0;
		uint32_t family = vbus == VBUS_SDP ? FAMILY_USB_HOST : vbus == VBUS_CDP ? FAMILY_USB_DOWNSTREAM :
		    (vbus == VBUS_DCP || vbus == VBUS_HVDCP) ? FAMILY_USB_DEDICATED : FAMILY_USB_UNKNOWN;
		const char *desc = vbus == VBUS_SDP ? "usb host" : vbus == VBUS_CDP ? "usb charging downstream port" :
		    vbus == VBUS_DCP ? "usb charger" : vbus == VBUS_HVDCP ? "usb hvdcp charger" : "usb";
		const char *names[] = { "Watts", "Current", "AdapterVoltage", "FamilyCode" };
		uint64_t values[] = { (uint64_t)(vbusMv * limitMa + 500000) / 1000000, limitMa, vbusMv, family };
		for (unsigned i = 0; external && i < 4; i++) {
			OSNumber *n = OSNumber::withNumber(values[i], 32);
			if (n != NULL) {
				adapter->setObject(names[i], n);
				n->release();
			}
		}
		if (external) {
			OSString *d = OSString::withCString(desc);
			if (d != NULL) {
				adapter->setObject("Description", d);
				d->release();
			}
			adapter->setObject("IsWireless", kOSBooleanFalse);
		}
		const OSSymbol *key = OSSymbol::withCString("AdapterDetails");
		if (key != NULL) {
			setPSProperty(key, adapter);
			key->release();
		}
		adapter->release();
	}

	/* IOPMCopyBatteryInfo's legacy dictionary */
	OSDictionary *legacy = OSDictionary::withCapacity(6);
	if (legacy != NULL) {
		const char *keys[] = { kIOBatteryCurrentChargeKey, kIOBatteryCapacityKey, kIOBatteryVoltageKey,
			               kIOBatteryAmperageKey, kIOBatteryCycleCountKey, kIOBatteryFlagsKey };
		uint32_t values[] = { s->fgOK ? s->rm : fcc * soc / 100, fcc, s->fgOK ? s->mv : 4000,
			              (uint32_t)(s->fgOK ? s->avgMa : 0), s->fgOK ? s->cycles : 0,
			              (uint32_t)(kIOBatteryInstalled | (external ? kIOBatteryChargerConnect : 0) |
			              (charging ? kIOBatteryCharge : 0)) };
		for (unsigned i = 0; i < 6; i++) {
			OSNumber *n = OSNumber::withNumber(values[i], 32);
			if (n != NULL) {
				legacy->setObject(keys[i], n);
				n->release();
			}
		}
		setLegacyIOBatteryInfo(legacy);
		legacy->release();
	}
	updateStatus();

	if (!fHaveSample || external != fLastExternal || vbus != fLastVbus || chrg != fLastChrg || soc != fLastSoc ||
	    fLastLPM != (gVivaLowPower != 0)) {
		IOLog("VIVA BATTERY: %u%% %umV %dmA %d.%dC, %s%s%s, vbus %u chrg %u%s%s\n", soc, s->fgOK ? s->mv : 0,
		    s->fgOK ? s->avgMa : 0, s->fgOK ? s->tempDeciC / 10 : 0, s->fgOK ? tenths(s->tempDeciC) : 0,
		    external ? "external" : "battery", charging ? ", charging" : "", full ? ", full" : "", vbus, chrg,
		    s->fgOK ? "" : ", NO GAUGE", s->chgOK ? "" : ", NO CHARGER");
	}
	fLastExternal = external;
	fLastVbus = vbus;
	fLastChrg = chrg;
	fLastSoc = soc;
	fLastLPM = gVivaLowPower != 0;
	fHaveSample = true;
}

void
VivaBattery::poll(void)
{
	VivaBatterySample s;
	fPolls++;
	readSample(&s);
	maintainCharger(&s);
	publish(&s);
	fLast = s;
}

void
VivaBattery::timerFired(OSObject *owner, IOTimerEventSource *sender)
{
	VivaBattery *self = OSDynamicCast(VivaBattery, owner);
	if (self != NULL) {
		self->poll();
		sender->setTimeoutMS(POLL_MS);
	}
}

/* Charger INT (EINT 11): plug, unplug, charge state change. Poll shortly after (VBUS detection settles). */
void
VivaBattery::interruptOccurred(OSObject *owner, IOInterruptEventSource *src __unused, int count __unused)
{
	VivaBattery *self = OSDynamicCast(VivaBattery, owner);
	if (self != NULL) {
		self->fIrqs++;
		self->fTimer->setTimeoutMS(IRQ_DELAY_MS);
	}
}

void
VivaBattery::pollNow(void)
{
	if (fTimer != NULL) {
		fTimer->setTimeoutMS(1);
	}
}

void
VivaBattery::lowPowerChanged(void)
{
	pollNow();
}

bool
VivaBattery::start(IOService *provider)
{
	int enable = 1;
	PE_parse_boot_argn("vivabattery", &enable, sizeof(enable));
	if (!enable) {
		return false;
	}
	fBus = OSDynamicCast(VivaI2C, provider->getProvider());
	fFgAddr = (uint8_t)dtU32(provider, "i2c-address", 0x55);
	fChgAddr = (uint8_t)dtU32(provider, "charger-address", 0x6a);
	fDesignDefault = dtU32(provider, "design-capacity", 5000);
	fMaxMv = MIN(dtU32(provider, "charge-voltage-max", 4448), 4448u);
	fMaxMa = MIN(dtU32(provider, "charge-current-max", 3000), 3000u);
	fChargeMode = 1;
	PE_parse_boot_argn("vivacharge", &fChargeMode, sizeof(fChargeMode));
	fInputMa = 2000;
	PE_parse_boot_argn("vivachargein", &fInputMa, sizeof(fInputMa));      /* wall charger input limit, 0 = never set */
	fInputMa = fInputMa != 0 ? MIN(MAX(fInputMa, 500u), 3000u) : 0;
	PE_parse_boot_argn("vivachargelimit", &gVivaChargeLimit, sizeof(gVivaChargeLimit));
	gVivaChargeLimit = MIN(MAX(gVivaChargeLimit, 50), 100);
	PE_parse_boot_argn("vivalpmgpu", &gVivaLPMGpuMHz, sizeof(gVivaLPMGpuMHz));
	PE_parse_boot_argn("vivalpmduty", &gVivaLPMDuty, sizeof(gVivaLPMDuty));
	gVivaLPMDuty = MIN(MAX(gVivaLPMDuty, 1u), 100u);
	fWorkLoop = IOWorkLoop::workLoop();
	if (fBus == NULL || fWorkLoop == NULL || !IOPMPowerSource::start(provider)) {
		IOLog("VIVA BATTERY: no I2C bus or work loop\n");
		return false;
	}
	fGate = IOCommandGate::commandGate(this);
	fTimer = IOTimerEventSource::timerEventSource(this, &VivaBattery::timerFired);
	if (fGate == NULL || fTimer == NULL || fWorkLoop->addEventSource(fGate) != kIOReturnSuccess ||
	    fWorkLoop->addEventSource(fTimer) != kIOReturnSuccess) {
		IOLog("VIVA BATTERY: no timer\n");
		return false;
	}
	/* a complete source before registerService wakes powerd */
	fGate->runActionBlock(^IOReturn {
		poll();
		return kIOReturnSuccess;
	});
	IOLog("VIVA BATTERY: gauge 0x%02x %s (design %u mAh, full %u mAh, health %u %%, %u cycles), charger 0x%02x %s "
	    "(PN %u rev %u), writes %s, limits %u mV %u mA\n", fFgAddr, fLast.fgOK ? "ok" : "NOT ANSWERING",
	    fLast.design, fLast.fcc, fLast.soh, fLast.cycles, fChgAddr, fLast.chgOK ? "ok" : "NOT ANSWERING",
	    (fLast.r[CHG_REG14] >> 3) & 7, fLast.r[CHG_REG14] & 3, fChargeMode ? "on" : "off", fMaxMv, fMaxMa);
	fInterrupt = IOInterruptEventSource::interruptEventSource(this, &VivaBattery::interruptOccurred, provider, 0);
	if (fInterrupt != NULL && fWorkLoop->addEventSource(fInterrupt) == kIOReturnSuccess) {
		fInterrupt->enable();
	} else {
		IOLog("VIVA BATTERY: no charger interrupt, polling only\n");
		OSSafeReleaseNULL(fInterrupt);
	}
	fTimer->setTimeoutMS(POLL_MS);
	gVivaBattery = this;
	registerService();
	return true;
}

void
VivaBattery::free(void)
{
	if (fWorkLoop != NULL) {
		if (fInterrupt != NULL) {
			fInterrupt->disable();
			fWorkLoop->removeEventSource(fInterrupt);
		}
		if (fTimer != NULL) {
			fTimer->cancelTimeout();
			fWorkLoop->removeEventSource(fTimer);
		}
		if (fGate != NULL) {
			fWorkLoop->removeEventSource(fGate);
		}
	}
	OSSafeReleaseNULL(fInterrupt);
	OSSafeReleaseNULL(fTimer);
	OSSafeReleaseNULL(fGate);
	OSSafeReleaseNULL(fWorkLoop);
	IOPMPowerSource::free();
}

#define say(...) do { if (o < len) o += (size_t)snprintf(out + o, len - o, __VA_ARGS__); } while (0)

void
VivaBattery::link(const char *args, char *out, size_t len)
{
	size_t o = 0;
	while (*args == ' ') {
		args++;
	}
	if (strncmp(args, "lpm", 3) == 0) {
		const char *a = args + 3;
		while (*a == ' ') {
			a++;
		}
		if (strncmp(a, "on", 2) == 0 || strncmp(a, "off", 3) == 0) {
			int on = a[1] == 'n';
			if (on != gVivaLowPower) {
				gVivaLowPower = on;
				viva_power_apply();
			}
		}
		say("low power mode %s (GPU ceiling %u MHz, panel duty %u %% when on)\n", gVivaLowPower ? "on" : "off",
		    gVivaLPMGpuMHz, gVivaLPMDuty);
		return;
	}
	fGate->runActionBlock(^IOReturn {
		if (strncmp(args, "poll", 4) == 0) {
			poll();
		}
		return kIOReturnSuccess;
	});
	const VivaBatterySample &s = fLast;
	say("gauge 0x%02x %s: %u%% (rm %u / fcc %u / design %u mAh), health %u%%, %u cycles, %u mV, now %d mA, avg %d mA, "
	    "%d.%d C, status 0x%04x%s%s%s%s, tte %u ttf %u min, wants %u mV %u mA\n", fFgAddr, s.fgOK ? "ok" : "ERROR",
	    s.soc, s.rm, s.fcc, s.design, s.soh, s.cycles, s.mv, s.ma, s.avgMa, s.tempDeciC / 10, tenths(s.tempDeciC),
	    s.status, (s.status & FG_STATUS_INIT) ? " INIT" : "", (s.status & FG_STATUS_DSG) ? " DSG" : "",
	    (s.status & FG_STATUS_FC) ? " FC" : "", (s.status & FG_STATUS_FD) ? " FD" : "", s.tte, s.ttf, s.chgMv, s.chgMa);
	uint32_t pn = (s.r[CHG_REG14] >> 3) & 7, step = pn == PN_SC89890H ? 60 : 64;
	say("charger 0x%02x %s: PN %u, vbus_stat %u chrg_stat %u pg %u, vbus %u mV%s, ichg %u mA (set %u), input limit %u mA, "
	    "vreg %u mV, faults 0x%02x, wd %s, charging %s\n", fChgAddr, s.chgOK ? "ok" : "ERROR", pn, s.r[CHG_REG0B] >> 5,
	    (s.r[CHG_REG0B] >> 3) & 3, (s.r[CHG_REG0B] >> 2) & 1, 2600 + 100 * (s.r[CHG_REG11] & 0x7f),
	    (s.r[CHG_REG11] & 0x80) ? " good" : "", 50 * (s.r[CHG_REG12] & 0x7f), step * (s.r[CHG_REG04] & R04_ICHG),
	    100 + 50 * (s.r[CHG_REG13] & 0x3f), VREG_BASE_MV + VREG_STEP_MV * (s.r[CHG_REG06] >> R06_VREG_SHIFT),
	    s.r[CHG_REG0C], (s.r[CHG_REG07] & R07_WD_MASK) == 0 ? "off" : "on",
	    (s.r[CHG_REG03] & R03_CHG_CONFIG) ? "enabled" : "DISABLED");
	say("charger regs:");
	for (unsigned i = 0; i < sizeof(s.r); i++) {
		say(" %02x", s.r[i]);
	}
	say("\nwrites %s (%u, %u failed; last %s), limit %d%%%s, low power %s, polls %u, irqs %u, errors gauge %u charger %u\n",
	    fChargeMode ? "on" : "off", fWrites, fWriteFails, fLastWrite[0] ? fLastWrite : "-", gVivaChargeLimit,
	    fLimitHold ? " (holding)" : "", gVivaLowPower ? "on" : "off", fPolls, fIrqs, fFgErrors, fChgErrors);
}

#undef say

extern "C" void
viva_battery_link(const char *args, char *out, size_t len)
{
	if (len == 0) {
		return;
	}
	out[0] = 0;
	if (gVivaBattery == NULL) {
		snprintf(out, len, "battery: VivaBattery not started (vivabattery=0, no /arm-io/i2c3/battery, or I2C3 failed)\n");
		return;
	}
	gVivaBattery->link(args, out, len);
}

/* SweetBattery steps aside when this driver will run. */
extern "C" bool
viva_battery_present(void)
{
	int enable = 1;
	PE_parse_boot_argn("vivabattery", &enable, sizeof(enable));
	if (!enable) {
		return false;
	}
	IORegistryEntry *e = IORegistryEntry::fromPath("/arm-io/i2c3/battery", gIODTPlane);
	if (e == NULL) {
		return false;
	}
	e->release();
	return true;
}
#endif /* SWEET */
