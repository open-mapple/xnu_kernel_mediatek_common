/*
 * viva (MT6781): Goodix GT9896 touch controller, the built-in touch screen.
 * Matches /arm-io/i2c0/touch ("goodix,gt9896"), a child of VivaI2C
 * (../VivaIO.cpp); it is a SweetHIDTouch, so contacts reach backboardd as
 * the same digitizer collection events (fingers, edge tips) as on sweet.
 *
 * Protocol and power sequence from Xiaomi's driver for this phone
 * (viva-r-oss drivers/input/touchscreen/goodix_gtx8). This GT9896 is a
 * "Yellowstone" IC: its register map comes from the vendor's
 * goodix_cfg_group.bin (ic_type "yellowstone", pid "9896"), copied into the
 * ADT: coordinates 0x4180, pid 0x4022, fw_request 0x4180 (2026-11-03).
 *  - power: avdd GPIO high, 100 us, dvdd GPIO high (load switches); reset
 *    GPIO low 2 ms, high, 110 ms, then the IC answers on I2C
 *  - registers have 16-bit big-endian addresses
 *  - interrupt (EINT 13, falling edge) -> 8-byte event head at coor:
 *    byte 0 bit 7 touch event, bit 6 request; byte 2 low nibble = contacts;
 *    head checksum: sum of bytes 0-5 == big-endian bytes 6-7. Then 8 bytes
 *    per contact (id << 4 | type, -, x BE16, y BE16, w BE16) and a BE16
 *    sum of the contact bytes; write 0 to coor when done
 *  - coordinates 0..panel-max (8640 x 19200 = 8x the 1080 x 2400 panel)
 * Reads go in 8-byte pieces (VivaI2C FIFO transfers).
 *
 * ADT: i2c-address, avdd-gpio, dvdd-gpio, reset-gpio, max-x, max-y,
 * reg-coor, reg-pid, reg-fw-request; the interrupt is
 * interrupts/interrupt-parent (VivaEINT).
 */
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOInterruptEventSource.h>

#include "SweetHIDTouch.h"
#include "../VivaIO.h"

extern "C" void viva_cpufreq_ui_activity(void);        /* osfmk/arm64/viva_cpufreq.c: 0183 UI boost */

#define GT_REG_TEST             0x4100
#define GT_EVENT_TOUCH          0x80
#define GT_EVENT_REQUEST        0x40
#define GT_BYTES_PER_COORD      8
#define GT_HEAD_LEN             8       /* Yellowstone IRQ_HEAD_LEN_YS */
#define GT_MAX_TOUCH            10
#define GT_POINT_STYLUS         0x01
#define GT_POINT_STYLUS_HOVER   0x03
#define GT_REQUEST_CONFIG       0x01
#define GT_REQUEST_RESET        0x03
#define PANEL_W                 1080
#define PANEL_H                 2400

class VivaGT9896 : public SweetHIDTouch {
	OSDeclareDefaultStructors(VivaGT9896);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOWorkLoop *getWorkLoop(void) const APPLE_KEXT_OVERRIDE;

protected:
	virtual OSString *getProduct() APPLE_KEXT_OVERRIDE { return OSString::withCString("GT9896"); }
	virtual UInt32 getVendorID() APPLE_KEXT_OVERRIDE { return 0x27c6; }        /* Goodix */
	virtual UInt32 getProductID() APPLE_KEXT_OVERRIDE { return 0x9896; }

private:
	static void interruptOccurred(OSObject *owner, IOInterruptEventSource *src, int count);
	IOReturn readReg(uint16_t reg, uint8_t *buf, uint32_t len);
	IOReturn writeReg(uint16_t reg, const uint8_t *buf, uint32_t len);
	bool powerOn(void);
	void handleEvent(void);
	void handleRequest(void);

	VivaI2C                *fBus;
	VivaGPIO               *fGPIO;
	IOWorkLoop             *fWorkLoop;
	IOInterruptEventSource *fInterrupt;
	uint8_t                 fAddr;
	uint32_t                fAvdd, fDvdd, fReset, fMaxX, fMaxY;
	uint16_t                fRegCoor, fRegPid, fRegRequest;
	uint32_t                fHeadErrors;
	uint32_t                fPrevMap;
	uint32_t                fEvents, fBadSums;
};

OSDefineMetaClassAndStructors(VivaGT9896, SweetHIDTouch);

static uint32_t
dtU32(IOService *provider, const char *key, uint32_t dflt)
{
	OSData *d = OSDynamicCast(OSData, provider->getProperty(key));
	return (d != NULL && d->getLength() >= 4) ? *(const uint32_t *)d->getBytesNoCopy() : dflt;
}

IOWorkLoop *
VivaGT9896::getWorkLoop(void) const
{
	return fWorkLoop;
}

IOReturn
VivaGT9896::readReg(uint16_t reg, uint8_t *buf, uint32_t len)
{
	while (len > 0) {
		uint32_t n = len < (uint32_t)VivaI2C::kFIFOSize ? len : (uint32_t)VivaI2C::kFIFOSize;
		uint8_t a[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
		IOReturn ret = fBus->transfer(fAddr, a, 2, buf, n);
		if (ret != kIOReturnSuccess) {
			return ret;
		}
		reg += n;
		buf += n;
		len -= n;
	}
	return kIOReturnSuccess;
}

IOReturn
VivaGT9896::writeReg(uint16_t reg, const uint8_t *buf, uint32_t len)
{
	uint8_t w[VivaI2C::kFIFOSize] = { (uint8_t)(reg >> 8), (uint8_t)reg };
	if (len > VivaI2C::kFIFOSize - 2) {
		return kIOReturnBadArgument;
	}
	memcpy(w + 2, buf, len);
	return fBus->transfer(fAddr, w, 2 + len, NULL, 0);
}

/* goodix_ts_power_on + goodix_ts_dev_confirm */
bool
VivaGT9896::powerOn(void)
{
	fGPIO->setOutput(fReset, false);
	fGPIO->setOutput(fAvdd, true);
	IODelay(100);
	fGPIO->setOutput(fDvdd, true);
	IOSleep(10);
	for (int retry = 0; retry < 3; retry++) {
		fGPIO->setOutput(fReset, false);
		IODelay(2000);
		fGPIO->setOutput(fReset, true);
		IOSleep(110);
		uint8_t test = 0;
		if (readReg(GT_REG_TEST, &test, 1) == kIOReturnSuccess) {
			IOSleep(95);
			return true;
		}
	}
	return false;
}

bool
VivaGT9896::start(IOService *provider)
{
	fBus = OSDynamicCast(VivaI2C, provider->getProvider());
	fGPIO = VivaGPIO::wait();
	fWorkLoop = IOWorkLoop::workLoop();
	fAddr = (uint8_t)dtU32(provider, "i2c-address", 0x5d);
	fAvdd = dtU32(provider, "avdd-gpio", 166);
	fDvdd = dtU32(provider, "dvdd-gpio", 73);
	fReset = dtU32(provider, "reset-gpio", 17);
	fMaxX = dtU32(provider, "max-x", 8640);
	fMaxY = dtU32(provider, "max-y", 19200);
	fRegCoor = (uint16_t)dtU32(provider, "reg-coor", 0x4180);
	fRegPid = (uint16_t)dtU32(provider, "reg-pid", 0x4022);
	fRegRequest = (uint16_t)dtU32(provider, "reg-fw-request", 0x4180);
	if (fBus == NULL || fGPIO == NULL || fWorkLoop == NULL) {
		IOLog("VIVA TOUCH: missing I2C bus, GPIO or work loop\n");
		return false;
	}
	if (!powerOn()) {
		IOLog("VIVA TOUCH: GT9896 at 0x%02x does not answer after reset\n", fAddr);
		return false;
	}
	uint8_t pid[4] = { 0 };
	readReg(fRegPid, pid, sizeof(pid));
	IOLog("VIVA TOUCH: pid %02x %02x %02x %02x ('%c%c%c%c'), coordinates at 0x%04x\n",
	    pid[0], pid[1], pid[2], pid[3], pid[0] >= 0x20 && pid[0] < 0x7f ? pid[0] : '.',
	    pid[1] >= 0x20 && pid[1] < 0x7f ? pid[1] : '.', pid[2] >= 0x20 && pid[2] < 0x7f ? pid[2] : '.',
	    pid[3] >= 0x20 && pid[3] < 0x7f ? pid[3] : '.', fRegCoor);
	uint8_t zero = 0;
	writeReg(fRegCoor, &zero, 1);

	/* HID service (SweetHIDTouch publishes the digitizer properties) */
	if (!SweetHIDTouch::start(provider)) {
		return false;
	}
	fInterrupt = IOInterruptEventSource::interruptEventSource(this, &VivaGT9896::interruptOccurred, provider, 0);
	if (fInterrupt == NULL || fWorkLoop->addEventSource(fInterrupt) != kIOReturnSuccess) {
		IOLog("VIVA TOUCH: no interrupt\n");
		return false;
	}
	fInterrupt->enable();
	IOLog("VIVA TOUCH: GT9896 at 0x%02x on, %ux%u\n", fAddr, fMaxX, fMaxY);
	return true;
}

void
VivaGT9896::free(void)
{
	if (fInterrupt != NULL) {
		fInterrupt->disable();
		if (fWorkLoop != NULL) {
			fWorkLoop->removeEventSource(fInterrupt);
		}
	}
	OSSafeReleaseNULL(fInterrupt);
	SweetHIDTouch::free();
	OSSafeReleaseNULL(fWorkLoop);
}

void
VivaGT9896::interruptOccurred(OSObject *owner, IOInterruptEventSource *src __unused, int count __unused)
{
	VivaGT9896 *self = OSDynamicCast(VivaGT9896, owner);
	if (self != NULL) {
		self->handleEvent();
	}
}

/* goodix_request_handler: the IC asks for something. */
void
VivaGT9896::handleRequest(void)
{
	uint8_t req = 0, zero = 0;
	readReg(fRegRequest, &req, 1);
	IOLog("VIVA TOUCH: request 0x%02x%s\n", req,
	    req == GT_REQUEST_CONFIG ? " (config: not sent, the IC keeps its flash config)" :
	    req == GT_REQUEST_RESET ? " (reset)" : "");
	if (req == GT_REQUEST_RESET) {
		fGPIO->setOutput(fReset, false);
		IODelay(2000);
		fGPIO->setOutput(fReset, true);
		IOSleep(100);
	}
	writeReg(fRegRequest, &zero, 1);
}

static inline uint16_t
be16(const uint8_t *p)
{
	return (uint16_t)(p[0] << 8 | p[1]);
}

/* checksum_u8_ys: the byte sum of all but the last 2 bytes equals them (big endian). */
static bool
sumOK(const uint8_t *p, uint32_t len)
{
	uint16_t sum = 0;
	for (uint32_t i = 0; i < len - 2; i++) {
		sum += p[i];
	}
	return sum == be16(p + len - 2);
}

/* Work loop: goodix_event_handler + goodix_touch_handler_ys. */
void
VivaGT9896::handleEvent(void)
{
	uint8_t buf[GT_HEAD_LEN + GT_BYTES_PER_COORD * GT_MAX_TOUCH + 2];
	uint32_t pre = GT_HEAD_LEN + GT_BYTES_PER_COORD + 2;
	uint8_t zero = 0;

	if (readReg(fRegCoor, buf, pre) != kIOReturnSuccess) {
		return;
	}
	if (!sumOK(buf, GT_HEAD_LEN)) {
		if (++fHeadErrors <= 8) {
			IOLog("VIVA TOUCH: head checksum %02x %02x %02x %02x %02x %02x %02x %02x\n",
			    buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
		}
		if (fHeadErrors % 2 == 0) {
			writeReg(fRegCoor, &zero, 1);
		}
		return;
	}
	if (!(buf[0] & GT_EVENT_TOUCH)) {
		if (buf[0] & GT_EVENT_REQUEST) {
			handleRequest();
		}
		writeReg(fRegCoor, &zero, 1);
		return;
	}
	uint32_t n = buf[2] & 0x0f;
	if (n > GT_MAX_TOUCH) {
		writeReg(fRegCoor, &zero, 1);
		return;
	}
	if (n > 1 && readReg(fRegCoor + pre, buf + pre, (n - 1) * GT_BYTES_PER_COORD) != kIOReturnSuccess) {
		return;
	}
	writeReg(fRegCoor, &zero, 1);

	if (n > 0 && !sumOK(buf + GT_HEAD_LEN, n * GT_BYTES_PER_COORD + 2)) {
		if (++fBadSums <= 8) {
			IOLog("VIVA TOUCH: contact checksum (n %u)\n", n);
		}
		return;
	}
	if (n > 0) {
		uint8_t type = buf[GT_HEAD_LEN + (n - 1) * GT_BYTES_PER_COORD] & 0x0f;
		if (type == GT_POINT_STYLUS || type == GT_POINT_STYLUS_HOVER) {
			return;         /* pen report: not a finger */
		}
	}

	fEvents++;
	viva_cpufreq_ui_activity();     /* 0183: the CPU clusters ramp while a finger is down */
	uint32_t map = 0;
	const uint8_t *c = buf + GT_HEAD_LEN;
	for (uint32_t i = 0; i < n; i++, c += GT_BYTES_PER_COORD) {
		uint32_t id = (c[0] >> 4) & 0x0f;
		if (id >= GT_MAX_TOUCH || id >= SWEET_TOUCH_FINGERS) {
			break;
		}
		uint32_t x = be16(c + 2), y = be16(c + 4);
		map |= 1u << id;
		touch((int)id, true, (int)(x * PANEL_W / fMaxX), (int)(y * PANEL_H / fMaxY));
	}
	for (uint32_t id = 0; id < SWEET_TOUCH_FINGERS; id++) {
		if ((fPrevMap & (1u << id)) && !(map & (1u << id))) {
			touch((int)id, false, 0, 0);
		}
	}
	fPrevMap = map;
}
