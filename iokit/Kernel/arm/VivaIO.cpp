/*
 * viva (MT6781): GPIO and I2C bus drivers (see VivaIO.h).
 *
 * GPIO (0x10005000, Xiaomi viva-r-oss pinctrl-mt6781.c /
 * pinctrl-mtk-common-v2.c): DIR 0x000, DOUT 0x100, DIN 0x200, + 0x10 per
 * 32 pins, race-free SET at +4 and CLR at +8. Pin modes are left as LK set
 * them (the touch pins are already GPIO).
 *
 * I2C v2 (i2c-mtk.c, i2c_common: ver 2, set_ltiming, cnt_constraint,
 * set_dt_div, ext_time_config 0x1801): source clock from the ADT
 * ("source-clock", Hz, before "clock-div"), timings computed like
 * mtk_i2c_calculate_speed/i2c_set_speed (45 % duty above 100 kHz), FIFO
 * transfers of up to 8 bytes, completion on the controller's interrupt.
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include "VivaIO.h"

/* ---- GPIO ---- */

#define GPIO_DIR        0x000
#define GPIO_DOUT       0x100
#define GPIO_DIN        0x200
#define GPIO_SET        0x4
#define GPIO_CLR        0x8
#define GPIO_PINS       203

OSDefineMetaClassAndStructors(VivaGPIO, IOService);

static VivaGPIO *gVivaGPIO;

bool
VivaGPIO::start(IOService *provider)
{
	if (!IOService::start(provider)) {
		return false;
	}
	fMap = provider->mapDeviceMemoryWithIndex(0);
	if (fMap == NULL) {
		return false;
	}
	fRegs = (vm_offset_t)fMap->getVirtualAddress();
	fOutputs = OSDynamicCast(OSData, provider->getProperty("output-pins"));
	if (fOutputs != NULL) {
		fOutputs->retain();
	}
	IOLog("VIVA GPIO: %u output pins allowed\n", fOutputs ? fOutputs->getLength() / 4 : 0);
	gVivaGPIO = this;
	registerService();
	return true;
}

VivaGPIO *
VivaGPIO::wait(void)
{
	for (int i = 0; i < 100 && gVivaGPIO == NULL; i++) {
		IOSleep(50);
	}
	return gVivaGPIO;
}

bool
VivaGPIO::outputAllowed(uint32_t pin)
{
	if (fOutputs == NULL) {
		return false;
	}
	const uint32_t *pins = (const uint32_t *)fOutputs->getBytesNoCopy();
	for (unsigned int i = 0; i < fOutputs->getLength() / 4; i++) {
		if (pins[i] == pin) {
			return true;
		}
	}
	return false;
}

IOReturn
VivaGPIO::setOutput(uint32_t pin, bool high)
{
	if (pin >= GPIO_PINS || !outputAllowed(pin)) {
		IOLog("VIVA GPIO: refused output on pin %u\n", pin);
		return kIOReturnNotPermitted;
	}
	uint32_t bank = 0x10 * (pin / 32), bit = 1u << (pin % 32);
	wr32(GPIO_DOUT + bank + (high ? GPIO_SET : GPIO_CLR), bit);   /* level first, then drive */
	wr32(GPIO_DIR + bank + GPIO_SET, bit);
	return kIOReturnSuccess;
}

IOReturn
VivaGPIO::getInput(uint32_t pin, bool *high)
{
	if (pin >= GPIO_PINS || high == NULL) {
		return kIOReturnBadArgument;
	}
	*high = (rd32(GPIO_DIN + 0x10 * (pin / 32)) >> (pin % 32)) & 1;
	return kIOReturnSuccess;
}

/* ---- I2C ---- */

#define I2C_DATA_PORT           0x00
#define I2C_SLAVE_ADDR          0x04
#define I2C_INTR_MASK           0x08
#define I2C_INTR_STAT           0x0c
#define I2C_CONTROL             0x10
#define I2C_TRANSFER_LEN        0x14
#define I2C_TRANSAC_LEN         0x18
#define I2C_DELAY_LEN           0x1c
#define I2C_TIMING              0x20
#define I2C_START               0x24
#define I2C_EXT_CONF            0x28
#define I2C_LTIMING             0x2c
#define I2C_HS                  0x30
#define I2C_IO_CONFIG           0x34
#define I2C_FIFO_ADDR_CLR       0x38
#define I2C_MCU_INTR            0x40
#define I2C_TRANSFER_LEN_AUX    0x44
#define I2C_CLOCK_DIV           0x48
#define I2C_HW_TIMEOUT          0x4c
#define I2C_SOFTRESET           0x50

#define INTR_BUS_ERR            (1u << 8)
#define INTR_TIMEOUT            (1u << 5)
#define INTR_ARB_LOST           (1u << 3)
#define INTR_HS_NACKERR         (1u << 2)
#define INTR_ACKERR             (1u << 1)
#define INTR_TRANSAC_COMP       (1u << 0)
#define INTR_ALL                0x1ff
#define INTR_ERRORS             (INTR_BUS_ERR | INTR_TIMEOUT | INTR_HS_NACKERR | INTR_ACKERR | INTR_ARB_LOST)

#define CONTROL_RS              (1u << 1)
#define CONTROL_CLK_EXT_EN      (1u << 3)
#define CONTROL_DIR_CHANGE      (1u << 4)
#define CONTROL_ACKERR_DET_EN   (1u << 5)

#define TIMING_TIMEOUT_EN       0x0001
#define IO_CONFIG_OPEN_DRAIN    0x0003
#define DELAY_LEN               0x000a
#define MAX_STEP_CNT_DIV        64
#define MAX_SAMPLE_CNT_DIV      8
#define MAX_SCL_LOW_TIME_MS     2

OSDefineMetaClassAndStructors(VivaI2C, IOService);

static uint32_t
dtU32(IOService *provider, const char *key, uint32_t dflt)
{
	OSData *d = OSDynamicCast(OSData, provider->getProperty(key));
	return (d != NULL && d->getLength() >= 4) ? *(const uint32_t *)d->getBytesNoCopy() : dflt;
}

IOWorkLoop *
VivaI2C::getWorkLoop(void) const
{
	return fWorkLoop;
}

/* i2c-mtk.c mtk_i2c_calculate_speed, ST/FS range, cnt_constraint set. */
bool
VivaI2C::calcSpeed(uint32_t srcHz, uint32_t speedHz, uint32_t *step, uint32_t *sample)
{
	uint32_t khz = speedHz / 1000, hclk = srcHz / 1000;
	uint32_t min_div = ((hclk >> 1) + khz - 1) / khz;
	uint32_t best = MAX_SAMPLE_CNT_DIV * MAX_STEP_CNT_DIV, sdiv = MAX_SAMPLE_CNT_DIV, stdiv = MAX_STEP_CNT_DIV;

	for (uint32_t s = 1; s <= MAX_SAMPLE_CNT_DIV; s++) {
		uint32_t st = (min_div + s - 1) / s;
		if (st > MAX_STEP_CNT_DIV) {
			continue;
		}
		if (st * s < best) {
			best = st * s;
			sdiv = s;
			stdiv = st;
			if (best == min_div) {
				break;
			}
		}
	}
	if (hclk / (2 * sdiv * stdiv) > khz) {
		return false;
	}
	if (--sdiv) {
		stdiv--;
	}
	*step = stdiv;
	*sample = sdiv;
	return true;
}

bool
VivaI2C::start(IOService *provider)
{
	fWorkLoop = IOWorkLoop::workLoop();
	if (fWorkLoop == NULL || !IOService::start(provider)) {
		return false;
	}
	fMap = provider->mapDeviceMemoryWithIndex(0);
	if (fMap == NULL) {
		IOLog("VIVA I2C: cannot map registers\n");
		return false;
	}
	fRegs = (vm_offset_t)fMap->getVirtualAddress();
	fClockDiv = dtU32(provider, "clock-div", 1);
	fSrcHz = dtU32(provider, "source-clock", 0) / (fClockDiv ? fClockDiv : 1);
	fSpeedHz = dtU32(provider, "clock-frequency", 400000);
	fExtConf = (uint16_t)dtU32(provider, "ext-time-config", 0x1801);

	/* 45 % duty above 100 kHz (i2c_set_speed with set_ltiming), high and low phases */
	uint32_t duty = fSpeedHz > 100000 ? 45 : 50, step, sample, lstep, lsample;
	if (fSrcHz == 0 || fSpeedHz > 400000 || fClockDiv > 8 ||
	    !calcSpeed(fSrcHz, fSpeedHz * 50 / duty, &step, &sample) ||
	    !calcSpeed(fSrcHz, fSpeedHz * 50 / (100 - duty), &lstep, &lsample)) {
		IOLog("VIVA I2C: unsupported clocks (source %u Hz, speed %u Hz, div %u)\n", fSrcHz, fSpeedHz, fClockDiv);
		return false;
	}
	fTiming = (uint16_t)(((sample & 7) << 8) | (step & 0x3f));
	fLTiming = (uint16_t)(((lsample & 7) << 6) | (lstep & 0x3f));
	uint32_t tv1 = fSrcHz / 1000 * MAX_SCL_LOW_TIME_MS, tv2 = ((fLTiming & 0x3f) + 1) * (((fLTiming & 0x1c0) >> 6) + 1);
	fHWTimeout = (uint16_t)((tv1 + tv2 - 1) / tv2);

	fGate = IOCommandGate::commandGate(this);
	fInterrupt = IOInterruptEventSource::interruptEventSource(this, &VivaI2C::interruptOccurred, provider, 0);
	if (fGate == NULL || fInterrupt == NULL || fWorkLoop->addEventSource(fGate) != kIOReturnSuccess ||
	    fWorkLoop->addEventSource(fInterrupt) != kIOReturnSuccess) {
		IOLog("VIVA I2C: no gate/interrupt\n");
		return false;
	}
	initHW();
	fInterrupt->enable();
	IOLog("VIVA I2C: %s, source %u Hz, %u Hz: timing 0x%04x ltiming 0x%04x hw_timeout 0x%04x\n",
	    provider->getName(), fSrcHz, fSpeedHz, fTiming, fLTiming, fHWTimeout);
	registerService();
	VivaDTNub::publishChildren(this, provider);
	return true;
}

void
VivaI2C::free(void)
{
	if (fWorkLoop != NULL) {
		if (fInterrupt != NULL) {
			fInterrupt->disable();
			fWorkLoop->removeEventSource(fInterrupt);
		}
		if (fGate != NULL) {
			fWorkLoop->removeEventSource(fGate);
		}
	}
	OSSafeReleaseNULL(fInterrupt);
	OSSafeReleaseNULL(fGate);
	OSSafeReleaseNULL(fMap);
	OSSafeReleaseNULL(fWorkLoop);
	IOService::free();
}

/* mt_i2c_init_hw: interrupts off and cleared, soft reset, static configuration. */
void
VivaI2C::initHW(void)
{
	wr16(I2C_INTR_MASK, 0);
	wr16(I2C_INTR_STAT, rd16(I2C_INTR_STAT));
	wr16(I2C_SOFTRESET, 1);
	wr16(I2C_IO_CONFIG, IO_CONFIG_OPEN_DRAIN);
	wr16(I2C_TIMING, fTiming | TIMING_TIMEOUT_EN);
	wr16(I2C_LTIMING, fLTiming);
	wr16(I2C_HS, 0);
}

/* Work loop: the controller's interrupt. */
void
VivaI2C::interruptOccurred(OSObject *owner, IOInterruptEventSource *src __unused, int count __unused)
{
	VivaI2C *self = OSDynamicCast(VivaI2C, owner);
	if (self == NULL) {
		return;
	}
	uint16_t stat = self->rd16(I2C_INTR_STAT);
	self->wr16(I2C_INTR_STAT, stat);
	if (stat == 0) {
		return;
	}
	self->fIrqStat |= stat;
	self->fDone = true;
	self->fGate->commandWakeup(&self->fDone);
}

IOReturn
VivaI2C::transfer(uint8_t addr, const uint8_t *wbuf, uint32_t wlen, uint8_t *rbuf, uint32_t rlen)
{
	if (addr > 0x7f || wlen == 0 || wlen > kFIFOSize || rlen > kFIFOSize || (rlen && rbuf == NULL)) {
		return kIOReturnBadArgument;
	}
	return fGate->runActionBlock(^IOReturn {
		return transferGated(addr, wbuf, wlen, rbuf, rlen);
	});
}

/* i2c-mtk.c mt_i2c_do_transfer, FIFO path (op WR or WRRD). */
IOReturn
VivaI2C::transferGated(uint8_t addr, const uint8_t *wbuf, uint32_t wlen, uint8_t *rbuf, uint32_t rlen)
{
	bool wrrd = rlen != 0;

	wr16(I2C_HW_TIMEOUT, fHWTimeout);
	wr16(I2C_CLOCK_DIV, (uint16_t)(((fClockDiv - 1) << 8) | (fClockDiv - 1)));
	uint16_t control = CONTROL_ACKERR_DET_EN | CONTROL_CLK_EXT_EN;
	if (wrrd) {
		control |= CONTROL_DIR_CHANGE | CONTROL_RS;
	}
	wr16(I2C_CONTROL, control);
	wr16(I2C_EXT_CONF, fSpeedHz <= 100000 ? 0x8001 : fExtConf);
	wr16(I2C_DELAY_LEN, DELAY_LEN);
	wr16(I2C_IO_CONFIG, IO_CONFIG_OPEN_DRAIN);
	wr16(I2C_TIMING, fTiming | TIMING_TIMEOUT_EN);
	wr16(I2C_LTIMING, fLTiming);
	wr16(I2C_HS, 0);
	wr16(I2C_SLAVE_ADDR, (uint16_t)(addr << 1));
	wr16(I2C_INTR_STAT, INTR_ALL);
	wr16(I2C_FIFO_ADDR_CLR, 1);
	wr16(I2C_INTR_MASK, INTR_ERRORS | INTR_TRANSAC_COMP);
	wr16(I2C_TRANSFER_LEN, (uint16_t)wlen);
	if (wrrd) {
		wr16(I2C_TRANSFER_LEN_AUX, (uint16_t)rlen);
		wr16(I2C_TRANSAC_LEN, 2);
	} else {
		wr16(I2C_TRANSAC_LEN, 1);
	}
	for (uint32_t i = 0; i < wlen; i++) {
		*(volatile uint8_t *)(fRegs + I2C_DATA_PORT) = wbuf[i];
	}
	wr16(I2C_MCU_INTR, 1);
	fIrqStat = 0;
	fDone = false;
	OSMemoryBarrier();
	wr16(I2C_START, 1);

	AbsoluteTime deadline;
	clock_interval_to_deadline(100, kMillisecondScale, &deadline);
	while (!fDone) {
		if (fGate->commandSleep(&fDone, deadline, THREAD_UNINT) == THREAD_TIMED_OUT) {
			break;
		}
	}
	uint16_t stat = fIrqStat;
	if (!fDone || (stat & INTR_ERRORS)) {
		fErrors++;
		if (fErrors <= 8) {
			IOLog("VIVA I2C: addr 0x%02x %s (stat 0x%04x start 0x%04x)\n", addr,
			    fDone ? "error" : "timeout", stat, rd16(I2C_START));
		}
		wr16(I2C_FIFO_ADDR_CLR, 1);
		initHW();
		return fDone ? kIOReturnIOError : kIOReturnTimeout;
	}
	for (uint32_t i = 0; i < rlen; i++) {
		rbuf[i] = *(volatile uint8_t *)(fRegs + I2C_DATA_PORT);
	}
	return kIOReturnSuccess;
}
#endif /* SWEET */
