/*
 * viva (MT6781): Goodix GT9896 touch controller, the built-in touch screen.
 * Matches /arm-io/i2c0/touch ("goodix,gt9896"), a child of VivaI2C
 * (../VivaIO.cpp); it is a SweetHIDTouch, so contacts reach backboardd as
 * the same digitizer collection events (fingers, edge tips) as on sweet.
 *
 * Protocol and power sequence from Xiaomi's driver for this phone
 * (viva-r-oss drivers/input/touchscreen/goodix_gtx8, GT9896 = "Normandy"):
 *  - power: avdd GPIO high, 100 us, dvdd GPIO high (load switches); reset
 *    GPIO low 2 ms, high, 110 ms, then the IC answers on I2C
 *  - registers have 16-bit big-endian addresses; version at 0x8240
 *  - interrupt (EINT 13, falling edge) -> event head at 0x824E:
 *    byte 0 bit 7 touch event, bit 6 request; byte 1 low nibble = contacts;
 *    then 8 bytes per contact (id, x LE16, y LE16, w, ...) and 2 more bytes;
 *    the byte sum of the first 4 + 8 * n bytes is 0; write 0 to 0x824E
 *    when done
 *  - coordinates 0..panel-max (8640 x 19200 = 8x the 1080 x 2400 panel)
 * Reads go in 8-byte pieces (VivaI2C FIFO transfers).
 *
 * ADT: i2c-address, avdd-gpio, dvdd-gpio, reset-gpio, max-x, max-y; the
 * interrupt is interrupts/interrupt-parent (VivaEINT).
 */
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOInterruptEventSource.h>

#include "SweetHIDTouch.h"
#include "../VivaIO.h"

#define GT_REG_VERSION          0x8240
#define GT_REG_COORDS           0x824e
#define GT_REG_TEST             0x4100
#define GT_EVENT_TOUCH          0x80
#define GT_EVENT_REQUEST        0x40
#define GT_BYTES_PER_COORD      8
#define GT_HEAD_LEN             2
#define GT_MAX_TOUCH            10
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

	VivaI2C                *fBus;
	VivaGPIO               *fGPIO;
	IOWorkLoop             *fWorkLoop;
	IOInterruptEventSource *fInterrupt;
	uint8_t                 fAddr;
	uint32_t                fAvdd, fDvdd, fReset, fMaxX, fMaxY;
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
	if (fBus == NULL || fGPIO == NULL || fWorkLoop == NULL) {
		IOLog("VIVA TOUCH: missing I2C bus, GPIO or work loop\n");
		return false;
	}
	if (!powerOn()) {
		IOLog("VIVA TOUCH: GT9896 at 0x%02x does not answer after reset\n", fAddr);
		return false;
	}
	uint8_t ver[16] = { 0 };
	readReg(GT_REG_VERSION, ver, sizeof(ver));
	IOLog("VIVA TOUCH: version %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x"
	    " ('%c%c%c%c')\n", ver[0], ver[1], ver[2], ver[3], ver[4], ver[5], ver[6], ver[7], ver[8], ver[9],
	    ver[10], ver[11], ver[12], ver[13], ver[14], ver[15],
	    ver[0] >= 0x20 && ver[0] < 0x7f ? ver[0] : '.', ver[1] >= 0x20 && ver[1] < 0x7f ? ver[1] : '.',
	    ver[2] >= 0x20 && ver[2] < 0x7f ? ver[2] : '.', ver[3] >= 0x20 && ver[3] < 0x7f ? ver[3] : '.');
	uint8_t zero = 0;
	writeReg(GT_REG_COORDS, &zero, 1);

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

/* Work loop: goodix_event_handler + goodix_touch_handler_nor. */
void
VivaGT9896::handleEvent(void)
{
	uint8_t buf[GT_HEAD_LEN + GT_BYTES_PER_COORD * GT_MAX_TOUCH + 2];
	uint32_t pre = GT_HEAD_LEN + GT_BYTES_PER_COORD + 2;
	uint8_t zero = 0;

	if (readReg(GT_REG_COORDS, buf, pre) != kIOReturnSuccess) {
		return;
	}
	if (!(buf[0] & GT_EVENT_TOUCH)) {
		if (buf[0] & GT_EVENT_REQUEST) {
			IOLog("VIVA TOUCH: request event 0x%02x 0x%02x\n", buf[0], buf[1]);
		}
		writeReg(GT_REG_COORDS, &zero, 1);
		return;
	}
	uint32_t n = buf[1] & 0x0f;
	if (n > GT_MAX_TOUCH) {
		writeReg(GT_REG_COORDS, &zero, 1);
		return;
	}
	if (n > 1 && readReg(GT_REG_COORDS + pre, buf + pre, (n - 1) * GT_BYTES_PER_COORD) != kIOReturnSuccess) {
		return;
	}
	writeReg(GT_REG_COORDS, &zero, 1);

	uint8_t sum = 0;
	for (uint32_t i = 0; i < n * GT_BYTES_PER_COORD + 4; i++) {
		sum += buf[i];
	}
	if (sum != 0) {
		if (++fBadSums <= 8) {
			IOLog("VIVA TOUCH: checksum 0x%02x (n %u)\n", sum, n);
		}
		return;
	}
	if (n >= 1 && buf[(n - 1) * GT_BYTES_PER_COORD + 2] >= 0x80) {
		return;         /* pen report: not a finger */
	}

	fEvents++;
	uint32_t map = 0;
	const uint8_t *c = buf + GT_HEAD_LEN;
	for (uint32_t i = 0; i < n; i++, c += GT_BYTES_PER_COORD) {
		uint32_t id = c[0];
		if (id >= GT_MAX_TOUCH || id >= SWEET_TOUCH_FINGERS) {
			break;
		}
		uint32_t x = (uint32_t)c[1] | (uint32_t)c[2] << 8;
		uint32_t y = (uint32_t)c[3] | (uint32_t)c[4] << 8;
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
