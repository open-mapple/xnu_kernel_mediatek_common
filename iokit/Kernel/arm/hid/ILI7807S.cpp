/*
 * Mapple (MT6781, realme 8i): the touch half of the ILI7807S (display and
 * touch in one chip) as the built-in touch screen. Matches the ADT node
 * 'touch' ("ilitek,ili7807s"); a MappleHIDTouch, like GT9896.cpp.
 *
 * The chip has no flash: mtkloader sends it its firmware and leaves SPI4's
 * pins as GPIOs, and they are driven by hand here (mode 0, MSB first), since
 * SPI4's controller hangs on anything but single short transfers. EINT 12
 * (MTKEINT.cpp) gives a short low pulse per report; the report's layout is
 * in MappleILIProtocol.h. A finger with no report for 150 ms is lifted.
 *
 * The chip loses its firmware by itself, or keeps its ACK but never raises
 * INT again. When it answers without its ACK the firmware is sent again from
 * mtkloader's copy, as Android's driver does on resume: ICE mode with the MCU
 * stopped, the chip ID checked, blocks AP, DATA and TUNING each checked
 * against the chip's DMA CRC and its last four bytes, code reset, ICE mode
 * left; then edge-palm rejection on (without it a hand holding the phone is a
 * finger at x = 0). Unless mp_ili_quiet=0, after QUIET_MS with the screen on
 * and no interrupt the firmware's version is asked for; with no interrupt
 * ANSWER_MS later (Android's wait for a command's INT pulse) the firmware is
 * sent again, starting with Android's reset pulse instead of the PLL wake.
 * Only a firmware whose info block says host download is sent; at most
 * RESENDS sends a boot, at most every 2 s.
 *
 * Boot-args: mp_ili_touch=0 leaves the chip alone; mp_ili_fw=<phys> is
 * mtkloader's firmware table (without it the firmware is never sent again);
 * mp_ili_quiet=0 leaves a quiet chip alone; mp_touchlog=1 logs every
 * landing and lifting.
 * ADT: spi-gpios <mosi miso clk cs>, reset-gpios <rst>, max-x, max-y (the chip's grid),
 * panel-size <w h>, interrupts/interrupt-parent (MTKEINT). GPIO registers
 * through the gpio node's reg (MTKIO.cpp).
 *
 * The chip's framing and commands from the ILITEK driver Android runs
 * (ili9882n), read, not copied; GPIO layout from MediaTek's MT6781 tables
 * (pinctrl-mtk-mt6781.h).
 */
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>

#include "MappleHIDTouch.h"
#include "MappleILIProtocol.h"
#include "../MTKIO.h"

#define GPIO_DIR_SET    0x004
#define GPIO_DIR_CLR    0x008
#define GPIO_DOUT_SET   0x104
#define GPIO_DOUT_CLR   0x108
#define GPIO_DIN        0x200
#define GPIO_MODE       0x300                   /* four bits a pin, eight pins per 0x10 */

#define STALE_MS        150
#define WATCH_MS        1000
#define RELOAD_GAP_MS   2000
#define RESENDS         2
#define QUIET_MS        2000                    /* Android's touch ESD period (WQ_ESD_DELAY) */
#define ANSWER_MS       600                     /* Android's wait for a command's INT (AP_INT_TIMEOUT) */
#define CHUNK           2048                    /* bytes of a block in one SPI write */
#define BAD_LOGGED      20

class ILI7807S : public MappleHIDTouch {
	OSDeclareDefaultStructors(ILI7807S);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOWorkLoop *getWorkLoop(void) const APPLE_KEXT_OVERRIDE;

protected:
	virtual OSString *getProduct() APPLE_KEXT_OVERRIDE { return OSString::withCString("ILI7807S"); }
	virtual UInt32 getVendorID() APPLE_KEXT_OVERRIDE { return 0x222a; }        /* ILITEK */
	virtual UInt32 getProductID() APPLE_KEXT_OVERRIDE { return 0x7807; }

private:
	static void interruptOccurred(OSObject *owner, IOInterruptEventSource *src, int count);
	static void timerFired(OSObject *owner, IOTimerEventSource *src);
	static const volatile uint8_t *mapPhys(void *ctx, uint64_t pa, uint32_t len);

	inline uint32_t rd(uint32_t off) { return *(volatile uint32_t *)(fGPIO + off); }
	inline void wr(uint32_t off, uint32_t v)
	{
		*(volatile uint32_t *)(fGPIO + off) = v;
		__asm__ volatile ("dsb sy" ::: "memory");
	}
	inline void pinSet(uint32_t pin, bool high) { wr((high ? GPIO_DOUT_SET : GPIO_DOUT_CLR) + 0x10 * (pin / 32), 1u << (pin % 32)); }
	inline uint32_t pinGet(uint32_t pin) { return (rd(GPIO_DIN + 0x10 * (pin / 32)) >> (pin % 32)) & 1; }
	void pinMode(uint32_t pin, uint32_t func);
	void pinsByHand(void);
	void spiXfer(const uint8_t *tx, uint32_t txLen, uint8_t *rx, uint32_t len);
	void spiRead(uint8_t *rx, uint32_t len);

	void findFirmware(void);
	void iceWrite(uint32_t addr, uint32_t value, uint32_t n);
	void iceBytes(uint32_t addr, uint8_t *b, uint32_t n);
	uint32_t dmaCRC(uint32_t start, uint32_t size);
	void resetPulse(void);
	bool reload(bool android);
	void resend(const char *why, bool android);
	void edgePalmOn(void);

	int readReport(void);
	void liftAll(const char *why);
	void quiet(void);
	void watch(void);

	IOWorkLoop             *fWorkLoop;
	IOInterruptEventSource *fInterrupt;
	IOTimerEventSource     *fTimer;
	IOMemoryMap            *fGPIOMap;
	IOMemoryMap            *fFWMap[ILI_BLOCKS + 1];
	unsigned int            fFWMaps;
	vm_offset_t             fGPIO;
	uint32_t                fMO, fMI, fCLK, fCS, fINT, fRST;
	uint32_t                fMaxX, fMaxY, fPanelW, fPanelH;
	uint32_t                fLog, fQuiet;
	struct ili_fw_block     fFW[ILI_BLOCKS];
	bool                    fHaveFW, fAlive, fAsked, fGaveUp;
	uint32_t                fResends, fAsks, fAnswers, fBad;
	uint32_t                fIrqs, fReports, fNoAck, fLowLong;
	uint64_t                fLastIrqAt, fLastReportAt, fLastBadAt;
	int                     fLastBad;
public:
	void diag(char *out, size_t len);
private:
	uint64_t                fLastTry, fReloadGap, fAskedAt, fQuietFrom;
	uint32_t                fDownMap;
	uint32_t                fLastX[ILI_FINGERS], fLastY[ILI_FINGERS];
};

OSDefineMetaClassAndStructors(ILI7807S, MappleHIDTouch);

static ILI7807S *gTouch;

extern "C" bool mapple_fb_display_on(void);     /* MappleFramebuffer.cpp */

static uint64_t
msSince(uint64_t at)
{
	uint64_t ns;

	if (at == 0) {
		return UINT64_MAX;
	}
	absolutetime_to_nanoseconds(mach_absolute_time() - at, &ns);
	return ns / NSEC_PER_MSEC;
}

static uint32_t
dtU32(IOService *provider, const char *key, unsigned int index, uint32_t dflt)
{
	OSData *d = OSDynamicCast(OSData, provider->getProperty(key));
	return (d != NULL && d->getLength() >= 4 * (index + 1)) ? ((const uint32_t *)d->getBytesNoCopy())[index] : dflt;
}

IOWorkLoop *
ILI7807S::getWorkLoop(void) const
{
	return fWorkLoop;
}

void
ILI7807S::pinMode(uint32_t pin, uint32_t func)
{
	uint32_t reg = GPIO_MODE + 0x10 * (pin / 8), shift = (pin % 8) * 4;

	wr(reg, (rd(reg) & ~(0xfu << shift)) | (func << shift));
}

/* as mtkloader leaves them: levels first, then directions, then mode 0 (GPIO) taken from SPI4's function */
void
ILI7807S::pinsByHand(void)
{
	pinSet(fCS, true);
	pinSet(fCLK, false);
	pinSet(fMO, false);
	wr(GPIO_DIR_SET + 0x10 * (fCS / 32), 1u << (fCS % 32));
	wr(GPIO_DIR_SET + 0x10 * (fCLK / 32), 1u << (fCLK % 32));
	wr(GPIO_DIR_SET + 0x10 * (fMO / 32), 1u << (fMO % 32));
	wr(GPIO_DIR_CLR + 0x10 * (fMI / 32), 1u << (fMI % 32));
	pinMode(fCS, 0);
	pinMode(fCLK, 0);
	pinMode(fMO, 0);
	pinMode(fMI, 0);
}

/*
 * One transfer, chip select held throughout: txLen bytes of tx, then zeros,
 * len bytes in all; rx NULL keeps nothing. Data is set while the clock is low
 * and sampled just after it rises; each extra read of DIN lets the pad settle.
 */
void
ILI7807S::spiXfer(const uint8_t *tx, uint32_t txLen, uint8_t *rx, uint32_t len)
{
	pinSet(fCS, false);
	(void)pinGet(fMI);
	for (uint32_t i = 0; i < len; i++) {
		uint8_t out = i < txLen ? tx[i] : 0, in = 0;

		for (int bit = 7; bit >= 0; bit--) {
			pinSet(fMO, (out >> bit) & 1);
			pinSet(fCLK, true);
			in = (uint8_t)(in << 1 | pinGet(fMI));
			pinSet(fCLK, false);
			(void)pinGet(fMI);
		}
		if (rx != NULL) {
			rx[i] = in;
		}
	}
	pinSet(fCS, true);
	(void)pinGet(fMI);
}

void
ILI7807S::spiRead(uint8_t *rx, uint32_t len)
{
	static const uint8_t readCmd = 0x83;

	spiXfer(&readCmd, 1, rx, len);
}

const volatile uint8_t *
ILI7807S::mapPhys(void *ctx, uint64_t pa, uint32_t len)
{
	ILI7807S *self = (ILI7807S *)ctx;
	uint64_t off = pa & PAGE_MASK;
	IOMemoryDescriptor *md;
	IOMemoryMap *map = NULL;

	if (self->fFWMaps >= ILI_BLOCKS + 1) {
		return NULL;
	}
	md = IOMemoryDescriptor::withPhysicalAddress(trunc_page(pa), round_page(off + len), kIODirectionInOut);
	if (md != NULL) {
		map = md->map();
		md->release();
	}
	if (map == NULL) {
		return NULL;
	}
	self->fFWMap[self->fFWMaps++] = map;
	return (const volatile uint8_t *)(map->getVirtualAddress() + off);
}

/* mtkloader's copy of the firmware, which stays in memory under XNU */
void
ILI7807S::findFirmware(void)
{
	uint64_t table = 0;
	const volatile struct ili_loader_block *t;

	if (!PE_parse_boot_argn_alias("mp_ili_fw", "ili_fw", &table, sizeof(table)) || table == 0) {
		IOLog("MAPPLE TOUCH: no mp_ili_fw=: the firmware cannot be sent again\n");
		return;
	}
	t = (const volatile struct ili_loader_block *)mapPhys(this, table, ILI_BLOCKS * sizeof(*t));
	if (t == NULL) {
		IOLog("MAPPLE TOUCH: cannot map the firmware table at %#llx\n", table);
		return;
	}
	int good = ili_fw_walk(t, fFW, &ILI7807S::mapPhys, this);
	if (good < ILI_BLOCKS) {
		if (fFW[good].data == NULL) {
			IOLog("MAPPLE TOUCH: the table at %#llx does not look like mtkloader's (block %d: %#x, %u bytes): not used\n",
			    table, good, fFW[good].mem, fFW[good].len);
		} else {
			IOLog("MAPPLE TOUCH: firmware block %d has CRC %08x, not %08x: not used\n", good, fFW[good].got, fFW[good].crc);
		}
		return;
	}
	if (!ili_fw_host_download(fFW[0].data, fFW[0].len)) {
		IOLog("MAPPLE TOUCH: the firmware's info block does not say host download (%02x): not used\n",
		    fFW[0].len > 0x50 ? fFW[0].data[fFW[0].len - 0x50] : 0);
		return;
	}
	fHaveFW = true;
	IOLog("MAPPLE TOUCH: mtkloader's firmware at %#llx: %u + %u + %u bytes, CRCs %08x %08x %08x right\n", table,
	    fFW[0].len, fFW[1].len, fFW[2].len, fFW[0].crc, fFW[1].crc, fFW[2].crc);
}

void
ILI7807S::iceWrite(uint32_t addr, uint32_t value, uint32_t n)
{
	uint8_t w[9] = { 0x82, 0x25, (uint8_t)addr, (uint8_t)(addr >> 8), (uint8_t)(addr >> 16) };

	for (uint32_t i = 0; i < n; i++) {
		w[5 + i] = (uint8_t)(value >> (8 * i));
	}
	spiXfer(w, 5 + n, NULL, 5 + n);
}

/* n (<= 4) bytes of the chip's memory, in the order they come */
void
ILI7807S::iceBytes(uint32_t addr, uint8_t *b, uint32_t n)
{
	uint8_t w[5] = { 0x82, 0x25, (uint8_t)addr, (uint8_t)(addr >> 8), (uint8_t)(addr >> 16) }, in[5];

	spiXfer(w, 5, NULL, 5);
	spiRead(in, 1 + n);
	for (uint32_t i = 0; i < n; i++) {
		b[i] = in[1 + i];
	}
}

/* the chip's DMA CRC over [start, start + size) (the ILITEK driver's calc_hw_dma_crc); 0 when it does not finish */
uint32_t
ILI7807S::dmaCRC(uint32_t start, uint32_t size)
{
	uint8_t b[4] = { 0 };

	iceWrite(0x072104, start, 4);
	iceWrite(0x072108, 0x80000001, 4);
	iceWrite(0x072114, 0x0002725c, 4);
	iceWrite(0x072118, 0x80000000, 4);
	iceWrite(0x07211c, size, 4);
	iceWrite(0x041016, 0x00, 1);
	iceWrite(0x041017, 0x03, 1);
	iceWrite(0x072100, 0x02000000, 4);
	iceWrite(0x041016, 0x01, 1);
	iceWrite(0x048006, 0x2, 1);
	iceWrite(0x072100, 0x01000000, 4);
	for (int i = 0; i < 50; i++) {
		IOSleep(1);
		iceBytes(0x048006, b, 1);
		if (b[0] & 2) {
			iceBytes(0x04101c, b, 4);
			return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24;
		}
	}
	return 0;
}

/* Android's ili_tp_reset: high 1 ms, low 5 ms, high, then 10 ms (rst_edge_delay); busy waits as there */
void
ILI7807S::resetPulse(void)
{
	pinSet(fRST, true);
	wr(GPIO_DIR_SET + 0x10 * (fRST / 32), 1u << (fRST % 32));
	IODelay(1000);
	pinSet(fRST, false);
	IODelay(5000);
	pinSet(fRST, true);
	IODelay(10000);
}

/* android: the reset pulse first and no PLL wake, as Android's driver in AP mode */
bool
ILI7807S::reload(bool android)
{
	static const uint8_t wake[12] = { 0x82, 0x00, 0x09, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3 };
	static const uint8_t iceStop[5] = { 0x82, 0x25, 0x62, 0x10, 0x18 };
	static const uint8_t iceOff[5] = { 0x82, 0x1b, 0x62, 0x10, 0x18 };
	static uint8_t buf[5 + CHUNK + 4];
	uint32_t dma[ILI_BLOCKS] = { 0 }, iram[ILI_BLOCKS] = { 0 };
	uint8_t id[4];
	bool good = true;

	if (android) {
		resetPulse();
	} else {
		spiXfer(wake, sizeof(wake), NULL, sizeof(wake));
	}
	spiXfer(iceStop, sizeof(iceStop), NULL, sizeof(iceStop));
	IOSleep(1);
	iceBytes(0x4009c, id, 4);
	uint32_t pid = (uint32_t)id[0] | (uint32_t)id[1] << 8 | (uint32_t)id[2] << 16 | (uint32_t)id[3] << 24;
	if (!ili_chip_ok(pid)) {
		spiXfer(iceOff, sizeof(iceOff), NULL, sizeof(iceOff));
		IOLog("MAPPLE TOUCH: chip ID %08x in ICE mode, not the ILI7807S's: the firmware is not sent\n", pid);
		return false;
	}
	for (int i = 0; i < ILI_BLOCKS && good; i++) {
		uint8_t b[4];

		for (uint32_t at = 0; at < fFW[i].len; at += CHUNK) {
			uint32_t n = fFW[i].len - at < CHUNK ? fFW[i].len - at : CHUNK, a = fFW[i].mem + at;

			buf[0] = 0x82;
			buf[1] = 0x25;
			buf[2] = (uint8_t)a;
			buf[3] = (uint8_t)(a >> 8);
			buf[4] = (uint8_t)(a >> 16);
			for (uint32_t k = 0; k < n; k++) {
				buf[5 + k] = fFW[i].data[at + k];
			}
			while (n % 4) {
				buf[5 + n++] = 0xff;
			}
			spiXfer(buf, 5 + n, NULL, 5 + n);
		}
		dma[i] = dmaCRC(fFW[i].mem, fFW[i].len - 4);
		iceBytes(fFW[i].mem + fFW[i].len - 4, b, 4);
		iram[i] = (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3];
		good = dma[i] == fFW[i].crc && iram[i] == fFW[i].crc;
	}
	if (good) {
		iceWrite(0x40040, 0xae, 1);             /* code reset: the MCU starts the firmware */
	}
	spiXfer(iceOff, sizeof(iceOff), NULL, sizeof(iceOff));
	IOSleep(100);
	IOLog("MAPPLE TOUCH: firmware sent again (%s, chip %08x%s): dma/iram %08x/%08x %08x/%08x %08x/%08x\n",
	    good ? "CRCs right" : "A CRC WRONG", pid, android ? ", reset pulse" : "", dma[0], iram[0], dma[1], iram[1],
	    dma[2], iram[2]);
	return good;
}

void
ILI7807S::resend(const char *why, bool android)
{
	uint8_t rx[ILI_RX_LEN];

	if (fResends == RESENDS) {
		if (!fGaveUp) {
			fGaveUp = true;
			IOLog("MAPPLE TOUCH: %s after %u re-sends: giving up until the next boot\n", why, RESENDS);
		}
		return;
	}
	fResends++;
	IOLog("MAPPLE TOUCH: re-send %u of %u: %s\n", fResends, RESENDS, why);
	reload(android);
	fLastTry = mach_absolute_time();
	fQuietFrom = fLastTry;
	fAsked = false;
	spiRead(rx, sizeof(rx));
	fAlive = rx[1] == ILI_ACK;
	IOLog("MAPPLE TOUCH: firmware %s after sending it again (ACK %02x)\n", fAlive ? "answers" : "STILL GONE", rx[1]);
	if (fAlive) {
		edgePalmOn();
	}
}

/* a command, framed 0x82, length (two bytes), then 01 12 01 */
void
ILI7807S::edgePalmOn(void)
{
	static const uint8_t cmd[6] = { 0x82, 0x00, 0x03, 0x01, 0x12, 0x01 };

	spiXfer(cmd, sizeof(cmd), NULL, sizeof(cmd));
	IOLog("MAPPLE TOUCH: edge-palm rejection on\n");
}

void
ILI7807S::liftAll(const char *why)
{
	for (uint32_t f = 0; f < ILI_FINGERS; f++) {
		if (fDownMap & (1u << f)) {
			if (fLog) {
				IOLog("MAPPLE TOUCH: f%u up %u,%u%s\n", f, fLastX[f], fLastY[f], why);
			}
			touch((int)f, false, (int)fLastX[f], (int)fLastY[f]);
		}
	}
	fDownMap = 0;
}

/* Work loop: one report; each finger it names lands or moves, each one it no longer names lifts. ili_parse's result. */
int
ILI7807S::readReport(void)
{
	uint8_t rx[ILI_RX_LEN];
	struct ili_point pt[ILI_FINGERS];

	spiRead(rx, sizeof(rx));
	int map = ili_parse(rx, pt);
	if (map == ILI_ANSWER) {
		fAnswers++;
		return map;
	}
	if (map < 0) {
		fNoAck += map == ILI_NO_ACK;
		fLastBad = map;
		fLastBadAt = mach_absolute_time();
	} else {
		fReports++;
		fLastReportAt = mach_absolute_time();
	}
	if ((map != ILI_NO_ACK) != fAlive) {
		fAlive = map != ILI_NO_ACK;
		IOLog("MAPPLE TOUCH: firmware %s (ACK %02x)\n", fAlive ? "answers again" : "GONE", rx[1]);
		if (!fAlive) {
			fTimer->setTimeoutMS(1);
		}
	}
	if (map < 0) {
		if (map == ILI_BAD && fBad++ < BAD_LOGGED) {
			IOLog("MAPPLE TOUCH: not a report: packet %02x, sum byte %02x\n", rx[3], rx[ILI_RX_LEN - 1]);
		}
		return map;
	}
	for (uint32_t f = 0; f < ILI_FINGERS; f++) {
		if (!(map & (1 << f))) {
			continue;
		}
		fLastX[f] = ili_scale(pt[f].x, fMaxX, fPanelW);
		fLastY[f] = ili_scale(pt[f].y, fMaxY, fPanelH);
		if (fLog && !(fDownMap & (1u << f))) {
			IOLog("MAPPLE TOUCH: f%u down %u,%u (grid %u,%u) p%u\n", f, fLastX[f], fLastY[f], pt[f].x, pt[f].y, pt[f].pressure);
		}
		touch((int)f, true, (int)fLastX[f], (int)fLastY[f]);
	}
	for (uint32_t f = 0; f < ILI_FINGERS; f++) {
		if ((fDownMap & (1u << f)) && !(map & (1 << f))) {
			if (fLog) {
				IOLog("MAPPLE TOUCH: f%u up %u,%u\n", f, fLastX[f], fLastY[f]);
			}
			touch((int)f, false, (int)fLastX[f], (int)fLastY[f]);
		}
	}
	fDownMap = (uint32_t)map;
	if (fDownMap != 0) {
		fTimer->setTimeoutMS(STALE_MS);
	}
	return map;
}

/*
 * An idle chip raises no interrupt (one per report, reports only with
 * fingers), so a quiet one is asked for its version, which it answers with an
 * interrupt as it does every command. Any interrupt clears fAsked.
 */
void
ILI7807S::quiet(void)
{
	static const uint8_t pre[5] = { 0x82, 0x00, 0x02, 0xf6, ILI_FW_VERSION };
	static const uint8_t ask[4] = { 0x82, 0x00, 0x01, ILI_FW_VERSION };
	uint64_t from = fLastIrqAt > fQuietFrom ? fLastIrqAt : fQuietFrom;
	char why[96];

	if (!mapple_fb_display_on() || fDownMap != 0) {
		fAsked = false;
		fQuietFrom = mach_absolute_time();
		return;
	}
	if (fAsked) {
		if (msSince(fAskedAt) < ANSWER_MS) {
			return;
		}
		fAsked = false;
		int got = readReport();
		if (got == ILI_NO_ACK) {
			return;
		}
		snprintf(why, sizeof(why), "ACK but no interrupt %u ms after asking the version (%s)", ANSWER_MS,
		    got == ILI_ANSWER ? "answer there" : "no answer");
		resend(why, true);
		return;
	}
	if (msSince(from) >= QUIET_MS) {
		spiXfer(pre, sizeof(pre), NULL, sizeof(pre));
		spiXfer(ask, sizeof(ask), NULL, sizeof(ask));
		fAsks++;
		fAsked = true;
		fAskedAt = mach_absolute_time();
	}
}

/*
 * Timer: it fires STALE_MS after the last report that had fingers, at once
 * when the firmware is found gone, ANSWER_MS after the version is asked for,
 * and every WATCH_MS otherwise. A line held low (no edge to catch) is read too.
 */
void
ILI7807S::watch(void)
{
	liftAll(" (no report)");
	if (!pinGet(fINT) && msSince(fLastIrqAt) > 250) {
		fLowLong++;
	}
	if (!pinGet(fINT)) {
		fAsked = false;
		readReport();
	}
	if (fQuiet && fAlive && fHaveFW && !fGaveUp) {
		quiet();
	}
	if (!fAlive && fHaveFW && !fGaveUp && (fLastTry == 0 || mach_absolute_time() - fLastTry >= fReloadGap)) {
		resend("firmware gone (no ACK)", false);
	}
	if (fDownMap != 0) {
		fTimer->setTimeoutMS(STALE_MS);
	} else if (!fAlive && fHaveFW && !fGaveUp) {
		fTimer->setTimeoutMS(RELOAD_GAP_MS);
	} else if (fAsked) {
		fTimer->setTimeoutMS(ANSWER_MS);
	} else {
		fTimer->setTimeoutMS(WATCH_MS);
	}
}

bool
ILI7807S::start(IOService *provider)
{
	uint32_t on = 1;
	uint8_t rx[ILI_RX_LEN];

	PE_parse_boot_argn_alias("mp_ili_touch", "mappletouch", &on, sizeof(on));
	if (!on) {
		return false;
	}
	PE_parse_boot_argn_alias("mp_touchlog", "touchlog", &fLog, sizeof(fLog));
	fQuiet = 1;
	PE_parse_boot_argn("mp_ili_quiet", &fQuiet, sizeof(fQuiet));
	fMO = dtU32(provider, "spi-gpios", 0, 13);
	fMI = dtU32(provider, "spi-gpios", 1, 14);
	fCLK = dtU32(provider, "spi-gpios", 2, 144);
	fCS = dtU32(provider, "spi-gpios", 3, 145);
	fRST = dtU32(provider, "reset-gpios", 0, 121);
	fINT = dtU32(provider, "interrupts", 0, 12);    /* EINT n is GPIO n here */
	fMaxX = dtU32(provider, "max-x", 0, 2047);
	fMaxY = dtU32(provider, "max-y", 0, 2047);
	fPanelW = dtU32(provider, "panel-size", 0, 1080);
	fPanelH = dtU32(provider, "panel-size", 1, 2412);
	clock_interval_to_absolutetime_interval(RELOAD_GAP_MS, kMillisecondScale, &fReloadGap);

	MTKGPIO *gpio = MTKGPIO::wait();
	fWorkLoop = IOWorkLoop::workLoop();
	if (gpio != NULL) {
		fGPIOMap = gpio->getProvider()->mapDeviceMemoryWithIndex(0);
	}
	if (fGPIOMap == NULL || fWorkLoop == NULL || fMaxX == 0 || fMaxY == 0 || fPanelW == 0 || fPanelH == 0) {
		IOLog("MAPPLE TOUCH: no GPIO registers, work loop or grid\n");
		return false;
	}
	fGPIO = (vm_offset_t)fGPIOMap->getVirtualAddress();
	pinsByHand();
	findFirmware();
	spiRead(rx, sizeof(rx));
	fAlive = rx[1] == ILI_ACK;
	IOLog("MAPPLE TOUCH: ILI7807S on GPIO %u/%u/%u/%u, firmware %s (ACK %02x), INT %s, grid %ux%u to %ux%u%s\n",
	    fMO, fMI, fCLK, fCS, fAlive ? "answers" : "GONE", rx[1], pinGet(fINT) ? "high" : "low",
	    fMaxX, fMaxY, fPanelW, fPanelH, fLog ? ", mp_touchlog on" : "");
	if (fAlive) {
		edgePalmOn();
	}

	if (!MappleHIDTouch::start(provider)) {
		return false;
	}
	fTimer = IOTimerEventSource::timerEventSource(this, &ILI7807S::timerFired);
	fInterrupt = IOInterruptEventSource::interruptEventSource(this, &ILI7807S::interruptOccurred, provider, 0);
	if (fTimer == NULL || fInterrupt == NULL || fWorkLoop->addEventSource(fTimer) != kIOReturnSuccess ||
	    fWorkLoop->addEventSource(fInterrupt) != kIOReturnSuccess) {
		IOLog("MAPPLE TOUCH: no timer or interrupt\n");
		return false;
	}
	fInterrupt->enable();
	fQuietFrom = mach_absolute_time();
	fTimer->setTimeoutMS(fAlive ? WATCH_MS : 1);
	gTouch = this;
	return true;
}

void
ILI7807S::free(void)
{
	if (fInterrupt != NULL) {
		fInterrupt->disable();
		if (fWorkLoop != NULL) {
			fWorkLoop->removeEventSource(fInterrupt);
		}
	}
	if (fTimer != NULL) {
		fTimer->cancelTimeout();
		if (fWorkLoop != NULL) {
			fWorkLoop->removeEventSource(fTimer);
		}
	}
	OSSafeReleaseNULL(fInterrupt);
	OSSafeReleaseNULL(fTimer);
	for (unsigned int i = 0; i < fFWMaps; i++) {
		OSSafeReleaseNULL(fFWMap[i]);
	}
	OSSafeReleaseNULL(fGPIOMap);
	MappleHIDTouch::free();
	OSSafeReleaseNULL(fWorkLoop);
}

void
ILI7807S::interruptOccurred(OSObject *owner, IOInterruptEventSource *src __unused, int count __unused)
{
	ILI7807S *self = OSDynamicCast(ILI7807S, owner);
	if (self != NULL) {
		self->fIrqs++;
		self->fLastIrqAt = mach_absolute_time();
		self->fAsked = false;
		self->readReport();
	}
}

void
ILI7807S::timerFired(OSObject *owner, IOTimerEventSource *src __unused)
{
	ILI7807S *self = OSDynamicCast(ILI7807S, owner);
	if (self != NULL) {
		self->watch();
	}
}

/* Read only, for the USB link's "touch": counters since boot and the INT line now. */
void
ILI7807S::diag(char *out, size_t len)
{
	snprintf(out, len, "touch: INT %s; %u interrupts, %u reports, %u bad (%u no ACK, %u not a report), last bad %d "
	    "%llu ms ago; INT held low past 250 ms without an interrupt %u times; last interrupt %llu ms ago, last report %llu "
	    "ms ago; firmware %s; %u version queries, %u answers; %u re-sends\n", pinGet(fINT) ? "high" : "low", fIrqs,
	    fReports, fNoAck + fBad, fNoAck, fBad, fLastBad, msSince(fLastBadAt), fLowLong, msSince(fLastIrqAt),
	    msSince(fLastReportAt), fAlive ? "answers" : "gone", fAsks, fAnswers, fResends);
}

extern "C" void mapple_touch_link(char *out, size_t len);

extern "C" void
mapple_touch_link(char *out, size_t len)
{
	if (gTouch == NULL) {
		snprintf(out, len, "touch: no ILI7807S\n");
		return;
	}
	gTouch->diag(out, len);
}
