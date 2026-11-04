/*
 * Built-in IOMobileFramebuffer for SWEET: the ABL's continuous-splash
 * framebuffer (1080x2400 a8r8g8b8 in RAM, the panel scans it out by
 * itself) as the main display, driven by CoreAnimation's CPU renderer.
 *
 * IOMobileFramebuffer.framework (iOS 16.7) counts /arm-io children with
 * device_type "display-subsystem", waits for that many services matching
 * IOServiceMatching("IOMobileFramebuffer"), opens each with type 0 and
 * reads canvas-width/canvas-height from the service's provider. The class
 * here (AppleCLCD, an IOMobileFramebuffer) matches the /arm-io/disp0 nub.
 *
 * User client selectors used by CoreAnimation:
 *   3  GetLayerDefaultSurface [layer, ?] -> [IOSurface port name]
 *   4  SwapBegin -> [swap id]
 *   5  SwapEnd, struct 0x46c: +0x50 swap id, +0x54 IOSurface id per layer
 *      (4 layers), +0x64 rect a / +0xc4 rect b per layer (int x,y,w,h),
 *      +0x104 enabled layer mask
 *   6  SwapWait [swap id, options, timeout ms]: CoreAnimation (IOMFBDisplay::can_update_) renders into a
 *      framebuffer page again only once SwapWait(its swap, 0x80000000) returns 0. options[2:0]: 0 retired (not
 *      queued, not waiting for the frame end, not the surface the layer reads), 1 displayed (not queued, not
 *      waiting), 2 nothing queued or waiting; bit 31 = do not block (kIOReturnBusy). Apple:
 *      IOMobileFramebufferLegacy swap_wait_gated (gpu/docs/ANALISI-SCANOUT-RE.md). 7 GetID,
 *      8 GetDisplaySize -> [w, h],
 *   12 RequestPowerChange [state]: 0 = display off (backboardd, power
 *      button / auto-lock), else on. Off fades the panel out over a few
 *      vsyncs (halving each pixel) and keeps it black until on again;
 *      the panel itself stays powered (AMOLED black pixels are off).
 *   18 IsMainDisplay,
 *   28 GetDotPitch, 72 Enable/DisableNotifications [func, refcon, type, 0]
 * Everything else is accepted and logged once.
 *
 * SwapEnd copies the enabled layers' surfaces into the panel memory (no
 * scaling, CPU copy). Vsync (notification type 5) is a 60 Hz timer that
 * sends (count, mach_absolute_time, 0) as 32-bit halves to clients that
 * enabled it, through the port they registered (IOConnectSetNotificationPort).
 *
 * viva direct scanout (boot-arg vivascanout=1, or "disp scanout on" on the
 * USB link): LK leaves the MediaTek display pipe running in video mode, one
 * OVL layer reading the panel memory. When a swap shows a single full-screen
 * BGRA surface that is physically contiguous below 4 GiB (SweetSurface
 * allocates display-sized surfaces that way when scanout is on), only that
 * layer's address and pitch are changed to point at the surface (the
 * centered 1080x2400 window of the 1125x2436 surface): no CPU copy. Anything
 * else falls back to the copy. The OVL shadow registers are bypassed on
 * MT6781: a layer register write takes effect at once, so it is made at a
 * frame end (below).
 *
 * viva double buffer (K0d, boot-arg sweetfb_flip, 1 default): the copy goes
 * to the panel buffer the layer is not reading, LK's buffer (fPhys) or the
 * next one in LK's memory (LK reserves three), and the layer is switched to
 * it at the next frame end. The frame end is the OVL's frame-complete status
 * bit (INTSTA bit 1, polled; the OVL interrupt itself stays unused). The
 * vsync timer is phase-locked to it: each tick fires fLead before the
 * predicted frame end and, when a switch is pending, polls the bit until it
 * rises (the switch happens with interrupts off right then, only if the bit
 * was seen within a few microseconds of rising). Without a usable frame end,
 * with the kernel console on screen, a swap of the default surface or a
 * frame that does not cover the whole panel, the copy goes to the buffer on
 * screen as before (a frame may tear). A panic points the layer back at
 * LK's buffer, where the console draws (sweet_fb_panic_hook).
 *
 * GPU-drawn frames (K0c): backboardd hands a surface to SwapEnd right after
 * committing its command buffer, without waiting for the GPU (on Apple the
 * display driver waits). Swaps are therefore queued in order (the last
 * SWEET_FB_QUEUE) and a tick shows the newest one whose surfaces have no GPU
 * job pending (viva_gpu_surface_idle), dropping it and the older ones. If
 * none is ready nothing is shown and the next vsync tick tries again (no
 * waiting in the timer). A copy during which a new job used one of the
 * surfaces is redone at the next tick. After sweetfb_gpuwait ticks (default
 * 12, 200 ms) the newest frame is shown anyway; sweetfb_gpuwait=0 turns the
 * wait off.
 *
 * Early present (K0g, boot-arg sweetfb_early, 1 default; "disp early on|off"):
 * the same is also tried when a swap arrives and whenever a GPU job that used
 * IOSurfaces completes, on this work loop, so a finished frame is copied off
 * screen at once and switched at the next frame end instead of one tick later.
 *
 * The panel memory itself is published as an IOSurface and returned as the
 * default framebuffer surface. The kernel text console keeps drawing over
 * the GUI by default: boot-arg sweetfb_noconsole=1 stops drawing it from
 * the first swap on (it still reaches the persistent log; panics draw).
 */
#include "SweetSurface.h"

extern "C" int sweet_console_muted;     /* osfmk/console/serial_console.c */
extern "C" void (*sweet_fb_panic_hook)(void);   /* pexpert/arm/pe_serial.c */

#if defined(SWEET)
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/system.h>
#include <pexpert/pexpert.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>
#include <sys/proc.h>

enum {
	kSweetFBGetLayerDefaultSurface  = 3,
	kSweetFBSwapBegin               = 4,
	kSweetFBSwapEnd                 = 5,
	kSweetFBSwapWait                = 6,
	kSweetFBGetID                   = 7,
	kSweetFBGetDisplaySize          = 8,
	kSweetFBRequestPowerChange      = 12,
	kSweetFBIsMainDisplay           = 18,
	kSweetFBGetDotPitch             = 28,
	kSweetFBNotifications           = 72,
};

#define SWEET_FB_SWAP_REC_SIZE          0x46c
#define SWEET_FB_LAYERS                 4
#define SWEET_FB_QUEUE                  4       /* swaps waiting for the GPU (K0c) */
#define SWEET_FB_LK_BUFFERS             3       /* panel buffers LK reserves (K0d uses one besides its own) */
#define SWEET_FB_NOTIFY_TYPES           8
#define SWEET_FB_NOTIFY_VSYNC           5
#define SWEET_FB_VSYNC_US               16667
#define SWEET_FB_DOT_PITCH              458     /* ppi, as the iPhone X panel */
/*
 * The iOS userland only has status bar / layout data for real iPhone
 * screens (UIKit: "No status bar visual provider for screen", the red
 * rdar://45025538 bar for 360x800 points). The display is therefore
 * presented as the iPhone X panel, 1125x2436 (375x812 points @3x), and the
 * centered 1080x2400 part of it is shown on the 1080x2400 panel.
 * Boot-arg sweetfb_native=1 presents the real panel size instead.
 */
#define SWEET_FB_LOGICAL_W              1125
#define SWEET_FB_LOGICAL_H              2436
#define SWEET_FB_FORMAT_BGRA            0x42475241
/*
 * Extended-range formats CoreAnimation composes in on a wide-colour display (Metal BGR10_XR / BGRA10_XR):
 * 'w30r' 32-bit, B/G/R in bits 0-9/10-19/20-29; 'w40a'/'w40m' 64-bit, B/G/R/A in 16-bit words, 10 bits in the
 * top of each; 'l10r' 32-bit ARGB2101010, full range. XR value v means (v - 384) / 510: converted to 8-bit
 * BGRA (clamped to [0, 1], i.e. sRGB) while copying to the panel.
 */
#define SWEET_FB_FORMAT_W30R            0x77333072
#define SWEET_FB_FORMAT_W40A            0x77343061
#define SWEET_FB_FORMAT_W40M            0x7734306d
#define SWEET_FB_FORMAT_L10R            0x6c313072

class IOMobileFramebuffer : public IOService {
	OSDeclareDefaultStructors(IOMobileFramebuffer);
};

OSDefineMetaClassAndStructors(IOMobileFramebuffer, IOService);

class IOMobileFramebufferUserClient;

class AppleCLCD : public IOMobileFramebuffer {
	OSDeclareDefaultStructors(AppleCLCD);

	IOWorkLoop         *fWorkLoop;
	IOTimerEventSource *fVsync;
	uint64_t            fVsyncCount;

	static void vsyncFired(OSObject *owner, IOTimerEventSource *sender);
	static void earlyFired(OSObject *owner, IOTimerEventSource *sender);
	void flushFrames(bool early, uint64_t count);

public:
	void earlyKick(void);
	uint64_t      fPhys;
	uint32_t      fWidth, fHeight, fRowBytes;
	uint32_t      fLogicalW, fLogicalH;     /* size reported to userland */
	int32_t       fOffX, fOffY;             /* logical origin of the panel */
	uintptr_t     fPixels;          /* kernel mapping of the panel memory */
	IOMemoryMap  *fMap;
	SweetSurface *fSurface;         /* the panel memory as an IOSurface */
	IOLock       *fLock;            /* swaps and client list */
	OSArray      *fClients;
	uint32_t      fSwapID;
	uint64_t      fSwapWaitBusy, fSwapWaitSlept;      /* SwapWait answered busy / had to block */
	bool          fConsolePaused;
	uint8_t       fPending[SWEET_FB_SWAP_REC_SIZE];     /* latest swap (redrawn when the display comes on) */
	bool          fDirty;
	uint8_t       fQueue[SWEET_FB_QUEUE][SWEET_FB_SWAP_REC_SIZE];       /* swaps not shown yet, oldest first */
	uint64_t      fQueueSeq[SWEET_FB_QUEUE];
	uint64_t      fQueueAt[SWEET_FB_QUEUE];       /* 0135: mach time of each queued swap's SwapEnd */
	/*
	 * 0135 frame latency ("disp"): SwapEnd -> chosen (GPU done, K0c), chosen -> on screen (frame end), SwapEnd to
	 * SwapEnd, last vsync notification -> SwapEnd (CoreAnimation's render time); sums, counts and maxima in mach time.
	 */
	struct LatStat { uint64_t sum, n, max; } fLatReady, fLatFlip, fLatSwap, fLatRender;
	uint64_t      fLatReadyHist[5];               /* SwapEnd -> chosen in frames: <1, <2, <3, <4, more */
	uint64_t      fLastSwapAt, fLastNotifyAt, fNextChosenAt;
	uint64_t      fTickPresents;                  /* 0135: frames chosen at a tick before its frame end (scanout) */
	static void latAdd(LatStat *s, uint64_t d) { s->sum += d; s->n++; if (d > s->max) s->max = d; }
	uint64_t periodOr60(void)
	{
		uint64_t p = fPeriod;
		if (p == 0) {
			nanoseconds_to_absolutetime(16666667, &p);
		}
		return p;
	}
	uint32_t      fQueueCount;
	uint64_t      fSeq;
	uint32_t      fFlushTicks;    /* vsync ticks between panel copies (sweetfb_hz) */
	uint32_t      fDuty;          /* no sweetfb_hz: copies take at most this % of one CPU (sweetfb_duty) */
	uint64_t      fNextCopy;      /* absolute time the next copy may start (fDuty) */
	uint64_t      fCopies, fCopyNs;
	bool          fPowerOff;      /* RequestPowerChange(0) */
	uint32_t      fFadeStep;      /* fade-out steps done while off */
	/* K0c: frames held back for the GPU */
	bool          fRetry;         /* the pending frame waits for the GPU: try again at the next tick */
	uint32_t      fGpuTicks;      /* consecutive ticks the pending frame waited */
	uint32_t      fGpuWaitMax;    /* sweetfb_gpuwait: ticks before showing it anyway */
	uint64_t      fGpuWaits, fGpuRecopies, fGpuForced;
	/* K0g early present (sweetfb_early, 1 default): a swap is tried when it arrives and when GPU jobs complete */
	IOTimerEventSource *fEarlyTimer;
	bool          fEarly;
	volatile UInt32 fEarlyPending;  /* fEarlyTimer armed */
	uint64_t      fEarlyRuns, fEarlyShown;

	/* viva direct scanout (see the top) */
	IOLock       *fScanLock;      /* scanout state: present() vs. the link command */
	IOMemoryMap  *fDispMap[4];    /* OVL0, OVL0_2L, SMI larb0, larb1 */
	vm_offset_t   fDisp[4];
	int           fScanOvl;       /* OVL of LK's layer (0/1), -1 = not found */
	uint32_t      fScanAddrReg, fScanPitchReg, fScanPitchMsbReg;
	uint32_t      fLkAddr, fLkPitch, fLkPitchMsb;
	char          fScanLayer[16];
	bool          fScanOn;        /* allowed */
	bool          fScanLive;      /* the last frame shown was scanned out (not copied) */
	uint64_t      fScanFrames, fScanFallbacks;
	const char   *fScanWhy;       /* last reason for falling back */

	/* what LK's layer reads (fScanLock): now, and from the next frame end */
	struct Target {
		uint32_t      addr, pitch, pitchMsb;
		int           buf;            /* panel buffer 0/1, -1 = an IOSurface (scanout) */
		SweetSurface *surface;        /* retained while the layer may read it */
		uint32_t      swap;           /* the swap whose surface the layer reads (scanout), 0 = a panel buffer */
	};
	volatile uint32_t fShownSwap, fNextSwap;      /* fShown.swap / fNext.swap (valid), read without fScanLock */
	Target        fShown, fNext;
	bool          fNextValid;
	/* K0d double buffer and frame end (see the top) */
	uintptr_t     fBufKVA[2];     /* 0 = LK's buffer (fPixels), 1 = another one in LK's memory (flipSetup) */
	uint64_t      fBufPA[2];
	IOMemoryMap  *fBuf1Map;
	bool          fFlipAllowed;   /* sweetfb_flip */
	bool          fFlipTried, fFlipReady;
	const char   *fFlipWhy;       /* why there are no flips */
	int           fVblankOvl;     /* OVL whose frame-complete bit is polled */
	bool          fVblankInten;   /* its FME_CPL interrupt enable had to be set for the bit to latch */
	uint64_t      fPeriod, fLastVblank, fLead, fSlack;    /* absolute time */
	uint32_t      fTickFrames;    /* panel frames per vsync tick (about 60 Hz) */
	uint32_t      fMisses;        /* consecutive pending switches without a frame end */
	bool          fResync;        /* the phase is unsure: poll up to a whole frame at the next switch */
	uint64_t      fFlips, fFlipMissed, fFlipUnsure, fFlipEarly, fInPlace, fPolls, fPollAbs;
	/* 0136: ticks with a switch pending that came after the predicted frame end (late) or too early for it */
	uint64_t      fTickLate, fTickLateSum, fTickLateMax, fTickEarly, fTickEarlySum, fTickWidePolls;

	bool dispMap(void);
	bool findLKLayer(char **out, char *end);
	bool layerReads(uint64_t lo, uint64_t hi);
	bool scanout(const uint8_t *rec);
	bool layersIdle(const uint8_t *rec, uint64_t *jobs);
	void scanoutStop(void);
	void dispCommand(const char *args, char *out, size_t len);
	Target bufTarget(int buf);
	void layerWrite(const Target &t);
	void layerApply(const Target &t);
	void show(const Target &t);
	void dropNext(void);
	bool swapDone(uint32_t swap, uint32_t op);
	bool flipSetup(char **out, char *end);
	bool vblankCalibrate(char **out, char *end);
	uint64_t vblankPoll(uint64_t budget, const Target *t, bool *applied);
	void vblankSeen(uint64_t at);
	void vblankTick(void);
	void rearm(IOTimerEventSource *timer);

	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	void addClient(IOMobileFramebufferUserClient *client);
	void removeClient(IOMobileFramebufferUserClient *client);
	void swap(const uint8_t *rec);
	bool present(const uint8_t *rec, bool force, uint64_t jobs);
	int presentQueued(const uint8_t *recs, uint32_t n, bool early);
};

OSDefineMetaClassAndStructors(AppleCLCD, IOMobileFramebuffer);

static AppleCLCD *gSweetFB;     /* for the USB link "disp" command */
volatile uint32_t sweet_fb_gpu_waits;
static volatile uint32_t gSweetFbDutyCap;       /* sweet_fb_set_duty_cap (Low Power Mode); 0 = sweetfb_duty */

/* Low Power Mode (VivaBattery.cpp): panel copies take at most `pct` % of one CPU (0 = sweetfb_duty again). */
extern "C" void
sweet_fb_set_duty_cap(uint32_t pct)
{
	gSweetFbDutyCap = pct > 100 ? 100 : pct;
}

class IOMobileFramebufferUserClient : public IOUserClient {
	OSDeclareDefaultStructors(IOMobileFramebufferUserClient);

public:
	AppleCLCD          *fFB;
	task_t              fTask;
	mach_port_t         fPort;
	io_user_reference_t fFunc[SWEET_FB_NOTIFY_TYPES];
	io_user_reference_t fRefcon[SWEET_FB_NOTIFY_TYPES];

	bool initWithTask(task_t owningTask, void *securityID, UInt32 type) APPLE_KEXT_OVERRIDE;
	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	IOReturn registerNotificationPort(mach_port_t port, UInt32 type,
	    io_user_reference_t refCon) APPLE_KEXT_OVERRIDE;
	IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target,
	    void *reference) APPLE_KEXT_OVERRIDE;
	void notify(uint32_t type, const uint64_t *values, uint32_t count);
};

OSDefineMetaClassAndStructors(IOMobileFramebufferUserClient, IOUserClient);

bool
AppleCLCD::start(IOService *provider)
{
	PE_Video video;

	if (!IOService::start(provider)) {
		return false;
	}
	PE_current_console(&video);
	fPhys = video.v_baseAddr;
	fWidth = (uint32_t)video.v_width;
	fHeight = (uint32_t)video.v_height;
	fRowBytes = (uint32_t)video.v_rowBytes;
	if (fPhys == 0 || fWidth == 0 || fHeight == 0 || fRowBytes < fWidth * 4) {
		IOLog("SWEET FB: no boot framebuffer\n");
		return false;
	}
	int native = 0;
	PE_parse_boot_argn("sweetfb_native", &native, sizeof(native));
	fLogicalW = fWidth;
	fLogicalH = fHeight;
	if (!native && fWidth <= SWEET_FB_LOGICAL_W && fHeight <= SWEET_FB_LOGICAL_H) {
		fLogicalW = SWEET_FB_LOGICAL_W;
		fLogicalH = SWEET_FB_LOGICAL_H;
	}
	fOffX = (int32_t)(fLogicalW - fWidth) / 2;
	fOffY = (int32_t)(fLogicalH - fHeight) / 2;
	fLock = IOLockAlloc();
	fClients = OSArray::withCapacity(4);
	fSurface = sweet_surface_create_physical(fPhys, fWidth, fHeight, fRowBytes, SWEET_FB_FORMAT_BGRA);
	fPixels = fSurface != NULL ? fSurface->kernelAddress() : 0;
	fWorkLoop = IOWorkLoop::workLoop();
	if (fLock == NULL || fClients == NULL || fPixels == 0 || fWorkLoop == NULL) {
		IOLog("SWEET FB: setup failed\n");
		OSSafeReleaseNULL(fSurface);    /* stays published: also held by the table */
		OSSafeReleaseNULL(fWorkLoop);
		return false;
	}
	fVsync = IOTimerEventSource::timerEventSource(this, &AppleCLCD::vsyncFired);
	if (fVsync == NULL || fWorkLoop->addEventSource(fVsync) != kIOReturnSuccess) {
		IOLog("SWEET FB: no vsync timer\n");
		return false;
	}
	int early = 1;
	PE_parse_boot_argn("sweetfb_early", &early, sizeof(early));
	fEarlyTimer = IOTimerEventSource::timerEventSource(this, &AppleCLCD::earlyFired);
	if (fEarlyTimer != NULL && fWorkLoop->addEventSource(fEarlyTimer) != kIOReturnSuccess) {
		OSSafeReleaseNULL(fEarlyTimer);
	}
	fEarly = early != 0 && fEarlyTimer != NULL;
	uint32_t hz = 10;
	fFlushTicks = 0;
	if (PE_parse_boot_argn("sweetfb_hz", &hz, sizeof(hz))) {
		fFlushTicks = (hz == 0 || hz >= 60) ? 1 : 60 / hz;
	}
	fDuty = 25;
	PE_parse_boot_argn("sweetfb_duty", &fDuty, sizeof(fDuty));
	fDuty = MIN(MAX(fDuty, 1u), 100u);
	fGpuWaitMax = 12;
	PE_parse_boot_argn("sweetfb_gpuwait", &fGpuWaitMax, sizeof(fGpuWaitMax));
	fScanOvl = -1;
	fScanLock = IOLockAlloc();
	if (fScanLock == NULL) {
		return false;
	}
	fShown.buf = 0;
	fBufKVA[0] = fPixels;
	fBufPA[0] = fPhys;
	fVblankOvl = -1;
	int flip = 1;
	PE_parse_boot_argn("sweetfb_flip", &flip, sizeof(flip));
	fFlipAllowed = flip != 0;
	uint32_t us = 1000;
	PE_parse_boot_argn("sweetfb_lead", &us, sizeof(us));        /* tick this long before the frame end */
	nanoseconds_to_absolutetime((uint64_t)MIN(MAX(us, 200u), 4000u) * 1000, &fLead);
	nanoseconds_to_absolutetime(20 * 1000, &fSlack);
	uint32_t scan = 0;
	if (PE_parse_boot_argn("vivascanout", &scan, sizeof(scan)) && scan != 0) {
		/* display-sized surfaces from now on contiguous; LK's layer is looked up at the first swap */
		sweet_surface_scanout_w = fLogicalW;
		sweet_surface_scanout_h = fLogicalH;
		fScanOn = true;
	}
	gSweetFB = this;
	fVsync->setTimeoutUS(SWEET_FB_VSYNC_US);

	setProperty(kIOUserClientClassKey, "IOMobileFramebufferUserClient");
	setProperty("external", false);
	setProperty("DisplayPipePlaneBaseAlignment", 64ULL, 32);
	registerService();
	IOLog("SWEET FB: AppleCLCD %ux%u stride %u at 0x%llx, default surface %u, presented as %ux%u\n",
	    fWidth, fHeight, fRowBytes, fPhys, fSurface->id, fLogicalW, fLogicalH);
	return true;
}

void
AppleCLCD::vsyncFired(OSObject *owner, IOTimerEventSource *sender)
{
	AppleCLCD *fb = (AppleCLCD *)owner;
	uint64_t count = ++fb->fVsyncCount;
	/*
	 * 0135, scanout: look for a finished frame before the switch, so one found now is on screen at this frame end
	 * (choosing it costs no copy). Before, it was chosen after vblankTick and switched one frame later; with
	 * SwapWait (0133) CoreAnimation only draws the next frame once the switch retired the old page, so every frame
	 * of latency here is a frame less per second. Copies (3 ms) still run after the switch.
	 */
	bool tickFirst = fb->fScanLive && fb->fFlipReady && !fb->fPowerOff;
	if (tickFirst) {
		uint64_t shownBefore = fb->fScanFrames;
		fb->flushFrames(false, count);
		fb->fTickPresents += fb->fScanFrames != shownBefore;
	}
	fb->vblankTick();               /* a pending layer switch happens at this frame end */
	uint64_t now = mach_absolute_time();
	fb->fLastNotifyAt = now;
	/* args 4/5: the refresh period (QuartzCore predicts the next vblank as time + period) */
	uint64_t period = fb->fPeriod;
	if (period == 0) {
		nanoseconds_to_absolutetime(16666667, &period);
	}
	uint64_t values[6] = {
		(uint32_t)count, (uint32_t)(count >> 32),
		(uint32_t)now, (uint32_t)(now >> 32),
		(uint32_t)period, (uint32_t)(period >> 32),
	};

	IOLockLock(fb->fLock);
	for (unsigned int i = 0; i < fb->fClients->getCount(); i++) {
		IOMobileFramebufferUserClient *client =
		    (IOMobileFramebufferUserClient *)fb->fClients->getObject(i);
		client->notify(SWEET_FB_NOTIFY_VSYNC, values, 6);
	}
	if (fb->fPowerOff) {
		/* Fade out: halve every channel each vsync, then black, then idle. */
		uint32_t step = fb->fFadeStep;
		if (step <= 6) {
			fb->fFadeStep = step + 1;
		}
		IOLockUnlock(fb->fLock);
		if (step <= 6) {
			/* the buffer on screen (a scanned-out surface: LK's buffer at once) */
			IOLockLock(fb->fScanLock);
			fb->dropNext();
			if (fb->fShown.buf < 0) {
				fb->layerApply(fb->bufTarget(0));
			}
			uintptr_t px = fb->fBufKVA[fb->fShown.buf];
			IOLockUnlock(fb->fScanLock);
			for (uint32_t y = 0; y < fb->fHeight; y++) {
				volatile uint32_t *row = (volatile uint32_t *)(px + (uint64_t)y * fb->fRowBytes);
				for (uint32_t x = 0; x < fb->fWidth; x++) {
					row[x] = step < 6 ? (row[x] >> 1) & 0x7f7f7f7f : 0;
				}
			}
		}
		fb->rearm(sender);
		return;
	}
	IOLockUnlock(fb->fLock);
	if (!tickFirst) {
		fb->flushFrames(false, count);
	}
	fb->rearm(sender);
}

/*
 * K0g early present: a swap whose surfaces the GPU has finished is shown as soon as it can be, not at the next
 * vsync tick. With the double buffer the copy then lands in the buffer off screen right away and the switch
 * happens at the next frame end, instead of copying at the tick (fLead before the frame end, too late for that
 * frame end) and switching one frame later: a frame of latency less. Same rules as the tick: K0c (only finished
 * frames), the CPU duty limit (sweetfb_duty), nothing while the display is off, never with a fixed sweetfb_hz.
 */
void
AppleCLCD::earlyKick(void)
{
	if (fEarly && fDirty && OSCompareAndSwap(0, 1, &fEarlyPending)) {
		fEarlyTimer->setTimeoutUS(1);
	}
}

void
AppleCLCD::earlyFired(OSObject *owner, IOTimerEventSource *sender __unused)
{
	AppleCLCD *fb = (AppleCLCD *)owner;
	fb->fEarlyPending = 0;          /* before looking: a kick from now on runs again */
	fb->fEarlyRuns++;
	fb->flushFrames(true, 0);
}

/* The vsync tick (early false, count = tick number) or an early present: show the newest finished queued swap. */
void
AppleCLCD::flushFrames(bool early, uint64_t count)
{
	uint64_t now = mach_absolute_time();
	IOLockLock(fLock);
	bool flush;
	if (early) {
		/* scanout costs no copy; a copy only within the duty budget (the tick covers the rest) */
		flush = fDirty && !fPowerOff && fFlushTicks == 0 && (fScanLive || now >= fNextCopy);
	} else {
		/* scanout or a frame held back for the GPU: every vsync */
		bool due = fFlushTicks ? (count % fFlushTicks) == 0 : now >= fNextCopy;
		flush = fDirty && (fScanLive || fRetry || due);
	}
	uint8_t *recs = NULL;
	uint64_t seqs[SWEET_FB_QUEUE], ats[SWEET_FB_QUEUE];
	uint32_t n = 0;
	if (flush) {
		recs = (uint8_t *)IOMallocData(SWEET_FB_QUEUE * SWEET_FB_SWAP_REC_SIZE);
		if (recs != NULL) {
			if (fQueueCount != 0) {
				n = fQueueCount;
				memcpy(recs, fQueue, n * SWEET_FB_SWAP_REC_SIZE);
				memcpy(seqs, fQueueSeq, n * sizeof(seqs[0]));
				memcpy(ats, fQueueAt, n * sizeof(ats[0]));
			} else {
				n = 1;                  /* redraw of the latest frame */
				memcpy(recs, fPending, SWEET_FB_SWAP_REC_SIZE);
				seqs[0] = 0;
				ats[0] = 0;
			}
			fDirty = false;
		}
	}
	IOLockUnlock(fLock);
	if (n != 0) {
		int shown = presentQueued(recs, n, early);
		IOLockLock(fLock);
		if (shown >= 0 && seqs[shown] != 0) {
			uint64_t t = mach_absolute_time(), period = periodOr60();
			if (ats[shown] != 0 && t > ats[shown]) {
				uint64_t d = t - ats[shown];
				latAdd(&fLatReady, d);
				fLatReadyHist[MIN(d / period, 4ull)]++;
			}
			uint32_t keep = 0;      /* drop the shown swap and the older ones */
			for (uint32_t i = 0; i < fQueueCount; i++) {
				if (fQueueSeq[i] > seqs[shown]) {
					if (keep != i) {
						memcpy(fQueue[keep], fQueue[i], SWEET_FB_SWAP_REC_SIZE);
						fQueueSeq[keep] = fQueueSeq[i];
						fQueueAt[keep] = fQueueAt[i];
					}
					keep++;
				}
			}
			fQueueCount = keep;
		}
		if (shown >= 0) {
			fRetry = false;
			fEarlyShown += early;
		} else if (!early) {
			fRetry = true;          /* held back for the GPU: try again at the next tick */
		}
		if (shown < 0 || fQueueCount != 0) {
			fDirty = true;
		}
		IOLockUnlock(fLock);
	}
	if (recs != NULL) {
		IOFreeData(recs, SWEET_FB_QUEUE * SWEET_FB_SWAP_REC_SIZE);
	}
}

void
sweet_fb_gpu_done(void)
{
	if (gSweetFB != NULL) {
		gSweetFB->earlyKick();
	}
}

/* Next vsync tick: fLead before a predicted frame end, about 60 Hz (without frame ends: every 16.7 ms). */
void
AppleCLCD::rearm(IOTimerEventSource *timer)
{
	if (!fFlipReady) {
		timer->setTimeoutUS(SWEET_FB_VSYNC_US);
		return;
	}
	uint64_t now = mach_absolute_time(), step = fPeriod * fTickFrames, next = fLastVblank + step;
	uint64_t min = 0;
	nanoseconds_to_absolutetime(300 * 1000, &min);
	if (next < now + fLead + min) {
		next += ((now + fLead + min - next) / step + 1) * step;
	}
	/*
	 * 0136: no leeway. Without kIOTimeOptionsWithLeeway the thread call gets the default timer coalescing slop, and
	 * a tick more than fLead late lands after the frame end: the pending switch then waited a whole frame (0135 on
	 * the phone: ~2 ticks deferred per switch, chosen -> on screen 27-32 ms).
	 */
	timer->wakeAtTime(kIOTimeOptionsWithLeeway, next - fLead, 0);
}

void
AppleCLCD::addClient(IOMobileFramebufferUserClient *client)
{
	IOLockLock(fLock);
	fClients->setObject(client);
	IOLockUnlock(fLock);
}

void
AppleCLCD::removeClient(IOMobileFramebufferUserClient *client)
{
	IOLockLock(fLock);
	unsigned int i = fClients->getNextIndexOfObject(client, 0);
	if (i != (unsigned int)-1) {
		fClients->removeObject(i);
	}
	IOLockUnlock(fLock);
}

static uint32_t
get32(const uint8_t *p, uint32_t off)
{
	uint32_t v;
	memcpy(&v, p + off, 4);
	return v;
}

static uint32_t sweet_fb_swaps_logged;
static uint64_t sweet_fb_swap_count;
static uint32_t sweet_fb_layer_format[SWEET_FB_LAYERS];        /* last fourcc seen per layer, for the log */
static uint8_t  sweet_fb_xr8[1024];                             /* XR 10-bit -> 8-bit */

static void
sweet_fb_fourcc(uint32_t f, char out[5])
{
	for (int i = 0; i < 4; i++) {
		char c = (char)(f >> (24 - 8 * i));
		out[i] = (c >= 0x20 && c < 0x7f) ? c : '?';
	}
	out[4] = 0;
}

/* One row of an extended-range (or 10-bit) source to 8-bit BGRA. */
static void
sweet_fb_convert_row(uint32_t *dst, const uint8_t *src, int32_t w, uint32_t fmt)
{
	if (sweet_fb_xr8[1023] == 0) {
		for (uint32_t v = 0; v < 1024; v++) {
			uint32_t x = v < 384 ? 0 : v - 384 > 510 ? 510 : v - 384;
			sweet_fb_xr8[v] = (uint8_t)((x * 255 + 255) / 510);
		}
	}
	if (fmt == SWEET_FB_FORMAT_W40A || fmt == SWEET_FB_FORMAT_W40M) {
		const uint16_t *p = (const uint16_t *)src;
		for (int32_t x = 0; x < w; x++, p += 4) {
			dst[x] = 0xff000000u | (uint32_t)sweet_fb_xr8[p[2] >> 6] << 16 | (uint32_t)sweet_fb_xr8[p[1] >> 6] << 8 |
			    sweet_fb_xr8[p[0] >> 6];
		}
		return;
	}
	const uint32_t *p = (const uint32_t *)src;
	bool xr = fmt == SWEET_FB_FORMAT_W30R;
	for (int32_t x = 0; x < w; x++) {
		uint32_t v = p[x], b = v & 0x3ff, g = (v >> 10) & 0x3ff, r = (v >> 20) & 0x3ff;
		dst[x] = 0xff000000u | (xr ? (uint32_t)sweet_fb_xr8[r] << 16 | (uint32_t)sweet_fb_xr8[g] << 8 | sweet_fb_xr8[b] :
		    (r >> 2) << 16 | (g >> 2) << 8 | (b >> 2));
	}
}

/* Copy each enabled layer's surface into the panel memory (no scaling). */
/*
 * A swap only records the frame: copying 10 MB into the panel on every
 * swap (backboardd presents at 60 Hz) took the whole CPU. The vsync timer
 * shows the latest frame every fFlushTicks ticks with boot-arg sweetfb_hz;
 * by default as often as the copies stay within sweetfb_duty percent of one
 * CPU (default 25: a 4 ms copy allows a frame every 16 ms, 60 Hz; an 8 ms
 * one every 32 ms, 30 Hz). Frames the GPU drew cost the CPU nothing else.
 */
void
AppleCLCD::swap(const uint8_t *rec)
{
	IOLockLock(fLock);
	memcpy(fPending, rec, SWEET_FB_SWAP_REC_SIZE);
	if (fQueueCount == SWEET_FB_QUEUE) {
		/*
		 * The oldest swap was never shown: drop it. One record at a time: a single memmove from fQueue[0]
		 * spans several rows, which the fortified memmove rejects (object size of fQueue[0] = one record).
		 */
		for (uint32_t i = 1; i < SWEET_FB_QUEUE; i++) {
			memcpy(fQueue[i - 1], fQueue[i], SWEET_FB_SWAP_REC_SIZE);
			fQueueSeq[i - 1] = fQueueSeq[i];
			fQueueAt[i - 1] = fQueueAt[i];
		}
		fQueueCount--;
	}
	memcpy(fQueue[fQueueCount], rec, SWEET_FB_SWAP_REC_SIZE);
	uint64_t now = mach_absolute_time();
	fQueueAt[fQueueCount] = now;
	fQueueSeq[fQueueCount++] = ++fSeq;
	if (fLastSwapAt != 0 && now - fLastSwapAt < 8 * periodOr60()) {
		latAdd(&fLatSwap, now - fLastSwapAt);
	}
	fLastSwapAt = now;
	if (fLastNotifyAt != 0 && now - fLastNotifyAt < 4 * periodOr60()) {
		latAdd(&fLatRender, now - fLastNotifyAt);
	}
	fDirty = true;
	IOLockUnlock(fLock);
	earlyKick();                    /* shown right away if the GPU has already finished it */
	/* the format of every layer CoreAnimation sets (SwapSetLayer), logged when it changes */
	uint32_t mask = get32(rec, 0x104);
	for (uint32_t layer = 0; layer < SWEET_FB_LAYERS; layer++) {
		uint32_t id = get32(rec, 0x54 + 4 * layer);
		if (!(mask & (1u << layer)) || id == 0) {
			continue;
		}
		SweetSurface *s = sweet_surface_copy(id);
		if (s != NULL && s->pixelFormat != sweet_fb_layer_format[layer]) {
			/* every format change of a layer (SwapSetLayer), whatever the swap count */
			char was[5], now[5];
			sweet_fb_fourcc(sweet_fb_layer_format[layer], was);
			sweet_fb_fourcc(s->pixelFormat, now);
			IOLog("SWEET FB: layer %u format '%s' (0x%x) -> '%s' (0x%x), surface %u %ux%u bpe %u bpr %u%s\n", layer,
			    was, sweet_fb_layer_format[layer], now, s->pixelFormat, id, s->width, s->height, s->bytesPerElement,
			    s->bytesPerRow, (s->pixelFormat == SWEET_FB_FORMAT_W30R || s->pixelFormat == SWEET_FB_FORMAT_W40A ||
			    s->pixelFormat == SWEET_FB_FORMAT_W40M || s->pixelFormat == SWEET_FB_FORMAT_L10R) ?
			    " (converted to BGRA8)" : (s->bytesPerElement != 4 ? " (NOT SHOWN: unsupported)" : ""));
			sweet_fb_layer_format[layer] = s->pixelFormat;
		}
		OSSafeReleaseNULL(s);
	}
	if ((++sweet_fb_swap_count % 1000) == 0) {
		IOLog("SWEET FB: %llu swaps\n", sweet_fb_swap_count);
	}
}

/*
 * K0c: no GPU job pending on any enabled layer's surface. *jobs = the GPU jobs that used them so far (it only
 * grows: a change means a new job touched one of them).
 */
bool
AppleCLCD::layersIdle(const uint8_t *rec, uint64_t *jobs)
{
	uint32_t mask = get32(rec, 0x104);
	bool idle = true;
	*jobs = 0;
	for (uint32_t layer = 0; layer < SWEET_FB_LAYERS; layer++) {
		uint32_t id = get32(rec, 0x54 + 4 * layer);
		if (!(mask & (1u << layer)) || id == 0) {
			continue;
		}
		SweetSurface *s = sweet_surface_copy(id);
		if (s == NULL || s == fSurface) {
			OSSafeReleaseNULL(s);
			continue;
		}
		uint64_t n = 0;
		if (!viva_gpu_surface_idle(s, &n)) {
			idle = false;
		}
		*jobs += n;
		s->release();
	}
	return idle;
}

/*
 * K0c: show the newest of the n queued swaps (oldest first) whose surfaces the GPU has finished. Returns its index,
 * or -1 when none is ready yet (or its copy has to be redone): nothing shown, retry at the next tick.
 */
int
AppleCLCD::presentQueued(const uint8_t *recs, uint32_t n, bool early)
{
	uint64_t jobs = 0;
	int pick = -1;
	for (int i = (int)n - 1; i >= 0; i--) {
		if (layersIdle(recs + (size_t)i * SWEET_FB_SWAP_REC_SIZE, &jobs)) {
			pick = i;
			break;
		}
	}
	bool force = false;
	if (pick < 0) {
		if (early) {
			return -1;              /* the GPU is not done yet: its completion or the next tick tries again */
		}
		if (fGpuWaitMax != 0 && fGpuTicks < fGpuWaitMax) {
			fGpuTicks++;
			fGpuWaits++;
			sweet_fb_gpu_waits++;   /* 0137: the GPU governor boosts */
			return -1;
		}
		pick = (int)n - 1;
		force = true;
		if (fGpuWaitMax != 0 && ++fGpuForced <= 8) {
			IOLog("SWEET FB: frame shown after waiting %u vsyncs for the GPU\n", fGpuTicks);
		}
	}
	return present(recs + (size_t)pick * SWEET_FB_SWAP_REC_SIZE, force, jobs) ? pick : -1;
}

/*
 * Show a swap whose surfaces the GPU has finished (jobs = their GPU job count then; force: shown as it is). false:
 * a new GPU job used a surface during the copy, which may be mixed: redo it at the next tick.
 */
bool
AppleCLCD::present(const uint8_t *rec, bool force, uint64_t jobs)
{
	uint32_t mask = get32(rec, 0x104);
	uint64_t jobsAfter = 0;

	IOLockLock(fScanLock);
	if (fFlipAllowed && !fFlipTried) {
		fFlipTried = true;              /* once: findLKLayer and the frame-end calibration (< 0.1 s) */
		flipSetup(NULL, NULL);
	}
	/*
	 * Scanout costs no copy, so a scanned-out frame skips the CPU duty limit; a frame that falls back to the copy
	 * does not (with scanout "on" but every frame copied, copies used to run at every vsync and early present).
	 */
	fScanLive = fScanOn && scanout(rec);
	if (fScanLive) {
		IOLockUnlock(fScanLock);
		fGpuTicks = 0;
		return true;
	}
	if (!fConsolePaused) {
		int quiet = 0;
		fConsolePaused = true;
		if (PE_parse_boot_argn("sweetfb_noconsole", &quiet, sizeof(quiet)) && quiet != 0) {
			/* Not PE_initialize_console(kPEDisableScreen): its console switch
			 * writes the read-only-late cons_ops_index and panics. */
			sweet_console_muted = 1;
		}
	}
	uint64_t t0 = mach_absolute_time();

	/* each enabled layer clipped to the panel first: which buffer to draw in depends on what the frame covers */
	struct Blit {
		SweetSurface *s;
		uintptr_t     src;
		int32_t       sx, sy, dx, dy, w, h;
		uint32_t      bpe;
	} blit[SWEET_FB_LAYERS];
	uint32_t nblit = 0;
	bool full = false, usesDefault = false;
	for (uint32_t layer = 0; layer < SWEET_FB_LAYERS; layer++) {
		uint32_t id = get32(rec, 0x54 + 4 * layer);
		const uint8_t *a = rec + 0x64 + 16 * layer;
		const uint8_t *b = rec + 0xc4 + 16 * layer;
		int32_t sx = (int32_t)get32(a, 0), sy = (int32_t)get32(a, 4);
		int32_t dx = (int32_t)get32(b, 0), dy = (int32_t)get32(b, 4);
		int32_t w = (int32_t)get32(b, 8), h = (int32_t)get32(b, 12);

		if (!(mask & (1u << layer)) || id == 0) {
			continue;
		}
		SweetSurface *s = sweet_surface_copy(id);
		if (sweet_fb_swaps_logged < 16) {
			sweet_fb_swaps_logged++;
			IOLog("SWEET FB: swap %u layer %u surface %u %ux%u fmt 0x%x bpr %u a %d,%d %ux%u b %d,%d %dx%d\n",
			    get32(rec, 0x50), layer, id, s ? s->width : 0, s ? s->height : 0,
			    s ? s->pixelFormat : 0, s ? s->bytesPerRow : 0, sx, sy, get32(a, 8),
			    get32(a, 12), dx, dy, w, h);
		}
		if (s == NULL || s == fSurface) {
			usesDefault |= s != NULL;       /* drawn straight into LK's buffer */
			OSSafeReleaseNULL(s);
			continue;
		}
		uintptr_t src = s->kernelAddress();
		uint32_t bpe = s->bytesPerElement ? s->bytesPerElement : 4;
		if (w <= 0 || h <= 0) {
			w = (int32_t)s->width;
			h = (int32_t)s->height;
		}
		if (sx < 0 || sy < 0) {
			sx = sy = 0;
		}
		if (dx < 0 || dy < 0) {
			dx = dy = 0;
		}
		/* Logical (presented) display coordinates -> panel coordinates. */
		dx -= fOffX;
		dy -= fOffY;
		if (dx < 0) {
			sx -= dx;
			w += dx;
			dx = 0;
		}
		if (dy < 0) {
			sy -= dy;
			h += dy;
			dy = 0;
		}
		if (w > (int32_t)s->width - sx) {
			w = (int32_t)s->width - sx;
		}
		if (h > (int32_t)s->height - sy) {
			h = (int32_t)s->height - sy;
		}
		if (w > (int32_t)fWidth - dx) {
			w = (int32_t)fWidth - dx;
		}
		if (h > (int32_t)fHeight - dy) {
			h = (int32_t)fHeight - dy;
		}
		uint64_t last = (uint64_t)s->offset + (uint64_t)(sy + h - 1) * s->bytesPerRow +
		    (uint64_t)(sx + w) * bpe;
		if (w <= 0 || h <= 0 || s->elementWidth != 1 || s->elementHeight != 1 ||
		    s->memory == NULL || last > s->memory->getLength()) {
			src = 0;
		}
		uint32_t fmt = s->pixelFormat;
		bool convert = fmt == SWEET_FB_FORMAT_W30R || fmt == SWEET_FB_FORMAT_L10R ||
		    ((fmt == SWEET_FB_FORMAT_W40A || fmt == SWEET_FB_FORMAT_W40M) && bpe == 8);
		if (src != 0 && !convert && bpe != 4) {
			src = 0;
		}
		if (src != 0 && dx == 0 && dy == 0 && w == (int32_t)fWidth && h == (int32_t)fHeight) {
			full = true;
		}
		blit[nblit++] = { s, src, sx, sy, dx, dy, w, h, bpe };
	}

	/*
	 * K0d: into the buffer not on screen when the frame covers the whole panel (switched at the next frame end),
	 * else into the one on screen (LK's when the console or the default surface must be seen).
	 */
	int shownBuf = fShown.buf;
	int buf;
	if (!sweet_console_muted || usesDefault) {
		buf = 0;
	} else if (fFlipReady && full) {
		buf = shownBuf == 0 ? 1 : 0;
	} else {
		buf = shownBuf >= 0 ? shownBuf : 0;
	}
	uintptr_t dst = fBufKVA[buf];
	for (uint32_t i = 0; i < nblit; i++) {
		Blit *bl = &blit[i];
		SweetSurface *s = bl->s;
		if (bl->src != 0) {
			sweet_surface_before_cpu_access(s);     /* the GPU may have drawn it: no stale CPU lines */
			uint32_t fmt = s->pixelFormat;
			bool convert = fmt == SWEET_FB_FORMAT_W30R || fmt == SWEET_FB_FORMAT_L10R ||
			    ((fmt == SWEET_FB_FORMAT_W40A || fmt == SWEET_FB_FORMAT_W40M) && bl->bpe == 8);
			for (int32_t y = 0; y < bl->h; y++) {
				uint32_t *to = (uint32_t *)(dst + (uint64_t)(bl->dy + y) * fRowBytes + (uint64_t)bl->dx * 4);
				const uint8_t *from = (const uint8_t *)(bl->src + s->offset + (uint64_t)(bl->sy + y) * s->bytesPerRow +
				    (uint64_t)bl->sx * bl->bpe);
				if (convert) {
					sweet_fb_convert_row(to, from, bl->w, fmt);
				} else {
					memcpy(to, from, (size_t)bl->w * 4);
				}
			}
		}
		s->release();
	}
	/* the next copy no sooner than its share of the CPU (sweetfb_duty) allows */
	uint64_t t1 = mach_absolute_time(), ns;
	uint32_t duty = gSweetFbDutyCap != 0 ? MIN(fDuty, gSweetFbDutyCap) : fDuty;   /* Low Power Mode */
	fNextCopy = t1 + (t1 - t0) * (100 - duty) / duty;
	absolutetime_to_nanoseconds(t1 - t0, &ns);
	fCopies++;
	fCopyNs += ns;
	/* a GPU job used a surface while it was copied (CoreAnimation reuses it): the copy may be mixed, redo it */
	if (!force && (!layersIdle(rec, &jobsAfter) || jobsAfter != jobs)) {
		IOLockUnlock(fScanLock);
		fGpuTicks++;
		fGpuRecopies++;
		return false;
	}
	if (buf == shownBuf) {
		dropNext();                     /* drawn in place: an older pending switch would go back in time */
		fInPlace++;
	} else {
		show(bufTarget(buf));
	}
	IOLockUnlock(fScanLock);
	fGpuTicks = 0;
	return true;
}

/* ---- viva direct scanout ---- */

#define DISP_OVL0_PA            0x14005000ull
#define DISP_OVL0_2L_PA         0x14006000ull
#define SMI_LARB0_PA            0x14003000ull
#define SMI_LARB1_PA            0x14004000ull
#define OVL_EN                  0x00c   /* bit 0 enable, bit 22 BYPASS_SHADOW */
#define OVL_ROI_SIZE            0x020
#define OVL_SRC_CON             0x02c   /* bit n: layer n */
#define OVL_L_CON(n)            (0x030 + 0x20 * (n))
#define OVL_L_SRC_SIZE(n)       (0x038 + 0x20 * (n))
#define OVL_L_OFFSET(n)         (0x03c + 0x20 * (n))
#define OVL_L_PITCH_MSB(n)      (0x040 + 0x20 * (n))
#define OVL_L_PITCH(n)          (0x044 + 0x20 * (n))
#define OVL_L_ADDR(n)           (0xf40 + 0x20 * (n))
#define OVL_DATAPATH_EXT_CON    0x324   /* bit k: ext layer k */
#define OVL_EL_CON(k)           (0x330 + 0x20 * (k))
#define OVL_EL_SRC_SIZE(k)      (0x338 + 0x20 * (k))
#define OVL_EL_PITCH_MSB(k)     (0x340 + 0x20 * (k))
#define OVL_EL_PITCH(k)         (0x344 + 0x20 * (k))
#define OVL_EL_ADDR(k)          (0xfb0 + 4 * (k))
#define LARB_NON_SEC_CON(p)     (0x380 + 4 * (p))       /* bit 0: port translated by the M4U (inferred) */
#define OVL_INTEN               0x004
#define OVL_INTSTA              0x008   /* write 0 to a bit to clear it */
#define OVL_INT_FME_CPL         (1u << 1)       /* frame complete */
#define OVL_ADDCON_DBG          0x244   /* [12:0] x, [28:16] y of the pixel being blended */


static inline uint32_t
disp_rd(vm_offset_t b, uint32_t o)
{
	return *(volatile uint32_t *)(b + o);
}

static inline void
disp_wr(vm_offset_t b, uint32_t o, uint32_t v)
{
	*(volatile uint32_t *)(b + o) = v;
}

static void
disp_say(char **p, char *end, const char *fmt, ...) __printflike(3, 4);

static uint64_t
absUsFB(uint64_t abs)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(abs, &ns);
	return ns / 1000;
}

static void
disp_say(char **p, char *end, const char *fmt, ...)
{
	if (p == NULL || *p >= end - 1) {
		return;
	}
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(*p, (size_t)(end - *p), fmt, ap);
	va_end(ap);
	if (n > 0) {
		*p += MIN((size_t)n, (size_t)(end - *p - 1));
	}
}

/* A panic draws into LK's buffer: show it (any context; registers only). */
static void
sweet_fb_panic_show(void)
{
	AppleCLCD *fb = gSweetFB;
	if (fb != NULL && fb->fScanOvl >= 0) {
		vm_offset_t base = fb->fDisp[fb->fScanOvl];
		disp_wr(base, fb->fScanPitchMsbReg, fb->fLkPitchMsb);
		disp_wr(base, fb->fScanPitchReg, fb->fLkPitch);
		disp_wr(base, fb->fScanAddrReg, fb->fLkAddr);
	}
}

bool
AppleCLCD::dispMap(void)
{
	static const uint64_t pa[4] = { DISP_OVL0_PA, DISP_OVL0_2L_PA, SMI_LARB0_PA, SMI_LARB1_PA };
	for (int i = 0; i < 4; i++) {
		if (fDispMap[i] != NULL) {
			continue;
		}
		IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(pa[i], 0x1000, kIODirectionInOut);
		fDispMap[i] = md != NULL ? md->map(kIOMapInhibitCache) : NULL;
		OSSafeReleaseNULL(md);
		if (fDispMap[i] == NULL) {
			return false;
		}
		fDisp[i] = (vm_offset_t)fDispMap[i]->getVirtualAddress();
	}
	return true;
}

/*
 * The enabled OVL layer that scans the panel memory (LK's three buffers): OVL0 layers 0-3, its ext layers
 * 0-2, OVL0_2L layers 0-1. Read only.
 */
bool
AppleCLCD::findLKLayer(char **out, char *end)
{
	uint64_t lo = fPhys, hi = fPhys + 3ull * fRowBytes * fHeight;
	if (!dispMap()) {
		disp_say(out, end, "display registers not mappable\n");
		return false;
	}
	for (int o = 0; o < 2; o++) {
		vm_offset_t b = fDisp[o];
		if (!(disp_rd(b, OVL_EN) & 1)) {
			continue;
		}
		for (int n = 0; n < (o ? 2 : 4); n++) {
			uint32_t addr = disp_rd(b, OVL_L_ADDR(n));
			if ((disp_rd(b, OVL_SRC_CON) & (1u << n)) && addr >= lo && addr < hi) {
				fScanOvl = o;
				fScanAddrReg = OVL_L_ADDR(n);
				fScanPitchReg = OVL_L_PITCH(n);
				fScanPitchMsbReg = OVL_L_PITCH_MSB(n);
				snprintf(fScanLayer, sizeof(fScanLayer), "%s L%d", o ? "OVL0_2L" : "OVL0", n);
			}
		}
		for (int k = 0; o == 0 && k < 3; k++) {
			uint32_t addr = disp_rd(b, OVL_EL_ADDR(k));
			if ((disp_rd(b, OVL_DATAPATH_EXT_CON) & (1u << k)) && addr >= lo && addr < hi) {
				fScanOvl = o;
				fScanAddrReg = OVL_EL_ADDR(k);
				fScanPitchReg = OVL_EL_PITCH(k);
				fScanPitchMsbReg = OVL_EL_PITCH_MSB(k);
				snprintf(fScanLayer, sizeof(fScanLayer), "OVL0 EL%d", k);
			}
		}
	}
	if (fScanOvl < 0) {
		disp_say(out, end, "no enabled OVL layer reads the panel memory 0x%llx..0x%llx\n", lo, hi);
		return false;
	}
	vm_offset_t b = fDisp[fScanOvl];
	fLkAddr = disp_rd(b, fScanAddrReg);
	fLkPitch = disp_rd(b, fScanPitchReg);
	fLkPitchMsb = disp_rd(b, fScanPitchMsbReg);
	sweet_fb_panic_hook = sweet_fb_panic_show;      /* from now on the layer may read something else */
	/* OVL0 reads through SMI larb0 ports 1-2, OVL0_2L through larb1 ports 0-1 */
	vm_offset_t larb = fDisp[fScanOvl ? 3 : 2];
	uint32_t p0 = disp_rd(larb, LARB_NON_SEC_CON(fScanOvl ? 0 : 1)), p1 = disp_rd(larb, LARB_NON_SEC_CON(fScanOvl ? 1 : 2));
	disp_say(out, end, "LK layer %s: addr 0x%08x pitch 0x%08x (msb 0x%x), larb ports 0x%08x 0x%08x%s\n", fScanLayer,
	    fLkAddr, fLkPitch, fLkPitchMsb, p0, p1, ((p0 | p1) & 1) ? " (bit 0 set: may go through the M4U!)" : "");
	return true;
}

/* Point LK's layer at the swap's surface, if the swap is one full-screen contiguous BGRA surface. */
bool
AppleCLCD::scanout(const uint8_t *rec)
{
	uint32_t mask = get32(rec, 0x104), layers = 0, id = 0, idx = 0;
	for (uint32_t l = 0; l < SWEET_FB_LAYERS; l++) {
		if ((mask & (1u << l)) && get32(rec, 0x54 + 4 * l) != 0) {
			layers++;
			id = get32(rec, 0x54 + 4 * l);
			idx = l;
		}
	}
	const uint8_t *a = rec + 0x64 + 16 * idx, *b = rec + 0xc4 + 16 * idx;
	SweetSurface *s = layers == 1 ? sweet_surface_copy(id) : NULL;
	const char *why = NULL;
	uint64_t pa = 0;
	if (layers != 1) {
		why = "not exactly one layer";
	} else if (s == NULL || !s->contig) {
		why = "surface not contiguous";
	} else if (s->width != fLogicalW || s->height != fLogicalH || s->bytesPerElement != 4 ||
	    s->pixelFormat != SWEET_FB_FORMAT_BGRA || s->bytesPerRow > 0xfffff) {
		why = "not a full-screen BGRA surface";
	} else if (get32(a, 0) != 0 || get32(a, 4) != 0 || get32(b, 0) != 0 || get32(b, 4) != 0 ||
	    (get32(b, 8) != 0 && get32(b, 8) != fLogicalW) || (get32(b, 12) != 0 && get32(b, 12) != fLogicalH)) {
		why = "layer not at the origin, full size";
	} else {
		IOByteCount len = 0;
		pa = s->memory->getPhysicalSegment(0, &len, kIOMemoryMapperNone);
		if (pa == 0 || len < s->allocSize || pa + s->allocSize > (1ull << 32)) {
			why = "memory not one range below 4 GiB";
		}
	}
	if (why == NULL && fScanOvl < 0 && !findLKLayer(NULL, NULL)) {
		why = "LK's layer not found";
	}
	if (why != NULL) {
		if (fScanWhy != why) {
			IOLog("SWEET FB: scanout: copying (%s)\n", why);
			fScanWhy = why;
		}
		OSSafeReleaseNULL(s);
		if (!fFlipReady) {
			scanoutStop();          /* LK's buffer at once; with flips the copy's buffer follows at a frame end */
		}
		fScanFallbacks++;
		return false;
	}
	sweet_surface_before_device_read(s);    /* CPU-drawn pixels out of the caches */
	Target t;
	t.addr = (uint32_t)(pa + s->offset + (uint64_t)fOffY * s->bytesPerRow + (uint64_t)fOffX * 4);
	t.pitchMsb = (fLkPitchMsb & ~0xfu) | ((s->bytesPerRow >> 16) & 0xf);
	t.pitch = (fLkPitch & ~0xffffu) | (s->bytesPerRow & 0xffff);
	t.buf = -1;
	t.surface = s;                          /* keeps the memory alive while the OVL reads it */
	t.swap = get32(rec, 0x50);
	if (fShown.buf >= 0 || fScanWhy != NULL) {
		IOLog("SWEET FB: scanout: %s reads surface %u at 0x%08x (pitch %u)%s\n", fScanLayer, s->id, t.addr,
		    s->bytesPerRow, fFlipReady ? " from the next frame end" : "");
		fScanWhy = NULL;
	}
	show(t);
	fScanFrames++;
	return true;
}

/* Back to LK's buffer (the CPU copy), at once. */
void
AppleCLCD::scanoutStop(void)
{
	dropNext();
	if (fShown.buf < 0) {
		layerApply(bufTarget(0));
	}
}

/* ---- K0d: layer switches at the panel's frame end ---- */

AppleCLCD::Target
AppleCLCD::bufTarget(int buf)
{
	Target t;
	t.addr = (uint32_t)fBufPA[buf];
	t.pitch = fLkPitch;
	t.pitchMsb = fLkPitchMsb;
	t.buf = buf;
	t.surface = NULL;
	t.swap = 0;
	return t;
}

void
AppleCLCD::layerWrite(const Target &t)
{
	if (fScanOvl < 0) {
		return;                 /* LK's layer unknown: only its own buffer is ever shown */
	}
	vm_offset_t base = fDisp[fScanOvl];
	disp_wr(base, fScanPitchMsbReg, t.pitchMsb);
	disp_wr(base, fScanPitchReg, t.pitch);
	disp_wr(base, fScanAddrReg, t.addr);
}

/* fScanLock: t is on the layer from now on (written already when written is true); takes t's surface reference. */
void
AppleCLCD::layerApply(const Target &t)
{
	layerWrite(t);
	SweetSurface *old = fShown.surface;
	fShown = t;
	fShownSwap = t.swap;
	if (fNextValid && fNext.swap == t.swap) {
		fNextSwap = 0;
	}
	OSSafeReleaseNULL(old);
}

/* fScanLock: show t from the next frame end (with flips) or at once; takes t's surface reference. */
void
AppleCLCD::show(const Target &t)
{
	if (!fFlipReady) {
		layerApply(t);
		return;
	}
	dropNext();
	fNext = t;
	fNextValid = true;
	fNextSwap = t.swap;
	fNextChosenAt = mach_absolute_time();
}

void
AppleCLCD::dropNext(void)
{
	if (fNextValid) {
		OSSafeReleaseNULL(fNext.surface);
		fNextValid = false;
	}
	fNextSwap = 0;
}

/*
 * SwapWait condition (see the top): op 0 retired, 1 displayed, 2 nothing queued or waiting. A copied frame never
 * holds its surface (the layer reads a panel buffer), so it is retired once it leaves the queue.
 */
bool
AppleCLCD::swapDone(uint32_t swap, uint32_t op)
{
	bool queued = false;
	IOLockLock(fLock);
	for (uint32_t i = 0; i < fQueueCount; i++) {
		if (op == 2 || get32(fQueue[i], 0x50) == swap) {
			queued = true;
			break;
		}
	}
	IOLockUnlock(fLock);
	uint32_t next = fNextSwap, shown = fShownSwap;
	if (queued) {
		return false;
	}
	switch (op) {
	case 0:
		return swap == 0 || (next != swap && shown != swap);
	case 1:
		return next != swap;
	case 2:
		return next == 0;
	default:
		return true;
	}
}

/* Any enabled OVL layer (except LK's) reading [lo, hi). */
bool
AppleCLCD::layerReads(uint64_t lo, uint64_t hi)
{
	for (int o = 0; o < 2; o++) {
		vm_offset_t b = fDisp[o];
		if (!(disp_rd(b, OVL_EN) & 1)) {
			continue;
		}
		for (int n = 0; n < (o ? 2 : 4); n++) {
			uint32_t addr = disp_rd(b, OVL_L_ADDR(n));
			if ((disp_rd(b, OVL_SRC_CON) & (1u << n)) && addr >= lo && addr < hi &&
			    !(o == fScanOvl && OVL_L_ADDR(n) == fScanAddrReg)) {
				return true;
			}
		}
		for (int k = 0; o == 0 && k < 3; k++) {
			uint32_t addr = disp_rd(b, OVL_EL_ADDR(k));
			if ((disp_rd(b, OVL_DATAPATH_EXT_CON) & (1u << k)) && addr >= lo && addr < hi &&
			    !(o == fScanOvl && OVL_EL_ADDR(k) == fScanAddrReg)) {
				return true;
			}
		}
	}
	return false;
}

/*
 * fScanLock. Double buffering: LK's layer must read LK's buffer (fPhys), one of LK's other buffers must be read by no
 * other layer, and the OVL must report frame ends. Otherwise fFlipWhy says why and everything stays as before.
 * LK reserves SWEET_FB_LK_BUFFERS panel buffers back to back (live DT mblock-18-framebuffer: 0x7bc10000, 0x1df0000
 * bytes = 3 x 1088 x 2400 x 4). 0128: some boots LK leaves OVL0 L1 enabled on its second buffer (seen after a panic
 * reboot: L1 1080x2400 pitch 4320 at 0x7c606000); the double buffer was then off and every copy went to the buffer on
 * screen (tearing bands while scrolling). The third buffer is used instead.
 */
bool
AppleCLCD::flipSetup(char **out, char *end)
{
	fFlipReady = false;
	dropNext();
	uint64_t size = (uint64_t)fRowBytes * fHeight, pa1 = 0;
	const char *why = NULL;
	if (fScanOvl < 0 && !findLKLayer(out, end)) {
		why = "LK's layer not found";
	} else if (fLkAddr != fPhys) {
		why = "LK's layer does not read the boot framebuffer";
	} else {
		why = "another OVL layer reads each of LK's other buffers";
		for (uint32_t k = 1; k < SWEET_FB_LK_BUFFERS; k++) {
			uint64_t pa = fPhys + size * k;
			if (pa + size > (1ull << 32)) {
				why = "LK's other buffers above 4 GiB";
				break;
			}
			if (!layerReads(pa, pa + size)) {
				pa1 = pa;
				why = NULL;
				break;
			}
		}
	}
	if (why == NULL) {
		if (fBuf1Map != NULL && fBufPA[1] != pa1) {
			fBuf1Map->release();    /* "disp flip on" after LK's layers changed: another buffer now */
			fBuf1Map = NULL;
		}
		if (fBuf1Map == NULL) {
			IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(pa1, size, kIODirectionInOut);
			fBuf1Map = md != NULL ? md->map(kIOMapWriteCombineCache) : NULL;
			OSSafeReleaseNULL(md);
		}
		if (fBuf1Map == NULL) {
			why = "LK's other buffer not mappable";
		} else {
			fBufKVA[1] = (uintptr_t)fBuf1Map->getVirtualAddress();
			fBufPA[1] = pa1;
			if (!vblankCalibrate(out, end)) {
				why = "no frame-end status from the OVL";
			}
		}
	}
	fFlipWhy = why;
	if (why != NULL) {
		IOLog("SWEET FB: double buffer off (%s): copies go to the buffer on screen\n", why);
		disp_say(out, end, "double buffer off: %s\n", why);
		return false;
	}
	uint64_t ns;
	absolutetime_to_nanoseconds(fPeriod, &ns);
	IOLog("SWEET FB: double buffer: 0x%llx / 0x%llx, frame end from %s INTSTA%s, panel frame %llu us, vsync every %u "
	    "frame(s)\n", fBufPA[0], fBufPA[1], fVblankOvl ? "OVL0_2L" : "OVL0", fVblankInten ? " (INTEN set)" : "",
	    ns / 1000, fTickFrames);
	disp_say(out, end, "double buffer on: 0x%llx / 0x%llx, frame %llu us\n", fBufPA[0], fBufPA[1], ns / 1000);
	fMisses = 0;
	fResync = false;
	fFlipReady = true;
	return true;
}

/*
 * fScanLock. Three frame ends in a row on LK's OVL (or the other one, which runs in the same pipe), first with the
 * status bit as LK left the interrupt enables, then with FME_CPL enabled (the OVL's GIC line has no handler: the GIC
 * driver masks it if it ever fires). Busy for at most ~0.25 s, once.
 */
bool
AppleCLCD::vblankCalibrate(char **out, char *end)
{
	uint64_t limit, lo, hi;
	nanoseconds_to_absolutetime(40 * 1000 * 1000, &limit);
	nanoseconds_to_absolutetime(4 * 1000 * 1000, &lo);
	nanoseconds_to_absolutetime(40 * 1000 * 1000, &hi);
	for (int k = 0; k < 2; k++) {
		int o = k == 0 ? fScanOvl : 1 - fScanOvl;
		vm_offset_t b = fDisp[o];
		if (!(disp_rd(b, OVL_EN) & 1)) {
			continue;
		}
		for (int inten = 0; inten < 2; inten++) {
			uint32_t was = disp_rd(b, OVL_INTEN);
			if (inten) {
				if (was & OVL_INT_FME_CPL) {
					break;
				}
				disp_wr(b, OVL_INTEN, was | OVL_INT_FME_CPL);
			}
			fVblankOvl = o;
			uint64_t t[3] = { 0, 0, 0 };
			bool applied;
			for (int i = 0; i < 3; i++) {
				t[i] = vblankPoll(limit, NULL, &applied);
				if (t[i] == 0) {
					break;
				}
			}
			uint64_t p1 = t[1] - t[0], p2 = t[2] - t[1];
			if (t[2] != 0 && p1 >= lo && p1 <= hi && (p1 > p2 ? p1 - p2 : p2 - p1) <= p1 / 10) {
				fVblankInten = inten != 0;
				fPeriod = (t[2] - t[0]) / 2;
				fLastVblank = t[2];
				uint64_t tick;
				nanoseconds_to_absolutetime((uint64_t)SWEET_FB_VSYNC_US * 1000, &tick);
				fTickFrames = (uint32_t)MAX((tick + fPeriod / 2) / fPeriod, 1ull);
				return true;
			}
			disp_say(out, end, "%s INTSTA (INTEN %s): frame ends at +0 / +%llu / +%llu ticks: not usable\n",
			    o ? "OVL0_2L" : "OVL0", inten ? "set" : "as LK left it", t[1] ? p1 : 0, t[2] ? p2 : 0);
			if (inten) {
				disp_wr(b, OVL_INTEN, was);
			}
		}
	}
	fVblankOvl = -1;
	return false;
}

/*
 * fScanLock. Busy-poll fVblankOvl's frame-complete bit for at most `budget` (absolute time), the bit cleared first
 * (older frame ends are of no use). Returns when it rose, 0 = not within the budget. With t, the layer registers
 * are written right then, interrupts off from reading the bit to the last write, unless the previous read was more
 * than fSlack earlier (the poll was held up: the frame end may be long past, too late to switch safely).
 */
uint64_t
AppleCLCD::vblankPoll(uint64_t budget, const Target *t, bool *applied)
{
	vm_offset_t b = fDisp[fVblankOvl];
	*applied = false;
	disp_wr(b, OVL_INTSTA, disp_rd(b, OVL_INTSTA) & ~OVL_INT_FME_CPL);
	uint64_t t0 = mach_absolute_time(), prev = t0, now = t0, edge = 0;
	while (now - t0 < budget) {
		boolean_t was = ml_set_interrupts_enabled(FALSE);
		uint32_t s = disp_rd(b, OVL_INTSTA);
		now = mach_absolute_time();
		if (s & OVL_INT_FME_CPL) {
			if (t != NULL && now - prev <= fSlack) {
				layerWrite(*t);
				*applied = true;
			}
			ml_set_interrupts_enabled(was);
			disp_wr(b, OVL_INTSTA, disp_rd(b, OVL_INTSTA) & ~OVL_INT_FME_CPL);
			edge = now;
			break;
		}
		ml_set_interrupts_enabled(was);
		prev = now;
	}
	fPolls++;
	fPollAbs += now - t0;
	return edge;
}

/* A frame end seen at `at`: the phase, and the period refined (panel clocks drift a little from the calibration). */
void
AppleCLCD::vblankSeen(uint64_t at)
{
	if (fLastVblank != 0 && at > fLastVblank) {
		uint64_t n = (at - fLastVblank + fPeriod / 2) / fPeriod;
		if (n >= 1 && n <= 600) {
			fPeriod = (fPeriod * 7 + (at - fLastVblank) / n) / 8;
		}
	}
	fLastVblank = at;
}

/*
 * Vsync tick (timer, before anything else): a pending switch happens at the predicted frame end, which is about
 * fLead away. Too far (the timer was late, or early): next tick. After a miss the next try polls up to a whole
 * frame to find the phase again; after 8 misses in a row double buffering stops.
 */
void
AppleCLCD::vblankTick(void)
{
	IOLockLock(fScanLock);
	if (!fFlipReady || !fNextValid) {
		IOLockUnlock(fScanLock);
		return;
	}
	uint64_t now = mach_absolute_time(), margin;
	nanoseconds_to_absolutetime(500 * 1000, &margin);
	uint64_t next = fLastVblank + fPeriod;
	if (next < now) {
		next += ((now - next) / fPeriod + 1) * fPeriod;
	}
	uint64_t budget, wide;
	nanoseconds_to_absolutetime(3000 * 1000, &wide);
	if (fResync) {
		budget = fPeriod + fPeriod / 4;
	} else if (next - now <= fLead + margin) {
		budget = next - now + margin;
	} else if (next - now <= wide) {
		budget = next - now + margin;   /* 0136: a little early: poll up to 3 ms rather than lose the frame */
		fTickWidePolls++;
	} else {
		fFlipEarly++;           /* not near a frame end now: the next tick is */
		uint64_t prevEnd = next - fPeriod, d = now - prevEnd;
		if (now >= prevEnd && d < fPeriod / 2) {
			fTickLate++;    /* the frame end it was armed for has just passed */
			fTickLateSum += d;
			fTickLateMax = MAX(fTickLateMax, d);
		} else {
			fTickEarly++;
			fTickEarlySum += next - now;
		}
		IOLockUnlock(fScanLock);
		return;
	}
	bool applied = false;
	uint64_t at = vblankPoll(budget, &fNext, &applied);
	if (at != 0) {
		vblankSeen(at);
		fResync = false;
		fMisses = 0;
	}
	if (applied) {
		if (fNextChosenAt != 0 && at > fNextChosenAt) {
			latAdd(&fLatFlip, at - fNextChosenAt);
		}
		layerApply(fNext);      /* written at the frame end already: bookkeeping */
		fNextValid = false;
		fNextSwap = 0;
		fFlips++;
	} else if (at != 0) {
		fFlipUnsure++;          /* held up while polling: switch at the next frame end */
	} else {
		fFlipMissed++;
		fResync = true;
		if (++fMisses >= 8) {
			fFlipReady = false;
			fFlipWhy = "frame ends stopped";
			IOLog("SWEET FB: double buffer off: no frame end in 8 tries\n");
			layerApply(fNext);
			fNextValid = false;
			fNextSwap = 0;
		}
	}
	IOLockUnlock(fScanLock);
}

/* USB link "disp [regs|scanout on|off|flip on|off]". */
void
AppleCLCD::dispCommand(const char *args, char *out, size_t len)
{
	char *p = out, *end = out + len;
	out[0] = 0;
	while (*args == ' ') {
		args++;
	}
	IOLockLock(fScanLock);
	if (strncmp(args, "regs", 4) == 0) {
		if (!dispMap()) {
			disp_say(&p, end, "display registers not mappable\n");
		} else {
			for (int o = 0; o < 2; o++) {
				vm_offset_t b = fDisp[o];
				disp_say(&p, end, "%s: en 0x%08x roi 0x%08x src_con 0x%08x ext 0x%08x inten 0x%08x intsta 0x%08x "
				    "addcon_dbg 0x%08x\n", o ? "OVL0_2L" : "OVL0", disp_rd(b, OVL_EN), disp_rd(b, OVL_ROI_SIZE),
				    disp_rd(b, OVL_SRC_CON), disp_rd(b, OVL_DATAPATH_EXT_CON), disp_rd(b, OVL_INTEN), disp_rd(b, OVL_INTSTA),
				    disp_rd(b, OVL_ADDCON_DBG));
				for (int n = 0; n < (o ? 2 : 4); n++) {
					disp_say(&p, end, "  L%d con 0x%08x size 0x%08x off 0x%08x pitch 0x%08x/0x%x addr 0x%08x\n", n,
					    disp_rd(b, OVL_L_CON(n)), disp_rd(b, OVL_L_SRC_SIZE(n)), disp_rd(b, OVL_L_OFFSET(n)),
					    disp_rd(b, OVL_L_PITCH(n)), disp_rd(b, OVL_L_PITCH_MSB(n)), disp_rd(b, OVL_L_ADDR(n)));
				}
				for (int k = 0; o == 0 && k < 3; k++) {
					disp_say(&p, end, "  EL%d con 0x%08x size 0x%08x pitch 0x%08x addr 0x%08x\n", k,
					    disp_rd(b, OVL_EL_CON(k)), disp_rd(b, OVL_EL_SRC_SIZE(k)), disp_rd(b, OVL_EL_PITCH(k)),
					    disp_rd(b, OVL_EL_ADDR(k)));
				}
			}
			disp_say(&p, end, "larb0 ports 0-3: 0x%08x 0x%08x 0x%08x 0x%08x | larb1 ports 0-3: 0x%08x 0x%08x 0x%08x 0x%08x\n",
			    disp_rd(fDisp[2], LARB_NON_SEC_CON(0)), disp_rd(fDisp[2], LARB_NON_SEC_CON(1)),
			    disp_rd(fDisp[2], LARB_NON_SEC_CON(2)), disp_rd(fDisp[2], LARB_NON_SEC_CON(3)),
			    disp_rd(fDisp[3], LARB_NON_SEC_CON(0)), disp_rd(fDisp[3], LARB_NON_SEC_CON(1)),
			    disp_rd(fDisp[3], LARB_NON_SEC_CON(2)), disp_rd(fDisp[3], LARB_NON_SEC_CON(3)));
			if (fScanOvl < 0) {
				findLKLayer(&p, end);
			}
		}
	} else if (strncmp(args, "scanout on", 10) == 0) {
		if (fScanOvl >= 0 || findLKLayer(&p, end)) {
			fScanOn = true;
			sweet_surface_scanout_w = fLogicalW;
			sweet_surface_scanout_h = fLogicalH;
			fDirty = true;
			disp_say(&p, end, "scanout on (surfaces created from now on can be scanned out; CA allocates its "
			    "framebuffers at start: boot with vivascanout=1)\n");
		}
	} else if (strncmp(args, "scanout off", 11) == 0) {
		fScanOn = false;
		fScanLive = false;
		scanoutStop();
		fDirty = true;
		disp_say(&p, end, "scanout off: LK's buffer and the CPU copy again\n");
	} else if (strncmp(args, "lat reset", 9) == 0) {
		IOLockLock(fLock);
		bzero(&fLatReady, sizeof(fLatReady));
		bzero(&fLatFlip, sizeof(fLatFlip));
		bzero(&fLatSwap, sizeof(fLatSwap));
		bzero(&fLatRender, sizeof(fLatRender));
		bzero(fLatReadyHist, sizeof(fLatReadyHist));
		fTickPresents = 0;
		fTickLate = fTickLateSum = fTickLateMax = fTickEarly = fTickEarlySum = fTickWidePolls = 0;
		fFlips = fFlipEarly = fFlipMissed = fFlipUnsure = 0;
		IOLockUnlock(fLock);
		disp_say(&p, end, "frame latency counters reset\n");
	} else if (strncmp(args, "early on", 8) == 0) {
		fEarly = fEarlyTimer != NULL;
		disp_say(&p, end, "early present %s\n", fEarly ? "on" : "off (no timer)");
	} else if (strncmp(args, "early off", 9) == 0) {
		fEarly = false;
		disp_say(&p, end, "early present off: swaps are shown at vsync ticks only\n");
	} else if (strncmp(args, "flip on", 7) == 0) {
		fFlipAllowed = fFlipTried = true;
		if (fFlipReady || flipSetup(&p, end)) {
			fDirty = true;
			disp_say(&p, end, "double buffer on\n");
		}
	} else if (strncmp(args, "flip off", 8) == 0) {
		fFlipAllowed = false;
		fFlipReady = false;
		fFlipWhy = "disp flip off";
		dropNext();
		if (fShown.buf == 1) {
			layerApply(bufTarget(0));       /* redrawn at the next tick */
		}
		fDirty = true;
		disp_say(&p, end, "double buffer off: copies go to LK's buffer on screen\n");
	} else {
		disp_say(&p, end, "display %ux%u (shown as %ux%u), scanout %s%s, layer %s, %llu frames scanned out, "
		    "%llu copied%s%s\n", fWidth, fHeight, fLogicalW, fLogicalH, fScanOn ? "on" : "off",
		    fShown.buf < 0 ? " (active)" : "", fScanOvl >= 0 ? fScanLayer : "?", fScanFrames, fScanFallbacks,
		    fScanWhy ? ", last reason: " : "", fScanWhy ? fScanWhy : "");
		disp_say(&p, end, "copies: %llu, avg %llu us, refresh %s%u\n", fCopies, fCopies ? fCopyNs / fCopies / 1000 : 0,
		    fFlushTicks ? "every N vsyncs, N = " : "adaptive, CPU duty % = ", fFlushTicks ? fFlushTicks : fDuty);
		disp_say(&p, end, "GPU sync (K0c): %llu vsyncs waited for the GPU, %llu copies redone, %llu frames shown anyway "
		    "(sweetfb_gpuwait=%u)\n", fGpuWaits, fGpuRecopies, fGpuForced, fGpuWaitMax);
		{
			uint64_t r = absUsFB(fLatReady.n ? fLatReady.sum / fLatReady.n : 0), rm = absUsFB(fLatReady.max);
			uint64_t f = absUsFB(fLatFlip.n ? fLatFlip.sum / fLatFlip.n : 0), fm = absUsFB(fLatFlip.max);
			uint64_t s = absUsFB(fLatSwap.n ? fLatSwap.sum / fLatSwap.n : 0);
			uint64_t c = absUsFB(fLatRender.n ? fLatRender.sum / fLatRender.n : 0), cm = absUsFB(fLatRender.max);
			disp_say(&p, end, "latency (0135, us): vsync -> SwapEnd avg %llu max %llu; SwapEnd -> GPU done avg %llu max %llu "
			    "(frames <1 %llu, <2 %llu, <3 %llu, <4 %llu, more %llu); chosen -> on screen avg %llu max %llu; SwapEnd every "
			    "%llu (%llu swaps); %llu chosen at the tick before its switch (disp lat reset)\n", c, cm, r, rm,
			    fLatReadyHist[0], fLatReadyHist[1], fLatReadyHist[2], fLatReadyHist[3], fLatReadyHist[4], f, fm, s,
			    fLatSwap.n, fTickPresents);
			disp_say(&p, end, "ticks with a switch pending (0136): %llu late (avg %llu max %llu us after the frame end), "
			    "%llu early (avg %llu us before it), %llu polled up to 3 ms ahead\n", fTickLate,
			    absUsFB(fTickLate ? fTickLateSum / fTickLate : 0), absUsFB(fTickLateMax), fTickEarly,
			    absUsFB(fTickEarly ? fTickEarlySum / fTickEarly : 0), fTickWidePolls);
		}
		disp_say(&p, end, "early present (K0g, sweetfb_early) %s: %llu tries, %llu frames shown before their tick\n",
		    fEarly ? "on" : "off", fEarlyRuns, fEarlyShown);
		disp_say(&p, end, "SwapWait: %llu busy answers (CA keeps the page), %llu blocking waits; layer reads swap %u, "
		    "next swap %u\n", fSwapWaitBusy, fSwapWaitSlept, fShownSwap, fNextSwap);
		uint64_t pns = 0, polls = fPolls ? fPolls : 1, pollNs;
		absolutetime_to_nanoseconds(fPeriod, &pns);
		absolutetime_to_nanoseconds(fPollAbs / polls, &pollNs);
		disp_say(&p, end, "double buffer (K0d) %s%s%s: on screen %s%d, frame %llu us, %llu switches at a frame end, %llu "
		    "in place, %llu missed, %llu unsure, %llu deferred; poll avg %llu us\n", fFlipReady ? "on" : "off",
		    fFlipWhy ? ": " : "", fFlipWhy ? fFlipWhy : "", fShown.buf < 0 ? "surface" : "buffer ",
		    fShown.buf < 0 ? 0 : fShown.buf, pns / 1000, fFlips, fInPlace, fFlipMissed, fFlipUnsure, fFlipEarly,
		    pollNs / 1000);
		disp_say(&p, end, "usage: disp [regs|scanout on|scanout off|flip on|flip off|early on|early off|lat reset]\n");
	}
	IOLockUnlock(fScanLock);
}

extern "C" void
viva_disp_link(const char *args, char *out, size_t outLen)
{
	if (gSweetFB == NULL) {
		snprintf(out, outLen, "disp: no AppleCLCD\n");
		return;
	}
	gSweetFB->dispCommand(args, out, outLen);
}

bool
IOMobileFramebufferUserClient::initWithTask(task_t owningTask, void *securityID, UInt32 type)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type)) {
		return false;
	}
	fTask = owningTask;
	return true;
}

bool
IOMobileFramebufferUserClient::start(IOService *provider)
{
	fFB = OSDynamicCast(AppleCLCD, provider);
	if (fFB == NULL || !IOUserClient::start(provider)) {
		return false;
	}
	fFB->addClient(this);
	return true;
}

IOReturn
IOMobileFramebufferUserClient::clientClose(void)
{
	if (fFB != NULL) {
		fFB->removeClient(this);
		IOLockLock(fFB->fLock);
		mach_port_t old = fPort;
		fPort = MACH_PORT_NULL;
		IOLockUnlock(fFB->fLock);
		if (old != MACH_PORT_NULL) {
			releaseNotificationPort(old);
		}
	}
	terminate();
	return kIOReturnSuccess;
}

IOReturn
IOMobileFramebufferUserClient::registerNotificationPort(mach_port_t port, __unused UInt32 type,
    __unused io_user_reference_t refCon)
{
	mach_port_t old;
	IOLockLock(fFB->fLock);
	old = fPort;
	fPort = port;
	IOLockUnlock(fFB->fLock);
	if (old != MACH_PORT_NULL) {
		releaseNotificationPort(old);
	}
	return kIOReturnSuccess;
}

/* Called with the framebuffer lock held. */
void
IOMobileFramebufferUserClient::notify(uint32_t type, const uint64_t *values, uint32_t count)
{
	OSAsyncReference64 ref;

	if (type >= SWEET_FB_NOTIFY_TYPES || fFunc[type] == 0 || fPort == MACH_PORT_NULL) {
		return;
	}
	bzero(ref, sizeof(ref));
	setAsyncReference64(ref, fPort, (mach_vm_address_t)fFunc[type], fRefcon[type], fTask);
	sendAsyncResult64WithOptions(ref, kIOReturnSuccess, (io_user_reference_t *)values, count,
	    kIOUserNotifyOptionCanDrop);
}

static uint64_t sweet_fb_logged;        /* selectors already logged (< 64) */

IOReturn
IOMobileFramebufferUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    __unused IOExternalMethodDispatch *dispatch, __unused OSObject *target,
    __unused void *reference)
{
	IOReturn ret = kIOReturnSuccess;
	uint64_t out[2] = { 0, 0 };
	uint32_t outCount = 0;

	switch (selector) {
	case kSweetFBGetLayerDefaultSurface:
		out[0] = sweet_surface_port_in_task(fFB->fSurface, fTask);
		outCount = 1;
		break;
	case kSweetFBSwapBegin:
		IOLockLock(fFB->fLock);
		out[0] = ++fFB->fSwapID;
		IOLockUnlock(fFB->fLock);
		outCount = 1;
		break;
	case kSweetFBSwapEnd: {
		uint8_t rec[SWEET_FB_SWAP_REC_SIZE];
		if (args->structureInputDescriptor != NULL ||
		    args->structureInputSize < SWEET_FB_SWAP_REC_SIZE) {
			ret = kIOReturnBadArgument;
			break;
		}
		memcpy(rec, args->structureInput, sizeof(rec));
		fFB->swap(rec);
		break;
	}
	case kSweetFBGetID:
		out[0] = 0;
		outCount = 1;
		break;
	case kSweetFBGetDisplaySize:
		out[0] = fFB->fLogicalW;
		out[1] = fFB->fLogicalH;
		outCount = 2;
		break;
	case kSweetFBIsMainDisplay:
		out[0] = 1;
		outCount = 1;
		break;
	case kSweetFBGetDotPitch:
		out[0] = SWEET_FB_DOT_PITCH;
		outCount = 1;
		break;
	case kSweetFBNotifications:
		if (args->scalarInputCount >= 3 && args->scalarInput[2] < SWEET_FB_NOTIFY_TYPES) {
			uint32_t type = (uint32_t)args->scalarInput[2];
			IOLockLock(fFB->fLock);
			fFunc[type] = args->scalarInput[0];
			fRefcon[type] = args->scalarInput[1];
			IOLockUnlock(fFB->fLock);
			char name[32];
			proc_selfname(name, sizeof(name));
			IOLog("SWEET FB: %s notifications type %u %s\n", name, type,
			    args->scalarInput[0] ? "on" : "off");
		}
		break;
	case kSweetFBRequestPowerChange:
		if (args->scalarInputCount >= 1) {
			bool off = args->scalarInput[0] == 0;
			IOLockLock(fFB->fLock);
			if (off != fFB->fPowerOff) {
				fFB->fPowerOff = off;
				fFB->fFadeStep = 0;
				fFB->fDirty = true;     /* on: redraw the latest frame */
			}
			IOLockUnlock(fFB->fLock);
			IOLog("SWEET FB: display %s\n", off ? "off" : "on");
		}
		break;
	case kSweetFBSwapWait: {
		uint32_t swap = args->scalarInputCount > 0 ? (uint32_t)args->scalarInput[0] : 0;
		uint32_t opts = args->scalarInputCount > 1 ? (uint32_t)args->scalarInput[1] : 0;
		uint32_t ms = args->scalarInputCount > 2 ? (uint32_t)args->scalarInput[2] : 0;
		uint32_t op = opts & 7;
		if (fFB->swapDone(swap, op)) {
			break;
		}
		if (opts & 0x80000000u) {
			ret = kIOReturnBusy;
			fFB->fSwapWaitBusy++;
			break;
		}
		/* block (the calling thread only): re-check every millisecond, at most the timeout or 5 s */
		ms = (ms == 0 || ms > 5000) ? 5000 : ms;
		ret = kIOReturnTimeout;
		for (uint32_t t = 0; t < ms; t++) {
			IOSleep(1);
			if (fFB->swapDone(swap, op)) {
				ret = kIOReturnSuccess;
				break;
			}
		}
		fFB->fSwapWaitSlept++;
		break;
	}
	default:
		break;
	}

	for (uint32_t i = 0; i < args->scalarOutputCount; i++) {
		args->scalarOutput[i] = i < outCount ? out[i] : 0;
	}
	if (args->structureOutputDescriptor == NULL && args->structureOutput != NULL) {
		bzero(args->structureOutput, args->structureOutputSize);
	}
	if (selector >= 64 || !(sweet_fb_logged & (1ULL << selector))) {
		if (selector < 64) {
			sweet_fb_logged |= 1ULL << selector;
		}
		char name[32];
		proc_selfname(name, sizeof(name));
		IOLog("SWEET FB: %s sel %u in %u/%u out %u/%u -> 0x%x\n", name, selector,
		    args->scalarInputCount, args->structureInputSize,
		    args->scalarOutputCount, args->structureOutputSize, ret);
	}
	return ret;
}
#endif /* SWEET */
