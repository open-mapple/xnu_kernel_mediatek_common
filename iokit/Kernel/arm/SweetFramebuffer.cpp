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
 *   6  SwapWait, 7 GetID, 8 GetDisplaySize -> [w, h],
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
 * else falls back to the copy with LK's address and pitch restored. The OVL
 * shadow registers are bypassed on MT6781, so the switch is not tied to
 * vblank yet (a frame may tear).
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
 * The panel memory itself is published as an IOSurface and returned as the
 * default framebuffer surface. The kernel text console keeps drawing over
 * the GUI by default: boot-arg sweetfb_noconsole=1 stops drawing it from
 * the first swap on (it still reaches the persistent log; panics draw).
 */
#include "SweetSurface.h"

extern "C" int sweet_console_muted;     /* osfmk/console/serial_console.c */

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

public:
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
	bool          fConsolePaused;
	uint8_t       fPending[SWEET_FB_SWAP_REC_SIZE];     /* latest swap (redrawn when the display comes on) */
	bool          fDirty;
	uint8_t       fQueue[SWEET_FB_QUEUE][SWEET_FB_SWAP_REC_SIZE];       /* swaps not shown yet, oldest first */
	uint64_t      fQueueSeq[SWEET_FB_QUEUE];
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

	/* viva direct scanout (see the top) */
	IOLock       *fScanLock;      /* scanout state: present() vs. the link command */
	IOMemoryMap  *fDispMap[4];    /* OVL0, OVL0_2L, SMI larb0, larb1 */
	vm_offset_t   fDisp[4];
	int           fScanOvl;       /* OVL of LK's layer (0/1), -1 = not found */
	uint32_t      fScanAddrReg, fScanPitchReg, fScanPitchMsbReg;
	uint32_t      fLkAddr, fLkPitch, fLkPitchMsb;
	char          fScanLayer[16];
	bool          fScanOn;        /* allowed */
	bool          fScanActive;    /* the layer points at fScanSurface */
	SweetSurface *fScanSurface;
	uint64_t      fScanFrames, fScanFallbacks;
	const char   *fScanWhy;       /* last reason for falling back */

	bool dispMap(void);
	bool findLKLayer(char **out, char *end);
	bool scanout(const uint8_t *rec);
	bool layersIdle(const uint8_t *rec, uint64_t *jobs);
	void scanoutStop(void);
	void dispCommand(const char *args, char *out, size_t len);

	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	void addClient(IOMobileFramebufferUserClient *client);
	void removeClient(IOMobileFramebufferUserClient *client);
	void swap(const uint8_t *rec);
	bool present(const uint8_t *rec, bool force, uint64_t jobs);
	int presentQueued(const uint8_t *recs, uint32_t n);
};

OSDefineMetaClassAndStructors(AppleCLCD, IOMobileFramebuffer);

static AppleCLCD *gSweetFB;     /* for the USB link "disp" command */

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
	uint64_t now = mach_absolute_time();
	uint64_t count = ++fb->fVsyncCount;
	uint64_t values[6] = {
		(uint32_t)count, (uint32_t)(count >> 32),
		(uint32_t)now, (uint32_t)(now >> 32),
		0, 0,
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
			for (uint32_t y = 0; y < fb->fHeight; y++) {
				volatile uint32_t *row = (volatile uint32_t *)(fb->fPixels + (uint64_t)y * fb->fRowBytes);
				for (uint32_t x = 0; x < fb->fWidth; x++) {
					row[x] = step < 6 ? (row[x] >> 1) & 0x7f7f7f7f : 0;
				}
			}
		}
		sender->setTimeoutUS(SWEET_FB_VSYNC_US);
		return;
	}
	/* scanout or a frame held back for the GPU: every vsync */
	bool due = fb->fFlushTicks ? (count % fb->fFlushTicks) == 0 : now >= fb->fNextCopy;
	bool flush = fb->fDirty && (fb->fScanOn || fb->fRetry || due);
	uint8_t *recs = NULL;
	uint64_t seqs[SWEET_FB_QUEUE];
	uint32_t n = 0;
	if (flush) {
		recs = (uint8_t *)IOMallocData(SWEET_FB_QUEUE * SWEET_FB_SWAP_REC_SIZE);
		if (recs != NULL) {
			if (fb->fQueueCount != 0) {
				n = fb->fQueueCount;
				memcpy(recs, fb->fQueue, n * SWEET_FB_SWAP_REC_SIZE);
				memcpy(seqs, fb->fQueueSeq, n * sizeof(seqs[0]));
			} else {
				n = 1;                  /* redraw of the latest frame */
				memcpy(recs, fb->fPending, SWEET_FB_SWAP_REC_SIZE);
				seqs[0] = 0;
			}
			fb->fDirty = false;
		}
	}
	IOLockUnlock(fb->fLock);
	if (n != 0) {
		int shown = fb->presentQueued(recs, n);
		IOLockLock(fb->fLock);
		if (shown >= 0 && seqs[shown] != 0) {
			uint32_t keep = 0;      /* drop the shown swap and the older ones */
			for (uint32_t i = 0; i < fb->fQueueCount; i++) {
				if (fb->fQueueSeq[i] > seqs[shown]) {
					if (keep != i) {
						memcpy(fb->fQueue[keep], fb->fQueue[i], SWEET_FB_SWAP_REC_SIZE);
						fb->fQueueSeq[keep] = fb->fQueueSeq[i];
					}
					keep++;
				}
			}
			fb->fQueueCount = keep;
		}
		fb->fRetry = shown < 0;         /* held back for the GPU: try again at the next tick */
		if (shown < 0 || fb->fQueueCount != 0) {
			fb->fDirty = true;
		}
		IOLockUnlock(fb->fLock);
	}
	if (recs != NULL) {
		IOFreeData(recs, SWEET_FB_QUEUE * SWEET_FB_SWAP_REC_SIZE);
	}
	sender->setTimeoutUS(SWEET_FB_VSYNC_US);
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
		}
		fQueueCount--;
	}
	memcpy(fQueue[fQueueCount], rec, SWEET_FB_SWAP_REC_SIZE);
	fQueueSeq[fQueueCount++] = ++fSeq;
	fDirty = true;
	IOLockUnlock(fLock);
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
AppleCLCD::presentQueued(const uint8_t *recs, uint32_t n)
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
		if (fGpuWaitMax != 0 && fGpuTicks < fGpuWaitMax) {
			fGpuTicks++;
			fGpuWaits++;
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
	bool scanned = fScanOn && scanout(rec);
	IOLockUnlock(fScanLock);
	if (scanned) {
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
			OSSafeReleaseNULL(s);
			continue;
		}
		uintptr_t src = s->kernelAddress();
		sweet_surface_before_cpu_access(s);     /* the GPU may have drawn it: no stale CPU lines */
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
		if (src != 0 && convert) {
			for (int32_t y = 0; y < h; y++) {
				sweet_fb_convert_row((uint32_t *)(fPixels + (uint64_t)(dy + y) * fRowBytes + (uint64_t)dx * 4),
				    (const uint8_t *)(src + s->offset + (uint64_t)(sy + y) * s->bytesPerRow + (uint64_t)sx * bpe),
				    w, fmt);
			}
		} else if (src != 0 && bpe == 4) {
			for (int32_t y = 0; y < h; y++) {
				memcpy((void *)(fPixels + (uint64_t)(dy + y) * fRowBytes + (uint64_t)dx * 4),
				    (const void *)(src + s->offset + (uint64_t)(sy + y) * s->bytesPerRow + (uint64_t)sx * 4),
				    (size_t)w * 4);
			}
		}
		s->release();
	}
	/* the next copy no sooner than its share of the CPU (sweetfb_duty) allows */
	uint64_t t1 = mach_absolute_time(), ns;
	fNextCopy = t1 + (t1 - t0) * (100 - fDuty) / fDuty;
	absolutetime_to_nanoseconds(t1 - t0, &ns);
	fCopies++;
	fCopyNs += ns;
	/* a GPU job used a surface while it was copied (CoreAnimation reuses it): the copy may be mixed, redo it */
	if (!force && (!layersIdle(rec, &jobsAfter) || jobsAfter != jobs)) {
		fGpuTicks++;
		fGpuRecopies++;
		return false;
	}
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
		scanoutStop();
		fScanFallbacks++;
		return false;
	}
	sweet_surface_before_device_read(s);    /* CPU-drawn pixels out of the caches */
	uint32_t addr = (uint32_t)(pa + s->offset + (uint64_t)fOffY * s->bytesPerRow + (uint64_t)fOffX * 4);
	vm_offset_t base = fDisp[fScanOvl];
	disp_wr(base, fScanPitchMsbReg, (fLkPitchMsb & ~0xfu) | ((s->bytesPerRow >> 16) & 0xf));
	disp_wr(base, fScanPitchReg, (fLkPitch & ~0xffffu) | (s->bytesPerRow & 0xffff));
	disp_wr(base, fScanAddrReg, addr);
	if (!fScanActive || fScanWhy != NULL) {
		IOLog("SWEET FB: scanout: %s reads surface %u at 0x%08x (pitch %u)\n", fScanLayer, s->id, addr,
		    s->bytesPerRow);
		fScanWhy = NULL;
	}
	fScanActive = true;
	SweetSurface *old = fScanSurface;
	fScanSurface = s;                       /* keeps the memory alive while the OVL reads it */
	OSSafeReleaseNULL(old);
	fScanFrames++;
	return true;
}

/* Back to LK's buffer (the CPU copy). */
void
AppleCLCD::scanoutStop(void)
{
	if (fScanActive && fScanOvl >= 0) {
		vm_offset_t base = fDisp[fScanOvl];
		disp_wr(base, fScanAddrReg, fLkAddr);
		disp_wr(base, fScanPitchMsbReg, fLkPitchMsb);
		disp_wr(base, fScanPitchReg, fLkPitch);
	}
	fScanActive = false;
	OSSafeReleaseNULL(fScanSurface);
}

/* USB link "disp [regs|scanout on|off]". */
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
				disp_say(&p, end, "%s: en 0x%08x roi 0x%08x src_con 0x%08x ext 0x%08x\n", o ? "OVL0_2L" : "OVL0",
				    disp_rd(b, OVL_EN), disp_rd(b, OVL_ROI_SIZE), disp_rd(b, OVL_SRC_CON), disp_rd(b, OVL_DATAPATH_EXT_CON));
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
		scanoutStop();
		fDirty = true;
		disp_say(&p, end, "scanout off: LK's buffer and the CPU copy again\n");
	} else {
		disp_say(&p, end, "display %ux%u (shown as %ux%u), scanout %s%s, layer %s, %llu frames scanned out, "
		    "%llu copied%s%s\n", fWidth, fHeight, fLogicalW, fLogicalH, fScanOn ? "on" : "off",
		    fScanActive ? " (active)" : "", fScanOvl >= 0 ? fScanLayer : "?", fScanFrames, fScanFallbacks,
		    fScanWhy ? ", last reason: " : "", fScanWhy ? fScanWhy : "");
		disp_say(&p, end, "copies: %llu, avg %llu us, refresh %s%u\n", fCopies, fCopies ? fCopyNs / fCopies / 1000 : 0,
		    fFlushTicks ? "every N vsyncs, N = " : "adaptive, CPU duty % = ", fFlushTicks ? fFlushTicks : fDuty);
		disp_say(&p, end, "GPU sync (K0c): %llu vsyncs waited for the GPU, %llu copies redone, %llu frames shown anyway "
		    "(sweetfb_gpuwait=%u)\n", fGpuWaits, fGpuRecopies, fGpuForced, fGpuWaitMax);
		disp_say(&p, end, "usage: disp [regs|scanout on|scanout off]\n");
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
	case kSweetFBSwapWait:
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
