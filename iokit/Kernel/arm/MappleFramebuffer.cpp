/*
 * Built-in IOMobileFramebuffer for the ABL's continuous-splash
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
 *      the panel itself stays powered. spaced's panel is an LCD, so once
 *      black the backlight goes to level 0 too, and back to iOS's level
 *      at on (mp_fb_bloff, 1 default; 0 leaves the backlight alone).
 *   18 IsMainDisplay,
 *   28 GetDotPitch, 72 Enable/DisableNotifications [func, refcon, type, 0]
 * Everything else is accepted and logged once.
 *
 * SwapEnd copies the enabled layers' surfaces into the panel memory (no
 * scaling, CPU copy). Vsync (notification type 5) is a 60 Hz timer that
 * sends (count, mach_absolute_time, 0) as 32-bit halves to clients that
 * enabled it, through the port they registered (IOConnectSetNotificationPort).
 *
 * direct scanout (boot-arg mp_fb_scanout=1, or "disp scanout on" on the
 * USB link): LK leaves the MediaTek display pipe running in video mode, one
 * OVL layer reading the panel memory. When a swap shows a single full-screen
 * BGRA surface that is physically contiguous below 4 GiB (MappleSurface
 * allocates display-sized surfaces that way when scanout is on), only that
 * layer's address and pitch are changed to point at the surface (the
 * centered 1080x2400 window of the 1125x2436 surface): no CPU copy. Anything
 * else falls back to the copy. The OVL shadow registers are bypassed on
 * MT6781: a layer register write takes effect at once, so it is made at a
 * frame end (below).
 *
 * double buffer (K0d, boot-arg mp_fb_flip, 1 default): the copy goes
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
 * LK's buffer, where the console draws (mapple_fb_panic_hook).
 *
 * GPU-drawn frames (K0c): backboardd hands a surface to SwapEnd right after
 * committing its command buffer, without waiting for the GPU (on Apple the
 * display driver waits). Swaps are therefore queued in order (the last
 * MAPPLE_FB_QUEUE) and a tick shows the newest one whose surfaces have no GPU
 * job pending (mapple_gpu_surface_idle), dropping it and the older ones. If
 * none is ready nothing is shown and the next vsync tick tries again (no
 * waiting in the timer). A copy during which a new job used one of the
 * surfaces is redone at the next tick. After mp_fb_gpuwait ticks (default
 * 12, 200 ms) the newest frame is shown anyway; mp_fb_gpuwait=0 turns the
 * wait off.
 *
 * Early present (K0g, boot-arg mp_fb_early, 1 default; "disp early on|off"):
 * the same is also tried when a swap arrives and whenever a GPU job that used
 * IOSurfaces completes, on this work loop, so a finished frame is copied off
 * screen at once and switched at the next frame end instead of one tick later.
 *
 * Panel refresh (0189, boot-arg mp_panel_hz=60|120|1, link "disp hz 60|120|auto"): the panel (Xiaomi
 * dsi_k6t_38_0c_0a_dsc_vdo, DSC video mode, 369.469 MHz pixel clock) runs at 60 Hz with VFP 2452 lines and at
 * 120 Hz with VFP 20, same clocks. The switch is the vendor's LCM_SEND_IN_VDO / DYNFPS_DSI_VFP path: four DCS
 * packets sent in the video-mode blanking through DSI VM_CMD (F0 5A 5A, 60 01 for 120 Hz / 60 21 for 60 Hz, F7 0B,
 * F0 A5 A5), two frames, then DSI_VFP_NL. The vsync tick then follows the panel (CoreAnimation gets 120 Hz with its
 * period), the frame end is calibrated again. Auto (1): 120 Hz from the first swap, 60 Hz after 500 ms without one
 * (mp_panel_idle=ms) or with the display off, as ProMotion does; the switch is locked to frame ends. Default 60. With 120 or auto at boot the service also publishes
 * IOMFBDisplayRefresh (min refresh interval 8.333 ms), which QuartzCore reads when backboardd starts.
 *
 * Brightness and Night Shift (0191). CoreBrightness sets IOMFBBrightnessLevel on this service (CFNumber, nits in 16.16
 * fixed point; SetIOMFBBrightness in CoreBrightness, iOS 16.7): the panel takes DCS 51 HH LL, 0..2047 for its
 * normal range (Xiaomi lcm_setbacklight_cmdq), sent through VM_CMD at the next vsync tick (fades coalesce);
 * mp_panel_nits (500) = the nits of level 2047; link "disp bl N". QuartzCore's color matrices arrive through
 * selector 55 (IOMobileFramebufferSetMatrix: scalar = which matrix, 9 x int64 signed 32.32, row-major); their product goes to the
 * display pipe's CCORR (0x1400b000, Xiaomi mtk_disp_ccorr.c: 3x3, 1.0 = 0x800 on MT6781, 13-bit signed, two per
 * register; CFG bit 0 relay / bit 1 engine). Identity = relay (bypass).
 *
 * Resizer (device.conf's resize through the device tree, or boot-arg mp_fb_rsz: 1 = read only, 2 = on; link
 * "disp rsz [on|off]"): the letterbox's scaling done by the display pipe's RSZ0 for frames that are scanned out (see
 * its section).
 *
 * The panel memory itself is published as an IOSurface and returned as the
 * default framebuffer surface. The kernel text console keeps drawing over
 * the GUI by default: boot-arg mp_fb_noconsole=1 stops drawing it from
 * the first swap on (it still reaches the persistent log; panics draw).
 */
#include "MappleSurface.h"

extern "C" int mapple_console_muted;     /* osfmk/console/serial_console.c */
extern "C" void (*mapple_fb_panic_hook)(void);   /* pexpert/arm/pe_serial.c */
extern "C" void mt6781_cpufreq_ui_activity(void);        /* osfmk/arm64/mt6781_cpufreq.c: 0183 UI boost */

#if defined(MT6781)
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

#include "MappleLetterbox.h"

enum {
	kMappleFBGetLayerDefaultSurface  = 3,
	kMappleFBSwapBegin               = 4,
	kMappleFBSwapEnd                 = 5,
	kMappleFBSwapWait                = 6,
	kMappleFBGetID                   = 7,
	kMappleFBGetDisplaySize          = 8,
	kMappleFBRequestPowerChange      = 12,
	kMappleFBIsMainDisplay           = 18,
	kMappleFBGetDotPitch             = 28,
	kMappleFBNotifications           = 72,
};

#define MAPPLE_FB_SWAP_REC_SIZE          0x46c
#define MAPPLE_FB_LAYERS                 4
#define MAPPLE_FB_QUEUE                  4       /* swaps waiting for the GPU (K0c) */
#define MAPPLE_FB_LK_BUFFERS             3       /* panel buffers LK reserves (K0d uses one besides its own) */
#define MAPPLE_FB_NOTIFY_TYPES           8
#define MAPPLE_FB_NOTIFY_VSYNC           5
#define MAPPLE_FB_VSYNC_US               16667
#define MAPPLE_FB_DOT_PITCH              458     /* ppi, as the iPhone X panel */
/*
 * The iOS userland only has status bar / layout data for real iPhone
 * screens (UIKit: "No status bar visual provider for screen", the red
 * rdar://45025538 bar for 360x800 points). The display is therefore
 * presented as the iPhone X panel, 1125x2436 (375x812 points @3x), and the
 * centered 1080x2400 part of it is shown on the 1080x2400 panel.
 * Boot-arg mp_fb_native=1 presents the real panel size instead; mp_fb_letterbox
 * scales the whole of it onto the panel under a black strip (MappleLetterbox.h).
 */
#define MAPPLE_FB_LOGICAL_W              1125
#define MAPPLE_FB_LOGICAL_H              2436
#define MAPPLE_FB_FORMAT_BGRA            0x42475241
/*
 * Extended-range formats CoreAnimation composes in on a wide-colour display (Metal BGR10_XR / BGRA10_XR):
 * 'w30r' 32-bit, B/G/R in bits 0-9/10-19/20-29; 'w40a'/'w40m' 64-bit, B/G/R/A in 16-bit words, 10 bits in the
 * top of each; 'l10r' 32-bit ARGB2101010, full range. XR value v means (v - 384) / 510: converted to 8-bit
 * BGRA (clamped to [0, 1], i.e. sRGB) while copying to the panel.
 */
#define MAPPLE_FB_FORMAT_W30R            0x77333072
#define MAPPLE_FB_FORMAT_W40A            0x77343061
#define MAPPLE_FB_FORMAT_W40M            0x7734306d
#define MAPPLE_FB_FORMAT_L10R            0x6c313072

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
	struct mapple_lb fLb;                   /* mp_fb_letterbox: the picture on the panel, w 0 = off */
	uint32_t     *fLbCol, *fLbRow;          /* logical position each picture column / row samples, 16.16 */
	uint64_t     *fLbLine;                  /* a logical row blended vertically (mapple_lb_spread lanes) */
	uint32_t     *fLbConv[2];               /* converted source rows of extended-range formats */
	void lbBlit(uintptr_t dst, MappleSurface *s, uintptr_t src, int32_t sx, int32_t sy, int32_t dx, int32_t dy,
	    int32_t w, int32_t h, uint32_t bpe, uint32_t fmt, bool convert);
	void lbBars(uintptr_t dst);
	uintptr_t     fPixels;          /* kernel mapping of the panel memory */
	IOMemoryMap  *fMap;
	MappleSurface *fSurface;         /* the panel memory as an IOSurface */
	IOLock       *fLock;            /* swaps and client list */
	OSArray      *fClients;
	uint32_t      fSwapID;
	uint64_t      fSwapWaitBusy, fSwapWaitSlept;      /* SwapWait answered busy / had to block */
	uint32_t      fPoolWant;                          /* 0184: mp_fb_pool */
	/*
	 * 0184 (P3 of gpu/docs/ANALISI-PRESTAZIONI-2026-11-06.md): why SwapWait was not done, by what held the swap
	 * when it was asked (WAIT_QUEUED: waiting for the GPU or a copy; WAIT_NEXT: chosen, switch at the next frame
	 * end; WAIT_SHOWN: the layer reads its page; WAIT_OTHER), for busy answers and blocking waits; and how long
	 * from the first "not done" until done (busy: the next SwapWait that said done; blocking: the wait itself).
	 */
	enum { WAIT_QUEUED, WAIT_NEXT, WAIT_SHOWN, WAIT_OTHER, WAIT_WHY };
	uint64_t      fWaitBusyWhy[WAIT_WHY], fWaitSleptWhy[WAIT_WHY];
	uint32_t      fBusySwap, fBusyWhy;            /* swap of the last busy answer, 0 = none open */
	uint64_t      fBusyAt;
	uint32_t      waitWhy(uint32_t swap);
	void          waitResult(uint32_t swap, bool done);
	bool          fConsolePaused;
	uint8_t       fPending[MAPPLE_FB_SWAP_REC_SIZE];     /* latest swap (redrawn when the display comes on) */
	bool          fDirty;
	uint8_t       fQueue[MAPPLE_FB_QUEUE][MAPPLE_FB_SWAP_REC_SIZE];       /* swaps not shown yet, oldest first */
	uint64_t      fQueueSeq[MAPPLE_FB_QUEUE];
	uint64_t      fQueueAt[MAPPLE_FB_QUEUE];       /* 0135: mach time of each queued swap's SwapEnd */
	/*
	 * 0135 frame latency ("disp"): SwapEnd -> chosen (GPU done, K0c), chosen -> on screen (frame end), SwapEnd to
	 * SwapEnd, last vsync notification -> SwapEnd (CoreAnimation's render time); sums, counts and maxima in mach time.
	 */
	struct LatStat { uint64_t sum, n, max; } fLatReady, fLatFlip, fLatSwap, fLatRender;
	LatStat       fLatWait[4];                    /* 0184: first "not done" -> done, per WAIT_* reason */
	uint64_t      fLatReadyHist[5];               /* SwapEnd -> chosen in frames: <1, <2, <3, <4, more */
	uint64_t      fSwapGapHist[5];                /* SwapEnd to SwapEnd: <= 17, 34, 50, 100 ms, more */
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
	uint32_t      fFlushTicks;    /* vsync ticks between panel copies (mp_fb_hz) */
	uint32_t      fDuty;          /* no mp_fb_hz: copies take at most this % of one CPU (mp_fb_duty) */
	uint64_t      fNextCopy;      /* absolute time the next copy may start (fDuty) */
	uint64_t      fCopies, fCopyNs;
	bool          fPowerOff;      /* RequestPowerChange(0) */
	uint32_t      fFadeStep;      /* fade-out steps done while off */
	/* K0c: frames held back for the GPU */
	bool          fRetry;         /* the pending frame waits for the GPU: try again at the next tick */
	uint32_t      fGpuTicks;      /* consecutive ticks the pending frame waited */
	uint32_t      fGpuWaitMax;    /* mp_fb_gpuwait: ticks before showing it anyway */
	uint64_t      fGpuWaits, fGpuRecopies, fGpuForced;
	/* K0g early present (mp_fb_early, 1 default): a swap is tried when it arrives and when GPU jobs complete */
	IOTimerEventSource *fEarlyTimer;
	bool          fEarly;
	volatile UInt32 fEarlyPending;  /* fEarlyTimer armed */
	uint64_t      fEarlyRuns, fEarlyShown;

	/* direct scanout (see the top) */
	IOLock       *fScanLock;      /* scanout state: present() vs. the link command */
	IOMemoryMap  *fDispMap[10];   /* OVL0, OVL0_2L, SMI larb0, larb1, DSI0 (0189), CCORR0 (0191); mp_fb_rsz: MMSYS, MUTEX, RDMA0, RSZ0 */
	vm_offset_t   fDisp[10];
	/* mp_fb_rsz (see the resizer section) */
	uint32_t      fRszArg;
	struct mapple_rsz fRsz;
	bool          fRszWant;       /* "disp rsz on|off" */
	bool          fRszReady;      /* rszPrepare done: clock on, RSZ0 configured, the pipe writes listed */
	bool          fRszLive;       /* the pipe runs OVL0 -> RSZ0 -> RDMA0 now */
	bool          fRszDumped;
	const char   *fRszWhy;        /* why the resizer is off for good, NULL = not failed */
	uint32_t      fRszBad;        /* the write that failed: block << 16 | offset */
	uint32_t      fRszN;          /* pipe writes, in switch-on order */
	uint8_t       fRszBlk[10];
	uint16_t      fRszOff[10];
	uint32_t      fRszMask[10], fRszOn[10], fRszSave[10];
	uint64_t      fRszSwitches;
	bool rszWrite(int blk, uint32_t off, uint32_t mask, uint32_t value);
	bool rszPrepare(void);
	void rszDump(char **out, char *end);
	void rszLog(void);
	bool rszPipe(bool on);
	/* 0189 panel refresh (see the top) */
	uint32_t      fPanelHz;       /* the panel's refresh now: 60 or 120 */
	uint32_t      fHzMode;        /* 60, 120, 1 = auto */
	uint32_t      fTickUs;        /* vsync tick target: 16667 (60 Hz) or 8333 (120 Hz) */
	volatile uint32_t fHzWant;    /* a switch to this refresh is pending (done at the next vsync tick), 0 = none */
	uint64_t      fHzSwitches, fHzFailures;
	const char   *fHzWhy;         /* last switch failure */
	volatile uint64_t fLastActivity;      /* mach time of the last swap (auto mode) */
	uint32_t      fHzIdleMs;      /* mp_panel_idle: auto mode back to 60 Hz after this long without a swap */
	uint32_t      fHzSeq;         /* mp_panel_seq / "disp hz seq N": where the porch goes relative to the commands */
	bool panelSetHz(uint32_t hz);
	void hzPolicy(void);
	/* 0191 brightness and color matrix (see the top) */
	IOMemoryMap  *fCcorrMap;
	vm_offset_t   fCcorr;
	uint32_t      fMaxNits;       /* mp_panel_nits: nits at level 2047 */
	volatile uint32_t fBlWant;    /* panel level to send at the next tick, 0xffffffff = none */
	uint32_t      fBlLevel;       /* last level sent */
	uint32_t      fBlOff;         /* mp_fb_bloff: backlight off while the display is */
	uint32_t      fBlResume;      /* level for when the display is on again */
	uint64_t      fBlSent, fMatrixCalls, fMatrixApplied;
	int32_t       fMatrix[16][9]; /* per matrix type (Night Shift = 9), fixed point 1.0 = 0x800 */
	bool          fMatrixUsed[16];
	volatile bool fMatrixDirty;
	volatile bool fSelTrace;      /* "disp seltrace on": log every user client call and swap record changes */
	uint8_t       fTraceRec[MAPPLE_FB_SWAP_REC_SIZE];
	uint32_t      fTraceLines;
	bool dsiSendDcs(const uint8_t *b, uint32_t n, bool vbp, bool wait = true);
	bool          fDcsInFlight;   /* a non-blocking DCS write was started (brightness) */
	class AppleARMBacklight *fBacklight;
	void backlightStart(void);
	void traceProperties(const char *who, OSDictionary *dict);
	void lightTick(void);
	void setMatrix(uint32_t type, const uint64_t bits[9]);
	IOReturn setProperties(OSObject *properties) APPLE_KEXT_OVERRIDE;
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
		MappleSurface *surface;        /* retained while the layer may read it */
		uint32_t      swap;           /* the swap whose surface the layer reads (scanout), 0 = a panel buffer */
		bool          rsz;            /* the whole surface, through the resizer (mp_fb_rsz) */
	};
	volatile uint32_t fShownSwap, fNextSwap;      /* fShown.swap / fNext.swap (valid), read without fScanLock */
	Target        fShown, fNext;
	bool          fNextValid;
	/* K0d double buffer and frame end (see the top) */
	uintptr_t     fBufKVA[2];     /* 0 = LK's buffer (fPixels), 1 = another one in LK's memory (flipSetup) */
	uint64_t      fBufPA[2];
	IOMemoryMap  *fBuf1Map;
	bool          fFlipAllowed;   /* mp_fb_flip */
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
	bool          fOneLayer;      /* mp_fb_onelayer */
	uint32_t      fOneOff[2];     /* per OVL: the layers oneLayer() switched off */
	uint32_t      fOneLkPitch;    /* LK's layer pitch before oneLayer() set it to fRowBytes (0: not changed) */

	bool dispMap(void);
	bool findLKLayer(char **out, char *end);
	bool layerReads(uint64_t lo, uint64_t hi);
	void oneLayer(bool on, char **out, char *end);
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

static AppleCLCD *gMappleFB;     /* for the USB link "disp" command */

/*
 * 0191: CoreBrightness (CBDisplayContaineriOS findBacklight, iOS 16.7) matches IOPropertyMatch {backlight-control: true}
 * and keeps the service that conforms to AppleARMBacklight; DisplayCreateWithID reads IODisplayParameters.brightness
 * {min, max, value}. Without it there is no display module: no brightness, no Night Shift. Brightness writes come back
 * through setProperties (logged with "disp seltrace on") and go to the panel through AppleCLCD.
 */
class AppleARMBacklight : public IOService {
	OSDeclareDefaultStructors(AppleARMBacklight);
public:
	AppleCLCD *fFB;
	IOReturn setProperties(OSObject *properties) APPLE_KEXT_OVERRIDE;
	void publishLevel(uint32_t level);
};
OSDefineMetaClassAndStructors(AppleARMBacklight, IOService);
volatile uint32_t mapple_fb_gpu_waits;
static volatile uint32_t gMappleFbDutyCap;       /* mapple_fb_set_duty_cap (Low Power Mode); 0 = mp_fb_duty */

/* Low Power Mode (BQ27Z561Battery.cpp): panel copies take at most `pct` % of one CPU (0 = mp_fb_duty again). */
extern "C" void
mapple_fb_set_duty_cap(uint32_t pct)
{
	gMappleFbDutyCap = pct > 100 ? 100 : pct;
}

class IOMobileFramebufferUserClient : public IOUserClient {
	OSDeclareDefaultStructors(IOMobileFramebufferUserClient);

public:
	AppleCLCD          *fFB;
	task_t              fTask;
	mach_port_t         fPort;
	io_user_reference_t fFunc[MAPPLE_FB_NOTIFY_TYPES];
	io_user_reference_t fRefcon[MAPPLE_FB_NOTIFY_TYPES];

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
		IOLog("FB: no boot framebuffer\n");
		return false;
	}
	int native = 0;
	PE_parse_boot_argn_alias("mp_fb_native", "sweetfb_native", &native, sizeof(native));
	fLogicalW = fWidth;
	fLogicalH = fHeight;
	if (!native && fWidth <= MAPPLE_FB_LOGICAL_W && fHeight <= MAPPLE_FB_LOGICAL_H) {
		fLogicalW = MAPPLE_FB_LOGICAL_W;
		fLogicalH = MAPPLE_FB_LOGICAL_H;
	}
	fOffX = (int32_t)(fLogicalW - fWidth) / 2;
	fOffY = (int32_t)(fLogicalH - fHeight) / 2;
	uint32_t lb = 0;
	PE_parse_boot_argn("mp_fb_letterbox", &lb, sizeof(lb));
	fRszArg = mapple_rsz_mode();
	if (lb == 0 && fRszArg >= 2) {
		lb = 1;                 /* frames the resizer cannot take are scaled on the CPU to the same picture */
	}
	if (lb != 0 && fLogicalW == MAPPLE_FB_LOGICAL_W) {
		fLb = mapple_lb_rect((int32_t)fWidth, (int32_t)fHeight, (int32_t)fLogicalW, (int32_t)fLogicalH, lb);
		fRsz = mapple_rsz_config((int32_t)fWidth, (int32_t)fHeight, (int32_t)fLogicalW, (int32_t)fLogicalH, fLb);
		fRszWant = fRszArg >= 2;
		void *m = IOMallocData((size_t)fLogicalW * 16 + (size_t)(fLb.w + fLb.h) * 4);
		if (m == NULL) {
			fLb.w = 0;
		} else {
			fLbLine = (uint64_t *)m;
			fLbConv[0] = (uint32_t *)(fLbLine + fLogicalW);
			fLbConv[1] = fLbConv[0] + fLogicalW;
			fLbCol = fLbConv[1] + fLogicalW;
			fLbRow = fLbCol + fLb.w;
			for (int32_t i = 0; i < fLb.w; i++) {
				fLbCol[i] = mapple_lb_src(i, fLb.w, (int32_t)fLogicalW);
			}
			for (int32_t j = 0; j < fLb.h; j++) {
				fLbRow[j] = mapple_lb_src(j, fLb.h, (int32_t)fLogicalH);
			}
			IOLog("FB: letterbox (mp_fb_letterbox=%u): picture %dx%d at %d,%d, black strip %d rows\n", lb,
			    fLb.w, fLb.h, fLb.x, fLb.y, fLb.y);
		}
	}
	fLock = IOLockAlloc();
	fClients = OSArray::withCapacity(4);
	fSurface = mapple_surface_create_physical(fPhys, fWidth, fHeight, fRowBytes, MAPPLE_FB_FORMAT_BGRA);
	fPixels = fSurface != NULL ? fSurface->kernelAddress() : 0;
	fWorkLoop = IOWorkLoop::workLoop();
	if (fLock == NULL || fClients == NULL || fPixels == 0 || fWorkLoop == NULL) {
		IOLog("FB: setup failed\n");
		OSSafeReleaseNULL(fSurface);    /* stays published: also held by the table */
		OSSafeReleaseNULL(fWorkLoop);
		return false;
	}
	fVsync = IOTimerEventSource::timerEventSource(this, &AppleCLCD::vsyncFired);
	if (fVsync == NULL || fWorkLoop->addEventSource(fVsync) != kIOReturnSuccess) {
		IOLog("FB: no vsync timer\n");
		return false;
	}
	int early = 1;
	PE_parse_boot_argn_alias("mp_fb_early", "sweetfb_early", &early, sizeof(early));
	fEarlyTimer = IOTimerEventSource::timerEventSource(this, &AppleCLCD::earlyFired);
	if (fEarlyTimer != NULL && fWorkLoop->addEventSource(fEarlyTimer) != kIOReturnSuccess) {
		OSSafeReleaseNULL(fEarlyTimer);
	}
	fEarly = early != 0 && fEarlyTimer != NULL;
	int one = fRszArg >= 2;         /* the resizer needs it */
	PE_parse_boot_argn_alias("mp_fb_onelayer", "sweetfb_onelayer", &one, sizeof(one));
	fOneLayer = one != 0;
	uint32_t hz = 10;
	fFlushTicks = 0;
	if (PE_parse_boot_argn_alias("mp_fb_hz", "sweetfb_hz", &hz, sizeof(hz))) {
		fFlushTicks = (hz == 0 || hz >= 60) ? 1 : 60 / hz;
	}
	fDuty = 25;
	PE_parse_boot_argn_alias("mp_fb_duty", "sweetfb_duty", &fDuty, sizeof(fDuty));
	fDuty = MIN(MAX(fDuty, 1u), 100u);
	fGpuWaitMax = 12;
	PE_parse_boot_argn_alias("mp_fb_gpuwait", "sweetfb_gpuwait", &fGpuWaitMax, sizeof(fGpuWaitMax));
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
	PE_parse_boot_argn_alias("mp_fb_flip", "sweetfb_flip", &flip, sizeof(flip));
	fFlipAllowed = flip != 0;
	fPanelHz = 60;
	fTickUs = MAPPLE_FB_VSYNC_US;
	fHzMode = 60;
	PE_parse_boot_argn_alias("mp_panel_hz", "vivapanelhz", &fHzMode, sizeof(fHzMode));
	if (fHzMode != 60 && fHzMode != 120 && fHzMode != 1) {
		fHzMode = 60;
	}
	if (fRszWant) {
		fHzMode = 60;           /* mp_fb_rsz switches the pipe in the 60 Hz blanking (8 ms; 120 Hz has 70 us) */
	}
	fHzWant = fHzMode == 120 ? 120 : 0;     /* done at the first vsync tick with frame ends known */
	fMaxNits = 500;
	PE_parse_boot_argn_alias("mp_panel_nits", "vivapanelnits", &fMaxNits, sizeof(fMaxNits));
	fMaxNits = MIN(MAX(fMaxNits, 50u), 2000u);
	fBlWant = 0xffffffffu;
	fBlOff = 1;
	PE_parse_boot_argn("mp_fb_bloff", &fBlOff, sizeof(fBlOff));
	fHzIdleMs = 500;
	PE_parse_boot_argn_alias("mp_panel_idle", "vivapanelidle", &fHzIdleMs, sizeof(fHzIdleMs));
	fHzIdleMs = MIN(MAX(fHzIdleMs, 100u), 60000u);
	PE_parse_boot_argn_alias("mp_panel_seq", "vivapanelseq", &fHzSeq, sizeof(fHzSeq));
	fHzSeq = MIN(fHzSeq, 2u);
	uint32_t us = 1000;
	PE_parse_boot_argn_alias("mp_fb_lead", "sweetfb_lead", &us, sizeof(us));        /* tick this long before the frame end */
	nanoseconds_to_absolutetime((uint64_t)MIN(MAX(us, 200u), 4000u) * 1000, &fLead);
	nanoseconds_to_absolutetime(20 * 1000, &fSlack);
	uint32_t scan = 0;
	if ((PE_parse_boot_argn_alias("mp_fb_scanout", "vivascanout", &scan, sizeof(scan)) && scan != 0) || fRszWant) {
		/* display-sized surfaces from now on contiguous; LK's layer is looked up at the first swap */
		mapple_surface_scanout_w = fLogicalW;
		mapple_surface_scanout_h = fLogicalH;
		fScanOn = true;
		/* 0184: CoreAnimation's pages from a pool taken now, before memory fragments (mp_fb_pool, 0 = none) */
		fPoolWant = 4;
		PE_parse_boot_argn_alias("mp_fb_pool", "vivafbpool", &fPoolWant, sizeof(fPoolWant));
		if (fPoolWant != 0) {
			mapple_surface_pool_init(fLogicalW, fLogicalH, fPoolWant);
		}
	}
	gMappleFB = this;
	fVsync->setTimeoutUS(MAPPLE_FB_VSYNC_US);

	setProperty(kIOUserClientClassKey, "IOMobileFramebufferUserClient");
	setProperty("external", false);
	setProperty("DisplayPipePlaneBaseAlignment", 64ULL, 32);
	setProperty("IOMFBBrightnessLevel", (unsigned long long)(200u << 16), 32);     /* 0191: 200 nits until set */
	if (fHzMode != 60) {
		/*
		 * 0189: QuartzCore (CA::WindowServer::AppleDisplay, iOS 16.7) reads IOMFBDisplayRefresh at start:
		 * displayMinRefreshInterval (int, seconds in 0.32 fixed point) gives its heartbeat when nothing else does
		 * (AppleCLCDDisplay::heartbeat_rate; CA_HEARTBEAT_RATE overrides it), displayRefreshStepMachTime (int, mach
		 * time) only on devices with APT. 120 Hz: 8.333 ms.
		 */
		OSDictionary *refresh = OSDictionary::withCapacity(2);
		uint64_t step = 0;
		nanoseconds_to_absolutetime(8333333, &step);
		OSNumber *minI = OSNumber::withNumber((unsigned long long)((8333333ull << 32) / 1000000000ull), 32);
		OSNumber *stepN = OSNumber::withNumber((unsigned long long)step, 32);
		if (refresh != NULL && minI != NULL && stepN != NULL) {
			refresh->setObject("displayMinRefreshInterval", minI);
			refresh->setObject("displayRefreshStepMachTime", stepN);
			setProperty("IOMFBDisplayRefresh", refresh);
		}
		OSSafeReleaseNULL(minI);
		OSSafeReleaseNULL(stepN);
		OSSafeReleaseNULL(refresh);
	}
	registerService();
	backlightStart();               /* 0191 */
	IOLog("FB: AppleCLCD %ux%u stride %u at 0x%llx, default surface %u, presented as %ux%u\n",
	    fWidth, fHeight, fRowBytes, fPhys, fSurface->id, fLogicalW, fLogicalH);
	return true;
}

void
AppleCLCD::vsyncFired(OSObject *owner, IOTimerEventSource *sender)
{
	AppleCLCD *fb = (AppleCLCD *)owner;
	fb->hzPolicy();                 /* 0189: a pending refresh switch happens here, on the work loop */
	fb->lightTick();                /* 0191: brightness / color matrix changes */
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
	/* args 4/5: the refresh period (QuartzCore predicts the next vblank as time + period): one tick, 0189 */
	uint64_t period = fb->fPeriod * (fb->fFlipReady ? fb->fTickFrames : 1);
	if (period == 0) {
		nanoseconds_to_absolutetime((uint64_t)fb->fTickUs * 1000, &period);
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
		client->notify(MAPPLE_FB_NOTIFY_VSYNC, values, 6);
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
 * frames), the CPU duty limit (mp_fb_duty), nothing while the display is off, never with a fixed mp_fb_hz.
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
	uint64_t seqs[MAPPLE_FB_QUEUE], ats[MAPPLE_FB_QUEUE];
	uint32_t n = 0;
	if (flush) {
		recs = (uint8_t *)IOMallocData(MAPPLE_FB_QUEUE * MAPPLE_FB_SWAP_REC_SIZE);
		if (recs != NULL) {
			if (fQueueCount != 0) {
				n = fQueueCount;
				memcpy(recs, fQueue, n * MAPPLE_FB_SWAP_REC_SIZE);
				memcpy(seqs, fQueueSeq, n * sizeof(seqs[0]));
				memcpy(ats, fQueueAt, n * sizeof(ats[0]));
			} else {
				n = 1;                  /* redraw of the latest frame */
				memcpy(recs, fPending, MAPPLE_FB_SWAP_REC_SIZE);
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
						memcpy(fQueue[keep], fQueue[i], MAPPLE_FB_SWAP_REC_SIZE);
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
		IOFreeData(recs, MAPPLE_FB_QUEUE * MAPPLE_FB_SWAP_REC_SIZE);
	}
}

void
mapple_fb_gpu_done(void)
{
	if (gMappleFB != NULL) {
		gMappleFB->earlyKick();
	}
}

/* Next vsync tick: fLead before a predicted frame end, about 60 Hz (without frame ends: every 16.7 ms). */
void
AppleCLCD::rearm(IOTimerEventSource *timer)
{
	if (!fFlipReady) {
		timer->setTimeoutUS(fTickUs);
		return;
	}
	uint64_t now = mach_absolute_time(), step = fPeriod * fTickFrames, next = fLastVblank + step;
	uint64_t min = 0;
	nanoseconds_to_absolutetime(300 * 1000, &min);
	if (next < now + fLead + min) {
		next += ((now + fLead + min - next) / step + 1) * step;
	}
	/*
	 * 0136/0139: no coalescing. The thread call otherwise gets the default timer coalescing slop (up to ~1 ms for a
	 * deadline 16 ms away), and a tick more than fLead late lands after the frame end: the pending switch then waits
	 * a whole frame. A zero leeway does not help (the slop is MAX(slop, leeway); 0136 on the phone: ticks avg 35 us
	 * after the frame end, ~2.4 deferred per switch, chosen -> on screen 38 ms); critical urgency has no slop.
	 */
	timer->wakeAtTime(kIOTimeOptionsSysCritical, next - fLead, 0);
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

static uint32_t mapple_fb_swaps_logged;
static uint64_t mapple_fb_swap_count;
static uint32_t mapple_fb_layer_format[MAPPLE_FB_LAYERS];        /* last fourcc seen per layer, for the log */
static uint8_t  mapple_fb_xr8[1024];                             /* XR 10-bit -> 8-bit */

static void
mapple_fb_fourcc(uint32_t f, char out[5])
{
	for (int i = 0; i < 4; i++) {
		char c = (char)(f >> (24 - 8 * i));
		out[i] = (c >= 0x20 && c < 0x7f) ? c : '?';
	}
	out[4] = 0;
}

/* One row of an extended-range (or 10-bit) source to 8-bit BGRA. */
static void
mapple_fb_convert_row(uint32_t *dst, const uint8_t *src, int32_t w, uint32_t fmt)
{
	if (mapple_fb_xr8[1023] == 0) {
		for (uint32_t v = 0; v < 1024; v++) {
			uint32_t x = v < 384 ? 0 : v - 384 > 510 ? 510 : v - 384;
			mapple_fb_xr8[v] = (uint8_t)((x * 255 + 255) / 510);
		}
	}
	if (fmt == MAPPLE_FB_FORMAT_W40A || fmt == MAPPLE_FB_FORMAT_W40M) {
		const uint16_t *p = (const uint16_t *)src;
		for (int32_t x = 0; x < w; x++, p += 4) {
			dst[x] = 0xff000000u | (uint32_t)mapple_fb_xr8[p[2] >> 6] << 16 | (uint32_t)mapple_fb_xr8[p[1] >> 6] << 8 |
			    mapple_fb_xr8[p[0] >> 6];
		}
		return;
	}
	const uint32_t *p = (const uint32_t *)src;
	bool xr = fmt == MAPPLE_FB_FORMAT_W30R;
	for (int32_t x = 0; x < w; x++) {
		uint32_t v = p[x], b = v & 0x3ff, g = (v >> 10) & 0x3ff, r = (v >> 20) & 0x3ff;
		dst[x] = 0xff000000u | (xr ? (uint32_t)mapple_fb_xr8[r] << 16 | (uint32_t)mapple_fb_xr8[g] << 8 | mapple_fb_xr8[b] :
		    (r >> 2) << 16 | (g >> 2) << 8 | (b >> 2));
	}
}

/* Copy each enabled layer's surface into the panel memory (no scaling). */
/*
 * A swap only records the frame: copying 10 MB into the panel on every
 * swap (backboardd presents at 60 Hz) took the whole CPU. The vsync timer
 * shows the latest frame every fFlushTicks ticks with boot-arg mp_fb_hz;
 * by default as often as the copies stay within mp_fb_duty percent of one
 * CPU (default 25: a 4 ms copy allows a frame every 16 ms, 60 Hz; an 8 ms
 * one every 32 ms, 30 Hz). Frames the GPU drew cost the CPU nothing else.
 */
static uint64_t absUsFB(uint64_t abs);

void
AppleCLCD::swap(const uint8_t *rec)
{
	IOLockLock(fLock);
	memcpy(fPending, rec, MAPPLE_FB_SWAP_REC_SIZE);
	if (fQueueCount == MAPPLE_FB_QUEUE) {
		/*
		 * The oldest swap was never shown: drop it. One record at a time: a single memmove from fQueue[0]
		 * spans several rows, which the fortified memmove rejects (object size of fQueue[0] = one record).
		 */
		for (uint32_t i = 1; i < MAPPLE_FB_QUEUE; i++) {
			memcpy(fQueue[i - 1], fQueue[i], MAPPLE_FB_SWAP_REC_SIZE);
			fQueueSeq[i - 1] = fQueueSeq[i];
			fQueueAt[i - 1] = fQueueAt[i];
		}
		fQueueCount--;
	}
	memcpy(fQueue[fQueueCount], rec, MAPPLE_FB_SWAP_REC_SIZE);
	uint64_t now = mach_absolute_time();
	fQueueAt[fQueueCount] = now;
	fQueueSeq[fQueueCount++] = ++fSeq;
	if (fLastSwapAt != 0 && now - fLastSwapAt < 8 * periodOr60()) {
		latAdd(&fLatSwap, now - fLastSwapAt);
	}
	if (fLastSwapAt != 0) {
		uint64_t us = absUsFB(now - fLastSwapAt);
		fSwapGapHist[us <= 17000 ? 0 : us <= 34000 ? 1 : us <= 50000 ? 2 : us <= 100000 ? 3 : 4]++;
	}
	fLastSwapAt = now;
	if (fLastNotifyAt != 0 && now - fLastNotifyAt < 4 * periodOr60()) {
		latAdd(&fLatRender, now - fLastNotifyAt);
	}
	fDirty = true;
	IOLockUnlock(fLock);
	fLastActivity = now;            /* 0189 auto refresh */
	if (fHzMode == 1 && fPanelHz != 120) {
		fHzWant = 120;
	}
	mt6781_cpufreq_ui_activity();     /* 0183: frames are being drawn: CPU clusters out of the floor */
	earlyKick();                    /* shown right away if the GPU has already finished it */
	/* the format of every layer CoreAnimation sets (SwapSetLayer), logged when it changes */
	uint32_t mask = get32(rec, 0x104);
	for (uint32_t layer = 0; layer < MAPPLE_FB_LAYERS; layer++) {
		uint32_t id = get32(rec, 0x54 + 4 * layer);
		if (!(mask & (1u << layer)) || id == 0) {
			continue;
		}
		MappleSurface *s = mapple_surface_copy(id);
		if (s != NULL && s->pixelFormat != mapple_fb_layer_format[layer]) {
			/* every format change of a layer (SwapSetLayer), whatever the swap count */
			char was[5], now[5];
			mapple_fb_fourcc(mapple_fb_layer_format[layer], was);
			mapple_fb_fourcc(s->pixelFormat, now);
			IOLog("FB: layer %u format '%s' (0x%x) -> '%s' (0x%x), surface %u %ux%u bpe %u bpr %u%s\n", layer,
			    was, mapple_fb_layer_format[layer], now, s->pixelFormat, id, s->width, s->height, s->bytesPerElement,
			    s->bytesPerRow, (s->pixelFormat == MAPPLE_FB_FORMAT_W30R || s->pixelFormat == MAPPLE_FB_FORMAT_W40A ||
			    s->pixelFormat == MAPPLE_FB_FORMAT_W40M || s->pixelFormat == MAPPLE_FB_FORMAT_L10R) ?
			    " (converted to BGRA8)" : (s->bytesPerElement != 4 ? " (NOT SHOWN: unsupported)" : ""));
			mapple_fb_layer_format[layer] = s->pixelFormat;
		}
		OSSafeReleaseNULL(s);
	}
	if ((++mapple_fb_swap_count % 1000) == 0) {
		IOLog("FB: %llu swaps\n", mapple_fb_swap_count);
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
	for (uint32_t layer = 0; layer < MAPPLE_FB_LAYERS; layer++) {
		uint32_t id = get32(rec, 0x54 + 4 * layer);
		if (!(mask & (1u << layer)) || id == 0) {
			continue;
		}
		MappleSurface *s = mapple_surface_copy(id);
		if (s == NULL || s == fSurface) {
			OSSafeReleaseNULL(s);
			continue;
		}
		uint64_t n = 0;
		if (!mapple_gpu_surface_idle(s, &n)) {
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
		if (layersIdle(recs + (size_t)i * MAPPLE_FB_SWAP_REC_SIZE, &jobs)) {
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
			mapple_fb_gpu_waits++;   /* 0137: the GPU governor boosts */
			return -1;
		}
		pick = (int)n - 1;
		force = true;
		if (fGpuWaitMax != 0 && ++fGpuForced <= 8) {
			IOLog("FB: frame shown after waiting %u vsyncs for the GPU\n", fGpuTicks);
		}
	}
	return present(recs + (size_t)pick * MAPPLE_FB_SWAP_REC_SIZE, force, jobs) ? pick : -1;
}

/* The letterbox's strip and side bars, opaque black: LK's layer blends by per-pixel alpha. */
void
AppleCLCD::lbBars(uintptr_t dst)
{
	for (int32_t y = 0; y < (int32_t)fHeight; y++) {
		uint32_t *row = (uint32_t *)(dst + (uint64_t)y * fRowBytes);
		int32_t a = y < fLb.y ? (int32_t)fWidth : fLb.x, b = y < fLb.y ? (int32_t)fWidth : fLb.x + fLb.w;
		for (int32_t x = 0; x < a; x++) {
			row[x] = 0xff000000u;
		}
		for (int32_t x = b; x < (int32_t)fWidth; x++) {
			row[x] = 0xff000000u;
		}
	}
}

/*
 * One layer (logical rect dx, dy, w x h, read from sx, sy of its surface) into the letterbox picture, bilinear: each
 * picture row blends two logical rows into fLbLine, each picture pixel two of its entries. Several times the cost of
 * the plain copy on the CPU; the MDP's resizer (MTKMDP.cpp) could take it over.
 */
void
AppleCLCD::lbBlit(uintptr_t dst, MappleSurface *s, uintptr_t src, int32_t sx, int32_t sy, int32_t dx, int32_t dy,
    int32_t w, int32_t h, uint32_t bpe, uint32_t fmt, bool convert)
{
	/* the picture columns and rows whose centres fall inside the layer */
	int32_t i0 = 0, i1, j0 = 0, j1;
	while (i0 < fLb.w && (int32_t)((fLbCol[i0] + 0x8000) >> 16) < dx) {
		i0++;
	}
	for (i1 = i0; i1 < fLb.w && (int32_t)((fLbCol[i1] + 0x8000) >> 16) < dx + w; i1++) {
	}
	while (j0 < fLb.h && (int32_t)((fLbRow[j0] + 0x8000) >> 16) < dy) {
		j0++;
	}
	for (j1 = j0; j1 < fLb.h && (int32_t)((fLbRow[j1] + 0x8000) >> 16) < dy + h; j1++) {
	}
	uintptr_t base = src + s->offset + (size_t)sx * bpe;
	uint32_t *conv[2] = { fLbConv[0], fLbConv[1] };
	int32_t convY[2] = { -1, -1 };
	for (int32_t j = j0; j < j1; j++) {
		int32_t ya = (int32_t)(fLbRow[j] >> 16) - dy;
		ya = ya < 0 ? 0 : ya > h - 1 ? h - 1 : ya;
		int32_t yb = ya < h - 1 ? ya + 1 : ya;
		uint32_t wy = (fLbRow[j] >> 8) & 0xff;
		uintptr_t pa = base + (size_t)(sy + ya) * s->bytesPerRow;
		uintptr_t pb = base + (size_t)(sy + yb) * s->bytesPerRow;
		const uint32_t *ra, *rb;
		if (convert) {
			if (convY[0] != ya && convY[1] == ya) {
				uint32_t *t = conv[0];
				conv[0] = conv[1];
				conv[1] = t;
				convY[1] = convY[0];
				convY[0] = ya;
			}
			if (convY[0] != ya) {
				mapple_fb_convert_row(conv[0], (const uint8_t *)pa, w, fmt);
				convY[0] = ya;
			}
			if (convY[1] != yb) {
				mapple_fb_convert_row(conv[1], (const uint8_t *)pb, w, fmt);
				convY[1] = yb;
			}
			ra = conv[0];
			rb = conv[1];
		} else {
			ra = (const uint32_t *)pa;
			rb = (const uint32_t *)pb;
		}
		for (int32_t x = 0; x < w; x++) {
			uint64_t a = mapple_lb_spread(ra[x]);
			fLbLine[x] = wy ? mapple_lb_mix(a, mapple_lb_spread(rb[x]), wy) : a;
		}
		uint32_t *to = (uint32_t *)(dst + (uint64_t)(fLb.y + j) * fRowBytes) + fLb.x;
		for (int32_t i = i0; i < i1; i++) {
			int32_t xa = (int32_t)(fLbCol[i] >> 16) - dx;
			xa = xa < 0 ? 0 : xa > w - 1 ? w - 1 : xa;
			int32_t xb = xa < w - 1 ? xa + 1 : xa;
			to[i] = mapple_lb_pack(mapple_lb_mix(fLbLine[xa], fLbLine[xb], (fLbCol[i] >> 8) & 0xff));
		}
	}
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
	if (fRszArg != 0 && !fRszDumped) {
		fRszDumped = true;
		rszLog();
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
		if (PE_parse_boot_argn_alias("mp_fb_noconsole", "sweetfb_noconsole", &quiet, sizeof(quiet)) && quiet != 0) {
			/* Not PE_initialize_console(kPEDisableScreen): its console switch
			 * writes the read-only-late cons_ops_index and panics. */
			mapple_console_muted = 1;
		}
	}
	uint64_t t0 = mach_absolute_time();

	/* each enabled layer clipped to the panel first: which buffer to draw in depends on what the frame covers */
	struct Blit {
		MappleSurface *s;
		uintptr_t     src;
		int32_t       sx, sy, dx, dy, w, h;
		uint32_t      bpe;
	} blit[MAPPLE_FB_LAYERS];
	uint32_t nblit = 0;
	bool full = false, usesDefault = false;
	for (uint32_t layer = 0; layer < MAPPLE_FB_LAYERS; layer++) {
		uint32_t id = get32(rec, 0x54 + 4 * layer);
		const uint8_t *a = rec + 0x64 + 16 * layer;
		const uint8_t *b = rec + 0xc4 + 16 * layer;
		int32_t sx = (int32_t)get32(a, 0), sy = (int32_t)get32(a, 4);
		int32_t dx = (int32_t)get32(b, 0), dy = (int32_t)get32(b, 4);
		int32_t w = (int32_t)get32(b, 8), h = (int32_t)get32(b, 12);

		if (!(mask & (1u << layer)) || id == 0) {
			continue;
		}
		MappleSurface *s = mapple_surface_copy(id);
		if (mapple_fb_swaps_logged < 16) {
			mapple_fb_swaps_logged++;
			IOLog("FB: swap %u layer %u surface %u %ux%u fmt 0x%x bpr %u a %d,%d %ux%u b %d,%d %dx%d\n",
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
		/* Logical (presented) display coordinates -> panel coordinates; the letterbox scales logical ones. */
		int32_t outW = (int32_t)fWidth, outH = (int32_t)fHeight;
		if (fLb.w != 0) {
			outW = (int32_t)fLogicalW;
			outH = (int32_t)fLogicalH;
		} else {
			dx -= fOffX;
			dy -= fOffY;
		}
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
		if (w > outW - dx) {
			w = outW - dx;
		}
		if (h > outH - dy) {
			h = outH - dy;
		}
		uint64_t last = (uint64_t)s->offset + (uint64_t)(sy + h - 1) * s->bytesPerRow +
		    (uint64_t)(sx + w) * bpe;
		if (w <= 0 || h <= 0 || s->elementWidth != 1 || s->elementHeight != 1 ||
		    s->memory == NULL || last > s->memory->getLength()) {
			src = 0;
		}
		uint32_t fmt = s->pixelFormat;
		bool convert = fmt == MAPPLE_FB_FORMAT_W30R || fmt == MAPPLE_FB_FORMAT_L10R ||
		    ((fmt == MAPPLE_FB_FORMAT_W40A || fmt == MAPPLE_FB_FORMAT_W40M) && bpe == 8);
		if (src != 0 && !convert && bpe != 4) {
			src = 0;
		}
		if (src != 0 && dx == 0 && dy == 0 && w == outW && h == outH) {
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
	if (!mapple_console_muted || usesDefault) {
		buf = 0;
	} else if (fFlipReady && full) {
		buf = shownBuf == 0 ? 1 : 0;
	} else {
		buf = shownBuf >= 0 ? shownBuf : 0;
	}
	uintptr_t dst = fBufKVA[buf];
	if (fLb.w != 0 && nblit != 0) {
		lbBars(dst);
	}
	for (uint32_t i = 0; i < nblit; i++) {
		Blit *bl = &blit[i];
		MappleSurface *s = bl->s;
		if (bl->src != 0) {
			mapple_surface_before_cpu_access(s);     /* the GPU may have drawn it: no stale CPU lines */
			uint32_t fmt = s->pixelFormat;
			bool convert = fmt == MAPPLE_FB_FORMAT_W30R || fmt == MAPPLE_FB_FORMAT_L10R ||
			    ((fmt == MAPPLE_FB_FORMAT_W40A || fmt == MAPPLE_FB_FORMAT_W40M) && bl->bpe == 8);
			if (fLb.w != 0) {
				lbBlit(dst, s, bl->src, bl->sx, bl->sy, bl->dx, bl->dy, bl->w, bl->h, bl->bpe, fmt,
				    convert);
			} else {
				for (int32_t y = 0; y < bl->h; y++) {
					uint32_t *to = (uint32_t *)(dst + (uint64_t)(bl->dy + y) * fRowBytes +
					    (uint64_t)bl->dx * 4);
					const uint8_t *from = (const uint8_t *)(bl->src + s->offset +
					    (uint64_t)(bl->sy + y) * s->bytesPerRow + (uint64_t)bl->sx * bl->bpe);
					if (convert) {
						mapple_fb_convert_row(to, from, bl->w, fmt);
					} else {
						memcpy(to, from, (size_t)bl->w * 4);
					}
				}
			}
		}
		s->release();
	}
	/* the next copy no sooner than its share of the CPU (mp_fb_duty) allows */
	uint64_t t1 = mach_absolute_time(), ns;
	uint32_t duty = gMappleFbDutyCap != 0 ? MIN(fDuty, gMappleFbDutyCap) : fDuty;   /* Low Power Mode */
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

/* ---- direct scanout ---- */

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
/* 0189: DSI0 (Xiaomi viva-r-oss drivers/gpu/drm/mediatek/mtk_dsi.c, CONFIG_MACH_MT6781 offsets) */
#define DISP_DSI0_PA            0x14013000ull
#define DSI_START               0x000   /* bit 0 video start, bit 16 VM_CMD_START */
#define DSI_VM_CMD_START        (1u << 16)
#define DSI_INTSTA              0x00c   /* bit 5 VM_CMD_DONE (write 0 to clear) */
#define DSI_VM_CMD_DONE         (1u << 5)
#define DSI_MODE_CTRL           0x014   /* [1:0] 0 = command mode */
#define DSI_VFP_NL              0x028
#define DSI_VM_CMD_CON          0x200   /* [0] enable, [1] long packet, [5] send in VFP, [15:8] type, [31:16] data/len */
#define DSI_VM_CMD_DATA0        0x208
#define DISP_CCORR0_PA          0x1400b000ull
#define CCORR_CFG               0x020   /* bit 0 relay (bypass), bit 1 engine */
#define CCORR_COEF(n)           (0x080 + 4 * (n))
#define PANEL_LEVEL_MAX         2047
#define PANEL_VFP_60            2452
#define PANEL_VFP_120           20
/*
 * mp_fb_rsz: MediaTek's DRM driver for MT6781 (realme kernel drivers/gpu/drm/mediatek: mtk_drm_ddp.c routing and mutex
 * bits, mtk_disp_rsz.c, mtk_disp_ovl.c; drivers/clk/mediatek/clk-mt6781.c MMSYS gates; dts mt6781.dts).
 */
#define DISP_MMSYS_PA           0x14000000ull
#define DISP_MUTEX_PA           0x14001000ull
#define DISP_RDMA0_PA           0x14007000ull
#define DISP_RSZ0_PA            0x14008000ull
#define D_MMSYS                 6       /* fDisp[] */
#define D_MUTEX                 7
#define D_RDMA0                 8
#define D_RSZ0                  9
#define MMSYS_CG_CON0           0x100   /* 1 = clock gated */
#define MMSYS_CG_SET0           0x104
#define MMSYS_CG_CLR0           0x108
#define CG_MUTEX0               (1u << 0)
#define CG_OVL0                 (1u << 2)
#define CG_RDMA0                (1u << 3)
#define CG_OVL0_2L              (1u << 4)
#define CG_RSZ0                 (1u << 7)
#define CG_POSTMASK0            (1u << 14)
#define MMSYS_OVL_CON           0xf04   /* bit 0 OVL0 blends to its output, bit 3 OVL0_2L into OVL0's background */
#define MMSYS_OVL0_2L_MOUT_EN   0xf14
#define MMSYS_OVL0_MOUT_EN      0xf18   /* bit 0 RDMA0, 1 RSZ0, 2 WDMA0 */
#define MMSYS_RSZ0_MOUT_EN      0xf1c   /* bit 0 RDMA0 */
#define MMSYS_RSZ0_SEL_IN       0xf24   /* 0 OVL0_2L, 1 OVL0 */
#define MMSYS_RDMA0_SEL_IN      0xf28   /* 0 OVL0, 1 RSZ0, 2 OVL0_2L */
#define MMSYS_DSI0_SEL_IN       0xf30
#define MUTEX_EN(n)             (0x20 + 0x20 * (n))
#define MUTEX_SOF(n)            (0x2c + 0x20 * (n))
#define MUTEX_MOD(n)            (0x30 + 0x20 * (n))     /* bit 0 OVL0, 1 OVL0_2L, 2 RDMA0, 3 RSZ0 */
#define OVL_DATAPATH_CON        0x024   /* bit 2: background from the previous OVL */
#define OVL_ROI_BGCLR           0x028
#define OVL_L_EN(n)             (1u << (n))
#define RDMA_INT_STATUS         0x004   /* bit 4 FIFO underflow */
#define RDMA_GLOBAL_CON         0x010
#define RDMA_SIZE_CON_0         0x014
#define RDMA_SIZE_CON_1         0x018
#define RSZ_ENABLE              0x000   /* bit 0 */
#define RSZ_CON_1               0x004   /* bit 0 horizontal, bit 1 vertical */
#define RSZ_CON_2               0x008   /* bit 9 power saving: Android's RGB999 (its RGB888's bit 28 reads 0 on MT6781) */
#define RSZ_INT_FLAG            0x00c   /* bit 4 size error */
#define RSZ_IN_IMAGE            0x010
#define RSZ_OUT_IMAGE           0x014
#define RSZ_STEP_H              0x018
#define RSZ_STEP_V              0x01c
#define RSZ_H_INT               0x020
#define RSZ_H_SUB               0x024
#define RSZ_V_INT               0x028
#define RSZ_V_SUB               0x02c
#define RSZ_DEBUG               0x048
#define RSZ_SHADOW_CTRL         0x0f0   /* bit 0 bypass the shadow registers (Android does on MT6781) */
#define RSZ_IN_W_MAX            1200    /* MT6781's tile length */
#define RSZ_IN_H_MAX            4096


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
mapple_fb_panic_show(void)
{
	AppleCLCD *fb = gMappleFB;
	if (fb != NULL && fb->fScanOvl >= 0) {
		if (fb->fRszLive) {
			fb->rszPipe(false);
		}
		vm_offset_t base = fb->fDisp[fb->fScanOvl];
		disp_wr(base, fb->fScanPitchMsbReg, fb->fLkPitchMsb);
		disp_wr(base, fb->fScanPitchReg, fb->fLkPitch);
		disp_wr(base, fb->fScanAddrReg, fb->fLkAddr);
	}
}

bool
AppleCLCD::dispMap(void)
{
	static const uint64_t pa[10] = { DISP_OVL0_PA, DISP_OVL0_2L_PA, SMI_LARB0_PA, SMI_LARB1_PA, DISP_DSI0_PA, DISP_CCORR0_PA,
		DISP_MMSYS_PA, DISP_MUTEX_PA, DISP_RDMA0_PA, DISP_RSZ0_PA };
	for (int i = 0; i < (fRszArg != 0 ? 10 : 6); i++) {
		if (fDispMap[i] != NULL) {
			continue;
		}
		IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(pa[i], 0x1000, kIODirectionInOut);
		fDispMap[i] = md != NULL ? md->map(kIOMapInhibitCache) : NULL;
		OSSafeReleaseNULL(md);
		if (fDispMap[i] == NULL) {
			if (i < 6) {
				return false;
			}
			continue;               /* mp_fb_rsz's blocks: their users check fDisp[] */
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
	bool exact = false;     /* a layer on the boot framebuffer itself beats one on another of LK's buffers */
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
			if ((disp_rd(b, OVL_SRC_CON) & (1u << n)) && addr >= lo && addr < hi && (addr == lo || !exact)) {
				exact = addr == lo;
				fScanOvl = o;
				fScanAddrReg = OVL_L_ADDR(n);
				fScanPitchReg = OVL_L_PITCH(n);
				fScanPitchMsbReg = OVL_L_PITCH_MSB(n);
				snprintf(fScanLayer, sizeof(fScanLayer), "%s L%d", o ? "OVL0_2L" : "OVL0", n);
			}
		}
		for (int k = 0; o == 0 && k < 3; k++) {
			uint32_t addr = disp_rd(b, OVL_EL_ADDR(k));
			if ((disp_rd(b, OVL_DATAPATH_EXT_CON) & (1u << k)) && addr >= lo && addr < hi && (addr == lo || !exact)) {
				exact = addr == lo;
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
	mapple_fb_panic_hook = mapple_fb_panic_show;      /* from now on the layer may read something else */
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
	for (uint32_t l = 0; l < MAPPLE_FB_LAYERS; l++) {
		if ((mask & (1u << l)) && get32(rec, 0x54 + 4 * l) != 0) {
			layers++;
			id = get32(rec, 0x54 + 4 * l);
			idx = l;
		}
	}
	const uint8_t *a = rec + 0x64 + 16 * idx, *b = rec + 0xc4 + 16 * idx;
	MappleSurface *s = layers == 1 ? mapple_surface_copy(id) : NULL;
	const char *why = NULL;
	uint64_t pa = 0;
	bool rsz = fLb.w != 0 && fRszWant && fRszWhy == NULL && fFlipReady && fPanelHz == 60;
	if (fLb.w != 0 && !rsz) {
		why = "letterbox: the picture is scaled";
	} else if (layers != 1) {
		why = "not exactly one layer";
	} else if (s == NULL || !s->contig) {
		why = "surface not contiguous";
	} else if (s->width != fLogicalW || s->height != fLogicalH || s->bytesPerElement != 4 ||
	    s->pixelFormat != MAPPLE_FB_FORMAT_BGRA || s->bytesPerRow > 0xfffff) {
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
	if (why == NULL && rsz && !fRszReady && !rszPrepare()) {
		why = fRszWhy;
	}
	if (why != NULL) {
		if (fScanWhy != why) {
			IOLog("FB: scanout: copying (%s)\n", why);
			fScanWhy = why;
		}
		OSSafeReleaseNULL(s);
		if (!fFlipReady) {
			scanoutStop();          /* LK's buffer at once; with flips the copy's buffer follows at a frame end */
		}
		fScanFallbacks++;
		return false;
	}
	mapple_surface_before_device_read(s);    /* CPU-drawn pixels out of the caches */
	Target t;
	t.rsz = rsz;
	t.addr = (uint32_t)(pa + s->offset + (rsz ? 0 : (uint64_t)fOffY * s->bytesPerRow + (uint64_t)fOffX * 4));
	t.pitchMsb = (fLkPitchMsb & ~0xfu) | ((s->bytesPerRow >> 16) & 0xf);
	t.pitch = (fLkPitch & ~0xffffu) | (s->bytesPerRow & 0xffff);
	t.buf = -1;
	t.surface = s;                          /* keeps the memory alive while the OVL reads it */
	t.swap = get32(rec, 0x50);
	if (fShown.buf >= 0 || fScanWhy != NULL) {
		IOLog("FB: scanout: %s reads surface %u at 0x%08x (pitch %u)%s%s\n", fScanLayer, s->id, t.addr,
		    s->bytesPerRow, fFlipReady ? " from the next frame end" : "", rsz ? ", through the resizer" : "");
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

/*
 * ---- mp_fb_rsz: the display pipe's resizer ----
 * 1: read only, the routing, mutex, OVL0/OVL0_2L, RDMA0 and RSZ0 state and the clock gates logged at the first frame
 * ("disp rsz" on the link). 2: the letterbox (MappleLetterbox.h) on RSZ0 for frames scanned out: OVL0 composes the
 * whole surface at (lx, ly) of a bigger frame over its black background, MMSYS routes OVL0 -> RSZ0 -> RDMA0 instead of
 * OVL0 -> RDMA0, RSZ0 (in mutex 0) scales the frame onto the panel. Frames that cannot be scanned out go through LK's
 * path and the CPU letterbox: the same picture. The pipe changes only at a frame end (shadow registers bypassed on
 * MT6781: writes take effect at once), from LK's values read beforehand and put back the same way.
 * Android never routes RSZ0 this way: its MT6781 RSZ0 sits between OVL0_2L and OVL0 and only scales up.
 */

/* mp_fb_rsz writes nothing else: block, register, the bits that may change. */
static const struct {
	uint8_t  blk;
	uint16_t off;
	uint32_t mask;
} mapple_rsz_regs[] = {
	{ D_MMSYS, MMSYS_CG_SET0, CG_RSZ0 }, { D_MMSYS, MMSYS_CG_CLR0, CG_RSZ0 },
	{ D_MMSYS, MMSYS_OVL0_MOUT_EN, 0x3 }, { D_MMSYS, MMSYS_RSZ0_MOUT_EN, 0x1 },
	{ D_MMSYS, MMSYS_RSZ0_SEL_IN, 0x1 }, { D_MMSYS, MMSYS_RDMA0_SEL_IN, 0x3 },
	{ D_MUTEX, MUTEX_MOD(0), 1u << 3 },
	{ 0, OVL_ROI_SIZE, 0x1fff1fff }, { 1, OVL_ROI_SIZE, 0x1fff1fff },
	{ 0, OVL_L_SRC_SIZE(0), 0x1fff1fff }, { 0, OVL_L_SRC_SIZE(1), 0x1fff1fff }, { 0, OVL_L_SRC_SIZE(2), 0x1fff1fff },
	{ 0, OVL_L_SRC_SIZE(3), 0x1fff1fff }, { 1, OVL_L_SRC_SIZE(0), 0x1fff1fff }, { 1, OVL_L_SRC_SIZE(1), 0x1fff1fff },
	{ 0, OVL_L_OFFSET(0), 0x1fff1fff }, { 0, OVL_L_OFFSET(1), 0x1fff1fff }, { 0, OVL_L_OFFSET(2), 0x1fff1fff },
	{ 0, OVL_L_OFFSET(3), 0x1fff1fff }, { 1, OVL_L_OFFSET(0), 0x1fff1fff }, { 1, OVL_L_OFFSET(1), 0x1fff1fff },
	{ D_RSZ0, RSZ_ENABLE, 0x1 }, { D_RSZ0, RSZ_SHADOW_CTRL, 0x1 },
	{ D_RSZ0, RSZ_CON_1, ~0u }, { D_RSZ0, RSZ_CON_2, ~0u }, { D_RSZ0, RSZ_IN_IMAGE, ~0u }, { D_RSZ0, RSZ_OUT_IMAGE, ~0u },
	{ D_RSZ0, RSZ_STEP_H, ~0u }, { D_RSZ0, RSZ_STEP_V, ~0u }, { D_RSZ0, RSZ_H_INT, 0xffff }, { D_RSZ0, RSZ_H_SUB, 0x1fffff },
	{ D_RSZ0, RSZ_V_INT, 0xffff }, { D_RSZ0, RSZ_V_SUB, 0x1fffff },
};

/* The one way to write: whitelisted bits only, read-modify-write and read back; the gate SET/CLR pair as given. */
bool
AppleCLCD::rszWrite(int blk, uint32_t off, uint32_t mask, uint32_t value)
{
	size_t i, n = sizeof(mapple_rsz_regs) / sizeof(mapple_rsz_regs[0]);
	for (i = 0; i < n && !(mapple_rsz_regs[i].blk == blk && mapple_rsz_regs[i].off == off); i++) {
	}
	fRszBad = ((uint32_t)blk << 16) | off;
	if (i == n || mask == 0 || (mask & ~mapple_rsz_regs[i].mask) || (value & ~mask)) {
		return false;
	}
	vm_offset_t b = fDisp[blk];
	if (blk == D_MMSYS && (off == MMSYS_CG_SET0 || off == MMSYS_CG_CLR0)) {
		disp_wr(b, off, value);
		return true;
	}
	disp_wr(b, off, (disp_rd(b, off) & ~mask) | value);
	return (disp_rd(b, off) & mask) == value;
}

/*
 * fScanLock, once, at the first frame that could go through the resizer: LK's path checked (read only), RSZ0's clock
 * on and RSZ0 configured (in no path yet, disabled), the pipe writes listed with LK's values. false: fRszWhy.
 */
bool
AppleCLCD::rszPrepare(void)
{
	const char *why = NULL;
	if (!dispMap() || fDisp[D_MMSYS] == 0 || fDisp[D_MUTEX] == 0 || fDisp[D_RDMA0] == 0 || fDisp[D_RSZ0] == 0) {
		fRszWhy = "display registers not mappable";
		IOLog("FB: rsz: off (%s): frames scaled on the CPU\n", fRszWhy);
		return false;
	}
	vm_offset_t m = fDisp[D_MMSYS], x = fDisp[D_MUTEX], z = fDisp[D_RSZ0], o0 = fDisp[0], o1 = fDisp[1];
	uint32_t panel = (fHeight << 16) | fWidth, a = fScanAddrReg - OVL_L_ADDR(0), n = a / 0x20;
	uint32_t cg = disp_rd(m, MMSYS_CG_CON0);
	bool two = !(cg & CG_OVL0_2L) && (disp_rd(o1, OVL_EN) & 1);        /* OVL0_2L under OVL0 */
	if (cg & (CG_MUTEX0 | CG_OVL0 | CG_RDMA0)) {
		why = "a display clock is gated";
	} else if (fScanAddrReg < OVL_L_ADDR(0) || a % 0x20 != 0 || n >= (fScanOvl ? 2u : 4u)) {
		why = "LK's layer is an extended layer";
	} else if (!(disp_rd(o0, OVL_EN) & 1) || disp_rd(o0, OVL_ROI_SIZE) != panel) {
		why = "OVL0 off or not panel-sized";
	} else if ((disp_rd(m, MMSYS_OVL0_MOUT_EN) & 7) != 1 || disp_rd(m, MMSYS_RDMA0_SEL_IN) != 0 ||
	    disp_rd(m, MMSYS_RSZ0_MOUT_EN) != 0) {
		why = "the path is not OVL0 -> RDMA0";
	} else if (!(disp_rd(x, MUTEX_EN(0)) & 1) || (disp_rd(x, MUTEX_MOD(0)) & 0xf) != (two ? 0x7u : 0x5u)) {
		why = "LK's path is not mutex 0's";
	} else if (two && (!(disp_rd(m, MMSYS_OVL_CON) & (1u << 3)) || !(disp_rd(o0, OVL_DATAPATH_CON) & (1u << 2)) ||
	    disp_rd(o1, OVL_ROI_SIZE) != panel)) {
		why = "OVL0_2L is on but not OVL0's panel-sized background";
	} else if (fScanOvl == 1 && !two) {
		why = "LK's layer is on an OVL outside the path";
	} else if ((disp_rd(fDisp[two ? 1 : 0], OVL_ROI_BGCLR) & 0xffffff) != 0) {
		why = "the OVL background is not black";
	} else if (fRsz.in_w > RSZ_IN_W_MAX || fRsz.in_h > RSZ_IN_H_MAX || fRsz.lx + (int32_t)fLogicalW > fRsz.in_w ||
	    fRsz.ly + (int32_t)fLogicalH > fRsz.in_h) {
		why = "the frame does not fit RSZ0";
	}
	for (int k = 0; why == NULL && k < 2; k++) {
		uint32_t on = disp_rd(fDisp[k], OVL_SRC_CON) & (k ? 0x3u : 0xfu);
		if (k == fScanOvl) {
			on &= ~OVL_L_EN(n);
		}
		if ((k == 0 || two) && (on != 0 || (disp_rd(fDisp[k], OVL_DATAPATH_EXT_CON) & 7) != 0)) {
			why = "other OVL layers are on (boot with mp_fb_onelayer=1)";
		}
	}
	bool opened = false;
	if (why == NULL && (cg & CG_RSZ0)) {
		opened = rszWrite(D_MMSYS, MMSYS_CG_CLR0, CG_RSZ0, CG_RSZ0) && !(disp_rd(m, MMSYS_CG_CON0) & CG_RSZ0);
		if (!opened) {
			why = "RSZ0's clock did not start";
		}
	}
	if (why == NULL && (disp_rd(z, RSZ_ENABLE) & 1)) {
		why = "RSZ0 is in use";
	}
	const struct mapple_rsz &c = fRsz;
	if (why == NULL && (!rszWrite(D_RSZ0, RSZ_SHADOW_CTRL, 1, 1) || !rszWrite(D_RSZ0, RSZ_CON_1, ~0u, 0x3) ||
	    !rszWrite(D_RSZ0, RSZ_CON_2, ~0u, 1u << 9) ||
	    !rszWrite(D_RSZ0, RSZ_IN_IMAGE, ~0u, ((uint32_t)c.in_h << 16) | (uint32_t)c.in_w) ||
	    !rszWrite(D_RSZ0, RSZ_OUT_IMAGE, ~0u, panel) || !rszWrite(D_RSZ0, RSZ_STEP_H, ~0u, c.step_x) ||
	    !rszWrite(D_RSZ0, RSZ_STEP_V, ~0u, c.step_y) || !rszWrite(D_RSZ0, RSZ_H_INT, 0xffff, c.off_x >> 15) ||
	    !rszWrite(D_RSZ0, RSZ_H_SUB, 0x1fffff, c.off_x & 0x7fff) || !rszWrite(D_RSZ0, RSZ_V_INT, 0xffff, c.off_y >> 15) ||
	    !rszWrite(D_RSZ0, RSZ_V_SUB, 0x1fffff, c.off_y & 0x7fff))) {
		why = "RSZ0 did not take its configuration";
	}
	if (why != NULL) {
		if (opened) {
			rszWrite(D_MMSYS, MMSYS_CG_SET0, CG_RSZ0, CG_RSZ0);
		}
		fRszWhy = why;
		IOLog("FB: rsz: off (%s): frames scaled on the CPU\n", why);
		return false;
	}
	const struct {
		uint8_t  blk;
		uint16_t off;
		uint32_t mask, on;
	} pipe[] = {
		{ D_RSZ0, RSZ_ENABLE, 0x1, 0x1 },
		{ 0, OVL_ROI_SIZE, 0x1fff1fff, ((uint32_t)c.in_h << 16) | (uint32_t)c.in_w },
		{ 1, OVL_ROI_SIZE, 0x1fff1fff, ((uint32_t)c.in_h << 16) | (uint32_t)c.in_w },
		{ (uint8_t)fScanOvl, (uint16_t)OVL_L_SRC_SIZE(n), 0x1fff1fff, (fLogicalH << 16) | fLogicalW },
		{ (uint8_t)fScanOvl, (uint16_t)OVL_L_OFFSET(n), 0x1fff1fff, ((uint32_t)c.ly << 16) | (uint32_t)c.lx },
		{ D_MUTEX, MUTEX_MOD(0), 1u << 3, 1u << 3 },
		{ D_MMSYS, MMSYS_RSZ0_SEL_IN, 0x1, 0x1 },       /* from OVL0 */
		{ D_MMSYS, MMSYS_RSZ0_MOUT_EN, 0x1, 0x1 },      /* to RDMA0 */
		{ D_MMSYS, MMSYS_RDMA0_SEL_IN, 0x3, 0x1 },      /* from RSZ0 */
		{ D_MMSYS, MMSYS_OVL0_MOUT_EN, 0x3, 0x2 },      /* to RSZ0 */
	};
	fRszN = 0;
	for (size_t i = 0; i < sizeof(pipe) / sizeof(pipe[0]); i++) {
		if (pipe[i].off == OVL_ROI_SIZE && pipe[i].blk == 1 && !two) {
			continue;
		}
		fRszBlk[fRszN] = pipe[i].blk;
		fRszOff[fRszN] = pipe[i].off;
		fRszMask[fRszN] = pipe[i].mask;
		fRszOn[fRszN] = pipe[i].on;
		fRszSave[fRszN] = disp_rd(fDisp[pipe[i].blk], pipe[i].off) & pipe[i].mask;
		fRszN++;
	}
	fRszReady = true;
	IOLog("FB: rsz: ready: %s frame %dx%d, %s at %d,%d, steps %u/%u, offsets %u/%u -> picture %dx%d at %d,%d\n",
	    two ? "OVL0_2L+OVL0" : "OVL0", c.in_w, c.in_h, fScanLayer, c.lx, c.ly, c.step_x, c.step_y, c.off_x, c.off_y,
	    fLb.w, fLb.h, fLb.x, fLb.y);
	return true;
}

/*
 * At a frame end with interrupts off, or in a panic: the pipe through RSZ0 (on, in the listed order) or LK's values back
 * (in reverse). A switch on that fails part-way is undone at once. No logging (panic).
 */
bool
AppleCLCD::rszPipe(bool on)
{
	bool ok = on && fRszReady;
	for (uint32_t i = 0; ok && i < fRszN; i++) {
		ok = rszWrite(fRszBlk[i], fRszOff[i], fRszMask[i], fRszOn[i]);
	}
	if (ok) {
		fRszLive = true;
		fRszSwitches++;
		return true;
	}
	uint32_t bad = fRszBad;
	bool back = true;
	for (uint32_t i = fRszN; i-- > 0;) {
		back = rszWrite(fRszBlk[i], fRszOff[i], fRszMask[i], fRszSave[i]) && back;
	}
	fRszLive = false;
	if (on) {
		fRszBad = bad;
		return false;
	}
	return back;
}

/* Read only. Registers of a gated block are never read. */
void
AppleCLCD::rszDump(char **out, char *end)
{
	if (!dispMap() || fDisp[D_MMSYS] == 0 || fDisp[D_MUTEX] == 0 || fDisp[D_RDMA0] == 0 || fDisp[D_RSZ0] == 0) {
		disp_say(out, end, "rsz: display registers not mappable\n");
		return;
	}
	vm_offset_t m = fDisp[D_MMSYS], x = fDisp[D_MUTEX], r = fDisp[D_RDMA0], z = fDisp[D_RSZ0];
	uint32_t cg = disp_rd(m, MMSYS_CG_CON0);
	disp_say(out, end, "rsz: MMSYS CG_CON0 0x%08x (gated: RSZ0 %u, POSTMASK0 %u), OVL_CON 0x%x, mout OVL0_2L 0x%x OVL0 0x%x "
	    "RSZ0 0x%x, sel RSZ0 0x%x RDMA0 0x%x DSI0 0x%x\n", cg, (cg & CG_RSZ0) >> 7, (cg & CG_POSTMASK0) >> 14,
	    disp_rd(m, MMSYS_OVL_CON), disp_rd(m, MMSYS_OVL0_2L_MOUT_EN), disp_rd(m, MMSYS_OVL0_MOUT_EN),
	    disp_rd(m, MMSYS_RSZ0_MOUT_EN), disp_rd(m, MMSYS_RSZ0_SEL_IN), disp_rd(m, MMSYS_RDMA0_SEL_IN),
	    disp_rd(m, MMSYS_DSI0_SEL_IN));
	for (int n = 0; n < 4 && !(cg & CG_MUTEX0); n++) {
		disp_say(out, end, "rsz: mutex%d en 0x%x sof 0x%x mod 0x%x\n", n, disp_rd(x, MUTEX_EN(n)), disp_rd(x, MUTEX_SOF(n)),
		    disp_rd(x, MUTEX_MOD(n)));
	}
	for (int o = 0; o < 2; o++) {
		vm_offset_t b = fDisp[o];
		if (cg & (o ? CG_OVL0_2L : CG_OVL0)) {
			disp_say(out, end, "rsz: %s clock gated\n", o ? "OVL0_2L" : "OVL0");
			continue;
		}
		disp_say(out, end, "rsz: %s en 0x%x roi 0x%08x datapath 0x%x bgclr 0x%08x src_con 0x%x ext 0x%x\n",
		    o ? "OVL0_2L" : "OVL0", disp_rd(b, OVL_EN), disp_rd(b, OVL_ROI_SIZE), disp_rd(b, OVL_DATAPATH_CON),
		    disp_rd(b, OVL_ROI_BGCLR), disp_rd(b, OVL_SRC_CON), disp_rd(b, OVL_DATAPATH_EXT_CON));
	}
	if (!(cg & CG_RDMA0)) {
		disp_say(out, end, "rsz: RDMA0 global 0x%x size 0x%x 0x%x int_status 0x%x\n", disp_rd(r, RDMA_GLOBAL_CON),
		    disp_rd(r, RDMA_SIZE_CON_0), disp_rd(r, RDMA_SIZE_CON_1), disp_rd(r, RDMA_INT_STATUS));
	}
	if (!(cg & CG_RSZ0)) {
		disp_say(out, end, "rsz: RSZ0 en 0x%x con 0x%x 0x%x int 0x%x in 0x%08x out 0x%08x step 0x%x 0x%x h %u.%u v %u.%u "
		    "debug 0x%x shadow 0x%x\n", disp_rd(z, RSZ_ENABLE), disp_rd(z, RSZ_CON_1), disp_rd(z, RSZ_CON_2),
		    disp_rd(z, RSZ_INT_FLAG), disp_rd(z, RSZ_IN_IMAGE), disp_rd(z, RSZ_OUT_IMAGE), disp_rd(z, RSZ_STEP_H),
		    disp_rd(z, RSZ_STEP_V), disp_rd(z, RSZ_H_INT), disp_rd(z, RSZ_H_SUB), disp_rd(z, RSZ_V_INT),
		    disp_rd(z, RSZ_V_SUB), disp_rd(z, RSZ_DEBUG), disp_rd(z, RSZ_SHADOW_CTRL));
	}
	if (fLb.w != 0) {
		disp_say(out, end, "rsz: plan: frame %dx%d, display at %d,%d, steps %u/%u, offsets %u/%u (1/32768) -> picture "
		    "%dx%d at %d,%d\n", fRsz.in_w, fRsz.in_h, fRsz.lx, fRsz.ly, fRsz.step_x, fRsz.step_y, fRsz.off_x, fRsz.off_y,
		    fLb.w, fLb.h, fLb.x, fLb.y);
	}
	disp_say(out, end, "rsz: mp_fb_rsz=%u, %s%s%s%s, %llu switches to the resizer\n", fRszArg,
	    fRszLive ? "in the pipe" : fRszReady ? "ready" : "not prepared", fRszWant ? "" : " (disp rsz off)",
	    fRszWhy ? ", off: " : "", fRszWhy ? fRszWhy : "", fRszSwitches);
}

void
AppleCLCD::rszLog(void)
{
	static char buf[3072];
	char *p = buf, *line = buf;
	buf[0] = 0;
	rszDump(&p, buf + sizeof(buf));
	for (char *q = buf; *q != 0; q++) {
		if (*q == '\n') {
			*q = 0;
			IOLog("FB: %s\n", line);
			line = q + 1;
		}
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
	t.rsz = false;
	return t;
}

void
AppleCLCD::layerWrite(const Target &t)
{
	if (fScanOvl < 0) {
		return;                 /* LK's layer unknown: only its own buffer is ever shown */
	}
	bool rsz = t.rsz && fRszWhy == NULL;
	if (rsz != fRszLive) {
		IODelay(150);           /* RSZ0 and RDMA0 still hold the frame's last lines after the OVL's end (~3.4 us a line) */
	}
	if (rsz != fRszLive && !rszPipe(rsz)) {
		fRszWhy = rsz ? "a pipe register did not take its value: back to LK's path" : "LK's path not restored";
		IOLog("FB: rsz: %s (block %u offset 0x%x)\n", fRszWhy, fRszBad >> 16, fRszBad & 0xffff);
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
	bool applied = false;
	if ((t.rsz && fRszWhy == NULL) != fRszLive && fVblankOvl >= 0 && fPeriod != 0) {
		vblankPoll(2 * fPeriod, &t, &applied);          /* the pipe changes at a frame end only */
	}
	if (!applied) {
		layerWrite(t);
	}
	MappleSurface *old = fShown.surface;
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

/* 0184: what holds a swap that SwapWait found not done. */
uint32_t
AppleCLCD::waitWhy(uint32_t swap)
{
	uint32_t why = WAIT_OTHER;
	IOLockLock(fLock);
	for (uint32_t i = 0; i < fQueueCount; i++) {
		if (get32(fQueue[i], 0x50) == swap) {
			why = WAIT_QUEUED;
		}
	}
	IOLockUnlock(fLock);
	if (why == WAIT_OTHER && swap != 0 && fNextSwap == swap) {
		why = WAIT_NEXT;
	} else if (why == WAIT_OTHER && swap != 0 && fShownSwap == swap) {
		why = WAIT_SHOWN;
	}
	return why;
}

/* 0184: a non-blocking SwapWait(swap) answered done or busy. Counters only. */
void
AppleCLCD::waitResult(uint32_t swap, bool done)
{
	uint64_t now = mach_absolute_time();
	if (!done) {
		uint32_t why = waitWhy(swap);
		fWaitBusyWhy[why]++;
		if (fBusySwap != swap) {
			fBusySwap = swap;
			fBusyWhy = why;
			fBusyAt = now;
		}
	} else if (fBusySwap == swap && swap != 0) {
		IOLockLock(fLock);
		latAdd(&fLatWait[fBusyWhy], now - fBusyAt);
		IOLockUnlock(fLock);
		fBusySwap = 0;
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
 * LK reserves MAPPLE_FB_LK_BUFFERS panel buffers back to back (live DT mblock-18-framebuffer: 0x7bc10000, 0x1df0000
 * bytes = 3 x 1088 x 2400 x 4). 0128: some boots LK leaves OVL0 L1 enabled on its second buffer (seen after a panic
 * reboot: L1 1080x2400 pitch 4320 at 0x7c606000); the double buffer was then off and every copy went to the buffer on
 * screen (tearing bands while scrolling). The third buffer is used instead.
 */
/*
 * 0189: switch the panel to `hz` (60 or 120), see the top. fScanLock. Busy for ~3 frames (two waited for the panel to
 * take the mode, one to calibrate the frame end again).
 */
bool
AppleCLCD::panelSetHz(uint32_t hz)
{
	if (!dispMap()) {
		fHzWhy = "display registers not mappable";
		return false;
	}
	vm_offset_t d = fDisp[4];
	if (fDcsInFlight) {             /* 0191: a brightness write still waiting for its blanking */
		for (int k = 0; k < 1000 && !(disp_rd(d, DSI_INTSTA) & DSI_VM_CMD_DONE); k++) {
			IODelay(50);
		}
		fDcsInFlight = false;
	}
	uint32_t vfp = disp_rd(d, DSI_VFP_NL);
	if ((disp_rd(d, DSI_MODE_CTRL) & 3) == 0) {
		fHzWhy = "DSI not in video mode";
		return false;
	}
	if (vfp != PANEL_VFP_60 && vfp != PANEL_VFP_120) {
		fHzWhy = "unknown panel timing (VFP not 2452 / 20)";
		return false;
	}
	/* {VM_CMD_CON, DATA0}: enable | send in VFP, DCS long (0x39, 3 bytes) or short with a parameter (0x15) */
	/* vendor mipi_dsi_send_vm_cmd: going to 60 Hz the first two go in VSA/VBP ([4:3]) instead of the VFP ([5]) */
	const uint32_t cmds[4][2] = {
		{ hz == 120 ? 0x00033923u : 0x0003391bu, 0x005a5af0 },  /* F0 5A 5A */
		{ hz == 120 ? 0x01601521u : 0x21601519u, 0 },           /* 60 01 / 60 21 */
		{ 0x0bf71521, 0 },                                      /* F7 0B */
		{ 0x00033923, 0x00a5a5f0 },                             /* F0 A5 A5 */
	};
	uint32_t newVfp = hz == 120 ? PANEL_VFP_120 : PANEL_VFP_60;
	/*
	 * Frame-locked as the vendor's GCE sequence (wait SOF, commands, EOF/SOF/EOF, porch after a SOF): a porch written
	 * while the DSI is inside the blanking gave one odd frame, a dark flash on the AMOLED (2026-11-07). The OVL frame
	 * end is the end of the active lines; the DSI's start of frame comes the old VFP (+ VSA + VBP) later.
	 */
	bool applied;
	uint64_t frame2, sofDelay, t;
	nanoseconds_to_absolutetime(40 * NSEC_PER_MSEC, &frame2);
	uint32_t lines = vfp + 12;                      /* VFP + VSA 2 + VBP 10 */
	nanoseconds_to_absolutetime((uint64_t)lines * 1266ull * 1000000ull / 369469ull + 500000ull, &sofDelay);
	bool locked = fVblankOvl >= 0;
	if (locked && (t = vblankPoll(frame2, NULL, &applied)) != 0) {
		while (mach_absolute_time() < t + sofDelay) {           /* into the next frame's active area */
			IODelay(20);
		}
	}
	if (fHzSeq == 2) {
		disp_wr(d, DSI_VFP_NL, newVfp);         /* seq 2: porch at the start of the frame, commands in its blanking */
	}
	for (int i = 0; i < 4; i++) {
		if (cmds[i][1] != 0) {
			disp_wr(d, DSI_VM_CMD_DATA0, cmds[i][1]);
		}
		disp_wr(d, DSI_VM_CMD_CON, cmds[i][0]);
		disp_wr(d, DSI_INTSTA, disp_rd(d, DSI_INTSTA) & ~DSI_VM_CMD_DONE);
		disp_wr(d, DSI_START, disp_rd(d, DSI_START) & ~DSI_VM_CMD_START);
		disp_wr(d, DSI_START, disp_rd(d, DSI_START) | DSI_VM_CMD_START);
		bool done = false;
		for (int k = 0; k < 1000 && !done; k++) {      /* a VFP comes at least every 16.7 ms */
			done = (disp_rd(d, DSI_INTSTA) & DSI_VM_CMD_DONE) != 0;
			if (!done) {
				IODelay(50);
			}
		}
		disp_wr(d, DSI_START, disp_rd(d, DSI_START) & ~DSI_VM_CMD_START);
		disp_wr(d, DSI_INTSTA, disp_rd(d, DSI_INTSTA) & ~DSI_VM_CMD_DONE);
		if (!done) {
			fHzWhy = "panel command not sent (VM_CMD_DONE timeout)";
			return false;
		}
		if (i == 2 && fHzSeq == 1) {
			disp_wr(d, DSI_VFP_NL, newVfp);         /* seq 1: porch in the blanking that carried F7 0B */
		}
	}
	/* seq 0, vendor send_cmd_need_delay: EOF, SOF, EOF; then the porch right after the following SOF */
	t = 0;
	for (int i = 0; locked && fHzSeq == 0 && i < 2; i++) {
		t = vblankPoll(frame2, NULL, &applied);
	}
	if (t != 0) {
		while (mach_absolute_time() < t + sofDelay) {
			IODelay(20);
		}
	} else if (fHzSeq == 0) {
		IOSleep(34);
	}
	if (fHzSeq == 0) {
		disp_wr(d, DSI_VFP_NL, newVfp);
	}
	fPanelHz = hz;
	fTickUs = hz == 120 ? 8333 : MAPPLE_FB_VSYNC_US;
	if (fFlipReady && !vblankCalibrate(NULL, NULL)) {
		fFlipReady = false;
		fFlipWhy = "no frame end after the refresh switch";
	}
	fHzSwitches++;
	IOLog("FB: panel %u Hz (frame %llu us)\n", hz, fPeriod ? absUsFB(fPeriod) : 0);
	return true;
}

/* 0189, vsync tick: the auto policy and a pending switch. */
void
AppleCLCD::hzPolicy(void)
{
	uint32_t want = fHzWant;
	if (fHzMode == 1 && fPanelHz == 120 && want == 0) {
		uint64_t idle;
		nanoseconds_to_absolutetime((uint64_t)fHzIdleMs * NSEC_PER_MSEC, &idle);
		if (fPowerOff || mach_absolute_time() - fLastActivity > idle) {
			want = 60;
		}
	}
	if (want == 0 || !fFlipReady) {
		return;                 /* switches wait for the frame end to be known (first swap) */
	}
	fHzWant = 0;
	if (want == 120 && (fRszWant || fRszLive)) {
		return;
	}
	if (want == fPanelHz || (fPowerOff && want == 120)) {
		return;
	}
	IOLockLock(fScanLock);
	if (!panelSetHz(want)) {
		fHzFailures++;
		if (fHzFailures <= 8) {
			IOLog("FB: panel %u Hz failed: %s\n", want, fHzWhy);
		}
		if (fHzMode == 1) {
			fHzMode = 60;           /* auto off after a failure */
		}
	}
	IOLockUnlock(fScanLock);
}

/* 0191: one DCS write (1-3 bytes, write or long write) in the video-mode blanking. fScanLock. */
bool
AppleCLCD::dsiSendDcs(const uint8_t *b, uint32_t n, bool vbp, bool wait)
{
	if (!dispMap() || n == 0 || n > 4) {
		return false;
	}
	vm_offset_t d = fDisp[4];
	if (fDcsInFlight) {
		/* the previous non-blocking write: still waiting for its blanking -> try again next tick */
		if (!(disp_rd(d, DSI_INTSTA) & DSI_VM_CMD_DONE) && (disp_rd(d, DSI_START) & DSI_VM_CMD_START)) {
			if (!wait) {
				return false;
			}
			for (int k = 0; k < 1000 && !(disp_rd(d, DSI_INTSTA) & DSI_VM_CMD_DONE); k++) {
				IODelay(50);
			}
		}
		fDcsInFlight = false;
	}
	uint32_t timing = vbp ? 0x18u : 0x20u;          /* [4:3] VSA/VBP or [5] VFP */
	uint32_t con;
	if (n <= 2) {
		con = ((n == 2 ? (uint32_t)b[1] : 0u) << 24) | ((uint32_t)b[0] << 16) | ((n == 2 ? 0x15u : 0x05u) << 8) | 1u | timing;
	} else {
		disp_wr(d, DSI_VM_CMD_DATA0, (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) |
		    (n > 3 ? (uint32_t)b[3] << 24 : 0u));
		con = (n << 16) | (0x39u << 8) | 2u | 1u | timing;
	}
	disp_wr(d, DSI_VM_CMD_CON, con);
	disp_wr(d, DSI_INTSTA, disp_rd(d, DSI_INTSTA) & ~DSI_VM_CMD_DONE);
	disp_wr(d, DSI_START, disp_rd(d, DSI_START) & ~DSI_VM_CMD_START);
	disp_wr(d, DSI_START, disp_rd(d, DSI_START) | DSI_VM_CMD_START);
	if (!wait) {
		fDcsInFlight = true;            /* goes out in the next blanking; checked at the next write */
		return true;
	}
	bool done = false;
	for (int k = 0; k < 1000 && !done; k++) {
		done = (disp_rd(d, DSI_INTSTA) & DSI_VM_CMD_DONE) != 0;
		if (!done) {
			IODelay(50);
		}
	}
	disp_wr(d, DSI_START, disp_rd(d, DSI_START) & ~DSI_VM_CMD_START);
	disp_wr(d, DSI_INTSTA, disp_rd(d, DSI_INTSTA) & ~DSI_VM_CMD_DONE);
	return done;
}

/* 0191, vsync tick: pending brightness level and color matrix. */
void
AppleCLCD::lightTick(void)
{
	uint32_t bl = fBlWant;
	bool matrix = fMatrixDirty;
	bool dark = fBlOff && fPowerOff && fFadeStep > 6;
	if (dark) {
		/* faded to black: backlight off, and what iOS asks for meanwhile kept for the display on */
		if (bl != 0xffffffffu) {
			fBlResume = bl;
			fBlWant = 0xffffffffu;
		}
		bl = fBlLevel != 0 ? 0 : 0xffffffffu;
	}
	if (bl == 0xffffffffu && !matrix) {
		return;
	}
	IOLockLock(fScanLock);
	if (bl != 0xffffffffu) {
		uint8_t cmd[3] = { 0x51, (uint8_t)(bl >> 8), (uint8_t)bl };
		if (bl == fBlLevel) {
			fBlWant = 0xffffffffu;
		} else if (dsiSendDcs(cmd, 3, false, false)) {      /* non-blocking: a fade must not hold the frames */
			if (fBlWant == bl) {
				fBlWant = 0xffffffffu;
			}
			fBlLevel = bl;
			fBlSent++;
			if (fBacklight != NULL && !dark) {
				fBacklight->publishLevel(bl);
			}
		}
	}
	if (matrix) {
		fMatrixDirty = false;
		/* product of the matrices in type order, fixed point 1.0 = 0x800 */
		int64_t m[9] = { 0x800, 0, 0, 0, 0x800, 0, 0, 0, 0x800 };
		for (int t = 0; t < 16; t++) {
			if (!fMatrixUsed[t]) {
				continue;
			}
			int64_t r[9];
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					int64_t v = 0;
					for (int k = 0; k < 3; k++) {
						v += (int64_t)fMatrix[t][i * 3 + k] * m[k * 3 + j];
					}
					r[i * 3 + j] = v / 0x800;
				}
			}
			memcpy(m, r, sizeof(m));
		}
		bool identity = true;
		int32_t c[9];
		for (int i = 0; i < 9; i++) {
			c[i] = (int32_t)MIN(MAX(m[i], -4096ll), 4095ll);
			identity = identity && c[i] == ((i % 4) == 0 ? 0x800 : 0);
		}
		fCcorr = dispMap() ? fDisp[5] : 0;
		if (fCcorr != 0) {
			disp_wr(fCcorr, CCORR_COEF(0), ((uint32_t)(c[0] & 0x1fff) << 16) | (uint32_t)(c[1] & 0x1fff));
			disp_wr(fCcorr, CCORR_COEF(1), ((uint32_t)(c[2] & 0x1fff) << 16) | (uint32_t)(c[3] & 0x1fff));
			disp_wr(fCcorr, CCORR_COEF(2), ((uint32_t)(c[4] & 0x1fff) << 16) | (uint32_t)(c[5] & 0x1fff));
			disp_wr(fCcorr, CCORR_COEF(3), ((uint32_t)(c[6] & 0x0fff) << 16) | (uint32_t)(c[7] & 0x1fff));
			disp_wr(fCcorr, CCORR_COEF(4), (uint32_t)(c[8] & 0x1fff) << 16);
			uint32_t cfg = disp_rd(fCcorr, CCORR_CFG);
			disp_wr(fCcorr, CCORR_CFG, identity ? ((cfg | 1u) & ~2u) : ((cfg & ~1u) | 2u));
			fMatrixApplied++;
		}
	}
	IOLockUnlock(fScanLock);
}

/* 0191: selector 55, a color matrix of the given type (row-major, signed 32.32 fixed point; seen on the phone). */
void
AppleCLCD::setMatrix(uint32_t type, const uint64_t bits[9])
{
	fMatrixCalls++;
	if (type >= 16) {
		return;
	}
	int32_t m[9];
	bool identity = true;
	for (int i = 0; i < 9; i++) {
		int64_t v = ((int64_t)bits[i] + (1ll << 20)) >> 21;     /* 32.32 -> 1.0 = 0x800 */
		m[i] = (int32_t)MIN(MAX(v, -0x4000ll), 0x4000ll);
		identity = identity && m[i] == ((i % 4) == 0 ? 0x800 : 0);
	}
	if (fMatrixCalls <= 16) {
		IOLog("FB: matrix %u: %d %d %d / %d %d %d / %d %d %d (1.0 = 2048)\n", type, m[0], m[1], m[2], m[3], m[4],
		    m[5], m[6], m[7], m[8]);
	}
	memcpy(fMatrix[type], m, sizeof(m));
	fMatrixUsed[type] = !identity;
	fMatrixDirty = true;
}

/* 0191: the AppleARMBacklight nub (see its class). */
void
AppleCLCD::backlightStart(void)
{
	AppleARMBacklight *b = OSTypeAlloc(AppleARMBacklight);
	if (b == NULL || !b->init()) {
		OSSafeReleaseNULL(b);
		return;
	}
	b->fFB = this;
	b->setProperty("backlight-control", true);
	b->publishLevel(0x219);         /* LK's level */
	if (!b->attach(this)) {
		b->release();
		return;
	}
	b->registerService();
	fBacklight = b;
}

void
AppleARMBacklight::publishLevel(uint32_t level)
{
	OSDictionary *params = OSDictionary::withCapacity(1), *bright = OSDictionary::withCapacity(3);
	OSNumber *mn = OSNumber::withNumber(0ull, 32), *mx = OSNumber::withNumber((unsigned long long)PANEL_LEVEL_MAX, 32);
	OSNumber *v = OSNumber::withNumber((unsigned long long)level, 32);
	if (params != NULL && bright != NULL && mn != NULL && mx != NULL && v != NULL) {
		bright->setObject("min", mn);
		bright->setObject("max", mx);
		bright->setObject("value", v);
		params->setObject("brightness", bright);
		setProperty("IODisplayParameters", params);
	}
	OSSafeReleaseNULL(mn);
	OSSafeReleaseNULL(mx);
	OSSafeReleaseNULL(v);
	OSSafeReleaseNULL(bright);
	OSSafeReleaseNULL(params);
}

/* A brightness number as CoreBrightness may send it: a number, or a dictionary with "value". */
static bool
mapple_fb_brightness_value(OSObject *o, uint64_t *v)
{
	OSNumber *n = OSDynamicCast(OSNumber, o);
	OSDictionary *d = OSDynamicCast(OSDictionary, o);
	if (n == NULL && d != NULL) {
		n = OSDynamicCast(OSNumber, d->getObject("value"));
	}
	if (n == NULL) {
		return false;
	}
	*v = n->unsigned64BitValue();
	return true;
}

IOReturn
AppleARMBacklight::setProperties(OSObject *properties)
{
	OSDictionary *dict = OSDynamicCast(OSDictionary, properties);
	if (dict == NULL || fFB == NULL) {
		return kIOReturnBadArgument;
	}
	fFB->traceProperties("backlight", dict);
	uint64_t v = 0;
	OSDictionary *params = OSDynamicCast(OSDictionary, dict->getObject("IODisplayParameters"));
	bool have = mapple_fb_brightness_value(dict->getObject("brightness"), &v) ||
	    (params != NULL && mapple_fb_brightness_value(params->getObject("brightness"), &v));
	if (!have) {
		return kIOReturnSuccess;        /* accepted (logged), nothing for the panel */
	}
	uint32_t level = (uint32_t)MIN(v, (uint64_t)PANEL_LEVEL_MAX);
	fFB->fBlWant = level;
	publishLevel(level);
	return kIOReturnSuccess;
}

/* 0191, "disp seltrace on": every property a client sets on us or the backlight nub. */
void
AppleCLCD::traceProperties(const char *who, OSDictionary *dict)
{
	if (!fSelTrace || dict == NULL || fTraceLines >= 400) {
		return;
	}
	OSCollectionIterator *it = OSCollectionIterator::withCollection(dict);
	OSSymbol *k;
	char name[32];
	proc_selfname(name, sizeof(name));
	while (it != NULL && (k = OSDynamicCast(OSSymbol, it->getNextObject())) != NULL) {
		OSObject *o = dict->getObject(k);
		OSNumber *v = OSDynamicCast(OSNumber, o);
		fTraceLines++;
		IOLog("FB: trace %s %s set %s = %s%llu\n", name, who, k->getCStringNoCopy(),
		    v ? "" : OSDynamicCast(OSDictionary, o) ? "(dictionary) " : "(other) ", v ? v->unsigned64BitValue() : 0);
		OSDictionary *sub = OSDynamicCast(OSDictionary, o);
		if (sub != NULL) {
			traceProperties(who, sub);
		}
	}
	OSSafeReleaseNULL(it);
}

/* 0191: CoreBrightness's IOMFBBrightnessLevel (nits << 16). */
IOReturn
AppleCLCD::setProperties(OSObject *properties)
{
	OSDictionary *dict = OSDynamicCast(OSDictionary, properties);
	traceProperties("framebuffer", dict);
	OSNumber *n = dict != NULL ? OSDynamicCast(OSNumber, dict->getObject("IOMFBBrightnessLevel")) : NULL;
	if (n == NULL) {
		return kIOReturnUnsupported;
	}
	uint64_t nits16 = n->unsigned32BitValue();
	uint64_t level = (nits16 * PANEL_LEVEL_MAX + ((uint64_t)fMaxNits << 15)) / ((uint64_t)fMaxNits << 16);
	if (nits16 != 0 && level == 0) {
		level = 1;
	}
	fBlWant = (uint32_t)MIN(level, (uint64_t)PANEL_LEVEL_MAX);
	setProperty("IOMFBBrightnessLevel", n);
	return kIOReturnSuccess;
}

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
		for (uint32_t k = 1; k < MAPPLE_FB_LK_BUFFERS; k++) {
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
		IOLog("FB: double buffer off (%s): copies go to the buffer on screen\n", why);
		disp_say(out, end, "double buffer off: %s\n", why);
		return false;
	}
	uint64_t ns;
	absolutetime_to_nanoseconds(fPeriod, &ns);
	IOLog("FB: double buffer: 0x%llx / 0x%llx, frame end from %s INTSTA%s, panel frame %llu us, vsync every %u "
	    "frame(s)\n", fBufPA[0], fBufPA[1], fVblankOvl ? "OVL0_2L" : "OVL0", fVblankInten ? " (INTEN set)" : "",
	    ns / 1000, fTickFrames);
	disp_say(out, end, "double buffer on: 0x%llx / 0x%llx, frame %llu us\n", fBufPA[0], fBufPA[1], ns / 1000);
	fMisses = 0;
	fResync = false;
	fFlipReady = true;
	if (fOneLayer) {
		oneLayer(true, out, end);
	}
	return true;
}

/*
 * fScanLock. Other enabled OVL layers that read any of LK's buffers, switched off (on) or back on (off), and
 * LK's layer given the pitch the buffers are written with (fRowBytes, the boot framebuffer's). On the realme 8i
 * mtkloader's OVL0 L1 (pitch 4320 = fRowBytes) sits above LK's OVL0_2L L0 (pitch 4352) on buffer 0: with double
 * buffering only every other frame is visible, the half-written one as well, and L0 shows through it sheared.
 */
void
AppleCLCD::oneLayer(bool on, char **out, char *end)
{
	uint64_t lo = fPhys, hi = fPhys + 3ull * fRowBytes * fHeight;     /* LK's buffers, as findLKLayer */
	if (on && fOneLkPitch == 0 && (fLkPitch & 0xffff) != fRowBytes) {
		fOneLkPitch = fLkPitch;
		fLkPitch = (fLkPitch & ~0xffffu) | fRowBytes;
	} else if (!on && fOneLkPitch != 0) {
		fLkPitch = fOneLkPitch;
		fOneLkPitch = 0;
	}
	if (fShown.buf >= 0) {          /* the buffer on screen now, at the new pitch; the next switch uses it too */
		fShown.pitch = fLkPitch;
		layerWrite(fShown);
	}
	disp_say(out, end, "LK's layer pitch 0x%x\n", fLkPitch & 0xffff);
	for (int o = 0; o < 2; o++) {
		vm_offset_t b = fDisp[o];
		uint32_t con = disp_rd(b, OVL_SRC_CON), off = 0;
		if (!on) {
			if (fOneOff[o]) {
				disp_wr(b, OVL_SRC_CON, con | fOneOff[o]);
				disp_say(out, end, "%s SRC_CON 0x%x -> 0x%x\n", o ? "OVL0_2L" : "OVL0", con, con | fOneOff[o]);
				fOneOff[o] = 0;
			}
			continue;
		}
		if (!(disp_rd(b, OVL_EN) & 1)) {
			continue;
		}
		for (int n = 0; n < (o ? 2 : 4); n++) {
			uint32_t addr = disp_rd(b, OVL_L_ADDR(n));
			if ((con & (1u << n)) && addr >= lo && addr < hi && !(o == fScanOvl && OVL_L_ADDR(n) == fScanAddrReg)) {
				off |= 1u << n;
			}
		}
		if (off) {
			disp_wr(b, OVL_SRC_CON, con & ~off);
			fOneOff[o] |= off;
			IOLog("FB: one layer: %s SRC_CON 0x%x -> 0x%x\n", o ? "OVL0_2L" : "OVL0", con, con & ~off);
			disp_say(out, end, "%s SRC_CON 0x%x -> 0x%x\n", o ? "OVL0_2L" : "OVL0", con, con & ~off);
		}
	}
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
				nanoseconds_to_absolutetime((uint64_t)fTickUs * 1000, &tick);
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
			IOLog("FB: double buffer off: no frame end in 8 tries\n");
			if (fRszLive) {
				fRszWhy = "frame ends stopped with the resizer in the pipe: back to LK's path";
				IOLog("FB: rsz: %s\n", fRszWhy);
			}
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
			mapple_surface_scanout_w = fLogicalW;
			mapple_surface_scanout_h = fLogicalH;
			if (fPoolWant == 0) {
				fPoolWant = 4;
				PE_parse_boot_argn_alias("mp_fb_pool", "vivafbpool", &fPoolWant, sizeof(fPoolWant));
			}
			if (fPoolWant != 0) {
				IOLockUnlock(fScanLock);        /* allocates (may take a while): never under the display locks */
				mapple_surface_pool_init(fLogicalW, fLogicalH, fPoolWant);
				IOLockLock(fScanLock);
			}
			fDirty = true;
			disp_say(&p, end, "scanout on (surfaces created from now on can be scanned out; CA allocates its "
			    "framebuffers at start: boot with mp_fb_scanout=1)\n");
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
		bzero(fSwapGapHist, sizeof(fSwapGapHist));
		bzero(fLatWait, sizeof(fLatWait));
		bzero(fWaitBusyWhy, sizeof(fWaitBusyWhy));
		bzero(fWaitSleptWhy, sizeof(fWaitSleptWhy));
		fSwapWaitBusy = fSwapWaitSlept = 0;
		fBusySwap = 0;
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
	} else if (strncmp(args, "onelayer on", 11) == 0) {
		if (!fFlipReady) {
			disp_say(&p, end, "one layer: needs the double buffer (%s)\n", fFlipWhy ? fFlipWhy : "off");
		} else {
			fOneLayer = true;
			oneLayer(true, &p, end);
			fDirty = true;
			disp_say(&p, end, "one layer on\n");
		}
	} else if (strncmp(args, "onelayer off", 12) == 0) {
		fOneLayer = false;
		oneLayer(false, &p, end);
		fDirty = true;
		disp_say(&p, end, "one layer off: the other layers back as they were\n");
	} else if (strncmp(args, "surfpitch", 9) == 0) {
		const char *a = args + 9;
		while (*a == ' ') {
			a++;
		}
		int m = mapple_surface_pitch_mode(*a >= '0' && *a <= '2' ? *a - '0' : -1);
		disp_say(&p, end, "mp_surface_pitch64 %d for new surfaces (0 never, 1 default, 2 also explicit sizes); %llu rounded so "
		    "far\n", m, mapple_surface_pitch_rounded());
	} else if (strncmp(args, "hz", 2) == 0) {
		const char *a = args + 2;
		while (*a == ' ') {
			a++;
		}
		if (strncmp(a, "120", 3) == 0) {
			fHzMode = 120;
			fHzWant = 120;
		} else if (strncmp(a, "60", 2) == 0) {
			fHzMode = 60;
			fHzWant = 60;
		} else if (strncmp(a, "auto", 4) == 0) {
			fHzMode = 1;
			fHzWant = 120;
		} else if (strncmp(a, "seq ", 4) == 0 && a[4] >= '0' && a[4] <= '2') {
			fHzSeq = (uint32_t)(a[4] - '0');
		} else if (strncmp(a, "idle ", 5) == 0) {
			uint32_t ms = 0;
			for (a += 5; *a >= '0' && *a <= '9'; a++) {
				ms = ms * 10 + (uint32_t)(*a - '0');
			}
			fHzIdleMs = MIN(MAX(ms, 100u), 60000u);
		}
		disp_say(&p, end, "panel %u Hz, mode %s (idle %u ms, seq %u)%s, %llu switches, %llu failures%s%s\n", fPanelHz,
		    fHzMode == 1 ? "auto" : fHzMode == 120 ? "120" : "60", fHzIdleMs, fHzSeq, fHzWant ? " (switch pending)" : "", fHzSwitches,
		    fHzFailures, fHzWhy ? ", last: " : "", fHzWhy ? fHzWhy : "");
	} else if (strncmp(args, "seltrace", 8) == 0) {
		fSelTrace = strncmp(args + 8, " on", 3) == 0;
		fTraceLines = 0;
		bzero(fTraceRec, sizeof(fTraceRec));
		disp_say(&p, end, "user client trace %s (log: \"FB: trace\", at most 400 lines)\n", fSelTrace ? "on" : "off");
	} else if (strncmp(args, "bl", 2) == 0) {
		const char *a = args + 2;
		while (*a == ' ') {
			a++;
		}
		if (*a >= '0' && *a <= '9') {
			uint32_t v = 0;
			for (; *a >= '0' && *a <= '9'; a++) {
				v = v * 10 + (uint32_t)(*a - '0');
			}
			fBlWant = MIN(v, (uint32_t)PANEL_LEVEL_MAX);
		}
		disp_say(&p, end, "panel brightness level %u of %u (%u nits max), %llu sent; color matrices %llu calls, %llu applied\n",
		    fBlLevel, PANEL_LEVEL_MAX, fMaxNits, fBlSent, fMatrixCalls, fMatrixApplied);
	} else if (strncmp(args, "rsz", 3) == 0) {
		if (fRszArg == 0) {
			disp_say(&p, end, "rsz: boot with mp_fb_rsz=1 (read only) or 2 (the resizer)\n");
		} else {
			if (strncmp(args + 3, " off", 4) == 0) {
				fRszWant = false;       /* the next frame goes through LK's path and the CPU letterbox */
				fDirty = true;
			} else if (strncmp(args + 3, " on", 3) == 0 && fRszArg >= 2 && fLb.w != 0) {
				fRszWant = true;
				fDirty = true;
			}
			rszDump(&p, end);
		}
	} else if (strncmp(args, "kv", 2) == 0) {
		/* KA1 (gpu/docs/RICHIESTE-KERNEL.md): stable key=value lines for Viva Manager */
		IOLockLock(fLock);
		uint64_t r = absUsFB(fLatReady.n ? fLatReady.sum / fLatReady.n : 0);
		uint64_t f = absUsFB(fLatFlip.n ? fLatFlip.sum / fLatFlip.n : 0);
		uint64_t swaps = fLatSwap.n;
		IOLockUnlock(fLock);
		disp_say(&p, end, "width=%u\nheight=%u\nlogical_width=%u\nlogical_height=%u\nscanout=%d\nscanout_live=%d\n",
		    fWidth, fHeight, fLogicalW, fLogicalH, fScanOn ? 1 : 0, fShown.buf < 0 ? 1 : 0);
		disp_say(&p, end, "frames_scanned=%llu\nframes_copied=%llu\nswaps=%llu\ngpu_waits=%llu\nframes_forced=%llu\n"
		    "copies=%llu\ncopy_avg_us=%llu\n", fScanFrames, fScanFallbacks, swaps, fGpuWaits, fGpuForced, fCopies,
		    fCopies ? fCopyNs / fCopies / 1000 : 0);
		disp_say(&p, end, "lat_swap_gpu_avg_us=%llu\nlat_chosen_screen_avg_us=%llu\nearly_shown=%llu\nflip=%d\n"
		    "swapwait_busy=%llu\n", r, f, fEarlyShown, fFlipReady ? 1 : 0, fSwapWaitBusy);
		disp_say(&p, end, "panel_hz=%u\nhz_mode=%u\nhz_switches=%llu\nhz_failures=%llu\n", fPanelHz, fHzMode,
		    fHzSwitches, fHzFailures);
		disp_say(&p, end, "brightness=%u\nbrightness_max=%u\nmatrix_calls=%llu\n", fBlLevel, PANEL_LEVEL_MAX, fMatrixCalls);
		{
			uint32_t pb = 0, pu = 0, pk = 0;
			uint64_t fp = 0, fc = 0, sp = 0;
			mapple_surface_pool_stats(&pb, &pu, &fp, &fc, &sp, &pk);
			disp_say(&p, end, "pool_buffers=%u\npool_in_use=%u\npages_pool=%llu\npages_contig=%llu\npages_sparse=%llu\n"
			    "pitch_rounded=%llu\n", pb, pu, fp, fc, sp, mapple_surface_pitch_rounded());
			disp_say(&p, end, "swapwait_busy_queued=%llu\nswapwait_busy_next=%llu\nswapwait_busy_shown=%llu\n"
			    "swapwait_slept=%llu\n", fWaitBusyWhy[WAIT_QUEUED], fWaitBusyWhy[WAIT_NEXT], fWaitBusyWhy[WAIT_SHOWN],
			    fSwapWaitSlept);
		}
	} else {
		disp_say(&p, end, "display %ux%u (shown as %ux%u), scanout %s%s, layer %s, %llu frames scanned out, "
		    "%llu copied%s%s\n", fWidth, fHeight, fLogicalW, fLogicalH, fScanOn ? "on" : "off",
		    fShown.buf < 0 ? " (active)" : "", fScanOvl >= 0 ? fScanLayer : "?", fScanFrames, fScanFallbacks,
		    fScanWhy ? ", last reason: " : "", fScanWhy ? fScanWhy : "");
		disp_say(&p, end, "copies: %llu, avg %llu us, refresh %s%u\n", fCopies, fCopies ? fCopyNs / fCopies / 1000 : 0,
		    fFlushTicks ? "every N vsyncs, N = " : "adaptive, CPU duty % = ", fFlushTicks ? fFlushTicks : fDuty);
		disp_say(&p, end, "GPU sync (K0c): %llu vsyncs waited for the GPU, %llu copies redone, %llu frames shown anyway "
		    "(mp_fb_gpuwait=%u)\n", fGpuWaits, fGpuRecopies, fGpuForced, fGpuWaitMax);
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
			disp_say(&p, end, "SwapEnd to SwapEnd: <=17 ms %llu, <=34 %llu, <=50 %llu, <=100 %llu, more %llu\n",
			    fSwapGapHist[0], fSwapGapHist[1], fSwapGapHist[2], fSwapGapHist[3], fSwapGapHist[4]);
			disp_say(&p, end, "ticks with a switch pending (0136): %llu late (avg %llu max %llu us after the frame end), "
			    "%llu early (avg %llu us before it), %llu polled up to 3 ms ahead\n", fTickLate,
			    absUsFB(fTickLate ? fTickLateSum / fTickLate : 0), absUsFB(fTickLateMax), fTickEarly,
			    absUsFB(fTickEarly ? fTickEarlySum / fTickEarly : 0), fTickWidePolls);
		}
		disp_say(&p, end, "early present (K0g, mp_fb_early) %s: %llu tries, %llu frames shown before their tick\n",
		    fEarly ? "on" : "off", fEarlyRuns, fEarlyShown);
		disp_say(&p, end, "SwapWait: %llu busy answers (CA keeps the page), %llu blocking waits; layer reads swap %u, "
		    "next swap %u\n", fSwapWaitBusy, fSwapWaitSlept, fShownSwap, fNextSwap);
		{
			static const char *const kWhy[WAIT_WHY] = { "queued (GPU/copy)", "chosen, switch pending", "on screen", "other" };
			uint64_t swaps = fLatSwap.n ? fLatSwap.n : 1;
			disp_say(&p, end, "SwapWait not done (0184), per 100 swaps busy/blocking, first not-done -> done avg/max us:");
			for (int w = 0; w < WAIT_WHY; w++) {
				disp_say(&p, end, "%s %s %llu/%llu %llu/%llu", w ? ";" : "", kWhy[w], fWaitBusyWhy[w] * 100 / swaps,
				    fWaitSleptWhy[w] * 100 / swaps, absUsFB(fLatWait[w].n ? fLatWait[w].sum / fLatWait[w].n : 0),
				    absUsFB(fLatWait[w].max));
			}
			disp_say(&p, end, "\n");
			uint32_t pb = 0, pu = 0, pk = 0;
			uint64_t fp = 0, fc = 0, sp = 0;
			mapple_surface_pool_stats(&pb, &pu, &fp, &fc, &sp, &pk);
			disp_say(&p, end, "display pages (0184): pool %u x %u KiB (%u in use, mp_fb_pool), created from the pool %llu, "
			    "contiguous %llu, sparse %llu; client pitches rounded to 64: %llu\n", pb, pk, pu, fp, fc, sp,
			    mapple_surface_pitch_rounded());
		}
		uint64_t pns = 0, polls = fPolls ? fPolls : 1, pollNs;
		absolutetime_to_nanoseconds(fPeriod, &pns);
		absolutetime_to_nanoseconds(fPollAbs / polls, &pollNs);
		disp_say(&p, end, "double buffer (K0d) %s%s%s: on screen %s%d, frame %llu us, %llu switches at a frame end, %llu "
		    "in place, %llu missed, %llu unsure, %llu deferred; poll avg %llu us\n", fFlipReady ? "on" : "off",
		    fFlipWhy ? ": " : "", fFlipWhy ? fFlipWhy : "", fShown.buf < 0 ? "surface" : "buffer ",
		    fShown.buf < 0 ? 0 : fShown.buf, pns / 1000, fFlips, fInPlace, fFlipMissed, fFlipUnsure, fFlipEarly,
		    pollNs / 1000);
		disp_say(&p, end, "one layer (mp_fb_onelayer) %s: switched off OVL0 0x%x, OVL0_2L 0x%x\n", fOneLayer ? "on" : "off",
		    fOneOff[0], fOneOff[1]);
		disp_say(&p, end, "panel refresh (0189): %u Hz, mode %s, %llu switches, %llu failures\n", fPanelHz,
		    fHzMode == 1 ? "auto" : fHzMode == 120 ? "120" : "60", fHzSwitches, fHzFailures);
		if (fRszArg != 0) {
			disp_say(&p, end, "resizer (mp_fb_rsz=%u, disp rsz [on|off]): %s%s%s%s, %llu switches to it\n", fRszArg,
			    fRszLive ? "in the pipe" : fRszReady ? "ready" : "not prepared", fRszWant ? "" : " (off)",
			    fRszWhy ? ", off: " : "", fRszWhy ? fRszWhy : "", fRszSwitches);
		}
		disp_say(&p, end, "usage: disp [regs|scanout on|scanout off|flip on|flip off|early on|early off|onelayer on|onelayer off|surfpitch 0|1|2|"
		    "lat reset|hz 60|120|auto]\n");
	}
	IOLockUnlock(fScanLock);
}

extern "C" void
mapple_disp_link(const char *args, char *out, size_t outLen)
{
	if (gMappleFB == NULL) {
		snprintf(out, outLen, "disp: no AppleCLCD\n");
		return;
	}
	gMappleFB->dispCommand(args, out, outLen);
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

	if (type >= MAPPLE_FB_NOTIFY_TYPES || fFunc[type] == 0 || fPort == MACH_PORT_NULL) {
		return;
	}
	bzero(ref, sizeof(ref));
	setAsyncReference64(ref, fPort, (mach_vm_address_t)fFunc[type], fRefcon[type], fTask);
	sendAsyncResult64WithOptions(ref, kIOReturnSuccess, (io_user_reference_t *)values, count,
	    kIOUserNotifyOptionCanDrop);
}

static uint64_t mapple_fb_logged;        /* selectors already logged (< 64) */

IOReturn
IOMobileFramebufferUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    __unused IOExternalMethodDispatch *dispatch, __unused OSObject *target,
    __unused void *reference)
{
	IOReturn ret = kIOReturnSuccess;
	uint64_t out[2] = { 0, 0 };
	uint32_t outCount = 0;

	if (fFB->fSelTrace && fFB->fTraceLines < 400) {
		/* 0191: what QuartzCore / CoreBrightness call (swap records: only words that changed past the layer fields) */
		char name[32];
		proc_selfname(name, sizeof(name));
		if (selector == kMappleFBSwapEnd && args->structureInputDescriptor == NULL && args->structureInputSize >= MAPPLE_FB_SWAP_REC_SIZE) {
			const uint8_t *r = (const uint8_t *)args->structureInput;
			for (uint32_t o = 0x108; o + 4 <= MAPPLE_FB_SWAP_REC_SIZE && fFB->fTraceLines < 400; o += 4) {
				uint32_t v, was;
				memcpy(&v, r + o, 4);
				memcpy(&was, fFB->fTraceRec + o, 4);
				if (v != was) {
					fFB->fTraceLines++;
					IOLog("FB: trace %s swap +0x%x 0x%08x -> 0x%08x\n", name, o, was, v);
				}
			}
			memcpy(fFB->fTraceRec, r, MAPPLE_FB_SWAP_REC_SIZE);
		} else if (selector != kMappleFBSwapBegin && selector != kMappleFBSwapWait) {
			uint32_t w[8] = { 0 };
			if (args->structureInputDescriptor == NULL && args->structureInput != NULL) {
				memcpy(w, args->structureInput, MIN(args->structureInputSize, (uint32_t)sizeof(w)));
			}
			fFB->fTraceLines++;
			IOLog("FB: trace %s sel %u scalars %u [0x%llx 0x%llx 0x%llx] struct %u [%08x %08x %08x %08x %08x %08x %08x %08x] out %u/%u\n",
			    name, selector, args->scalarInputCount, args->scalarInputCount > 0 ? args->scalarInput[0] : 0,
			    args->scalarInputCount > 1 ? args->scalarInput[1] : 0, args->scalarInputCount > 2 ? args->scalarInput[2] : 0,
			    args->structureInputSize, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7], args->scalarOutputCount,
			    args->structureOutputSize);
		}
	}

	switch (selector) {
	case kMappleFBGetLayerDefaultSurface:
		out[0] = mapple_surface_port_in_task(fFB->fSurface, fTask);
		outCount = 1;
		break;
	case kMappleFBSwapBegin:
		IOLockLock(fFB->fLock);
		out[0] = ++fFB->fSwapID;
		IOLockUnlock(fFB->fLock);
		outCount = 1;
		break;
	case kMappleFBSwapEnd: {
		uint8_t rec[MAPPLE_FB_SWAP_REC_SIZE];
		if (args->structureInputDescriptor != NULL ||
		    args->structureInputSize < MAPPLE_FB_SWAP_REC_SIZE) {
			ret = kIOReturnBadArgument;
			break;
		}
		memcpy(rec, args->structureInput, sizeof(rec));
		fFB->swap(rec);
		break;
	}
	case kMappleFBGetID:
		out[0] = 0;
		outCount = 1;
		break;
	case kMappleFBGetDisplaySize:
		out[0] = fFB->fLogicalW;
		out[1] = fFB->fLogicalH;
		outCount = 2;
		break;
	case kMappleFBIsMainDisplay:
		out[0] = 1;
		outCount = 1;
		break;
	case kMappleFBGetDotPitch:
		out[0] = MAPPLE_FB_DOT_PITCH;
		outCount = 1;
		break;
	case kMappleFBNotifications:
		if (args->scalarInputCount >= 3 && args->scalarInput[2] < MAPPLE_FB_NOTIFY_TYPES) {
			uint32_t type = (uint32_t)args->scalarInput[2];
			IOLockLock(fFB->fLock);
			fFunc[type] = args->scalarInput[0];
			fRefcon[type] = args->scalarInput[1];
			IOLockUnlock(fFB->fLock);
			char name[32];
			proc_selfname(name, sizeof(name));
			IOLog("FB: %s notifications type %u %s\n", name, type,
			    args->scalarInput[0] ? "on" : "off");
		}
		break;
	case kMappleFBRequestPowerChange:
		if (args->scalarInputCount >= 1) {
			bool off = args->scalarInput[0] == 0;
			IOLockLock(fFB->fLock);
			if (off != fFB->fPowerOff) {
				fFB->fPowerOff = off;
				fFB->fFadeStep = 0;
				fFB->fDirty = true;     /* on: redraw the latest frame */
				if (off) {
					fFB->fBlResume = fFB->fBlWant != 0xffffffffu ? fFB->fBlWant : fFB->fBlLevel;
				} else if (fFB->fBlOff && fFB->fBlLevel == 0 && fFB->fBlWant == 0xffffffffu) {
					fFB->fBlWant = fFB->fBlResume;
				}
			}
			IOLockUnlock(fFB->fLock);
			IOLog("FB: display %s\n", off ? "off" : "on");
		}
		break;
	case 55: {      /* 0191: IOMobileFramebufferSetMatrix(type, 3x3 signed 32.32) */
		if (args->scalarInputCount >= 1 && args->structureInputDescriptor == NULL && args->structureInputSize >= 72) {
			uint64_t bits[9];
			memcpy(bits, args->structureInput, sizeof(bits));
			fFB->setMatrix((uint32_t)args->scalarInput[0], bits);
		}
		break;
	}
	case kMappleFBSwapWait: {
		uint32_t swap = args->scalarInputCount > 0 ? (uint32_t)args->scalarInput[0] : 0;
		uint32_t opts = args->scalarInputCount > 1 ? (uint32_t)args->scalarInput[1] : 0;
		uint32_t ms = args->scalarInputCount > 2 ? (uint32_t)args->scalarInput[2] : 0;
		uint32_t op = opts & 7;
		if (fFB->swapDone(swap, op)) {
			fFB->waitResult(swap, true);
			break;
		}
		if (opts & 0x80000000u) {
			ret = kIOReturnBusy;
			fFB->fSwapWaitBusy++;
			fFB->waitResult(swap, false);
			break;
		}
		/* block (the calling thread only): re-check every millisecond, at most the timeout or 5 s */
		uint32_t why = fFB->waitWhy(swap);
		uint64_t t0 = mach_absolute_time();
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
		fFB->fWaitSleptWhy[why]++;
		if (ret == kIOReturnSuccess) {
			IOLockLock(fFB->fLock);
			AppleCLCD::latAdd(&fFB->fLatWait[why], mach_absolute_time() - t0);
			IOLockUnlock(fFB->fLock);
		}
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
	if (selector >= 64 || !(mapple_fb_logged & (1ULL << selector))) {
		if (selector < 64) {
			mapple_fb_logged |= 1ULL << selector;
		}
		char name[32];
		proc_selfname(name, sizeof(name));
		IOLog("FB: %s sel %u in %u/%u out %u/%u -> 0x%x\n", name, selector,
		    args->scalarInputCount, args->structureInputSize,
		    args->scalarOutputCount, args->structureOutputSize, ret);
	}
	return ret;
}
#endif /* MT6781 */
