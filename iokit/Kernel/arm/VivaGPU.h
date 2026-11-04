/*
 * viva (MT6781): Mali-G57 MC2 GPU (Valhall v9, job manager) for the
 * Metal driver (VivaMTLDriver.bundle, Mesa Panfrost in userspace).
 *
 * VivaGPUAccelerator (/arm-io/gpu, "mediatek,mt6781-mali") powers the MFG
 * subsystem (MT6366 rails through VivaPwrap, MFGPLL, mfg_sel, MTCMOS
 * MFG0-3, clock gate), brings up the Mali (soft reset, L2 and cores),
 * runs its MMU (one AArch64 4K address space per connection) and its job
 * slots, and publishes itself as an IOAcceleratorES with the Metal plugin
 * properties. VivaGPUUserClient implements the kernel/userspace contract
 * of viva_gpu_uapi.h (the semantics of Linux's DRM_PANFROST_* ioctls and
 * DRM syncobjs, plus IOSurface and user memory import).
 *
 * Boot-arg vivagpu: 0 (default) the driver does not start; 1 it starts
 * unpowered for bring-up from the USB link ("gpu ..." commands); 2 it
 * also powers the GPU at boot and publishes the accelerator for userspace.
 *
 * Locking: everything runs under the accelerator's work-loop gate (the
 * GPU, MMU and job interrupts, the job timer, user client calls, link
 * commands). Waits sleep on the gate with commandSleep(). The one exception
 * is the job interrupt's filter (primary interrupt context, K0e fast kick):
 * it only touches fArm/fIrqDone/fIrqAt and the job slot registers, under
 * fFastLock (a spinlock taken with interrupts off on the gated side). The per-selector call counters ("gpu calls")
 * have their own spinlock, fCallLock: the time of a call is known only after it left the gate.
 */
#ifndef _IOKIT_ARM_VIVAGPU_H
#define _IOKIT_ARM_VIVAGPU_H

#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOCommandGate.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOLocks.h>

#include "viva_gpu_uapi.h"

class VivaPwrap;

/*
 * CPU cache maintenance (to PoC): clean (dc cvac) or flush = clean + invalidate (dc civac), before the GPU reads
 * and after it wrote. No plain invalidate (dc ivac): userspace keeps writing other parts of a BO while a job that
 * uses it runs, and dropping those dirty lines loses its writes (stale job descriptors: GPU faults, missing tiles).
 */
enum { kVivaCacheClean, kVivaCacheFlush };
enum { kVivaK0jDepth = 512 };          /* dependsOnPromiseOf stack frames (= its largest budget) */
/*
 * Userspace's CLOCK_MONOTONIC in ns, the clock of every absolute timeout of the UAPI (Mesa: os_time_get_nano). On
 * Darwin it is NOT mach_continuous_time: Libc computes it as gettimeofday - kern.boottime, so it follows the calendar
 * (set by timed/NTP, slewed) and drifts from Mach time. Same formula here, from the same kernel values.
 */
uint64_t viva_gpu_monotonic_ns(void);
void cacheBuffer(IOBufferMemoryDescriptor *md, uint64_t offset, uint64_t len, int op);
class VivaGPUAccelerator;
class VivaGPUBO;
class VivaGPUVM;
class SweetSurface;

/* Metal of iOS 16 looks GPUs up with IOServiceMatching("IOAcceleratorES") (IOGPUFamily is not in this kernel). */
class IOAcceleratorES : public IOService {
	OSDeclareDefaultStructors(IOAcceleratorES);
};

/*
 * Completion of one job chain (or a syncobj signalled by hand). Threads waiting for one fence sleep on the fence
 * itself (VivaGPUAccelerator::sleepOn), so a job's completion wakes only them, not every waiter of every process.
 */
class VivaGPUFence : public OSObject {
	OSDeclareDefaultStructors(VivaGPUFence);
public:
	static VivaGPUFence *create(bool signalled);
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	bool     signalled;
	IOReturn error;
	uint64_t signalAt;      /* mach_absolute_time of the job's interrupt (0: signalled by hand or not yet) */
	uint32_t waiters;       /* threads in sleepOn() for it (gated) */
	/* K0i promise (SURFACE_PROMISE): signals when the fence it is linked to does, or at `deadline` */
	bool       promise;
	VivaGPUVM *owner;       /* connection that made it (compared only, not retained) */
	uint64_t   deadline;    /* mach_absolute_time */
	OSArray   *followers;   /* promises linked to this fence: signalled with it */
	/* K0j: what it waits for while unsignalled (its job's dependencies; a resolved promise: the fence it follows) */
	OSArray   *deps;
	/* K0j cache: "depends on a pending promise of k0jVM", valid while k0jEpoch == the accelerator's fK0jEpoch */
	uint64_t   k0jEpoch;
	VivaGPUVM *k0jVM;
	bool       k0jDep;
	uint64_t   k0jWalk;     /* last dependsOnPromiseOf walk that reached it (0128: each node once per walk) */
	uint64_t   k0jCollect;  /* last collectWait that reached it */
};

/* A GPU address space: AArch64 4K page tables plus the VA allocator for [VGPU_VA_START, VGPU_VA_END). */
struct VivaGPUJob;

class VivaGPUVM : public OSObject {
	OSDeclareDefaultStructors(VivaGPUVM);
public:
	static VivaGPUVM *create(VivaGPUAccelerator *gpu);
	virtual void free(void) APPLE_KEXT_OVERRIDE;

	/* VA: first fit, `align` a power of two >= 16 KiB. 0 = no room. */
	uint64_t allocVA(uint64_t size, uint64_t align);
	void freeVA(uint64_t va);
	/* Map `size` bytes of `md` from `offset` (16 KiB multiples) at `va`; pte = attribute bits for the leaves. */
	bool map(uint64_t va, IOMemoryDescriptor *md, uint64_t offset, uint64_t size, uint64_t pte);
	void unmap(uint64_t va, uint64_t size);
	/* Leaf attributes for ordinary GPU memory (read/write, GPU-cacheable), executable or not. */
	static uint64_t leafAttrs(bool exec);
	/* MMU fault report: what `addr` is (live BO and its PTE, VA freed lately, outside the range). Gated. */
	void describe(uint64_t addr, char *buf, size_t len);

	VivaGPUAccelerator *gpu;
	uint64_t  rootPA;               /* level 0 table */
	int       asSlot;               /* hardware address space, -1 = none */
	uint32_t  running;              /* jobs on the hardware with this VM */
	uint64_t  lastUse;              /* LRU stamp for address-space eviction */
	uint32_t  faults;
	bool      asFaulted;            /* its AS was disabled by a fault: reprogram before the next job */
	/* accounting ("gpu status", limits per process) */
	int        pid;
	char       proc[20];
	int64_t    bytes;               /* GPU memory charged: BOs, heap chunks, page tables */
	int64_t    imported;            /* IOSurface / user memory mapped (not charged) */
	uint32_t   bos;
	VivaGPUVM *nextVM;              /* the accelerator's list of live VMs */
	/*
	 * K0h completion page: jobs numbered per connection (seq), unfinished ones in submit order (jobHead..jobTail),
	 * page->completed = the highest seq with every job up to it finished.
	 */
	uint64_t   seqNext, seqDone;
	VivaGPUJob *jobHead, *jobTail;
	IOBufferMemoryDescriptor *completionMD;
	struct vgpu_completion   *completion;
	void       seqUpdate(void);

	/*
	 * Lazy invalidation (vivagpulazy=1): CPU-maintained BOs the GPU used since the CPU last dropped their lines.
	 * Not retained (a BO leaves the list when freed); flushed when userspace waits (WAIT_BO, syncobj wait).
	 */
	void invAdd(VivaGPUBO *bo);
	void invRemove(VivaGPUBO *bo);
	VivaGPUBO **inv;
	uint32_t    invCount, invCap;

private:
	uint64_t *tableFor(uint64_t va, bool create);
	uint64_t *tableKVA(uint64_t pa);
	uint64_t  newTable(void);
	OSArray  *fPool;                /* IOBufferMemoryDescriptor: 16 KiB pages, 4 tables each */
	uint64_t  fFreePA;              /* next free 4 KiB table in the last pool page */
	uint32_t  fFreeLeft;
	enum { kPoolMax = 640 };        /* 2560 tables: 4 GiB of VA needs 2048 level-3 tables + 5 */
	uint64_t  fPoolPA[kPoolMax];
	uint8_t  *fPoolKVA[kPoolMax];
	uint32_t  fPoolCount;
	struct Range { uint64_t start, end; };
	Range    *fRanges;              /* allocated VA, sorted */
	uint32_t  fRangeCount, fRangeCap;
	struct Freed { uint64_t start, end, ms; };
	enum { kFreedMax = 64 };
	Freed     fFreed[kFreedMax];    /* last VA ranges freed (ring), for describe() */
	uint32_t  fFreedNext;
};

enum VivaGPUBOKind { kVivaBONormal, kVivaBOHeap, kVivaBOSurface, kVivaBOUser };

class VivaGPUBO : public OSObject {
	OSDeclareDefaultStructors(VivaGPUBO);
public:
	virtual bool init(void) APPLE_KEXT_OVERRIDE { return OSObject::init(); }
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	/* Heap BO: back the 2 MiB chunk containing `va` (GPU fault). */
	bool growHeap(uint64_t va);
	/* CPU cache maintenance (kVivaCache*) of [offset, offset + len) of the BO (len 0 = all); bytes done. */
	uint64_t cacheOp(uint64_t offset, uint64_t len, int op);

	VivaGPUVM          *vm;
	VivaGPUBOKind       kind;
	IOMemoryDescriptor *memory;     /* normal/user/surface pages; NULL for a heap */
	uint64_t            memOffset;  /* first mapped byte in `memory` (page-aligned) */
	uint64_t            gpuOffset;  /* GPU address of the content = va + gpuOffset (surface plane) */
	uint64_t            va, vaSize; /* GPU VA and mapped span (16 KiB multiple) */
	uint64_t            size;       /* size reported to userspace */
	uint32_t            flags, madv;
	bool                cacheMaint; /* CPU-mapped and cacheable: flush before and invalidate after every job */
	bool                lazySurface;/* cacheable IOSurface: cleaned only after a CPU write lock (SweetSurface) */
	bool                cacheable;  /* CPU-cacheable memory (SYNC_BO applies) */
	bool                prepared;   /* memory->prepare() done (user memory) */
	bool                invPending; /* in vm->inv: the GPU used it, the CPU lines are not dropped yet */
	VivaGPUFence       *busy;       /* fence of the last job using it (it signals after every earlier one) */
	IOBufferMemoryDescriptor **chunks;      /* heap: one per 2 MiB of VA, NULL until the GPU faults on it */
	uint32_t            chunkCount;
	SweetSurface       *surface;
	IOMemoryMap        *kmap;       /* kernel mapping for cache maintenance (non-buffer memory) */
};

struct VivaGPUJob;

class VivaGPUAccelerator : public IOAcceleratorES {
	OSDeclareDefaultStructors(VivaGPUAccelerator);
	friend class VivaGPUVM;
	friend class VivaGPUBO;
	friend class VivaGPUUserClient;
public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOWorkLoop *getWorkLoop(void) const APPLE_KEXT_OVERRIDE;
	virtual IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type,
	    IOUserClient **handler) APPLE_KEXT_OVERRIDE;

	IOCommandGate *gate(void) { return fGate; }
	/* USB link "gpu ..." (text reply in out). */
	void linkCommand(const char *args, char *out, size_t outLen);

	/* Gated. */
	IOReturn submit(VivaGPUVM *vm, uint64_t jc, uint32_t requirements, OSArray *deps, OSArray *bos,
	    VivaGPUFence *fence, uint64_t *seqOut = NULL);
	/* Gated: signal `fence` (once), wake its waiters, signal the promises linked to it. */
	void signalFence(VivaGPUFence *fence, IOReturn error, uint64_t at);
	/*
	 * Gated, K0i: the pending promise `p` is resolved by `to` (signals when `to` does; NULL or already signalled:
	 * now). makePromise: a new pending promise of `vm` (timeout ms), set as the last fence of the surfaces.
	 */
	void resolvePromise(VivaGPUFence *p, VivaGPUFence *to);
	VivaGPUFence *makePromise(VivaGPUVM *vm, uint32_t timeoutMs, SweetSurface **surfaces, uint32_t count);
	/* Gated: sleep until the fence signals or the absolute deadline (0 = forever) passes. */
	IOReturn waitFence(VivaGPUFence *fence, uint64_t deadline);
	/* Gated: no pending job uses the BO (for an IOSurface: in any connection). */
	static bool boIdle(VivaGPUBO *bo);
	/*
	 * Gated: the fence that ends GPU use of `bo` as seen by its connection: its last job, else (IOSurface) the
	 * surface's last fence, or what came before the connection's own pending promise (K0i). NULL = idle.
	 */
	static VivaGPUFence *boBusyFence(VivaGPUBO *bo);
	/*
	 * Gated, K0j: one fence WAIT_BO on `bo` must still sleep on (retained; NULL = done). Fences that depend on a
	 * pending promise of the BO's own connection come after its promised work: what they waited for counts instead.
	 */
	VivaGPUFence *boWaitFence(VivaGPUBO *bo);
	/* Gated, K0j: drop the connection's queued jobs (fences signal kIOReturnAborted), let its running ones finish. */
	void abortVM(VivaGPUVM *vm);
	/* Gated: start what can start (after a fence was signalled outside a job completion). */
	void kickNow(void) { kick(); }
	/* Gated: no pending job uses the surface (any connection); *jobs = jobs that used it so far. */
	static bool surfaceIdle(SweetSurface *surface, uint64_t *jobs);
	/*
	 * Gated, lazy invalidation: drop the CPU lines of the idle BOs of vm->inv (only = that BO alone, if listed),
	 * so the CPU reads what the GPU wrote. Called when userspace waits for the GPU.
	 */
	void invalidatePending(VivaGPUVM *vm, VivaGPUBO *only);
	bool lazyInvalidate(void) const { return fLazy; }
	bool writeCombined(void) const { return fWC; }
	/*
	 * Gated waits (kIOReturnTimeout, kIOReturnAborted; the caller re-checks its condition). sleepOn: until `fence`
	 * (retained by the caller) signals or is dropped from a syncobj. sleepUntil: until any fence signals or any
	 * syncobj changes (waits on several syncobjs, a syncobj without a fence).
	 */
	IOReturn sleepOn(VivaGPUFence *fence, uint64_t deadline);
	IOReturn sleepUntil(uint64_t deadline);
	/* Gated: wake the threads sleeping on `fence` (signalled, or no longer what a syncobj holds) and the generic ones. */
	void fenceWake(VivaGPUFence *fence);
	/* Gated: wake the generic waiters only (a syncobj changed). */
	void wakeWaiters(void);
	bool powered(void) const { return fPowered; }
	IOReturn getParam(uint32_t param, uint64_t *value);
	void getVersion(struct vgpu_version *v);
	/* Unknown user clients and property reads (any thread, own lock). */
	void probeNote(const char *proc, uint32_t type, uint32_t selector, uint32_t inSize, uint32_t outSize);
	void propertyRead(const char *key, bool found);
	void vmFlush(VivaGPUVM *vm, uint64_t va, uint64_t size);   /* page tables changed */
	void vmDestroyed(VivaGPUVM *vm);
	VivaGPUBO *findHeapBO(VivaGPUVM *vm, uint64_t va);
	void heapAdd(VivaGPUBO *bo);
	void heapRemove(VivaGPUBO *bo);
	/* GPU memory accounting: false = over the per-process or global limit (nothing charged). */
	bool chargeMemory(VivaGPUVM *vm, int64_t bytes);
	void vmLink(VivaGPUVM *vm);
	/*
	 * K0g: cost of the user client calls ("gpu calls"). callDone: any thread, a call of `selector` took `abs`
	 * (mach_absolute_time units) from entry to return, through the trap or the method path. gateWaited: gated,
	 * it waited `abs` for the gate.
	 */
	void callDone(uint32_t selector, uint64_t abs, bool trap);
	void gateWaited(uint32_t selector, uint64_t abs);

private:
	/* MFG power (MediaTek) */
	bool mfgOn(char **out, char *end);
	void mfgOff(char **out, char *end);
	void mfgDump(char **out, char *end);
	bool regulatorSet(uint32_t enAddr, uint32_t voselAddr, uint32_t sel);
	bool mtcmosOn(uint32_t idx, char **out, char *end);
	bool mtcmosOff(uint32_t idx, char **out, char *end);
	void mfgClock(uint32_t opp, char **out, char *end);
	void oppSels(uint32_t opp, uint32_t *vgpuSel, uint32_t *vsramSel);
	void railsRead(uint32_t *vgEn, uint32_t *vg, uint32_t *vsEn, uint32_t *vs);
	bool railsTo(uint32_t vgpuSel, uint32_t vsramSel);
	IOReturn setOpp(uint32_t opp, char **out, char *end);
	void protRelease(int from = 3);
	/* Mali */
	bool maliInit(char **out, char *end);
	bool softReset(void);
	bool powerCores(void);
	void readFeatures(void);
	IOReturn powerOn(char **out, char *end);
	void powerOff(char **out, char *end);
	/* MMU */
	bool asWait(int as);
	void asCommand(int as, uint32_t cmd);
	void asLock(int as, uint64_t va, uint64_t size);
	void asProgram(int as, VivaGPUVM *vm);
	void asDisable(int as);
	bool asBind(VivaGPUVM *vm);
	/* jobs */
	void addImplicitDep(OSArray *deps, VivaGPUFence *f, VivaGPUFence *self);
	/* K0j: promise-aware waits of connection `vm` */
	bool vmHasPromise(VivaGPUVM *vm);
	bool dependsOnPromiseOf(VivaGPUFence *f, VivaGPUVM *vm, uint32_t *budget);
	void collectWait(VivaGPUFence *f, VivaGPUVM *vm, OSArray *out, uint32_t depth);
	void addImplicitDepFor(OSArray *deps, VivaGPUFence *f, VivaGPUFence *self, VivaGPUVM *vm);
	void discardJob(VivaGPUJob *job, IOReturn error);
	void unlinkQueued(uint32_t slot, VivaGPUJob *job, VivaGPUJob *prev);
	VivaGPUJob *nextStartable(uint32_t slot, bool forArm);
	void kick(void);
	void startJob(uint32_t slot, VivaGPUJob *job);
	void jsWrite(uint32_t slot, VivaGPUJob *job, uint32_t config);
	uint32_t jsConfig(VivaGPUJob *job);
	void jobStarted(uint32_t slot, VivaGPUJob *job, uint64_t at, bool fromFilter);
	void completeJob(uint32_t slot, IOReturn error, uint32_t status, uint64_t at = 0);
	void failAll(IOReturn error);
	void resetGPU(const char *why);
	static void gpuInterrupt(OSObject *owner, IOInterruptEventSource *src, int count);
	static void mmuInterrupt(OSObject *owner, IOInterruptEventSource *src, int count);
	static void jobInterrupt(OSObject *owner, IOInterruptEventSource *src, int count);
	static bool jobFilter(OSObject *owner, IOFilterInterruptEventSource *src);
	bool jobFilterRun(void);
	/* K0e fast kick (gated): take over what the filter did; arm the queue heads that wait only for running jobs */
	void fastSync(void);
	void fastArm(void);
	void fastLive(bool live);
	/* where the GPU's time goes: a job on the hardware, jobs queued but none running, nothing queued */
	enum { kStateBusy, kStateWaiting, kStateIdle, kStateCount };
	void stateUpdate(uint64_t at);
	void latencyStats(char **out, char *end);
	void latencyReset(void);
	static void jobTimer(OSObject *owner, IOTimerEventSource *src);
	static void dvfsTimer(OSObject *owner, IOTimerEventSource *src);
	void dvfsSample(void);
	void selftest(bool fault, char **out, char *end);

	inline uint32_t rd(uint32_t off) { return *(volatile uint32_t *)(fMali + off); }
	inline void wr(uint32_t off, uint32_t v) { *(volatile uint32_t *)(fMali + off) = v; }

	IOWorkLoop             *fWorkLoop;
	IOCommandGate          *fGate;
	IOInterruptEventSource *fIrq[3];        /* GPU, MMU, JOB */
	IOTimerEventSource     *fTimer;
	IOTimerEventSource     *fDvfsTimer;
	IOMemoryMap            *fMaps[6];       /* mali, mfgcfg, spm, topckgen, apmixed, infracfg_ao */
	vm_offset_t             fMali, fMfgcfg, fSpm, fTopck, fApmixed, fInfra;
	VivaPwrap              *fPwrap;
	int                     fMode;          /* vivagpu boot-arg */
	uint32_t                fOpp;           /* OPP index (gVivaOpp) for the next power on / "gpu freq" */
	uint32_t                fClockOpp;      /* OPP the MFGPLL runs at */
	int                     fMarginMv;      /* vivagpumv: added to the table's VGPU */
	bool                    fPowered, fPublished, fAcpDone, fTimerArmed;
	char                    fFenceEvent;

	/* features (read once after the first power on) */
	uint32_t fGpuId, fL2Features, fCoreFeatures, fTilerFeatures, fMemFeatures, fMmuFeatures;
	uint32_t fAsPresent, fJsPresent, fThreadMaxThreads, fThreadMaxWg, fThreadMaxBarrier, fThreadFeatures;
	uint32_t fThreadTlsAlloc, fCoherencyFeatures, fAfbcFeatures, fTextureFeatures[4], fJsFeatures[16];
	uint64_t fShaderPresent, fTilerPresent, fL2Present, fStackPresent;

	/* address spaces */
	VivaGPUVM *fAS[16];
	uint32_t   fASCount;
	uint64_t   fLRU;

	/* jobs: per slot (0 fragment, 1 vertex/tiler/compute) a FIFO and the job on the hardware */
	VivaGPUJob *fQueue[2], *fQueueTail[2], *fRunning[2];
	uint64_t    fStopDeadline[2];       /* hard stop sent: reset if the slot is not done by then */
	VivaGPUBO **fHeapList;              /* heap BOs (not retained), for GPU faults */
	uint32_t    fHeapCount, fHeapCap;
	int64_t     fMemory;                /* bytes of GPU memory allocated */
	int64_t     fMemPeak, fMemLimit, fProcLimit;    /* boot-args vivagpumemtotal / vivagpumem (MiB) */
	uint64_t    fMemRefused;
	VivaGPUVM  *fVMs;                   /* live VMs (gated) */
	void memoryDump(char **out, char *end);
	VivaGPUVM  *fSelfVM;

	/* counters */
	uint64_t fJobs, fJobFaults, fMmuFaults, fGpuFaults, fResets, fTimeouts, fHeapGrows;
	/*
	 * CPU cache maintenance (vivagpucache: 0 every BO around every job, 1 only CPU-mapped BOs + lazy surfaces,
	 * 2 like 1 but every such BO of the connection with every job, listed in bo_handles or not: diagnosis)
	 */
	int      fCacheMode;
	uint64_t fSubmits, fSubmitNs, fSubmitCacheNs, fSubmitCacheBytes, fDoneCacheNs, fDoneCacheBytes;
	/*
	 * K0b: vivagpulazy (1 default): after a job only list the CPU-maintained BOs, their lines are dropped when
	 * userspace waits (0: after every job, as before). vivagpuwc (1 default): BOs without VGPU_BO_WB_MMAP are
	 * write-combined like on Linux (Mesa with DRM 1.5 expects that): no CPU cache maintenance at all.
	 */
	bool     fLazy, fWC;
	/* flush reduction (vivagpuflushid, 1 default): JS_FLUSH_ID_NEXT = GPU_LATEST_FLUSH at submit */
	bool     fFlushReduce;
	/* DVFS governor (vivagpudvfs, 1 default; fOpp is the floor, fDvfsFast from vivagpufreqmax) */
	bool     fDvfs, fDvfsArmed;
	uint32_t fDvfsFast, fLoadPct;
	uint64_t fBusyStart, fBusyNs, fWindowStart, fDvfsUpMs, fDvfsUps, fDvfsDowns;
	uint64_t fBusyTotal;            /* KA1 busy_ns: absolute time with a job running, since boot */
	bool     fDvfsBoost;                /* 0137 vivagpuboost: fastest OPP while the display waits for frames */
	uint32_t fDvfsSeenWaits;
	uint64_t fDvfsBoosts;
	uint64_t fImplicitDeps, fLazyListed, fLazyBytes, fLazyNs, fLazyWaits, fLazyOverflows;
	void cacheStats(char **out, char *end);
public:
	int cacheMode(void) const { return fCacheMode; }
	/* A BO gets a CPU mapping (user mmap): from now on it is maintained around jobs. Gated. */
	void boMapped(VivaGPUBO *bo);
private:
	uint64_t fFaultsNoReset;            /* chains killed by an MMU fault without a GPU reset */
	bool     fWatchFragment;            /* since then no fragment chain completed: short timeout on JS0 */

	/*
	 * K0e fast kick (vivagpufastkick, 1 default): the queue head of a slot whose unfinished dependencies are all
	 * jobs on the hardware is "armed" (AS bound and reserved, JS_CONFIG ready). The job interrupt's filter
	 * starts it the moment those jobs signal done, in primary interrupt context, instead of after the work-loop
	 * thread wakes up; the gated handler then completes the finished jobs and takes the started one over.
	 */
	struct FastArm {
		VivaGPUJob *job;
		uint32_t    wait;                   /* slots whose running job must be done (bit n = slot n) */
		uint32_t    config;
		bool        started;
		uint64_t    startAt;
	};
	IOSimpleLock *fFastLock;                /* fIrq[2] is the job interrupt's IOFilterInterruptEventSource */
	FastArm       fArm[2];
	uint32_t      fIrqDone;                 /* done bits the filter cleared, not handled yet */
	uint64_t      fIrqAt[2];                /* when the filter saw each slot's interrupt */
	bool          fFastKick, fFastOn;       /* boot-arg / link switch; GPU powered with interrupts set up */
	/* latency accounting ("gpu lat"), mach_absolute_time units */
	uint64_t fStatSince, fStateSince, fStateNs[kStateCount];
	uint32_t fState;
	uint64_t fLatN[3], fLatSum[3], fLatMax[3];  /* start - ready: ready at submit, after a dependency (work loop, filter) */
	uint64_t fExecN, fExecSum, fIrqN, fIrqSum, fIrqMax, fFastStarts, fFastArmed;

	/*
	 * K0g. Wakeups (gated): fGenericWaiters threads in sleepUntil (fFenceEvent is only woken when there are some);
	 * fence wakeups issued, generic wakeups issued, sleeps that ended with nothing changed (spurious).
	 */
	OSArray *fPromises;                 /* K0i: pending promises (their timeout, in jobTimer) */
	uint64_t fPromisesMade, fPromisesTimedOut;
	uint64_t fPromiseBypass, fQueueBypass, fJobsDiscarded;      /* K0j */
	uint32_t fArmWait;
	uint64_t fK0jEpoch;                 /* bumped when a promise is made, resolved or signalled, or a job dropped */
	uint64_t fK0jVisits, fK0jCached;    /* graph nodes walked / answered from the cache */                  /* nextStartable(forArm): the slots the armed job waits for */
	uint64_t fK0jWalk, fK0jCut, fK0jCollect;       /* 0128: walks / cut short by the budget; collectWait */
	struct K0jFrame {
		VivaGPUFence *f;
		uint32_t      next;     /* next index in f->deps */
		bool          cond;     /* its "no" leaned on a node still on the path or cut short: not cached */
	} *fK0jStack;                       /* 0128: dependsOnPromiseOf's explicit stack (was recursion: stack overflow) */
	void promiseTimeouts(void);
	uint32_t fGenericWaiters;
	uint64_t fWakeFence, fWakeGeneric, fSleeps, fSleepsSpurious;
	/* per selector: calls through the method / trap path, time from entry to return, time waiting for the gate */
	struct CallStat { uint64_t n, trapN, sum, max, gateN, gateSum, gateMax; };
	CallStat fCalls[VGPU_SEL_COUNT];
	IOSimpleLock *fCallLock;
	void callStats(char **out, char *end);
	void callReset(void);
	/* "gpu clocks": CPU, CCI and MFG PLLs from apmixedsys (read only) */
	void clocksDump(char **out, char *end);

	/* unknown user clients and property reads, for "gpu probe" (fProbeLock) */
	IOLock *fProbeLock;
	int     fProbeMode;                 /* vivagpuprobe: 0 refuse unknown types, 1 accept and log */
	struct ProbeEntry { char proc[20]; uint32_t type, selector, count, inSize, outSize; };
	enum { kProbeMax = 64, kPropMax = 64 };
	ProbeEntry fProbe[kProbeMax];
	uint32_t   fProbeCount, fProbeDropped;
	struct PropEntry { char proc[20]; char key[44]; uint32_t count; bool found; };
	PropEntry  fProps[kPropMax];
	uint32_t   fPropCount, fPropDropped;
	void probeDump(char **out, char *end);
};

/*
 * A user client of a type VivaGPUUserClient does not implement (IOGPU.framework's IOGPUDeviceCreate opens one on
 * the IOAcceleratorES it finds): every call is logged ("gpu probe" lists them) and answered kIOReturnUnsupported.
 */
class VivaGPUProbeClient : public IOUserClient {
	OSDeclareDefaultStructors(VivaGPUProbeClient);
public:
	virtual bool initWithTask(task_t owningTask, void *securityID, UInt32 type) APPLE_KEXT_OVERRIDE;
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target, void *reference) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientMemoryForType(UInt32 type, IOOptionBits *options,
	    IOMemoryDescriptor **memory) APPLE_KEXT_OVERRIDE;
	virtual IOReturn registerNotificationPort(mach_port_t port, UInt32 type,
	    io_user_reference_t refCon) APPLE_KEXT_OVERRIDE;
	virtual IOExternalTrap *getTargetAndTrapForIndex(IOService **targetP, UInt32 index) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setProperties(OSObject *properties) APPLE_KEXT_OVERRIDE;
	virtual IOReturn connectClient(IOUserClient *client) APPLE_KEXT_OVERRIDE;

private:
	void note(const char *what, uint32_t selector, uint32_t inSize, uint32_t outSize, const char *detail);
	VivaGPUAccelerator *fGPU;
	uint32_t            fType;
	int                 fPid;
	char                fProc[20];
	uint32_t            fLogged;
};

class VivaGPUUserClient : public IOUserClient {
	OSDeclareDefaultStructors(VivaGPUUserClient);
public:
	virtual bool initWithTask(task_t owningTask, void *securityID, UInt32 type) APPLE_KEXT_OVERRIDE;
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target, void *reference) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientMemoryForType(UInt32 type, IOOptionBits *options,
	    IOMemoryDescriptor **memory) APPLE_KEXT_OVERRIDE;
	virtual IOExternalTrap *getTargetAndTrapForIndex(IOService **targetP, UInt32 index) APPLE_KEXT_OVERRIDE;
	/* K0g trap VGPU_TRAP_CALL: (selector, user address of its structure), same semantics as the method path. */
	IOReturn trapCall(void *selector, void *uaddr, void *, void *, void *, void *);

private:
	IOReturn dispatch(uint32_t selector, void *arg);
	/* runActionBlock with the gate wait accounted to `selector` */
	IOReturn gated(uint32_t selector, IOReturn (^block)(void));
	IOReturn createBO(struct vgpu_create_bo *a);
	IOReturn submit(struct vgpu_submit *a);
	IOReturn submitBatch(struct vgpu_submit_batch *a);
	IOReturn surfacePromise(struct vgpu_surface_promise *a);
	/* gated part of a SUBMIT (syncs/handles already copied in) */
	IOReturn submitGated(struct vgpu_submit *a, const uint32_t *syncs, const uint32_t *handles);
	/* gated: the syncobj's fence becomes `f` (reference taken over): old waiters woken, a pending promise resolved */
	void syncobjSet(uint32_t handle, VivaGPUFence *f);
	IOReturn syncobjWait(struct vgpu_syncobj_wait *a);
	IOReturn importSurface(struct vgpu_import_iosurface *a);
	IOReturn importUser(struct vgpu_import_userptr *a);
	IOReturn syncBO(struct vgpu_sync_bo *a);
	uint32_t addBO(VivaGPUBO *bo);
	VivaGPUBO *bo(uint32_t handle);
	bool syncobjValid(uint32_t handle);
	void closeAll(void);

	VivaGPUAccelerator *fGPU;
	task_t              fTask;
	VivaGPUVM          *fVM;
	VivaGPUBO         **fBOs;           /* index = handle (0 unused) */
	uint32_t            fBOCap;
	struct Syncobj { bool used; VivaGPUFence *fence; };
	Syncobj            *fSyncobjs;      /* index = handle (0 unused) */
	uint32_t            fSyncCap;
	bool                fClosed;
};

#endif /* _IOKIT_ARM_VIVAGPU_H */
