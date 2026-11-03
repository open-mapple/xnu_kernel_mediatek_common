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
 * commands). Waits sleep on the gate with commandSleep().
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
#include <IOKit/IOTimerEventSource.h>

#include "viva_gpu_uapi.h"

class VivaPwrap;

/*
 * CPU cache maintenance (to PoC): clean (dc cvac) or flush = clean + invalidate (dc civac), before the GPU reads
 * and after it wrote. No plain invalidate (dc ivac): userspace keeps writing other parts of a BO while a job that
 * uses it runs, and dropping those dirty lines loses its writes (stale job descriptors: GPU faults, missing tiles).
 */
enum { kVivaCacheClean, kVivaCacheFlush };
void cacheBuffer(IOBufferMemoryDescriptor *md, uint64_t offset, uint64_t len, int op);
class VivaGPUAccelerator;
class SweetSurface;

/* Metal of iOS 16 looks GPUs up with IOServiceMatching("IOAcceleratorES") (IOGPUFamily is not in this kernel). */
class IOAcceleratorES : public IOService {
	OSDeclareDefaultStructors(IOAcceleratorES);
};

/* Completion of one job chain (or a syncobj signalled by hand). */
class VivaGPUFence : public OSObject {
	OSDeclareDefaultStructors(VivaGPUFence);
public:
	static VivaGPUFence *create(bool signalled);
	bool     signalled;
	IOReturn error;
};

/* A GPU address space: AArch64 4K page tables plus the VA allocator for [VGPU_VA_START, VGPU_VA_END). */
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
	VivaGPUFence       *busy;       /* fence of the last job using it */
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
	    VivaGPUFence *fence);
	/* Gated: sleep until the fence signals or the absolute deadline (0 = forever) passes. */
	IOReturn waitFence(VivaGPUFence *fence, uint64_t deadline);
	/* Gated: sleep until any fence or syncobj changes (kIOReturnTimeout, kIOReturnAborted). */
	IOReturn sleepUntil(uint64_t deadline);
	void wakeWaiters(void) { fGate->commandWakeup(&fFenceEvent, false); }
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
	void kick(void);
	void startJob(uint32_t slot, VivaGPUJob *job);
	void completeJob(uint32_t slot, IOReturn error, uint32_t status);
	void failAll(IOReturn error);
	void resetGPU(const char *why);
	static void gpuInterrupt(OSObject *owner, IOInterruptEventSource *src, int count);
	static void mmuInterrupt(OSObject *owner, IOInterruptEventSource *src, int count);
	static void jobInterrupt(OSObject *owner, IOInterruptEventSource *src, int count);
	static void jobTimer(OSObject *owner, IOTimerEventSource *src);
	void selftest(bool fault, char **out, char *end);

	inline uint32_t rd(uint32_t off) { return *(volatile uint32_t *)(fMali + off); }
	inline void wr(uint32_t off, uint32_t v) { *(volatile uint32_t *)(fMali + off) = v; }

	IOWorkLoop             *fWorkLoop;
	IOCommandGate          *fGate;
	IOInterruptEventSource *fIrq[3];        /* GPU, MMU, JOB */
	IOTimerEventSource     *fTimer;
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
	void cacheStats(char **out, char *end);
public:
	int cacheMode(void) const { return fCacheMode; }
	/* A BO gets a CPU mapping (user mmap): from now on it is maintained around jobs. Gated. */
	void boMapped(VivaGPUBO *bo);
private:
	uint64_t fFaultsNoReset;            /* chains killed by an MMU fault without a GPU reset */
	bool     fWatchFragment;            /* since then no fragment chain completed: short timeout on JS0 */

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

private:
	IOReturn dispatch(uint32_t selector, void *arg);
	IOReturn createBO(struct vgpu_create_bo *a);
	IOReturn submit(struct vgpu_submit *a);
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
