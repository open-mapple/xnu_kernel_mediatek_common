/*
 * MT6781: user client of the Mali driver (see MappleGPU.h), the
 * kernel side of mapple_gpu_uapi.h. One connection = one GPU address space
 * with its own BO and syncobj handles, like a DRM file of Linux's
 * panfrost; Mesa opens one per Metal device.
 *
 * Calls are IOConnectCallStructMethod(selector, &args, sizeof args, &args):
 * the structure is checked against vgpu_selector_size[], handled under the
 * accelerator's gate and copied back. Arrays the structures point to are
 * read from the calling task with copyin() before taking the gate. The same
 * call without a Mach message (K0g): IOConnectTrap2(conn, VGPU_TRAP_CALL,
 * selector, &args), the structure read with copyin() and written back with
 * copyout(). Every call's time and its wait for the gate are counted per
 * selector ("gpu calls" on the USB link).
 *
 * Semantics follow DRM_PANFROST_* and DRM binary syncobjs, with three
 * leniencies the userspace mock also has: an in-sync without a fence is no
 * dependency (DRM fails the submit), TRANSFER copies "no fence", and a
 * syncobj without a fence waits (times out) instead of failing with EINVAL.
 * Timeouts are absolute CLOCK_MONOTONIC nanoseconds as userspace reads it
 * (Darwin's Libc: calendar - boottime, mapple_gpu_monotonic_ns; not Mach time). WAIT_BO answers kIOReturnBusy (EBUSY) when the BO is still
 * busy at the deadline, what Panfrost expects.
 */
#include <pexpert/arm64/board_config.h>
#include <pexpert/pexpert.h>

#if defined(MT6781)
#include <IOKit/IOLib.h>
#include <IOKit/IOSubMemoryDescriptor.h>
#include <IOKit/system.h>
#include <kern/clock.h>
#include <mach/mach_time.h>
#include <sys/sysctl.h>

#include "MappleGPU.h"
#include "MappleSurface.h"

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define MAX_BOS_PER_SUBMIT      8192
#define MAX_SYNCS               256
#define MAX_SYNC_OPS            256
#define MAX_BO_SIZE             (1024ull << 20)
#define EXEC_MAX                (16ull << 20)   /* the shader program counter is 24-bit: never cross 16 MiB */
#define HEAP_ALIGN              (2ull << 20)

OSDefineMetaClassAndStructors(MappleGPUUserClient, IOUserClient);

static int mapple_gpu_dropmaps = 1;
SYSCTL_INT(_kern, OID_AUTO, mapple_gpu_dropmaps, CTLFLAG_RW | CTLFLAG_LOCKED, &mapple_gpu_dropmaps, 0,
    "GEM_CLOSE drops the process's CPU mappings of the BO still in place (0: they stay until the process exits)");

/* Absolute CLOCK_MONOTONIC ns -> absolute-time deadline (0 = forever); *expired if already past. */
static uint64_t
deadlineFor(int64_t timeoutNs, bool *expired)
{
	uint64_t now = mapple_gpu_monotonic_ns();     /* the clock Mesa computed timeoutNs from */
	*expired = false;
	if (timeoutNs < 0 || (uint64_t)timeoutNs >= now + 365ull * 24 * 3600 * NSEC_PER_SEC) {
		return 0;
	}
	if ((uint64_t)timeoutNs <= now) {
		*expired = true;
		return 0;
	}
	uint64_t rel;
	nanoseconds_to_absolutetime((uint64_t)timeoutNs - now, &rel);
	return mach_absolute_time() + rel;
}

static IOReturn
copyinArray(uint64_t uaddr, uint32_t count, uint32_t max, size_t elem, void **out, size_t *outSize)
{
	*out = NULL;
	*outSize = 0;
	if (count == 0) {
		return kIOReturnSuccess;
	}
	if (count > max || uaddr == 0) {
		return kIOReturnBadArgument;
	}
	size_t size = (size_t)count * elem;
	void *buf = IOMallocData(size);
	if (buf == NULL) {
		return kIOReturnNoMemory;
	}
	if (copyin((user_addr_t)uaddr, buf, size) != 0) {
		IOFreeData(buf, size);
		return kIOReturnBadArgument;
	}
	*out = buf;
	*outSize = size;
	return kIOReturnSuccess;
}

bool
MappleGPUUserClient::initWithTask(task_t owningTask, void *securityID, UInt32 type)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type)) {
		return false;
	}
	fTask = owningTask;
	fBOCap = 256;
	fBOs = IONewZero(MappleGPUBO *, fBOCap);
	fSyncCap = 64;
	fSyncobjs = IONewZero(Syncobj, fSyncCap);
	return fBOs != NULL && fSyncobjs != NULL;
}

bool
MappleGPUUserClient::start(IOService *provider)
{
	fGPU = OSDynamicCast(MappleGPUAccelerator, provider);
	if (fGPU == NULL || !IOUserClient::start(provider)) {
		return false;
	}
	fGPU->gate()->runActionBlock(^IOReturn {
		fVM = MappleGPUVM::create(fGPU);
		if (fVM != NULL) {
			task_reference(fTask);
			fVM->task = fTask;
		}
		return kIOReturnSuccess;
	});
	return fVM != NULL;
}

void
MappleGPUUserClient::closeAll(void)
{
	if (fGPU == NULL) {
		return;
	}
	fGPU->gate()->runActionBlock(^IOReturn {
		fGPU->abortVM(fVM);             /* K0j: no queued job of ours runs after our memory is gone */
		for (uint32_t h = 1; h < fBOCap; h++) {
			OSSafeReleaseNULL(fBOs[h]);
		}
		for (uint32_t h = 1; h < fSyncCap; h++) {
			syncobjSet(h, NULL);            /* its waiters see the syncobj gone; a promise in it signals */
			fSyncobjs[h].used = false;
		}
		OSSafeReleaseNULL(fVM);         /* jobs still running keep their own references */
		fGPU->wakeWaiters();
		return kIOReturnSuccess;
	});
}

IOReturn
MappleGPUUserClient::clientClose(void)
{
	if (!fClosed) {
		fClosed = true;
		closeAll();
	}
	terminate();
	return kIOReturnSuccess;
}

void
MappleGPUUserClient::free(void)
{
	if (!fClosed) {
		fClosed = true;
		closeAll();
	}
	if (fBOs != NULL) {
		IODelete(fBOs, MappleGPUBO *, fBOCap);
	}
	if (fSyncobjs != NULL) {
		IODelete(fSyncobjs, Syncobj, fSyncCap);
	}
	IOUserClient::free();
}

/* ---- handles (gated) ---- */

MappleGPUBO *
MappleGPUUserClient::bo(uint32_t handle)
{
	return (handle > 0 && handle < fBOCap) ? fBOs[handle] : NULL;
}

uint32_t
MappleGPUUserClient::addBO(MappleGPUBO *b)
{
	uint32_t h;
	for (h = 1; h < fBOCap && fBOs[h] != NULL; h++) {
	}
	if (h == fBOCap) {
		if (fBOCap >= 65536) {
			return 0;
		}
		MappleGPUBO **bigger = IONewZero(MappleGPUBO *, fBOCap * 2);
		if (bigger == NULL) {
			return 0;
		}
		bcopy(fBOs, bigger, fBOCap * sizeof(*bigger));
		IODelete(fBOs, MappleGPUBO *, fBOCap);
		fBOs = bigger;
		fBOCap *= 2;
	}
	fBOs[h] = b;
	return h;
}

bool
MappleGPUUserClient::syncobjValid(uint32_t handle)
{
	return handle > 0 && handle < fSyncCap && fSyncobjs[handle].used;
}

void
MappleGPUUserClient::syncobjSet(uint32_t handle, MappleGPUFence *f)
{
	MappleGPUFence *old = fSyncobjs[handle].fence;
	fSyncobjs[handle].fence = f;
	if (old != NULL && old != f) {
		fGPU->resolvePromise(old, f);   /* K0i: a pending promise follows what replaced it */
		fGPU->fenceWake(old);           /* its waiters re-check the syncobj */
	}
	fGPU->wakeWaiters();
	OSSafeReleaseNULL(old);
}

/* ---- calls ---- */

IOReturn
MappleGPUUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    __unused IOExternalMethodDispatch *dispatchTable, __unused OSObject *target, __unused void *reference)
{
	uint64_t buf[16] = { 0 };
	if (selector >= VGPU_SEL_COUNT || fClosed) {
		return kIOReturnBadArgument;
	}
	uint32_t size = vgpu_selector_size[selector];
	if (size > sizeof(buf) || args->structureInputSize != size || args->structureInput == NULL) {
		return kIOReturnBadArgument;
	}
	uint64_t t0 = mach_absolute_time();
	bcopy(args->structureInput, buf, size);
	IOReturn ret = dispatch(selector, buf);
	if (args->structureOutput != NULL && args->structureOutputSize >= size) {
		bcopy(buf, args->structureOutput, size);
		args->structureOutputSize = size;
	}
	fGPU->callDone(selector, mach_absolute_time() - t0, false);
	return ret;
}

/*
 * K0g: the struct method without the Mach message (MIG request and reply, port lookups, message copies): the
 * structure comes from the caller's memory and goes back there.
 */
static IOExternalTrap vgpu_traps[] = {
	[VGPU_TRAP_CALL] = { NULL, (IOTrap) & MappleGPUUserClient::trapCall },
};

IOExternalTrap *
MappleGPUUserClient::getTargetAndTrapForIndex(IOService **targetP, UInt32 index)
{
	if (index >= sizeof(vgpu_traps) / sizeof(vgpu_traps[0])) {
		return NULL;
	}
	*targetP = this;
	return &vgpu_traps[index];
}

IOReturn
MappleGPUUserClient::trapCall(void *selectorArg, void *uaddr, void *, void *, void *, void *)
{
	uint64_t buf[16] = { 0 };
	uint32_t selector = (uint32_t)(uintptr_t)selectorArg;
	if ((uintptr_t)selectorArg >= VGPU_SEL_COUNT || fClosed || uaddr == NULL) {
		return kIOReturnBadArgument;
	}
	uint32_t size = vgpu_selector_size[selector];
	if (size > sizeof(buf)) {
		return kIOReturnBadArgument;
	}
	uint64_t t0 = mach_absolute_time();
	if (copyin((user_addr_t)(uintptr_t)uaddr, buf, size) != 0) {
		return kIOReturnBadArgument;
	}
	IOReturn ret = dispatch(selector, buf);
	if (copyout(buf, (user_addr_t)(uintptr_t)uaddr, size) != 0 && ret == kIOReturnSuccess) {
		ret = kIOReturnBadArgument;
	}
	fGPU->callDone(selector, mach_absolute_time() - t0, true);
	return ret;
}

IOReturn
MappleGPUUserClient::gated(uint32_t selector, IOReturn (^block)(void))
{
	uint64_t t0 = mach_absolute_time();
	return fGPU->gate()->runActionBlock(^IOReturn {
		fGPU->gateWaited(selector, mach_absolute_time() - t0);
		return block();
	});
}

IOReturn
MappleGPUUserClient::dispatch(uint32_t selector, void *arg)
{
	switch (selector) {
	case VGPU_SEL_GET_VERSION:
		fGPU->getVersion((struct vgpu_version *)arg);
		return kIOReturnSuccess;
	case VGPU_SEL_GET_PARAM: {
		struct vgpu_get_param *a = (struct vgpu_get_param *)arg;
		return fGPU->getParam(a->param, &a->value);
	}
	case VGPU_SEL_CREATE_BO:
		return createBO((struct vgpu_create_bo *)arg);
	case VGPU_SEL_SUBMIT:
		return submit((struct vgpu_submit *)arg);
	case VGPU_SEL_SUBMIT_BATCH:
		return submitBatch((struct vgpu_submit_batch *)arg);
	case VGPU_SEL_SURFACE_PROMISE:
		return surfacePromise((struct vgpu_surface_promise *)arg);
	case VGPU_SEL_SYNCOBJ_WAIT:
		return syncobjWait((struct vgpu_syncobj_wait *)arg);
	case VGPU_SEL_IMPORT_IOSURFACE:
		return importSurface((struct vgpu_import_iosurface *)arg);
	case VGPU_SEL_IMPORT_USERPTR:
		return importUser((struct vgpu_import_userptr *)arg);
	case VGPU_SEL_SYNC_BO:
		return syncBO((struct vgpu_sync_bo *)arg);
	case VGPU_SEL_JM_CTX_CREATE:
		((struct vgpu_jm_ctx *)arg)->handle = 1;
		return kIOReturnSuccess;
	case VGPU_SEL_JM_CTX_DESTROY:
		return kIOReturnSuccess;
	case VGPU_SEL_SYNCOBJ_RESET:
	case VGPU_SEL_SYNCOBJ_SIGNAL: {
		struct vgpu_syncobj_array *a = (struct vgpu_syncobj_array *)arg;
		uint32_t *handles = NULL;
		size_t hsize = 0;
		IOReturn ret = copyinArray(a->handles, a->count_handles, MAX_SYNCS, sizeof(uint32_t), (void **)&handles,
		    &hsize);
		if (ret != kIOReturnSuccess) {
			return ret;
		}
		bool signal = selector == VGPU_SEL_SYNCOBJ_SIGNAL;
		ret = gated(selector, ^IOReturn {
			for (uint32_t i = 0; i < a->count_handles; i++) {
				if (!syncobjValid(handles[i])) {
					return kIOReturnBadArgument;
				}
			}
			for (uint32_t i = 0; i < a->count_handles; i++) {
				syncobjSet(handles[i], signal ? MappleGPUFence::create(true) : NULL);
			}
			fGPU->wakeWaiters();
			return kIOReturnSuccess;
		});
		if (handles != NULL) {
			IOFreeData(handles, hsize);
		}
		return ret;
	}
	default:
		break;
	}

	/* the rest: small, entirely under the gate */
	return gated(selector, ^IOReturn {
		switch (selector) {
		case VGPU_SEL_MMAP_BO: {
			struct vgpu_mmap_bo *a = (struct vgpu_mmap_bo *)arg;
			MappleGPUBO *b = bo(a->handle);
			if (b == NULL) {
				return kIOReturnNotFound;
			}
			a->offset = a->handle;  /* memoryType for IOConnectMapMemory64 */
			return kIOReturnSuccess;
		}
		case VGPU_SEL_GET_BO_OFFSET: {
			struct vgpu_get_bo_offset *a = (struct vgpu_get_bo_offset *)arg;
			MappleGPUBO *b = bo(a->handle);
			if (b == NULL) {
				return kIOReturnNotFound;
			}
			a->offset = b->va + b->gpuOffset;
			return kIOReturnSuccess;
		}
		case VGPU_SEL_GEM_CLOSE: {
			struct vgpu_gem_close *a = (struct vgpu_gem_close *)arg;
			MappleGPUBO *b = bo(a->handle);
			if (b == NULL) {
				return kIOReturnNotFound;
			}
			fBOs[a->handle] = NULL;
			/*
			 * CPU mappings still in the process go now: once the handle is gone, IOConnectUnmapMemory64 cannot
			 * find them (it asks clientMemoryForType again) and they would keep the pages until the process exits.
			 */
			IOMemoryMap *m;
			while (mapple_gpu_dropmaps && b->memory != NULL && (m = removeMappingForDescriptor(b->mapMD ? b->mapMD : b->memory)) != NULL) {
				m->release();
				fGPU->fMapsDropped++;
			}
			b->release();           /* jobs using it hold their own reference */
			return kIOReturnSuccess;
		}
		case VGPU_SEL_WAIT_BO: {
			struct vgpu_wait_bo *a = (struct vgpu_wait_bo *)arg;
			MappleGPUBO *b = bo(a->handle);
			if (b == NULL) {
				return kIOReturnNotFound;
			}
			bool expired;
			uint64_t deadline = deadlineFor(a->timeout_ns, &expired);
			b->retain();
			IOReturn ret = kIOReturnSuccess;
			MappleGPUFence *f;
			/*
			 * the BO's last job, else (an IOSurface) the last job on the surface in any connection; K0j: not a job
			 * that waits for a pending promise of ours (it comes after our promised work)
			 */
			while ((f = fGPU->boWaitFence(b)) != NULL) {
				if (expired) {
					f->release();
					ret = kIOReturnBusy;
					break;
				}
				IOReturn r = fGPU->sleepOn(f, deadline);
				f->release();
				if (r == kIOReturnTimeout) {
					expired = true;
				} else if (r != kIOReturnSuccess) {
					ret = r;
					break;
				}
			}
			if (ret == kIOReturnSuccess) {
				fGPU->invalidatePending(b->vm, b);     /* K0b: the CPU is about to read what the GPU wrote */
			}
			b->release();
			return ret;
		}
		case VGPU_SEL_MADVISE: {
			struct vgpu_madvise *a = (struct vgpu_madvise *)arg;
			MappleGPUBO *b = bo(a->handle);
			if (b == NULL) {
				return kIOReturnNotFound;
			}
			if (a->madv > VGPU_MADV_DONTNEED) {
				return kIOReturnBadArgument;
			}
			b->madv = a->madv;
			a->retained = 1;        /* never purged */
			return kIOReturnSuccess;
		}
		case VGPU_SEL_SYNCOBJ_CREATE: {
			struct vgpu_syncobj_create *a = (struct vgpu_syncobj_create *)arg;
			uint32_t h;
			for (h = 1; h < fSyncCap && fSyncobjs[h].used; h++) {
			}
			if (h == fSyncCap) {
				if (fSyncCap >= 65536) {
					return kIOReturnNoResources;
				}
				Syncobj *bigger = IONewZero(Syncobj, fSyncCap * 2);
				if (bigger == NULL) {
					return kIOReturnNoMemory;
				}
				bcopy(fSyncobjs, bigger, fSyncCap * sizeof(Syncobj));
				IODelete(fSyncobjs, Syncobj, fSyncCap);
				fSyncobjs = bigger;
				fSyncCap *= 2;
			}
			fSyncobjs[h].used = true;
			fSyncobjs[h].fence = (a->flags & VGPU_SYNCOBJ_CREATE_SIGNALED) ? MappleGPUFence::create(true) : NULL;
			a->handle = h;
			return kIOReturnSuccess;
		}
		case VGPU_SEL_SYNCOBJ_DESTROY: {
			struct vgpu_syncobj_destroy *a = (struct vgpu_syncobj_destroy *)arg;
			if (!syncobjValid(a->handle)) {
				return kIOReturnBadArgument;
			}
			syncobjSet(a->handle, NULL);
			fSyncobjs[a->handle].used = false;
			return kIOReturnSuccess;
		}
		case VGPU_SEL_SYNCOBJ_TRANSFER: {
			struct vgpu_syncobj_transfer *a = (struct vgpu_syncobj_transfer *)arg;
			if (!syncobjValid(a->src) || !syncobjValid(a->dst)) {
				return kIOReturnBadArgument;
			}
			MappleGPUFence *f = fSyncobjs[a->src].fence;
			if (f != NULL) {
				f->retain();
			}
			syncobjSet(a->dst, f);          /* K0i: TRANSFER into a promise's syncobj resolves it */
			return kIOReturnSuccess;
		}
		case VGPU_SEL_QUERY_BO_INFO: {
			struct vgpu_query_bo_info *a = (struct vgpu_query_bo_info *)arg;
			MappleGPUBO *b = bo(a->handle);
			if (b == NULL) {
				return kIOReturnNotFound;
			}
			a->create_flags = b->flags;
			a->extra_flags = (b->kind == kMappleBOSurface || b->kind == kMappleBOUser) ? VGPU_BO_IS_IMPORTED : 0;
			return kIOReturnSuccess;
		}
		default:
			return kIOReturnBadArgument;
		}
	});
}

IOReturn
MappleGPUUserClient::createBO(struct vgpu_create_bo *a)
{
	uint64_t size = round_page((uint64_t)a->size);
	uint32_t flags = a->flags;
	if (size == 0 || size > MAX_BO_SIZE || (flags & ~(VGPU_BO_NOEXEC | VGPU_BO_HEAP | VGPU_BO_WB_MMAP)) ||
	    ((flags & VGPU_BO_HEAP) && !(flags & VGPU_BO_NOEXEC))) {
		return kIOReturnBadArgument;
	}
	bool exec = !(flags & VGPU_BO_NOEXEC), heap = (flags & VGPU_BO_HEAP) != 0;
	uint64_t align = page_size;
	if (exec) {
		if (size > EXEC_MAX) {
			return kIOReturnBadArgument;
		}
		while (align < size) {
			align <<= 1;            /* aligned to its size: never crosses 16 MiB */
		}
	} else if (size >= HEAP_ALIGN || heap) {
		align = HEAP_ALIGN;
	}
	if (heap) {
		size = (size + HEAP_ALIGN - 1) & ~(HEAP_ALIGN - 1);
	}

	return gated(VGPU_SEL_CREATE_BO, ^IOReturn {
		if (fVM == NULL) {
			return kIOReturnNotOpen;
		}
		MappleGPUBO *b = new MappleGPUBO;
		if (b == NULL || !b->init()) {
			OSSafeReleaseNULL(b);
			return kIOReturnNoMemory;
		}
		b->vm = fVM;
		fVM->retain();
		fVM->bos++;
		b->flags = flags;
		b->size = b->vaSize = size;
		if (heap) {
			b->kind = kMappleBOHeap;
			b->chunkCount = (uint32_t)(size / HEAP_ALIGN);
			b->chunks = IONewZero(IOBufferMemoryDescriptor *, b->chunkCount);
			if (b->chunks == NULL) {
				b->release();
				return kIOReturnNoMemory;
			}
		} else {
			b->kind = kMappleBONormal;
			if (!fGPU->chargeMemory(fVM, (int64_t)size)) {
				b->vaSize = 0;          /* nothing charged: free() must not refund */
				b->release();
				return kIOReturnNoMemory;
			}
			/*
			 * K0b: write-combined unless userspace asks for VGPU_BO_WB_MMAP, as Linux maps panfrost BOs on a
			 * non-coherent GPU (Mesa with DRM 1.5 relies on it): CPU writes reach memory without any cache
			 * maintenance before or after jobs. mp_gpu_wc=0: cacheable, maintained by the kernel as before.
			 */
			bool wc = fGPU->writeCombined() && !(flags & VGPU_BO_WB_MMAP);
			IOBufferMemoryDescriptor *mem = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,
			    kIODirectionInOut | kIOMemoryKernelUserShared | (wc ? kIOMapWriteCombineCache : 0), size, page_size);
			if (mem == NULL) {
				fGPU->chargeMemory(fVM, -(int64_t)size);
				b->vaSize = 0;
				b->release();
				return kIOReturnNoMemory;
			}
			b->memory = mem;
			if (!wc) {
				/*
				 * Cacheable: the kernel keeps CPU and GPU coherent, zeros pushed out now, then a clean +
				 * invalidate before and after jobs. (Write-combined pages are zeroed and flushed by XNU when
				 * their cache mode changes: pmap_sync_page_attributes_phys, fixed for PAs above 4 GiB in 0112.)
				 */
				b->cacheable = true;
				b->cacheMaint = fGPU->cacheMode() == 0;        /* else from the first user mapping (boMapped) */
				cacheBuffer(mem, 0, size, kMappleCacheFlush);
			}
		}
		b->va = fVM->allocVA(size, align);
		if (b->va == 0) {
			b->release();
			return kIOReturnNoSpace;
		}
		if (heap) {
			fGPU->heapAdd(b);
		} else if (!fVM->map(b->va, b->memory, 0, size, MappleGPUVM::leafAttrs(exec))) {
			b->release();
			return kIOReturnNoMemory;
		}
		uint32_t h = addBO(b);
		if (h == 0) {
			b->release();
			return kIOReturnNoResources;
		}
		a->handle = h;
		a->offset = b->va;
		return kIOReturnSuccess;
	});
}

/* Gated: one SUBMIT, its arrays already read from the caller. */
IOReturn
MappleGPUUserClient::submitGated(struct vgpu_submit *a, const uint32_t *syncs, const uint32_t *handles)
{
	if (fVM == NULL) {
		return kIOReturnNotOpen;
	}
	if (a->jc == 0 || a->jc < VGPU_VA_START || a->jc >= VGPU_VA_END) {
		return kIOReturnBadArgument;
	}
	if (a->out_sync != 0 && !syncobjValid(a->out_sync)) {
		return kIOReturnNotFound;
	}
	OSArray *deps = OSArray::withCapacity(a->in_sync_count ? a->in_sync_count : 1);
	OSArray *bos = OSArray::withCapacity(a->bo_handle_count ? a->bo_handle_count : 1);
	MappleGPUFence *fence = MappleGPUFence::create(false);
	IOReturn r = (deps && bos && fence) ? kIOReturnSuccess : kIOReturnNoMemory;
	for (uint32_t i = 0; r == kIOReturnSuccess && i < a->in_sync_count; i++) {
		if (!syncobjValid(syncs[i])) {
			r = kIOReturnNotFound;
		} else if (fSyncobjs[syncs[i]].fence != NULL && !fSyncobjs[syncs[i]].fence->signalled) {
			deps->setObject(fSyncobjs[syncs[i]].fence);
		}
	}
	for (uint32_t i = 0; r == kIOReturnSuccess && i < a->bo_handle_count; i++) {
		MappleGPUBO *b = bo(handles[i]);
		if (b == NULL) {
			r = kIOReturnNotFound;
		} else {
			bos->setObject(b);
		}
	}
	/*
	 * mp_gpu_cache=2 (diagnosis): every CPU-maintained BO of the connection goes with the job, listed or
	 * not, so a BO userspace forgot in bo_handles still gets its CPU writes cleaned before the job.
	 */
	for (uint32_t h = 1; r == kIOReturnSuccess && fGPU->cacheMode() == 2 && h < fBOCap; h++) {
		MappleGPUBO *b = fBOs[h];
		if (b != NULL && (b->cacheMaint || b->lazySurface) && bos->getNextIndexOfObject(b, 0) == (unsigned int)-1) {
			bos->setObject(b);
		}
	}
	uint64_t seq = 0;
	if (r == kIOReturnSuccess) {
		r = fGPU->submit(fVM, a->jc, a->requirements, deps, bos, fence, &seq);
	}
	if (r == kIOReturnSuccess) {
		a->seq = (uint32_t)seq;
		if (a->out_sync != 0) {
			/* old fence's waiters re-check; a promise in the syncobj follows this job (K0i) */
			fence->retain();
			syncobjSet(a->out_sync, fence);
		}
	}
	OSSafeReleaseNULL(deps);
	OSSafeReleaseNULL(bos);
	OSSafeReleaseNULL(fence);
	return r;
}

IOReturn
MappleGPUUserClient::submit(struct vgpu_submit *a)
{
	uint32_t *syncs = NULL, *handles = NULL;
	size_t ssize = 0, hsize = 0;
	IOReturn ret = copyinArray(a->in_syncs, a->in_sync_count, MAX_SYNCS, sizeof(uint32_t), (void **)&syncs, &ssize);
	if (ret == kIOReturnSuccess) {
		ret = copyinArray(a->bo_handles, a->bo_handle_count, MAX_BOS_PER_SUBMIT, sizeof(uint32_t),
		    (void **)&handles, &hsize);
	}
	if (ret == kIOReturnSuccess) {
		ret = gated(VGPU_SEL_SUBMIT, ^IOReturn {
			return submitGated(a, syncs, handles);
		});
	}
	if (syncs != NULL) {
		IOFreeData(syncs, ssize);
	}
	if (handles != NULL) {
		IOFreeData(handles, hsize);
	}
	return ret;
}

/*
 * K0h: N submits, one call and one gate acquisition. Every array is read from the caller first (a bad pointer
 * fails the whole call before anything is queued); then they are queued in order until one fails.
 */
#define MAX_BATCH 64
IOReturn
MappleGPUUserClient::submitBatch(struct vgpu_submit_batch *a)
{
	a->done = 0;
	if (a->count == 0) {
		return kIOReturnSuccess;
	}
	if (a->count > MAX_BATCH || a->stride != sizeof(struct vgpu_submit) || a->submits == 0) {
		return kIOReturnBadArgument;
	}
	struct vgpu_submit *subs = NULL;
	size_t subSize = 0;
	IOReturn ret = copyinArray(a->submits, a->count, MAX_BATCH, sizeof(*subs), (void **)&subs, &subSize);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	uint32_t *syncs[MAX_BATCH] = { NULL }, *handles[MAX_BATCH] = { NULL };
	size_t ssize[MAX_BATCH] = { 0 }, hsize[MAX_BATCH] = { 0 };
	for (uint32_t i = 0; ret == kIOReturnSuccess && i < a->count; i++) {
		ret = copyinArray(subs[i].in_syncs, subs[i].in_sync_count, MAX_SYNCS, sizeof(uint32_t), (void **)&syncs[i],
		    &ssize[i]);
		if (ret == kIOReturnSuccess) {
			ret = copyinArray(subs[i].bo_handles, subs[i].bo_handle_count, MAX_BOS_PER_SUBMIT, sizeof(uint32_t),
			    (void **)&handles[i], &hsize[i]);
		}
	}
	__block uint32_t done = 0;
	if (ret == kIOReturnSuccess) {
		uint32_t *const *s = syncs, *const *h = handles;
		ret = gated(VGPU_SEL_SUBMIT_BATCH, ^IOReturn {
			for (uint32_t i = 0; i < a->count; i++) {
				IOReturn r = submitGated(&subs[i], s[i], h[i]);
				if (r != kIOReturnSuccess) {
					return r;
				}
				done++;
			}
			return kIOReturnSuccess;
		});
	}
	a->done = done;
	if (done != 0 && copyout(subs, (user_addr_t)a->submits, done * sizeof(*subs)) != 0 && ret == kIOReturnSuccess) {
		ret = kIOReturnBadArgument;     /* queued, but the seq numbers could not be written back */
	}
	for (uint32_t i = 0; i < a->count; i++) {
		if (syncs[i] != NULL) {
			IOFreeData(syncs[i], ssize[i]);
		}
		if (handles[i] != NULL) {
			IOFreeData(handles[i], hsize[i]);
		}
	}
	IOFreeData(subs, subSize);
	return ret;
}

/* K0i: see struct vgpu_surface_promise. */
#define MAX_PROMISE_SURFACES 16
IOReturn
MappleGPUUserClient::surfacePromise(struct vgpu_surface_promise *a)
{
	uint32_t *ids = NULL;
	size_t isize = 0;
	if (a->count == 0) {
		return kIOReturnBadArgument;
	}
	IOReturn ret = copyinArray(a->surface_ids, a->count, MAX_PROMISE_SURFACES, sizeof(uint32_t), (void **)&ids, &isize);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	MappleSurface *surfaces[MAX_PROMISE_SURFACES] = { NULL };
	for (uint32_t i = 0; ret == kIOReturnSuccess && i < a->count; i++) {
		surfaces[i] = mapple_surface_copy(ids[i]);
		if (surfaces[i] == NULL) {
			ret = kIOReturnNotFound;
		}
	}
	uint32_t timeoutMs = a->timeout_ms == 0 ? 1000 : MIN(a->timeout_ms, 5000u);
	if (ret == kIOReturnSuccess) {
		MappleSurface **list = surfaces;
		ret = gated(VGPU_SEL_SURFACE_PROMISE, ^IOReturn {
			if (fVM == NULL) {
				return kIOReturnNotOpen;
			}
			if (!syncobjValid(a->syncobj)) {
				return kIOReturnNotFound;
			}
			MappleGPUFence *p = fGPU->makePromise(fVM, timeoutMs, list, a->count);
			if (p == NULL) {
				return kIOReturnNoMemory;
			}
			syncobjSet(a->syncobj, p);      /* the reference moves into the syncobj */
			return kIOReturnSuccess;
		});
	}
	for (uint32_t i = 0; i < a->count; i++) {
		OSSafeReleaseNULL(surfaces[i]);
	}
	IOFreeData(ids, isize);
	return ret;
}

IOReturn
MappleGPUUserClient::syncobjWait(struct vgpu_syncobj_wait *a)
{
	uint32_t *handles = NULL;
	size_t hsize = 0;
	if (a->count_handles == 0) {
		return kIOReturnBadArgument;
	}
	IOReturn ret = copyinArray(a->handles, a->count_handles, MAX_SYNCS, sizeof(uint32_t), (void **)&handles, &hsize);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	ret = gated(VGPU_SEL_SYNCOBJ_WAIT, ^IOReturn {
		bool expired;
		uint64_t deadline = deadlineFor(a->timeout_nsec, &expired);
		bool all = (a->flags & VGPU_SYNCOBJ_WAIT_ALL) != 0;
		for (;;) {
			uint32_t signalled = 0, first = UINT32_MAX;
			MappleGPUFence *sleepFence = NULL;        /* the one fence that can complete the wait, if there is one */
			bool generic = false;
			for (uint32_t i = 0; i < a->count_handles; i++) {
				if (!syncobjValid(handles[i])) {
					return kIOReturnBadArgument;
				}
				MappleGPUFence *f = fSyncobjs[handles[i]].fence;
				if (f == NULL) {
					generic = true;         /* no fence yet: unsignalled (DRM would fail without WAIT_FOR_SUBMIT) */
					continue;
				}
				if (f->signalled) {
					signalled++;
					first = MIN(first, i);
				} else if (sleepFence == NULL) {
					sleepFence = f;
				}
			}
			if (all ? signalled == a->count_handles : signalled > 0) {
				a->first_signaled = first == UINT32_MAX ? 0 : first;
				fGPU->invalidatePending(fVM, NULL);    /* K0b: the CPU may now read what those jobs wrote */
				return kIOReturnSuccess;
			}
			if (expired) {
				return kIOReturnTimeout;
			}
			/*
			 * K0g: one handle, or all of them with a fence each: sleep on the first unsignalled fence (it must
			 * signal before the wait can end; replacing it in the syncobj wakes it too). Otherwise any fence may
			 * end the wait: the generic event.
			 */
			IOReturn r;
			if (sleepFence != NULL && !generic && (all || a->count_handles == 1)) {
				sleepFence->retain();
				r = fGPU->sleepOn(sleepFence, deadline);
				sleepFence->release();
			} else {
				r = fGPU->sleepUntil(deadline);
			}
			if (r == kIOReturnTimeout) {
				expired = true;
			} else if (r != kIOReturnSuccess) {
				return r;
			}
		}
	});
	IOFreeData(handles, hsize);
	return ret;
}

IOReturn
MappleGPUUserClient::importSurface(struct vgpu_import_iosurface *a)
{
	MappleSurface *s = mapple_surface_copy(a->surface_id);
	if (s == NULL) {
		return kIOReturnNotFound;
	}
	uint32_t planes = s->planeCount;
	if (s->memory == NULL || (planes == 0 ? a->plane != 0 : (a->plane >= planes || planes > MAPPLE_SURFACE_MAX_PLANES))) {
		s->release();
		return kIOReturnBadArgument;
	}
	uint64_t planeOff = planes ? s->planes[a->plane].offset : s->offset;
	uint64_t total = s->memory->getLength();
	if (planeOff >= total) {
		s->release();
		return kIOReturnBadArgument;
	}
	uint64_t mapStart = planeOff & ~(uint64_t)(page_size - 1);
	uint64_t mapSize = round_page(total - mapStart);

	IOReturn ret = gated(VGPU_SEL_IMPORT_IOSURFACE, ^IOReturn {
		if (fVM == NULL) {
			return kIOReturnNotOpen;
		}
		MappleGPUBO *b = new MappleGPUBO;
		if (b == NULL || !b->init()) {
			OSSafeReleaseNULL(b);
			return kIOReturnNoMemory;
		}
		b->vm = fVM;
		fVM->retain();
		fVM->bos++;
		b->kind = kMappleBOSurface;
		b->surface = s;
		s->retain();
		b->memory = s->memory;
		b->memory->retain();
		b->prepared = b->memory->prepare() == kIOReturnSuccess;
		if (b->prepared) {
			fVM->surfPrepared += (int64_t)mapSize;
			fVM->surfPrepares++;
			fGPU->fSurfPrepared += (int64_t)mapSize;
			fGPU->fSurfPrepares++;
		}
		b->memOffset = mapStart;
		b->size = b->vaSize = mapSize;
		fVM->imported += (int64_t)mapSize;
		b->cacheable = (s->client || OSDynamicCast(IOBufferMemoryDescriptor, s->memory) != NULL) &&
		    s->mapCache != kIOMapWriteCombineCache && s->mapCache != kIOMapInhibitCache;
		b->cacheMaint = b->cacheable && fGPU->cacheMode() == 0;
		b->lazySurface = b->cacheable && !b->cacheMaint;
		b->va = fVM->allocVA(mapSize, mapSize >= HEAP_ALIGN ? HEAP_ALIGN : page_size);
		b->gpuOffset = planeOff - mapStart;
		if (!b->prepared || b->va == 0 || !fVM->map(b->va, b->memory, mapStart, mapSize, MappleGPUVM::leafAttrs(false))) {
			b->release();           /* unmaps, frees the VA, completes the memory */
			return kIOReturnNoMemory;
		}
		uint32_t h = addBO(b);
		if (h == 0) {
			b->release();
			return kIOReturnNoResources;
		}
		a->handle = h;
		a->offset = b->va + b->gpuOffset;
		s->gpuVM = fVM;
		s->gpuVA = a->offset;
		a->size = total - planeOff;
		a->bytes_per_row = planes ? s->planes[a->plane].bytesPerRow : s->bytesPerRow;
		a->pixel_format = s->pixelFormat;
		a->width = planes ? s->planes[a->plane].width : s->width;
		a->height = planes ? s->planes[a->plane].height : s->height;
		return kIOReturnSuccess;
	});
	s->release();
	return ret;
}

IOReturn
MappleGPUUserClient::importUser(struct vgpu_import_userptr *a)
{
	static int on = -1;
	if (on < 0) {
		int v = 1;
		PE_parse_boot_argn("mp_gpu_userptr", &v, sizeof(v));
		on = v != 0;
	}
	if (!on) {
		return kIOReturnUnsupported;    /* mp_gpu_userptr=0: the driver copies, as before */
	}
	if (a->addr == 0 || a->size == 0 || a->size > MAX_BO_SIZE || (a->addr & (page_size - 1)) ||
	    (a->size & (page_size - 1))) {
		return kIOReturnBadArgument;
	}
	/* the caller's own pages, wired until the BO's last job is done; memory it cannot write: the GPU reads only */
	bool readOnly = false;
	IOMemoryDescriptor *md = IOMemoryDescriptor::withAddressRange(a->addr, a->size, kIODirectionInOut, fTask);
	IOReturn ret = md != NULL ? md->prepare() : kIOReturnNoMemory;
	if (ret != kIOReturnSuccess) {
		OSSafeReleaseNULL(md);
		md = IOMemoryDescriptor::withAddressRange(a->addr, a->size, kIODirectionOut, fTask);
		ret = md != NULL ? md->prepare(kIODirectionOut) : kIOReturnNoMemory;
		readOnly = true;
	}
	if (ret != kIOReturnSuccess) {
		OSSafeReleaseNULL(md);
		return ret;
	}
	ret = gated(VGPU_SEL_IMPORT_USERPTR, ^IOReturn {
		if (fVM == NULL) {
			return kIOReturnNotOpen;
		}
		MappleGPUBO *b = new MappleGPUBO;
		if (b == NULL || !b->init()) {
			OSSafeReleaseNULL(b);
			return kIOReturnNoMemory;
		}
		b->vm = fVM;
		fVM->retain();
		fVM->bos++;
		b->kind = kMappleBOUser;
		b->memory = md;
		md->retain();
		b->prepared = true;
		b->cacheable = true;
		b->size = b->vaSize = a->size;
		fVM->imported += (int64_t)a->size;
		b->cacheMaint = true;
		b->va = fVM->allocVA(a->size, a->size >= HEAP_ALIGN ? HEAP_ALIGN : page_size);
		uint32_t h = 0;
		if (b->va == 0 || !fVM->map(b->va, md, 0, a->size, MappleGPUVM::leafAttrs(false, readOnly)) ||
		    (h = addBO(b)) == 0) {
			b->prepared = false;    /* completed below */
			b->release();
			return kIOReturnNoMemory;
		}
		fVM->userWired += (int64_t)a->size;
		fVM->userPrepares++;
		fGPU->fUserWired += (int64_t)a->size;
		fGPU->fUserPrepares++;
		a->handle = h;
		a->offset = b->va;
		return kIOReturnSuccess;
	});
	if (ret != kIOReturnSuccess) {
		md->complete();
	}
	md->release();
	return ret;
}

IOReturn
MappleGPUUserClient::syncBO(struct vgpu_sync_bo *a)
{
	struct vgpu_bo_sync_op *ops = NULL;
	size_t osize = 0;
	IOReturn ret = copyinArray(a->ops, a->op_count, MAX_SYNC_OPS, sizeof(*ops), (void **)&ops, &osize);
	if (ret != kIOReturnSuccess || ops == NULL) {
		return ret;
	}
	ret = gated(VGPU_SEL_SYNC_BO, ^IOReturn {
		for (uint32_t i = 0; i < a->op_count; i++) {
			MappleGPUBO *b = bo(ops[i].handle);
			if (b == NULL) {
				return kIOReturnNotFound;
			}
			if (ops[i].type > VGPU_SYNC_CPU_CACHE_FLUSH_INVALIDATE) {
				return kIOReturnBadArgument;
			}
			if (b->cacheable) {
				b->cacheOp(ops[i].offset, ops[i].size,
				    ops[i].type == VGPU_SYNC_CPU_CACHE_FLUSH_INVALIDATE ? kMappleCacheFlush : kMappleCacheClean);
			}
		}
		return kIOReturnSuccess;
	});
	IOFreeData(ops, osize);
	return ret;
}

/* IOConnectMapMemory64(memoryType = BO handle): the BO's pages, in the cache mode they were created with. */
IOReturn
MappleGPUUserClient::clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory)
{
	return fGPU->gate()->runActionBlock(^IOReturn {
		if (type == VGPU_MAP_COMPLETION) {
			/* K0h: the completion page, read-only for the process */
			if (fVM == NULL || fVM->completionMD == NULL) {
				return kIOReturnNotOpen;
			}
			fVM->completionMD->retain();
			*options = kIOMapReadOnly;
			*memory = fVM->completionMD;
			return kIOReturnSuccess;
		}
		MappleGPUBO *b = bo(type);
		if (b == NULL || b->memory == NULL || b->kind == kMappleBOHeap) {
			return kIOReturnBadArgument;
		}
		fGPU->boMapped(b);
		if (b->memOffset != 0 && b->mapMD == NULL) {
			b->mapMD = IOSubMemoryDescriptor::withSubRange(b->memory, b->memOffset, b->vaSize, kIODirectionInOut);
			if (b->mapMD == NULL) {
				return kIOReturnNoMemory;
			}
		}
		IOMemoryDescriptor *md = b->mapMD ? b->mapMD : b->memory;
		md->retain();
		*options = 0;
		*memory = md;
		return kIOReturnSuccess;
	});
}
#endif /* MT6781 */
