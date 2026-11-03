/*
 * viva (MT6781): user client of the Mali driver (see VivaGPU.h), the
 * kernel side of viva_gpu_uapi.h. One connection = one GPU address space
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
 * Timeouts are absolute CLOCK_MONOTONIC nanoseconds (mach_continuous_time
 * on Darwin). WAIT_BO answers kIOReturnBusy (EBUSY) when the BO is still
 * busy at the deadline, what Panfrost expects.
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include <IOKit/IOSubMemoryDescriptor.h>
#include <IOKit/system.h>
#include <kern/clock.h>
#include <mach/mach_time.h>

#include "VivaGPU.h"
#include "SweetSurface.h"

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define MAX_BOS_PER_SUBMIT      8192
#define MAX_SYNCS               256
#define MAX_SYNC_OPS            256
#define MAX_BO_SIZE             (1024ull << 20)
#define EXEC_MAX                (16ull << 20)   /* the shader program counter is 24-bit: never cross 16 MiB */
#define HEAP_ALIGN              (2ull << 20)

OSDefineMetaClassAndStructors(VivaGPUUserClient, IOUserClient);

/* Absolute CLOCK_MONOTONIC ns -> absolute-time deadline (0 = forever); *expired if already past. */
static uint64_t
deadlineFor(int64_t timeoutNs, bool *expired)
{
	uint64_t now;
	absolutetime_to_nanoseconds(mach_continuous_time(), &now);
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
VivaGPUUserClient::initWithTask(task_t owningTask, void *securityID, UInt32 type)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type)) {
		return false;
	}
	fTask = owningTask;
	fBOCap = 256;
	fBOs = IONewZero(VivaGPUBO *, fBOCap);
	fSyncCap = 64;
	fSyncobjs = IONewZero(Syncobj, fSyncCap);
	return fBOs != NULL && fSyncobjs != NULL;
}

bool
VivaGPUUserClient::start(IOService *provider)
{
	fGPU = OSDynamicCast(VivaGPUAccelerator, provider);
	if (fGPU == NULL || !IOUserClient::start(provider)) {
		return false;
	}
	fGPU->gate()->runActionBlock(^IOReturn {
		fVM = VivaGPUVM::create(fGPU);
		return kIOReturnSuccess;
	});
	return fVM != NULL;
}

void
VivaGPUUserClient::closeAll(void)
{
	if (fGPU == NULL) {
		return;
	}
	fGPU->gate()->runActionBlock(^IOReturn {
		for (uint32_t h = 1; h < fBOCap; h++) {
			OSSafeReleaseNULL(fBOs[h]);
		}
		for (uint32_t h = 1; h < fSyncCap; h++) {
			fGPU->fenceWake(fSyncobjs[h].fence);    /* its waiters see the syncobj gone */
			OSSafeReleaseNULL(fSyncobjs[h].fence);
			fSyncobjs[h].used = false;
		}
		OSSafeReleaseNULL(fVM);         /* jobs still running keep their own references */
		fGPU->wakeWaiters();
		return kIOReturnSuccess;
	});
}

IOReturn
VivaGPUUserClient::clientClose(void)
{
	if (!fClosed) {
		fClosed = true;
		closeAll();
	}
	terminate();
	return kIOReturnSuccess;
}

void
VivaGPUUserClient::free(void)
{
	if (!fClosed) {
		fClosed = true;
		closeAll();
	}
	if (fBOs != NULL) {
		IODelete(fBOs, VivaGPUBO *, fBOCap);
	}
	if (fSyncobjs != NULL) {
		IODelete(fSyncobjs, Syncobj, fSyncCap);
	}
	IOUserClient::free();
}

/* ---- handles (gated) ---- */

VivaGPUBO *
VivaGPUUserClient::bo(uint32_t handle)
{
	return (handle > 0 && handle < fBOCap) ? fBOs[handle] : NULL;
}

uint32_t
VivaGPUUserClient::addBO(VivaGPUBO *b)
{
	uint32_t h;
	for (h = 1; h < fBOCap && fBOs[h] != NULL; h++) {
	}
	if (h == fBOCap) {
		if (fBOCap >= 65536) {
			return 0;
		}
		VivaGPUBO **bigger = IONewZero(VivaGPUBO *, fBOCap * 2);
		if (bigger == NULL) {
			return 0;
		}
		bcopy(fBOs, bigger, fBOCap * sizeof(*bigger));
		IODelete(fBOs, VivaGPUBO *, fBOCap);
		fBOs = bigger;
		fBOCap *= 2;
	}
	fBOs[h] = b;
	return h;
}

bool
VivaGPUUserClient::syncobjValid(uint32_t handle)
{
	return handle > 0 && handle < fSyncCap && fSyncobjs[handle].used;
}

/* ---- calls ---- */

IOReturn
VivaGPUUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
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
	[VGPU_TRAP_CALL] = { NULL, (IOTrap) & VivaGPUUserClient::trapCall },
};

IOExternalTrap *
VivaGPUUserClient::getTargetAndTrapForIndex(IOService **targetP, UInt32 index)
{
	if (index >= sizeof(vgpu_traps) / sizeof(vgpu_traps[0])) {
		return NULL;
	}
	*targetP = this;
	return &vgpu_traps[index];
}

IOReturn
VivaGPUUserClient::trapCall(void *selectorArg, void *uaddr, void *, void *, void *, void *)
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
VivaGPUUserClient::gated(uint32_t selector, IOReturn (^block)(void))
{
	uint64_t t0 = mach_absolute_time();
	return fGPU->gate()->runActionBlock(^IOReturn {
		fGPU->gateWaited(selector, mach_absolute_time() - t0);
		return block();
	});
}

IOReturn
VivaGPUUserClient::dispatch(uint32_t selector, void *arg)
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
				Syncobj *s = &fSyncobjs[handles[i]];
				fGPU->fenceWake(s->fence);      /* waiters of the old fence re-check the syncobj */
				OSSafeReleaseNULL(s->fence);
				if (signal) {
					s->fence = VivaGPUFence::create(true);
				}
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
			VivaGPUBO *b = bo(a->handle);
			if (b == NULL) {
				return kIOReturnNotFound;
			}
			a->offset = a->handle;  /* memoryType for IOConnectMapMemory64 */
			return kIOReturnSuccess;
		}
		case VGPU_SEL_GET_BO_OFFSET: {
			struct vgpu_get_bo_offset *a = (struct vgpu_get_bo_offset *)arg;
			VivaGPUBO *b = bo(a->handle);
			if (b == NULL) {
				return kIOReturnNotFound;
			}
			a->offset = b->va + b->gpuOffset;
			return kIOReturnSuccess;
		}
		case VGPU_SEL_GEM_CLOSE: {
			struct vgpu_gem_close *a = (struct vgpu_gem_close *)arg;
			VivaGPUBO *b = bo(a->handle);
			if (b == NULL) {
				return kIOReturnNotFound;
			}
			fBOs[a->handle] = NULL;
			b->release();           /* jobs using it hold their own reference */
			return kIOReturnSuccess;
		}
		case VGPU_SEL_WAIT_BO: {
			struct vgpu_wait_bo *a = (struct vgpu_wait_bo *)arg;
			VivaGPUBO *b = bo(a->handle);
			if (b == NULL) {
				return kIOReturnNotFound;
			}
			bool expired;
			uint64_t deadline = deadlineFor(a->timeout_ns, &expired);
			b->retain();
			IOReturn ret = kIOReturnSuccess;
			while (!VivaGPUAccelerator::boIdle(b)) {
				if (expired) {
					ret = kIOReturnBusy;
					break;
				}
				/* the BO's last job, else (an IOSurface) the last job on the surface in any connection */
				VivaGPUFence *f = (b->busy != NULL && !b->busy->signalled) ? b->busy :
				    OSDynamicCast(VivaGPUFence, b->surface->gpuFence);
				f->retain();
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
			VivaGPUBO *b = bo(a->handle);
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
			fSyncobjs[h].fence = (a->flags & VGPU_SYNCOBJ_CREATE_SIGNALED) ? VivaGPUFence::create(true) : NULL;
			a->handle = h;
			return kIOReturnSuccess;
		}
		case VGPU_SEL_SYNCOBJ_DESTROY: {
			struct vgpu_syncobj_destroy *a = (struct vgpu_syncobj_destroy *)arg;
			if (!syncobjValid(a->handle)) {
				return kIOReturnBadArgument;
			}
			fGPU->fenceWake(fSyncobjs[a->handle].fence);
			OSSafeReleaseNULL(fSyncobjs[a->handle].fence);
			fSyncobjs[a->handle].used = false;
			return kIOReturnSuccess;
		}
		case VGPU_SEL_SYNCOBJ_TRANSFER: {
			struct vgpu_syncobj_transfer *a = (struct vgpu_syncobj_transfer *)arg;
			if (!syncobjValid(a->src) || !syncobjValid(a->dst)) {
				return kIOReturnBadArgument;
			}
			VivaGPUFence *f = fSyncobjs[a->src].fence;
			if (f != NULL) {
				f->retain();
			}
			fGPU->fenceWake(fSyncobjs[a->dst].fence);
			OSSafeReleaseNULL(fSyncobjs[a->dst].fence);
			fSyncobjs[a->dst].fence = f;
			return kIOReturnSuccess;
		}
		case VGPU_SEL_QUERY_BO_INFO: {
			struct vgpu_query_bo_info *a = (struct vgpu_query_bo_info *)arg;
			VivaGPUBO *b = bo(a->handle);
			if (b == NULL) {
				return kIOReturnNotFound;
			}
			a->create_flags = b->flags;
			a->extra_flags = (b->kind == kVivaBOSurface || b->kind == kVivaBOUser) ? VGPU_BO_IS_IMPORTED : 0;
			return kIOReturnSuccess;
		}
		default:
			return kIOReturnBadArgument;
		}
	});
}

IOReturn
VivaGPUUserClient::createBO(struct vgpu_create_bo *a)
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
		VivaGPUBO *b = new VivaGPUBO;
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
			b->kind = kVivaBOHeap;
			b->chunkCount = (uint32_t)(size / HEAP_ALIGN);
			b->chunks = IONewZero(IOBufferMemoryDescriptor *, b->chunkCount);
			if (b->chunks == NULL) {
				b->release();
				return kIOReturnNoMemory;
			}
		} else {
			b->kind = kVivaBONormal;
			if (!fGPU->chargeMemory(fVM, (int64_t)size)) {
				b->vaSize = 0;          /* nothing charged: free() must not refund */
				b->release();
				return kIOReturnNoMemory;
			}
			/*
			 * K0b: write-combined unless userspace asks for VGPU_BO_WB_MMAP, as Linux maps panfrost BOs on a
			 * non-coherent GPU (Mesa with DRM 1.5 relies on it): CPU writes reach memory without any cache
			 * maintenance before or after jobs. vivagpuwc=0: cacheable, maintained by the kernel as before.
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
				cacheBuffer(mem, 0, size, kVivaCacheFlush);
			}
		}
		b->va = fVM->allocVA(size, align);
		if (b->va == 0) {
			b->release();
			return kIOReturnNoSpace;
		}
		if (heap) {
			fGPU->heapAdd(b);
		} else if (!fVM->map(b->va, b->memory, 0, size, VivaGPUVM::leafAttrs(exec))) {
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

IOReturn
VivaGPUUserClient::submit(struct vgpu_submit *a)
{
	if (a->jc == 0 || a->jc < VGPU_VA_START || a->jc >= VGPU_VA_END) {
		return kIOReturnBadArgument;
	}
	uint32_t *syncs = NULL, *handles = NULL;
	size_t ssize = 0, hsize = 0;
	IOReturn ret = copyinArray(a->in_syncs, a->in_sync_count, MAX_SYNCS, sizeof(uint32_t), (void **)&syncs, &ssize);
	if (ret == kIOReturnSuccess) {
		ret = copyinArray(a->bo_handles, a->bo_handle_count, MAX_BOS_PER_SUBMIT, sizeof(uint32_t),
		    (void **)&handles, &hsize);
	}
	if (ret == kIOReturnSuccess) {
		ret = gated(VGPU_SEL_SUBMIT, ^IOReturn {
			if (fVM == NULL) {
				return kIOReturnNotOpen;
			}
			if (a->out_sync != 0 && !syncobjValid(a->out_sync)) {
				return kIOReturnNotFound;
			}
			OSArray *deps = OSArray::withCapacity(a->in_sync_count ? a->in_sync_count : 1);
			OSArray *bos = OSArray::withCapacity(a->bo_handle_count ? a->bo_handle_count : 1);
			VivaGPUFence *fence = VivaGPUFence::create(false);
			IOReturn r = (deps && bos && fence) ? kIOReturnSuccess : kIOReturnNoMemory;
			for (uint32_t i = 0; r == kIOReturnSuccess && i < a->in_sync_count; i++) {
				if (!syncobjValid(syncs[i])) {
					r = kIOReturnNotFound;
				} else if (fSyncobjs[syncs[i]].fence != NULL && !fSyncobjs[syncs[i]].fence->signalled) {
					deps->setObject(fSyncobjs[syncs[i]].fence);
				}
			}
			for (uint32_t i = 0; r == kIOReturnSuccess && i < a->bo_handle_count; i++) {
				VivaGPUBO *b = bo(handles[i]);
				if (b == NULL) {
					r = kIOReturnNotFound;
				} else {
					bos->setObject(b);
				}
			}
			/*
			 * vivagpucache=2 (diagnosis): every CPU-maintained BO of the connection goes with the job, listed or
			 * not, so a BO userspace forgot in bo_handles still gets its CPU writes cleaned before the job.
			 */
			for (uint32_t h = 1; r == kIOReturnSuccess && fGPU->cacheMode() == 2 && h < fBOCap; h++) {
				VivaGPUBO *b = fBOs[h];
				if (b != NULL && (b->cacheMaint || b->lazySurface) && bos->getNextIndexOfObject(b, 0) == (unsigned int)-1) {
					bos->setObject(b);
				}
			}
			if (r == kIOReturnSuccess) {
				r = fGPU->submit(fVM, a->jc, a->requirements, deps, bos, fence);
			}
			if (r == kIOReturnSuccess && a->out_sync != 0) {
				/* waiters of the old fence re-check the syncobj; WAIT_FOR_SUBMIT waiters are generic ones */
				fGPU->fenceWake(fSyncobjs[a->out_sync].fence);
				OSSafeReleaseNULL(fSyncobjs[a->out_sync].fence);
				fSyncobjs[a->out_sync].fence = fence;
				fence->retain();
			}
			OSSafeReleaseNULL(deps);
			OSSafeReleaseNULL(bos);
			OSSafeReleaseNULL(fence);
			return r;
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

IOReturn
VivaGPUUserClient::syncobjWait(struct vgpu_syncobj_wait *a)
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
			VivaGPUFence *sleepFence = NULL;        /* the one fence that can complete the wait, if there is one */
			bool generic = false;
			for (uint32_t i = 0; i < a->count_handles; i++) {
				if (!syncobjValid(handles[i])) {
					return kIOReturnBadArgument;
				}
				VivaGPUFence *f = fSyncobjs[handles[i]].fence;
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
VivaGPUUserClient::importSurface(struct vgpu_import_iosurface *a)
{
	SweetSurface *s = sweet_surface_copy(a->surface_id);
	if (s == NULL) {
		return kIOReturnNotFound;
	}
	uint32_t planes = s->planeCount;
	if (s->memory == NULL || (planes == 0 ? a->plane != 0 : (a->plane >= planes || planes > SWEET_SURFACE_MAX_PLANES))) {
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
		VivaGPUBO *b = new VivaGPUBO;
		if (b == NULL || !b->init()) {
			OSSafeReleaseNULL(b);
			return kIOReturnNoMemory;
		}
		b->vm = fVM;
		fVM->retain();
		fVM->bos++;
		b->kind = kVivaBOSurface;
		b->surface = s;
		s->retain();
		b->memory = s->memory;
		b->memory->retain();
		b->prepared = b->memory->prepare() == kIOReturnSuccess;
		b->memOffset = mapStart;
		b->size = b->vaSize = mapSize;
		fVM->imported += (int64_t)mapSize;
		b->cacheable = OSDynamicCast(IOBufferMemoryDescriptor, s->memory) != NULL &&
		    s->mapCache != kIOMapWriteCombineCache && s->mapCache != kIOMapInhibitCache;
		b->cacheMaint = b->cacheable && fGPU->cacheMode() == 0;
		b->lazySurface = b->cacheable && !b->cacheMaint;
		b->va = fVM->allocVA(mapSize, mapSize >= HEAP_ALIGN ? HEAP_ALIGN : page_size);
		b->gpuOffset = planeOff - mapStart;
		if (!b->prepared || b->va == 0 || !fVM->map(b->va, b->memory, mapStart, mapSize, VivaGPUVM::leafAttrs(false))) {
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
VivaGPUUserClient::importUser(struct vgpu_import_userptr *a)
{
	if (a->addr == 0 || a->size == 0 || a->size > MAX_BO_SIZE || (a->addr & (page_size - 1)) ||
	    (a->size & (page_size - 1))) {
		return kIOReturnBadArgument;
	}
	IOMemoryDescriptor *md = IOMemoryDescriptor::withAddressRange(a->addr, a->size, kIODirectionInOut, fTask);
	if (md == NULL) {
		return kIOReturnNoMemory;
	}
	IOReturn ret = md->prepare();           /* wires the pages */
	if (ret != kIOReturnSuccess) {
		md->release();
		return ret;
	}
	ret = gated(VGPU_SEL_IMPORT_USERPTR, ^IOReturn {
		if (fVM == NULL) {
			return kIOReturnNotOpen;
		}
		VivaGPUBO *b = new VivaGPUBO;
		if (b == NULL || !b->init()) {
			OSSafeReleaseNULL(b);
			return kIOReturnNoMemory;
		}
		b->vm = fVM;
		fVM->retain();
		fVM->bos++;
		b->kind = kVivaBOUser;
		b->memory = md;
		md->retain();
		b->prepared = true;
		b->cacheable = true;
		b->size = b->vaSize = a->size;
		fVM->imported += (int64_t)a->size;
		b->cacheMaint = true;
		b->va = fVM->allocVA(a->size, a->size >= HEAP_ALIGN ? HEAP_ALIGN : page_size);
		uint32_t h = 0;
		if (b->va == 0 || !fVM->map(b->va, md, 0, a->size, VivaGPUVM::leafAttrs(false)) || (h = addBO(b)) == 0) {
			b->prepared = false;    /* completed below */
			b->release();
			return kIOReturnNoMemory;
		}
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
VivaGPUUserClient::syncBO(struct vgpu_sync_bo *a)
{
	struct vgpu_bo_sync_op *ops = NULL;
	size_t osize = 0;
	IOReturn ret = copyinArray(a->ops, a->op_count, MAX_SYNC_OPS, sizeof(*ops), (void **)&ops, &osize);
	if (ret != kIOReturnSuccess || ops == NULL) {
		return ret;
	}
	ret = gated(VGPU_SEL_SYNC_BO, ^IOReturn {
		for (uint32_t i = 0; i < a->op_count; i++) {
			VivaGPUBO *b = bo(ops[i].handle);
			if (b == NULL) {
				return kIOReturnNotFound;
			}
			if (ops[i].type > VGPU_SYNC_CPU_CACHE_FLUSH_INVALIDATE) {
				return kIOReturnBadArgument;
			}
			if (b->cacheable) {
				b->cacheOp(ops[i].offset, ops[i].size,
				    ops[i].type == VGPU_SYNC_CPU_CACHE_FLUSH_INVALIDATE ? kVivaCacheFlush : kVivaCacheClean);
			}
		}
		return kIOReturnSuccess;
	});
	IOFreeData(ops, osize);
	return ret;
}

/* IOConnectMapMemory64(memoryType = BO handle): the BO's pages, in the cache mode they were created with. */
IOReturn
VivaGPUUserClient::clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory)
{
	return fGPU->gate()->runActionBlock(^IOReturn {
		VivaGPUBO *b = bo(type);
		if (b == NULL || b->memory == NULL || b->kind == kVivaBOHeap) {
			return kIOReturnBadArgument;
		}
		fGPU->boMapped(b);
		IOMemoryDescriptor *md = b->memory;
		if (b->memOffset != 0) {
			md = IOSubMemoryDescriptor::withSubRange(b->memory, b->memOffset, b->vaSize, kIODirectionInOut);
			if (md == NULL) {
				return kIOReturnNoMemory;
			}
		} else {
			md->retain();
		}
		*options = 0;
		*memory = md;
		return kIOReturnSuccess;
	});
}
#endif /* SWEET */
