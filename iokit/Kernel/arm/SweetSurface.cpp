/*
 * Built-in IOSurface for SWEET.
 *
 * Apple's IOSurface kext is closed. IOSurface.framework (iOS 16.7) finds
 * IOServiceNameMatching("IOCoreSurfaceRoot"), opens it with type 0 and
 * talks to the user client below:
 *
 *   sel 13  limits: struct out 0x18, u16 at +0x16 must be 1 (ABI version),
 *           +0 offset alignment mask, +4 bytes-per-row alignment mask
 *   sel 0   create: scalar [parent id], struct in = serialized property
 *           dictionary (binary or XML), struct out = lock result (0xa68)
 *   sel 4   lookup by id: scalar [id], struct out = lock result
 *   sel 34  lookup by mach port: scalar [port name], struct out = lock result
 *   sel 35  create mach port: scalars [id, options], scalar out [port name]
 *   traps   0/1 use count +/- (id, category), 2 lock / 3 unlock
 *           (id, options, &seed), 4/5 release (id)
 *
 * The lock result tells the library where the surface lives in the
 * caller's address space: +0 pixels, +8 shared state page (registry id,
 * seed at +0xc), +0x10 timestamps, then id, sizes and geometry (layout in
 * sweet_surface_fill_result). Every surface is wired kernel memory mapped
 * into each task that creates or looks it up; mappings are counted per
 * client and dropped on release or close.
 *
 *   sel 9   SetValue: struct in {u32 id, u32 ?, binary-serialized [value,
 *           key] or [dict] (merge)}, struct out u32 new generation
 *   sel 10  copy the values: struct in {u32 id, u32 0, u8 0}, struct out
 *           (descriptor, >= 16 KiB) {u32 length, u32 generation, binary
 *           serialized dictionary}
 *   sel 11  RemoveValue: struct in {u32 id, u32 ?, key, NUL} (empty key =
 *           all), struct out u32 new generation
 *   sel 27  bulk attachments: struct in 0x88 {0x70 bytes, u64 size, u64
 *           mask of changed fields, u32 id}, copied into the state page
 * Values live in the kernel; the state page tells every client when its
 * cache is stale: +0x20 u32 generation ((g + 2) | 1 after each change),
 * +0x28 u32 serialized size (>= 1 once anything was set), +0x30 the bulk
 * attachments (colour primaries, transfer function, ... read by clients
 * straight from the page).
 *
 * Notifications, accelerators and shared events are accepted and ignored
 * (logged once per selector).
 */
#include "SweetSurface.h"

#if defined(SWEET)
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/system.h>
#include <libkern/c++/OSArray.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSUnserialize.h>
#include <sys/proc.h>

enum {
	kSweetSurfaceCreate             = 0,
	kSweetSurfaceLookup             = 4,
	kSweetSurfaceSetValue           = 9,
	kSweetSurfaceCopyValues         = 10,
	kSweetSurfaceRemoveValue        = 11,
	kSweetSurfaceLimits             = 13,
	kSweetSurfaceSetBulk            = 27,
	kSweetSurfaceLookupFromPort     = 34,
	kSweetSurfaceCreatePort         = 35,
};

enum {
	kSweetSurfaceTrapIncrementUse   = 0,
	kSweetSurfaceTrapDecrementUse   = 1,
	kSweetSurfaceTrapLock           = 2,
	kSweetSurfaceTrapUnlock         = 3,
	kSweetSurfaceTrapRelease        = 4,
	kSweetSurfaceTrapReleaseFailed  = 5,
};

#define SWEET_SURFACE_RESULT_SIZE       0xa68
#define SWEET_SURFACE_MAX_ID            4096
#define SWEET_SURFACE_MAX_DIM           16384
#define SWEET_SURFACE_MAX_ALLOC         (256u << 20)
#define SWEET_SURFACE_MAX_PROPS         (64u << 10)
#define SWEET_SURFACE_ROW_ALIGN         64
#define SWEET_SURFACE_TIMESTAMPS        0x800   /* offset in the state page */
#define SWEET_SURFACE_STATE_SEED        0xc     /* u32 in the state page */
#define SWEET_SURFACE_STATE_USE         0x18    /* u32 x 2: holders per category (IOSurfaceIsInUse reads the u64) */
#define SWEET_SURFACE_STATE_VALUE_GEN   0x20    /* u32 */
#define SWEET_SURFACE_STATE_VALUE_SIZE  0x28    /* u32 */
#define SWEET_SURFACE_STATE_BULK        0x30    /* 0x70 bytes */
#define SWEET_SURFACE_BULK_SIZE         0x70
#define SWEET_SURFACE_MAX_VALUES        (256u << 10)    /* serialized values per surface */
#define kIOSurfaceLockReadOnly          0x1
#define kIOSurfaceLockAvoidSync         0x2     /* do not wait for pending GPU access: kIOReturnCannotLock instead */
#define SWEET_SURFACE_LOCK_WAIT_MS      1000    /* longest wait for the GPU in IOSurfaceLock */

static IOLock       *sweet_surface_lock;
static SweetSurface *sweet_surface_table[SWEET_SURFACE_MAX_ID];
static uint32_t      sweet_surface_users[SWEET_SURFACE_MAX_ID];
static uint32_t      sweet_surface_next_id = 1;
static uint32_t      sweet_surface_creates_logged;
static uint64_t      sweet_surface_wired;       /* bytes of buffer surfaces */
uint32_t             sweet_surface_scanout_w, sweet_surface_scanout_h;
static uint64_t      sweet_surface_contig;      /* bytes of contiguous (scanout) surfaces */
#define SWEET_SURFACE_CONTIG_MAX        (96ull << 20)
#define SWEET_SURFACE_WIRED_MAX         (1536ull << 20)

OSDefineMetaClassAndStructors(SweetSurface, OSObject);

void
SweetSurface::free()
{
	if (OSDynamicCast(IOBufferMemoryDescriptor, memory) != NULL) {
		OSAddAtomic64(-(SInt64)allocSize, (volatile SInt64 *)&sweet_surface_wired);
	}
	if (contig) {
		OSAddAtomic64(-(SInt64)allocSize, (volatile SInt64 *)&sweet_surface_contig);
	}
	OSSafeReleaseNULL(kernelMap);
	OSSafeReleaseNULL(memory);
	OSSafeReleaseNULL(state);
	OSSafeReleaseNULL(values);
	OSSafeReleaseNULL(gpuFence);
	OSSafeReleaseNULL(gpuPrev);
	OSObject::free();
}

uintptr_t
SweetSurface::kernelAddress()
{
	IOBufferMemoryDescriptor *buffer = OSDynamicCast(IOBufferMemoryDescriptor, memory);
	if (buffer != NULL) {
		return (uintptr_t)buffer->getBytesNoCopy();
	}
	IOLockLock(sweet_surface_lock);
	if (kernelMap == NULL && memory != NULL) {
		kernelMap = memory->map(kIOMapWriteCombineCache);
	}
	IOLockUnlock(sweet_surface_lock);
	return kernelMap != NULL ? (uintptr_t)kernelMap->getVirtualAddress() : 0;
}

/* Clean + invalidate (dc civac: never loses a CPU write) the whole surface, through the kernel mapping. */
static uint64_t
sweet_surface_clean(SweetSurface *s)
{
	IOBufferMemoryDescriptor *buffer = OSDynamicCast(IOBufferMemoryDescriptor, s->memory);
	if (buffer == NULL) {
		return 0;               /* physical (panel) memory: write-combined, nothing cached */
	}
	uintptr_t p = (uintptr_t)buffer->getBytesNoCopy(), end = p + buffer->getLength();
	for (uintptr_t line = p & ~(uintptr_t)63; line < end; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r"(line) : "memory");
	}
	__asm__ volatile ("dsb sy" : : : "memory");
	return end - p;
}

uint64_t
sweet_surface_before_device_read(SweetSurface *s)
{
	return OSCompareAndSwap(1, 0, &s->cpuDirty) ? sweet_surface_clean(s) : 0;
}

/* New contents for whoever compares seeds (the Metal driver copies a surface in again only when it moved). */
void
sweet_surface_seed_bump(SweetSurface *s)
{
	if (s->state != NULL) {
		OSIncrementAtomic((volatile SInt32 *)((uint8_t *)s->state->getBytesNoCopy() + SWEET_SURFACE_STATE_SEED));
	}
}

/* The kernel wrote the pixels with the CPU (scaler): like an IOSurfaceUnlock after a write lock. */
void
sweet_surface_cpu_wrote(SweetSurface *s)
{
	if (OSDynamicCast(IOBufferMemoryDescriptor, s->memory) != NULL) {
		OSCompareAndSwap(0, 1, &s->cpuDirty);
	}
	sweet_surface_seed_bump(s);
}

uint64_t
sweet_surface_before_cpu_access(SweetSurface *s)
{
	return OSCompareAndSwap(1, 0, &s->gpuDirty) ? sweet_surface_clean(s) : 0;
}

void
sweet_surface_device_wrote(SweetSurface *s)
{
	if (OSDynamicCast(IOBufferMemoryDescriptor, s->memory) != NULL) {
		OSCompareAndSwap(0, 1, &s->gpuDirty);
	}
}

static uint32_t *
sweet_surface_seed(SweetSurface *s)
{
	return (uint32_t *)((uint8_t *)s->state->getBytesNoCopy() + SWEET_SURFACE_STATE_SEED);
}

/*
 * Give the surface an id and a table slot; the table holds one retain and a
 * user count (creators, client mappings, kernel holders). Lock held.
 */
static bool
sweet_surface_insert_locked(SweetSurface *s)
{
	for (uint32_t n = 0; n < SWEET_SURFACE_MAX_ID - 1; n++) {
		uint32_t id = sweet_surface_next_id;
		sweet_surface_next_id = id + 1 < SWEET_SURFACE_MAX_ID ? id + 1 : 1;
		if (sweet_surface_table[id] == NULL) {
			s->id = id;
			s->retain();
			sweet_surface_table[id] = s;
			sweet_surface_users[id] = 1;
			*(uint64_t *)s->state->getBytesNoCopy() = id;   /* registry id */
			return true;
		}
	}
	return false;
}

/* Add a user to a surface that may have left the table (port lookups). */
static bool
sweet_surface_get(SweetSurface *s)
{
	bool ok = true;
	IOLockLock(sweet_surface_lock);
	if (sweet_surface_table[s->id] == s) {
		sweet_surface_users[s->id]++;
	} else if (sweet_surface_table[s->id] == NULL) {
		s->retain();
		sweet_surface_table[s->id] = s;
		sweet_surface_users[s->id] = 1;
	} else {
		ok = false;
	}
	IOLockUnlock(sweet_surface_lock);
	return ok;
}

/* Drop a user; the last one removes the surface from the table. */
static void
sweet_surface_put(SweetSurface *s)
{
	bool drop = false;
	IOLockLock(sweet_surface_lock);
	if (sweet_surface_table[s->id] == s && --sweet_surface_users[s->id] == 0) {
		sweet_surface_table[s->id] = NULL;
		drop = true;
	}
	IOLockUnlock(sweet_surface_lock);
	if (drop) {
		s->release();
	}
}

SweetSurface *
sweet_surface_copy(uint32_t id)
{
	SweetSurface *s = NULL;
	if (id == 0 || id >= SWEET_SURFACE_MAX_ID || sweet_surface_lock == NULL) {
		return NULL;
	}
	IOLockLock(sweet_surface_lock);
	s = sweet_surface_table[id];
	if (s != NULL) {
		s->retain();
	}
	IOLockUnlock(sweet_surface_lock);
	return s;
}

/* The table lock; the framebuffer may publish a surface before the root starts. */
static void
sweet_surface_init_lock(void)
{
	if (sweet_surface_lock == NULL) {
		IOLock *lock = IOLockAlloc();
		if (!OSCompareAndSwapPtr(NULL, lock, &sweet_surface_lock)) {
			IOLockFree(lock);
		}
	}
}

static SweetSurface *
sweet_surface_alloc(void)
{
	sweet_surface_init_lock();
	SweetSurface *s = OSTypeAlloc(SweetSurface);
	if (s == NULL || !s->init()) {
		OSSafeReleaseNULL(s);
		return NULL;
	}
	s->state = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,
	    kIODirectionInOut | kIOMemoryKernelUserShared, page_size, page_size);
	if (s->state == NULL) {
		s->release();
		return NULL;
	}
	bzero(s->state->getBytesNoCopy(), page_size);
	return s;
}

static bool
sweet_surface_publish(SweetSurface *s)
{
	bool ok;
	IOLockLock(sweet_surface_lock);
	ok = sweet_surface_insert_locked(s);
	IOLockUnlock(sweet_surface_lock);
	return ok;
}

SweetSurface *
sweet_surface_create_physical(uint64_t phys, uint32_t width, uint32_t height,
    uint32_t bytesPerRow, uint32_t pixelFormat)
{
	SweetSurface *s = sweet_surface_alloc();
	if (s == NULL) {
		return NULL;
	}
	s->width = width;
	s->height = height;
	s->bytesPerRow = bytesPerRow;
	s->bytesPerElement = 4;
	s->elementWidth = s->elementHeight = 1;
	s->pixelFormat = pixelFormat;
	s->allocSize = (uint32_t)round_page(bytesPerRow * height);
	s->memory = IOMemoryDescriptor::withPhysicalAddress(phys, s->allocSize, kIODirectionInOut);
	s->mapCache = kIOMapWriteCombineCache;
	if (s->memory == NULL || !sweet_surface_publish(s)) {
		s->release();
		return NULL;
	}
	return s;
}

mach_port_name_t
sweet_surface_port_in_task(SweetSurface *s, task_t task)
{
	mach_port_name_t name = 0;
	IOUserClient::copyPortNameForObjectInTask(task, s, &name);
	return name;
}

static uint32_t
sweet_dict_u32(OSDictionary *dict, const char *key, uint32_t def)
{
	OSNumber *n = OSDynamicCast(OSNumber, dict->getObject(key));
	return n != NULL ? n->unsigned32BitValue() : def;
}

static uint32_t
sweet_div_up(uint32_t a, uint32_t b)
{
	return b > 1 ? (a + b - 1) / b : a;
}

static uint32_t
sweet_row_bytes(uint32_t width, uint32_t ew, uint32_t bpe)
{
	uint64_t row = (uint64_t)sweet_div_up(width, ew) * bpe;
	return (uint32_t)((row + SWEET_SURFACE_ROW_ALIGN - 1) & ~(uint64_t)(SWEET_SURFACE_ROW_ALIGN - 1));
}

/* Geometry from the creation properties (IOSurface.h keys). */
static IOReturn
sweet_surface_layout(SweetSurface *s, OSDictionary *props)
{
	uint64_t end;

	s->width = sweet_dict_u32(props, "IOSurfaceWidth", 0);
	s->height = sweet_dict_u32(props, "IOSurfaceHeight", 0);
	s->bytesPerElement = (uint16_t)sweet_dict_u32(props, "IOSurfaceBytesPerElement", 4);
	s->elementWidth = (uint8_t)sweet_dict_u32(props, "IOSurfaceElementWidth", 1);
	s->elementHeight = (uint8_t)sweet_dict_u32(props, "IOSurfaceElementHeight", 1);
	s->pixelFormat = sweet_dict_u32(props, "IOSurfacePixelFormat", 0);
	s->cacheMode = sweet_dict_u32(props, "IOSurfaceCacheMode", 0);
	s->offset = sweet_dict_u32(props, "IOSurfaceOffset", 0);
	if (s->width > SWEET_SURFACE_MAX_DIM || s->height > SWEET_SURFACE_MAX_DIM ||
	    s->bytesPerElement > 64 || s->elementWidth == 0 || s->elementHeight == 0) {
		return kIOReturnBadArgument;
	}
	s->bytesPerRow = sweet_dict_u32(props, "IOSurfaceBytesPerRow",
	    sweet_row_bytes(s->width, s->elementWidth, s->bytesPerElement));
	if (s->bytesPerRow < (uint64_t)sweet_div_up(s->width, s->elementWidth) * s->bytesPerElement) {
		return kIOReturnBadArgument;
	}
	end = (uint64_t)s->offset + (uint64_t)s->bytesPerRow * sweet_div_up(s->height, s->elementHeight);

	OSArray *planes = OSDynamicCast(OSArray, props->getObject("IOSurfacePlaneInfo"));
	s->planeCount = 0;
	if (planes != NULL) {
		uint64_t next = s->offset;
		if (planes->getCount() > SWEET_SURFACE_MAX_PLANES) {
			return kIOReturnBadArgument;
		}
		for (unsigned int i = 0; i < planes->getCount(); i++) {
			OSDictionary *pd = OSDynamicCast(OSDictionary, planes->getObject(i));
			struct sweet_surface_plane *p = &s->planes[i];
			if (pd == NULL) {
				return kIOReturnBadArgument;
			}
			p->width = sweet_dict_u32(pd, "IOSurfacePlaneWidth", s->width);
			p->height = sweet_dict_u32(pd, "IOSurfacePlaneHeight", s->height);
			p->bytesPerElement = (uint16_t)sweet_dict_u32(pd, "IOSurfacePlaneBytesPerElement", 1);
			p->elementWidth = (uint8_t)sweet_dict_u32(pd, "IOSurfacePlaneElementWidth", 1);
			p->elementHeight = (uint8_t)sweet_dict_u32(pd, "IOSurfacePlaneElementHeight", 1);
			if (p->width > SWEET_SURFACE_MAX_DIM || p->height > SWEET_SURFACE_MAX_DIM ||
			    p->bytesPerElement > 64 || p->elementWidth == 0 || p->elementHeight == 0) {
				return kIOReturnBadArgument;
			}
			p->bytesPerRow = sweet_dict_u32(pd, "IOSurfacePlaneBytesPerRow",
			    sweet_row_bytes(p->width, p->elementWidth, p->bytesPerElement));
			p->offset = sweet_dict_u32(pd, "IOSurfacePlaneOffset", (uint32_t)next);
			p->size = sweet_dict_u32(pd, "IOSurfacePlaneSize",
			    p->bytesPerRow * sweet_div_up(p->height, p->elementHeight));
			next = round_page((uint64_t)p->offset + p->size);
			if ((uint64_t)p->offset + p->size > end) {
				end = (uint64_t)p->offset + p->size;
			}
		}
		s->planeCount = planes->getCount();
	}

	uint64_t alloc = sweet_dict_u32(props, "IOSurfaceAllocSize", 0);
	if (alloc < end) {
		alloc = end;
	}
	alloc = round_page(alloc);
	if (alloc == 0 || alloc > SWEET_SURFACE_MAX_ALLOC) {
		return kIOReturnNoMemory;
	}
	s->allocSize = (uint32_t)alloc;
	return kIOReturnSuccess;
}

/* Per-client mapping of one surface. */
class SweetSurfaceRef : public OSObject {
	OSDeclareDefaultStructors(SweetSurfaceRef);

public:
	SweetSurface *surface;
	IOMemoryMap  *data;
	IOMemoryMap  *state;
	uint32_t      count;
	uint32_t      use[2];         /* this client's use count holds (traps 0/1), per category */

	bool init() APPLE_KEXT_OVERRIDE { use[0] = use[1] = 0; return OSObject::init(); }
	void free() APPLE_KEXT_OVERRIDE;
};

/*
 * IOSurface use counts across processes (IOSurface.framework: the library counts per process and calls trap 0 on
 * 0 -> 1, trap 1 on 1 -> 0; IOSurfaceIsInUse reads the u64 at state page +0x18, u32 per category). CoreAnimation
 * uses it to know when an app's backing store is free again (CABackingStore buffer_is_unused).
 */
static void
sweet_surface_use(SweetSurface *s, uint32_t category, int delta)
{
	volatile SInt32 *c = (volatile SInt32 *)((uint8_t *)s->state->getBytesNoCopy() + SWEET_SURFACE_STATE_USE +
	    4 * (category & 1));
	OSAddAtomic(delta, c);
}

OSDefineMetaClassAndStructors(SweetSurfaceRef, OSObject);

void
SweetSurfaceRef::free()
{
	OSSafeReleaseNULL(data);
	OSSafeReleaseNULL(state);
	if (surface != NULL) {
		for (uint32_t c = 0; c < 2; c++) {
			if (use[c] != 0) {
				sweet_surface_use(surface, c, -(int)use[c]);     /* the client went away holding it */
			}
		}
		sweet_surface_put(surface);
		surface->release();
		surface = NULL;
	}
	OSObject::free();
}

class IOCoreSurfaceRoot : public IOService {
	OSDeclareDefaultStructors(IOCoreSurfaceRoot);

public:
	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(IOCoreSurfaceRoot, IOService);

bool
IOCoreSurfaceRoot::start(IOService *provider)
{
	if (!IOService::start(provider)) {
		return false;
	}
	sweet_surface_init_lock();
	setProperty(kIOUserClientClassKey, "IOSurfaceRootUserClient");
	registerService();
	IOLog("SWEET SURF: IOCoreSurfaceRoot registered\n");
	return true;
}

class IOSurfaceRootUserClient : public IOUserClient {
	OSDeclareDefaultStructors(IOSurfaceRootUserClient);

	task_t   fTask;
	IOLock  *fLock;
	OSArray *fRefs;

	SweetSurfaceRef *findRef(uint32_t id, unsigned int *index);
	IOReturn addRef(SweetSurface *s, SweetSurfaceRef **refOut, bool *existing);
	IOReturn returnSurface(SweetSurface *s, IOExternalMethodArguments *args);
	IOReturn create(IOExternalMethodArguments *args);
	IOReturn values(uint32_t selector, IOExternalMethodArguments *args);

public:
	bool initWithTask(task_t owningTask, void *securityID, UInt32 type) APPLE_KEXT_OVERRIDE;
	void free() APPLE_KEXT_OVERRIDE;
	IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target,
	    void *reference) APPLE_KEXT_OVERRIDE;
	IOExternalTrap *getTargetAndTrapForIndex(IOService **targetP, UInt32 index) APPLE_KEXT_OVERRIDE;

	IOReturn trapUseCount(void *id, void *category, void *, void *, void *, void *);
	IOReturn trapUseCountDown(void *id, void *category, void *, void *, void *, void *);
	IOReturn useCount(uintptr_t id, uintptr_t category, int delta);
	IOReturn trapLock(void *id, void *options, void *seedOut, void *, void *, void *);
	IOReturn trapUnlock(void *id, void *options, void *seedOut, void *, void *, void *);
	IOReturn trapRelease(void *id, void *, void *, void *, void *, void *);
};

OSDefineMetaClassAndStructors(IOSurfaceRootUserClient, IOUserClient);

static uint64_t sweet_surf_logged;      /* selectors / traps already logged (< 64) */

static void
sweet_surf_log_once(const char *what, uint32_t n)
{
	uint32_t bit = (what[0] == 't' ? 48 : 0) + n;
	if (bit >= 64 || (sweet_surf_logged & (1ULL << bit))) {
		return;
	}
	sweet_surf_logged |= 1ULL << bit;
	char name[32];
	proc_selfname(name, sizeof(name));
	IOLog("SWEET SURF: %s %s %u\n", name, what, n);
}

bool
IOSurfaceRootUserClient::initWithTask(task_t owningTask, void *securityID, UInt32 type)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type)) {
		return false;
	}
	fTask = owningTask;
	fLock = IOLockAlloc();
	fRefs = OSArray::withCapacity(16);
	return fLock != NULL && fRefs != NULL;
}

void
IOSurfaceRootUserClient::free()
{
	OSSafeReleaseNULL(fRefs);
	if (fLock != NULL) {
		IOLockFree(fLock);
		fLock = NULL;
	}
	IOUserClient::free();
}

IOReturn
IOSurfaceRootUserClient::clientClose(void)
{
	IOLockLock(fLock);
	fRefs->flushCollection();
	IOLockUnlock(fLock);
	terminate();
	return kIOReturnSuccess;
}

/* Lock held. */
SweetSurfaceRef *
IOSurfaceRootUserClient::findRef(uint32_t id, unsigned int *index)
{
	for (unsigned int i = 0; i < fRefs->getCount(); i++) {
		SweetSurfaceRef *ref = (SweetSurfaceRef *)fRefs->getObject(i);
		if (ref->surface->id == id) {
			if (index != NULL) {
				*index = i;
			}
			return ref;
		}
	}
	return NULL;
}

/*
 * Count one more create/lookup of s by this client, mapping it into the
 * task the first time. Takes over the caller's user count on s.
 *
 * A lookup of a surface this client already has maps nothing and counts
 * nothing (*existing): IOSurface.framework then gets base address -1 in the
 * lock result and reuses its cached IOSurfaceClient. Counting it instead
 * made the framework build a second client for the same id; releasing that
 * one (traps 4 + 5) unmapped the memory under the first (backboardd
 * snapshot crash in CGDataProviderCreateWithData).
 */
IOReturn
IOSurfaceRootUserClient::addRef(SweetSurface *s, SweetSurfaceRef **refOut, bool *existing)
{
	*existing = false;
	IOLockLock(fLock);
	SweetSurfaceRef *ref = findRef(s->id, NULL);
	if (ref != NULL && ref->surface == s) {
		IOLockUnlock(fLock);
		sweet_surface_put(s);
		*refOut = ref;
		*existing = true;
		return kIOReturnSuccess;
	}
	IOLockUnlock(fLock);

	ref = OSTypeAlloc(SweetSurfaceRef);
	if (ref == NULL || !ref->init()) {
		OSSafeReleaseNULL(ref);
		sweet_surface_put(s);
		return kIOReturnNoMemory;
	}
	s->retain();
	ref->surface = s;
	ref->count = 1;
	ref->data = s->memory->createMappingInTask(fTask, 0, kIOMapAnywhere | s->mapCache);
	ref->state = s->state->createMappingInTask(fTask, 0, kIOMapAnywhere | kIOMapReadOnly);
	if (ref->data == NULL || ref->state == NULL) {
		ref->release();
		return kIOReturnVMError;
	}
	IOLockLock(fLock);
	SweetSurfaceRef *raced = findRef(s->id, NULL);
	if (raced != NULL && raced->surface == s) {
		/* Another thread mapped it meanwhile: count there, drop ours. */
		raced->count++;
		IOLockUnlock(fLock);
		ref->release();
		*refOut = raced;
		return kIOReturnSuccess;
	}
	fRefs->setObject(ref);
	IOLockUnlock(fLock);
	ref->release();         /* fRefs holds it */
	*refOut = ref;
	return kIOReturnSuccess;
}

static void
put32(uint8_t *p, uint32_t off, uint32_t v)
{
	memcpy(p + off, &v, 4);
}

static void
put64(uint8_t *p, uint32_t off, uint64_t v)
{
	memcpy(p + off, &v, 8);
}

/* IOSurface.framework's lock result (copied to IOSurfaceClient + 0x68). */
static void
sweet_surface_fill_result(SweetSurface *s, SweetSurfaceRef *ref, uint8_t *out)
{
	bzero(out, SWEET_SURFACE_RESULT_SIZE);
	put64(out, 0x00, ref->data->getAddress());
	put64(out, 0x08, ref->state->getAddress());
	put64(out, 0x10, ref->state->getAddress() + SWEET_SURFACE_TIMESTAMPS);
	put32(out, 0x18, s->id);
	put32(out, 0x1c, s->allocSize);
	put32(out, 0x20, s->width);
	put32(out, 0x24, s->height);
	put32(out, 0x28, s->bytesPerRow);
	put32(out, 0x2c, s->offset);
	put32(out, 0x30, s->pixelFormat);
	put32(out, 0x34, s->allocSize);
	put32(out, 0x38, s->planeCount);
	memcpy(out + 0x40, &s->bytesPerElement, 2);
	out[0x42] = s->elementWidth;
	out[0x43] = s->elementHeight;
	put32(out, 0x44, s->cacheMode);
	put32(out, 0x50, s->parentID);
	put32(out, 0x54, 1);
	for (uint32_t i = 0; i < s->planeCount; i++) {
		uint8_t *p = out + 0x68 + i * 0x80;
		const struct sweet_surface_plane *pl = &s->planes[i];
		put32(p, 0x00, pl->width);
		put32(p, 0x04, pl->height);
		put32(p, 0x08, pl->bytesPerRow);
		put32(p, 0x0c, pl->offset);
		put32(p, 0x14, pl->size);
		memcpy(p + 0x18, &pl->bytesPerElement, 2);
		p[0x1a] = pl->elementWidth;
		p[0x1b] = pl->elementHeight;
	}
}

static IOReturn
sweet_struct_out(IOExternalMethodArguments *args, const void *buf, uint32_t len)
{
	if (args->structureOutputDescriptor != NULL) {
		if (len > args->structureOutputDescriptor->getLength()) {
			return kIOReturnNoSpace;
		}
		args->structureOutputDescriptor->writeBytes(0, buf, len);
		args->structureOutputDescriptorSize = len;
		return kIOReturnSuccess;
	}
	if (args->structureOutput == NULL || len > args->structureOutputSize) {
		return kIOReturnNoSpace;
	}
	memcpy(args->structureOutput, buf, len);
	args->structureOutputSize = len;
	return kIOReturnSuccess;
}

/* Map s for this client and answer with its lock result. Takes s's user count. */
IOReturn
IOSurfaceRootUserClient::returnSurface(SweetSurface *s, IOExternalMethodArguments *args)
{
	SweetSurfaceRef *ref = NULL;
	bool existing = false;
	IOReturn ret = addRef(s, &ref, &existing);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	uint8_t *out = (uint8_t *)IOMallocZeroData(SWEET_SURFACE_RESULT_SIZE);
	if (out == NULL) {
		ret = kIOReturnNoMemory;
	} else {
		sweet_surface_fill_result(s, ref, out);
		if (existing) {
			put64(out, 0x00, ~0ULL);        /* "already yours": use the cached client */
		}
		ret = sweet_struct_out(args, out, SWEET_SURFACE_RESULT_SIZE);
		IOFreeData(out, SWEET_SURFACE_RESULT_SIZE);
	}
	if (ret != kIOReturnSuccess && !existing) {
		/* The caller never learns the id: undo this create/lookup. */
		trapRelease((void *)(uintptr_t)s->id, NULL, NULL, NULL, NULL, NULL);
	}
	return ret;
}

IOReturn
IOSurfaceRootUserClient::create(IOExternalMethodArguments *args)
{
	const char *buf = (const char *)args->structureInput;
	uint32_t len = args->structureInputSize;
	char *copy = NULL;
	IOReturn ret;

	if (args->structureInputDescriptor != NULL) {
		len = (uint32_t)args->structureInputDescriptor->getLength();
		if (len == 0 || len > SWEET_SURFACE_MAX_PROPS) {
			return kIOReturnBadArgument;
		}
		copy = (char *)IOMallocData(len);
		if (copy == NULL) {
			return kIOReturnNoMemory;
		}
		args->structureInputDescriptor->readBytes(0, copy, len);
		buf = copy;
	}
	OSObject *obj = (buf != NULL && len > 0) ? OSUnserializeXML(buf, len) : NULL;
	if (copy != NULL) {
		IOFreeData(copy, len);
	}
	OSDictionary *props = OSDynamicCast(OSDictionary, obj);
	if (props == NULL) {
		OSSafeReleaseNULL(obj);
		IOLog("SWEET SURF: create: bad property dictionary (%u bytes)\n", len);
		return kIOReturnBadArgument;
	}

	SweetSurface *s = sweet_surface_alloc();
	if (s == NULL) {
		props->release();
		return kIOReturnNoMemory;
	}
	s->parentID = args->scalarInputCount > 0 ? (uint32_t)args->scalarInput[0] : 0;
	ret = sweet_surface_layout(s, props);
	if (ret == kIOReturnSuccess) {
		IOBufferMemoryDescriptor *mem = NULL;
		if ((uint64_t)OSAddAtomic64(s->allocSize, (volatile SInt64 *)&sweet_surface_wired) +
		    s->allocSize <= SWEET_SURFACE_WIRED_MAX) {
			if (sweet_surface_scanout_w != 0 && s->width == sweet_surface_scanout_w &&
			    s->height == sweet_surface_scanout_h && s->bytesPerElement == 4 && s->planeCount == 0 &&
			    sweet_surface_contig + s->allocSize <= SWEET_SURFACE_CONTIG_MAX) {
				/* the display's DMA has no IOMMU here: one physical range below 4 GiB, 16 KiB aligned */
				mem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, kIODirectionInOut |
				    kIOMemoryKernelUserShared | kIOMemoryPhysicallyContiguous, s->allocSize, 0x00000000ffffc000ull);
				if (mem != NULL) {
					OSAddAtomic64(s->allocSize, (volatile SInt64 *)&sweet_surface_contig);
					s->contig = true;
				}
			}
			if (mem == NULL) {
				mem = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,
				    kIODirectionInOut | kIOMemoryKernelUserShared, s->allocSize, page_size);
			}
		}
		if (mem == NULL) {
			OSAddAtomic64(-(SInt64)s->allocSize, (volatile SInt64 *)&sweet_surface_wired);
			ret = kIOReturnNoMemory;
		} else {
			bzero(mem->getBytesNoCopy(), s->allocSize);
			s->memory = mem;
			s->cpuDirty = 1;        /* the zeros may still be in the CPU caches */
			s->mapCache = kIOMapDefaultCache;
			if (!sweet_surface_publish(s)) {
				ret = kIOReturnNoResources;
			}
		}
	}
	if (ret != kIOReturnSuccess || sweet_surface_creates_logged < 64) {
		char name[32];
		proc_selfname(name, sizeof(name));
		sweet_surface_creates_logged++;
		IOLog("SWEET SURF: %s create id %u %ux%u fmt 0x%x bpe %u bpr %u planes %u size %u keys %u -> 0x%x\n",
		    name, s->id, s->width, s->height, s->pixelFormat, s->bytesPerElement,
		    s->bytesPerRow, s->planeCount, s->allocSize, props->getCount(), ret);
	}
	props->release();
	if (ret != kIOReturnSuccess) {
		s->release();
		return ret;
	}
	ret = returnSurface(s, args);   /* takes the creator's user count */
	s->release();
	return ret;
}

static SweetSurface *sweet_surface_held(IOLock *lock, OSArray *refs, uint32_t id);

/* The struct input, inband or out of line (> 4096 bytes); IOFreeData(*buf, *len) when done. */
static IOReturn
sweet_struct_in(IOExternalMethodArguments *args, uint32_t max, uint8_t **buf, uint32_t *len)
{
	uint32_t n = args->structureInputDescriptor ? (uint32_t)args->structureInputDescriptor->getLength() :
	    args->structureInputSize;
	if (n == 0 || n > max) {
		return kIOReturnBadArgument;
	}
	*buf = (uint8_t *)IOMallocData(n);
	if (*buf == NULL) {
		return kIOReturnNoMemory;
	}
	if (args->structureInputDescriptor != NULL) {
		IOMemoryDescriptor *md = args->structureInputDescriptor;
		if (md->prepare() != kIOReturnSuccess) {
			IOFreeData(*buf, n);
			return kIOReturnVMError;
		}
		md->readBytes(0, *buf, n);
		md->complete();
	} else {
		memcpy(*buf, args->structureInput, n);
	}
	*len = n;
	return kIOReturnSuccess;
}

/* New value generation and size in the state page (surface table lock held). */
static uint32_t
sweet_surface_values_changed(SweetSurface *s)
{
	uint8_t *st = (uint8_t *)s->state->getBytesNoCopy();
	uint32_t gen, size = 1;
	memcpy(&gen, st + SWEET_SURFACE_STATE_VALUE_GEN, 4);
	gen = (gen + 2) | 1;
	OSSerialize *ser = OSSerialize::binaryWithCapacity(1024);
	if (ser != NULL && s->values != NULL && s->values->serialize(ser)) {
		size = ser->getLength();
	}
	OSSafeReleaseNULL(ser);
	memcpy(st + SWEET_SURFACE_STATE_VALUE_SIZE, &size, 4);
	__asm__ volatile ("dmb ish" : : : "memory");    /* size before the generation clients compare */
	memcpy(st + SWEET_SURFACE_STATE_VALUE_GEN, &gen, 4);
	return gen;
}

/* Bulk attachment fields: mask bit -> offset and length in the 0x70-byte block. */
static const struct { uint8_t off, len; } sweet_surface_bulk_fields[15] = {
	{ 0x00, 32 }, { 0x20, 16 }, { 0x30, 8 }, { 0x38, 1 }, { 0x39, 1 }, { 0x3a, 1 }, { 0x3b, 1 }, { 0x3c, 1 },
	{ 0x3d, 1 }, { 0x3e, 1 }, { 0x3f, 1 }, { 0x40, 24 }, { 0x58, 4 }, { 0x5c, 8 }, { 0x64, 1 },
};

/* IOSurfaceSetValue(s) / RemoveValue / CopyValue / bulk attachments (selectors 9, 11, 10, 27). */
IOReturn
IOSurfaceRootUserClient::values(uint32_t selector, IOExternalMethodArguments *args)
{
	uint8_t *in = NULL;
	uint32_t len = 0;
	IOReturn ret = sweet_struct_in(args, SWEET_SURFACE_MAX_VALUES + 16, &in, &len);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	uint32_t id;
	memcpy(&id, in + (selector == kSweetSurfaceSetBulk ? 0x80 : 0), 4);
	SweetSurface *s = (selector == kSweetSurfaceSetBulk ? len >= 0x84 : len >= 8) ?
	    sweet_surface_held(fLock, fRefs, id) : NULL;
	if (s == NULL) {
		IOFreeData(in, len);
		return kIOReturnNotFound;
	}
	uint32_t gen = 0;
	uint8_t *st = (uint8_t *)s->state->getBytesNoCopy();
	sweet_surf_log_once("sel", selector);

	switch (selector) {
	case kSweetSurfaceSetValue: {
		OSObject *obj = OSUnserializeXML((const char *)in + 8, len - 8);
		OSArray *arr = OSDynamicCast(OSArray, obj);
		OSDictionary *merge = arr && arr->getCount() == 1 ? OSDynamicCast(OSDictionary, arr->getObject(0)) : NULL;
		OSString *key = arr && arr->getCount() == 2 ? OSDynamicCast(OSString, arr->getObject(1)) : NULL;
		if (merge == NULL && key == NULL) {
			IOLog("SWEET SURF: SetValue on %u: unexpected payload (%u bytes)\n", id, len);
			ret = kIOReturnBadArgument;
		} else {
			IOLockLock(sweet_surface_lock);
			if (s->values == NULL) {
				s->values = OSDictionary::withCapacity(4);
			}
			if (s->values == NULL) {
				ret = kIOReturnNoMemory;
			} else if (merge != NULL) {
				s->values->merge(merge);
			} else {
				s->values->setObject(key, arr->getObject(0));
			}
			if (ret == kIOReturnSuccess) {
				gen = sweet_surface_values_changed(s);
				uint32_t size;
				memcpy(&size, st + SWEET_SURFACE_STATE_VALUE_SIZE, 4);
				if (size > SWEET_SURFACE_MAX_VALUES) {      /* keep it bounded: undo */
					if (merge != NULL) {
						s->values->flushCollection();
					} else {
						s->values->removeObject(key);
					}
					gen = sweet_surface_values_changed(s);
					ret = kIOReturnNoMemory;
				}
			}
			IOLockUnlock(sweet_surface_lock);
		}
		OSSafeReleaseNULL(obj);
		break;
	}
	case kSweetSurfaceRemoveValue: {
		char *key = (char *)in + 8;
		in[len - 1] = 0;
		IOLockLock(sweet_surface_lock);
		if (s->values != NULL) {
			if (len <= 9 || key[0] == 0) {
				s->values->flushCollection();
			} else {
				s->values->removeObject(key);
			}
		}
		gen = sweet_surface_values_changed(s);  /* size stays >= 1: other clients must notice */
		IOLockUnlock(sweet_surface_lock);
		break;
	}
	case kSweetSurfaceCopyValues: {
		IOLockLock(sweet_surface_lock);
		OSSerialize *ser = OSSerialize::binaryWithCapacity(1024);
		OSDictionary *empty = s->values == NULL ? OSDictionary::withCapacity(1) : NULL;
		OSDictionary *dict = s->values != NULL ? s->values : empty;
		if (ser == NULL || dict == NULL || !dict->serialize(ser)) {
			ret = kIOReturnNoMemory;
		} else {
			uint32_t n = ser->getLength(), total = 8 + n;
			uint8_t *out = (uint8_t *)IOMallocData(total);
			if (out == NULL) {
				ret = kIOReturnNoMemory;
			} else {
				memcpy(out, &n, 4);
				memcpy(out + 4, st + SWEET_SURFACE_STATE_VALUE_GEN, 4);
				memcpy(out + 8, ser->text(), n);
				IOMemoryDescriptor *md = args->structureOutputDescriptor;   /* >= 16 KiB: out of line */
				if (md != NULL && md->prepare() != kIOReturnSuccess) {
					ret = kIOReturnVMError;
				} else {
					ret = sweet_struct_out(args, out, total);   /* kIOReturnNoSpace: the client retries bigger */
					if (md != NULL) {
						md->complete();
					}
				}
				IOFreeData(out, total);
			}
		}
		OSSafeReleaseNULL(ser);
		OSSafeReleaseNULL(empty);
		IOLockUnlock(sweet_surface_lock);
		break;
	}
	case kSweetSurfaceSetBulk: {
		uint64_t size, mask;
		memcpy(&size, in + 0x70, 8);
		memcpy(&mask, in + 0x78, 8);
		IOLockLock(sweet_surface_lock);
		for (uint32_t bit = 0; bit < 15; bit++) {
			uint32_t off = sweet_surface_bulk_fields[bit].off, n = sweet_surface_bulk_fields[bit].len;
			if ((mask & (1ull << bit)) && off + n <= size && off + n <= SWEET_SURFACE_BULK_SIZE) {
				memcpy(st + SWEET_SURFACE_STATE_BULK + off, in + off, n);
			}
		}
		uint32_t vsize;
		memcpy(&vsize, st + SWEET_SURFACE_STATE_VALUE_SIZE, 4);
		if (vsize == 0) {
			sweet_surface_values_changed(s);        /* CopyAllValues returns nothing while the size is 0 */
		}
		IOLockUnlock(sweet_surface_lock);
		break;
	}
	}
	IOFreeData(in, len);
	s->release();
	if (selector == kSweetSurfaceSetValue || selector == kSweetSurfaceRemoveValue) {
		if (args->structureOutput != NULL && args->structureOutputSize >= 4) {
			memcpy(args->structureOutput, &gen, 4);
			args->structureOutputSize = 4;
		}
	}
	return ret;
}

IOReturn
IOSurfaceRootUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    __unused IOExternalMethodDispatch *dispatch, __unused OSObject *target,
    __unused void *reference)
{
	uint64_t arg0 = args->scalarInputCount > 0 ? args->scalarInput[0] : 0;

	switch (selector) {
	case kSweetSurfaceLimits: {
		uint8_t limits[0x18] = { 0 };
		put32(limits, 0x0, SWEET_SURFACE_ROW_ALIGN - 1);
		put32(limits, 0x4, SWEET_SURFACE_ROW_ALIGN - 1);
		limits[0x16] = 1;
		sweet_surf_log_once("sel", selector);
		return sweet_struct_out(args, limits, sizeof(limits));
	}
	case kSweetSurfaceCreate:
		return create(args);
	case kSweetSurfaceSetValue:
	case kSweetSurfaceCopyValues:
	case kSweetSurfaceRemoveValue:
	case kSweetSurfaceSetBulk:
		return values(selector, args);
	case kSweetSurfaceLookup: {
		SweetSurface *s = sweet_surface_copy((uint32_t)arg0);
		IOReturn ret = kIOReturnNotFound;
		if (s != NULL) {
			ret = sweet_surface_get(s) ? returnSurface(s, args) : kIOReturnNotFound;
			s->release();
		}
		sweet_surf_log_once("sel", selector);
		return ret;
	}
	case kSweetSurfaceLookupFromPort: {
		OSObject *obj = NULL;
		IOReturn ret = IOUserClient::copyObjectForPortNameInTask(fTask, (mach_port_name_t)arg0, &obj);
		SweetSurface *s = OSDynamicCast(SweetSurface, obj);
		if (ret == kIOReturnSuccess) {
			ret = (s != NULL && sweet_surface_get(s)) ? returnSurface(s, args) : kIOReturnBadArgument;
		}
		OSSafeReleaseNULL(obj);
		sweet_surf_log_once("sel", selector);
		return ret;
	}
	case kSweetSurfaceCreatePort: {
		SweetSurface *s = sweet_surface_copy((uint32_t)arg0);
		if (s == NULL || args->scalarOutputCount < 1) {
			OSSafeReleaseNULL(s);
			return kIOReturnNotFound;
		}
		args->scalarOutput[0] = sweet_surface_port_in_task(s, fTask);
		args->scalarOutputCount = 1;
		s->release();
		sweet_surf_log_once("sel", selector);
		return kIOReturnSuccess;
	}
	default:
		/* Values, notifications, ownership, timestamps...: accept and ignore. */
		for (uint32_t i = 0; i < args->scalarOutputCount; i++) {
			args->scalarOutput[i] = 0;
		}
		if (args->structureOutput != NULL) {
			bzero(args->structureOutput, args->structureOutputSize);
		}
		sweet_surf_log_once("sel", selector);
		return kIOReturnSuccess;
	}
}

IOReturn
IOSurfaceRootUserClient::useCount(uintptr_t id, uintptr_t category, int delta)
{
	IOLockLock(fLock);
	SweetSurfaceRef *ref = findRef((uint32_t)id, NULL);
	uint32_t c = (uint32_t)category & 1;
	if (ref != NULL && (delta > 0 || ref->use[c] != 0)) {
		ref->use[c] += delta > 0 ? 1 : -1;
		sweet_surface_use(ref->surface, c, delta > 0 ? 1 : -1);
	}
	IOLockUnlock(fLock);
	return ref != NULL ? kIOReturnSuccess : kIOReturnNotFound;
}

IOReturn
IOSurfaceRootUserClient::trapUseCount(void *id, void *category, void *, void *, void *, void *)
{
	return useCount((uintptr_t)id, (uintptr_t)category, 1);
}

IOReturn
IOSurfaceRootUserClient::trapUseCountDown(void *id, void *category, void *, void *, void *, void *)
{
	return useCount((uintptr_t)id, (uintptr_t)category, -1);
}

static IOReturn
sweet_surface_seed_out(SweetSurface *s, void *seedOut, bool bump)
{
	uint32_t seed = bump ? OSIncrementAtomic((volatile SInt32 *)sweet_surface_seed(s)) + 1 :
	    *sweet_surface_seed(s);
	if (seedOut != NULL && copyout(&seed, (user_addr_t)seedOut, sizeof(seed)) != 0) {
		return kIOReturnBadArgument;
	}
	return kIOReturnSuccess;
}

/* Retained surface this client created or looked up, or NULL. */
static SweetSurface *
sweet_surface_held(IOLock *lock, OSArray *refs, uint32_t id)
{
	SweetSurface *s = NULL;
	IOLockLock(lock);
	for (unsigned int i = 0; i < refs->getCount(); i++) {
		SweetSurfaceRef *ref = (SweetSurfaceRef *)refs->getObject(i);
		if (ref->surface->id == id) {
			s = ref->surface;
			s->retain();
			break;
		}
	}
	IOLockUnlock(lock);
	return s;
}

IOReturn
IOSurfaceRootUserClient::trapLock(void *id, void *options, void *seedOut,
    void *, void *, void *)
{
	SweetSurface *s = sweet_surface_held(fLock, fRefs, (uint32_t)(uintptr_t)id);
	if (s == NULL) {
		return kIOReturnNotFound;
	}
	/* like Apple's: the lock waits for the GPU jobs pending on the surface (AvoidSync: fail instead) */
	if (!viva_gpu_surface_idle(s, NULL)) {
		if ((uintptr_t)options & kIOSurfaceLockAvoidSync) {
			s->release();
			return kIOReturnCannotLock;
		}
		viva_gpu_surface_wait(s, SWEET_SURFACE_LOCK_WAIT_MS);
	}
	sweet_surface_before_cpu_access(s);
	if (!((uintptr_t)options & kIOSurfaceLockReadOnly) && OSDynamicCast(IOBufferMemoryDescriptor, s->memory)) {
		OSCompareAndSwap(0, 1, &s->cpuDirty);
	}
	IOReturn ret = sweet_surface_seed_out(s, seedOut, false);
	s->release();
	return ret;
}

IOReturn
IOSurfaceRootUserClient::trapUnlock(void *id, void *options, void *seedOut,
    void *, void *, void *)
{
	SweetSurface *s = sweet_surface_held(fLock, fRefs, (uint32_t)(uintptr_t)id);
	if (s == NULL) {
		return kIOReturnNotFound;
	}
	if (!((uintptr_t)options & kIOSurfaceLockReadOnly) && OSDynamicCast(IOBufferMemoryDescriptor, s->memory)) {
		OSCompareAndSwap(0, 1, &s->cpuDirty);   /* the writes made while locked */
	}
	IOReturn ret = sweet_surface_seed_out(s, seedOut,
	    !((uintptr_t)options & kIOSurfaceLockReadOnly));
	s->release();
	return ret;
}

IOReturn
IOSurfaceRootUserClient::trapRelease(void *id, void *, void *, void *, void *, void *)
{
	unsigned int index;
	IOLockLock(fLock);
	SweetSurfaceRef *ref = findRef((uint32_t)(uintptr_t)id, &index);
	if (ref != NULL && --ref->count == 0) {
		fRefs->removeObject(index);     /* unmaps and drops the user count */
	}
	IOLockUnlock(fLock);
	return ref != NULL ? kIOReturnSuccess : kIOReturnNotFound;
}

static IOExternalTrap sweet_surface_traps[] = {
	[kSweetSurfaceTrapIncrementUse]  = { NULL, (IOTrap) & IOSurfaceRootUserClient::trapUseCount },
	[kSweetSurfaceTrapDecrementUse]  = { NULL, (IOTrap) & IOSurfaceRootUserClient::trapUseCountDown },
	[kSweetSurfaceTrapLock]          = { NULL, (IOTrap) & IOSurfaceRootUserClient::trapLock },
	[kSweetSurfaceTrapUnlock]        = { NULL, (IOTrap) & IOSurfaceRootUserClient::trapUnlock },
	[kSweetSurfaceTrapRelease]       = { NULL, (IOTrap) & IOSurfaceRootUserClient::trapRelease },
	[kSweetSurfaceTrapReleaseFailed] = { NULL, (IOTrap) & IOSurfaceRootUserClient::trapRelease },
};

IOExternalTrap *
IOSurfaceRootUserClient::getTargetAndTrapForIndex(IOService **targetP, UInt32 index)
{
	if (index >= sizeof(sweet_surface_traps) / sizeof(sweet_surface_traps[0])) {
		sweet_surf_log_once("trap", index);
		return NULL;
	}
	sweet_surf_log_once("trap", index);
	*targetP = this;
	return &sweet_surface_traps[index];
}
#endif /* SWEET */
