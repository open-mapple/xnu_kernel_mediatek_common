/*
 * Built-in IOSurface for MT6781.
 *
 * Apple's IOSurface kext is closed. IOSurface.framework (iOS 16.7) finds
 * IOServiceNameMatching("IOCoreSurfaceRoot"), opens it with type 0 and
 * talks to the user client below:
 *
 *   sel 13  limits: struct out 0x18, u16 at +0x16 must be 1 (ABI version),
 *           +0 offset alignment mask, +4 bytes-per-row alignment mask,
 *           +0xc / +0x10 largest width / height (IOSurfaceGetPropertyMaximum;
 *           PhotoLibraryServices clamps its thumbnails to the smaller)
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
 * mapple_surface_fill_result). Every surface is wired kernel memory mapped
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
#include "MappleSurface.h"
#include "MappleSurfaceAddress.h"

#if defined(MT6781)
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
#include <pexpert/pexpert.h>

enum {
	kMappleSurfaceCreate             = 0,
	kMappleSurfaceLookup             = 4,
	kMappleSurfaceSetValue           = 9,
	kMappleSurfaceCopyValues         = 10,
	kMappleSurfaceRemoveValue        = 11,
	kMappleSurfaceLimits             = 13,
	kMappleSurfaceSetBulk            = 27,
	kMappleSurfaceLookupFromPort     = 34,
	kMappleSurfaceCreatePort         = 35,
};

enum {
	kMappleSurfaceTrapIncrementUse   = 0,
	kMappleSurfaceTrapDecrementUse   = 1,
	kMappleSurfaceTrapLock           = 2,
	kMappleSurfaceTrapUnlock         = 3,
	kMappleSurfaceTrapRelease        = 4,
	kMappleSurfaceTrapReleaseFailed  = 5,
};

#define MAPPLE_SURFACE_RESULT_SIZE       0xa68
#define MAPPLE_SURFACE_MAX_ID            4096
#define MAPPLE_SURFACE_MAX_DIM           16384
#define MAPPLE_SURFACE_MAX_ALLOC         (256u << 20)
#define MAPPLE_SURFACE_MAX_PROPS         (64u << 10)
#define MAPPLE_SURFACE_ROW_ALIGN         64
#define MAPPLE_SURFACE_TIMESTAMPS        0x800   /* offset in the state page */
#define MAPPLE_SURFACE_STATE_SEED        0xc     /* u32 in the state page */
#define MAPPLE_SURFACE_STATE_USE         0x18    /* u32 x 2: holders per category (IOSurfaceIsInUse reads the u64) */
#define MAPPLE_SURFACE_STATE_VALUE_GEN   0x20    /* u32 */
#define MAPPLE_SURFACE_STATE_VALUE_SIZE  0x28    /* u32 */
#define MAPPLE_SURFACE_STATE_BULK        0x30    /* 0x70 bytes */
#define MAPPLE_SURFACE_BULK_SIZE         0x70
#define MAPPLE_SURFACE_MAX_VALUES        (256u << 10)    /* serialized values per surface */
#define kIOSurfaceLockReadOnly          0x1
#define kIOSurfaceLockAvoidSync         0x2     /* do not wait for pending GPU access: kIOReturnCannotLock instead */
#define MAPPLE_SURFACE_LOCK_WAIT_MS      1000    /* longest wait for the GPU in IOSurfaceLock */

static IOLock       *mapple_surface_lock;
static MappleSurface *mapple_surface_table[MAPPLE_SURFACE_MAX_ID];
static uint32_t      mapple_surface_users[MAPPLE_SURFACE_MAX_ID];
static uint32_t      mapple_surface_next_id = 1;
static uint32_t      mapple_surface_creates_logged;
static uint64_t      mapple_surface_wired;       /* bytes of buffer surfaces */
uint32_t             mapple_surface_scanout_w, mapple_surface_scanout_h;
static uint64_t      mapple_surface_contig;      /* bytes of contiguous (scanout) surfaces */
#define MAPPLE_SURFACE_CONTIG_MAX        (96ull << 20)
#define MAPPLE_SURFACE_WIRED_MAX         (1536ull << 20)

/*
 * 0184 display page pool (MappleSurface.h). Lock-free: a buffer is published before the count that covers it, and a
 * slot is taken / given back with a compare-and-swap (a surface may be freed with the table lock held).
 */
#define MAPPLE_SURFACE_POOL_MAX          8
static IOBufferMemoryDescriptor *mapple_surface_pool[MAPPLE_SURFACE_POOL_MAX];
static volatile UInt32 mapple_surface_pool_used[MAPPLE_SURFACE_POOL_MAX];
static int           mapple_surface_pool_rc[MAPPLE_SURFACE_POOL_MAX];      /* retain count with only the pool holding it */
static volatile UInt32 mapple_surface_pool_count;
static uint32_t      mapple_surface_pool_size;
static uint64_t      mapple_surface_from_pool, mapple_surface_from_contig, mapple_surface_sparse;
/* 0184 (P6) */
static int           mapple_surface_pitch64 = -1;        /* mp_surface_pitch64, read at the first create */
static uint64_t      mapple_surface_pitch_rounds;

OSDefineMetaClassAndStructors(MappleSurface, OSObject);

extern "C" void mapple_task_graphics_footprint(task_t task, int64_t bytes);   /* osfmk/kern/task.c */

/* mp_gpu_footprint (1 default): an IOSurface's pixels count in the footprint of the task that made it, as on iOS */
static bool
mapple_surface_footprint(void)
{
	static int on = -1;
	if (on < 0) {
		int v = 1;
		PE_parse_boot_argn("mp_gpu_footprint", &v, sizeof(v));
		on = v != 0;
	}
	return on;
}

void
MappleSurface::free()
{
	if (OSDynamicCast(IOBufferMemoryDescriptor, memory) != NULL) {
		OSAddAtomic64(-(SInt64)allocSize, (volatile SInt64 *)&mapple_surface_wired);
	}
	if (owner != TASK_NULL) {
		mapple_task_graphics_footprint(owner, -(int64_t)allocSize);
		task_deallocate(owner);
	}
	if (contig && poolSlot < 0) {
		OSAddAtomic64(-(SInt64)allocSize, (volatile SInt64 *)&mapple_surface_contig);
	}
	OSSafeReleaseNULL(kernelMap);
	if (client && memory != NULL) {
		memory->complete();
	}
	OSSafeReleaseNULL(memory);
	if (poolSlot >= 0) {
		/* the pool keeps its own reference: the buffer is free again once the surface's is gone */
		OSCompareAndSwap(1, 0, &mapple_surface_pool_used[poolSlot]);
	}
	OSSafeReleaseNULL(state);
	OSSafeReleaseNULL(values);
	OSSafeReleaseNULL(gpuFence);
	OSSafeReleaseNULL(gpuPrev);
	OSObject::free();
}

uintptr_t
MappleSurface::kernelAddress()
{
	IOBufferMemoryDescriptor *buffer = OSDynamicCast(IOBufferMemoryDescriptor, memory);
	if (buffer != NULL) {
		return (uintptr_t)buffer->getBytesNoCopy();
	}
	IOLockLock(mapple_surface_lock);
	if (kernelMap == NULL && memory != NULL) {
		kernelMap = memory->map(client ? kIOMapDefaultCache : kIOMapWriteCombineCache);
	}
	IOLockUnlock(mapple_surface_lock);
	return kernelMap != NULL ? (uintptr_t)kernelMap->getVirtualAddress() : 0;
}

/* Buffer surfaces and the creator's pages are cacheable; physical (panel) memory is write-combined. */
static bool
mapple_surface_cacheable(MappleSurface *s)
{
	return s->client || OSDynamicCast(IOBufferMemoryDescriptor, s->memory) != NULL;
}

/* Clean + invalidate (dc civac: never loses a CPU write) the whole surface, through the kernel mapping. */
static uint64_t
mapple_surface_clean(MappleSurface *s)
{
	if (!mapple_surface_cacheable(s)) {
		return 0;
	}
	uintptr_t p = s->kernelAddress(), end = p + s->memory->getLength();
	if (p == 0) {
		return 0;
	}
	for (uintptr_t line = p & ~(uintptr_t)63; line < end; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r"(line) : "memory");
	}
	__asm__ volatile ("dsb sy" : : : "memory");
	return end - p;
}

uint64_t
mapple_surface_before_device_read(MappleSurface *s)
{
	/* the creator writes its own pages without IOSurfaceLock: always cleaned */
	return OSCompareAndSwap(1, 0, &s->cpuDirty) || s->client ? mapple_surface_clean(s) : 0;
}

/* New contents for whoever compares seeds (the Metal driver copies a surface in again only when it moved). */
void
mapple_surface_seed_bump(MappleSurface *s)
{
	if (s->state != NULL) {
		OSIncrementAtomic((volatile SInt32 *)((uint8_t *)s->state->getBytesNoCopy() + MAPPLE_SURFACE_STATE_SEED));
	}
}

/* The kernel wrote the pixels with the CPU (scaler): like an IOSurfaceUnlock after a write lock. */
void
mapple_surface_cpu_wrote(MappleSurface *s)
{
	if (mapple_surface_cacheable(s)) {
		OSCompareAndSwap(0, 1, &s->cpuDirty);
	}
	mapple_surface_seed_bump(s);
}

uint64_t
mapple_surface_before_cpu_access(MappleSurface *s)
{
	return OSCompareAndSwap(1, 0, &s->gpuDirty) ? mapple_surface_clean(s) : 0;
}

void
mapple_surface_device_wrote(MappleSurface *s)
{
	if (mapple_surface_cacheable(s)) {
		OSCompareAndSwap(0, 1, &s->gpuDirty);
	}
}

static uint32_t *
mapple_surface_seed(MappleSurface *s)
{
	return (uint32_t *)((uint8_t *)s->state->getBytesNoCopy() + MAPPLE_SURFACE_STATE_SEED);
}

/*
 * Give the surface an id and a table slot; the table holds one retain and a
 * user count (creators, client mappings, kernel holders). Lock held.
 */
static bool
mapple_surface_insert_locked(MappleSurface *s)
{
	for (uint32_t n = 0; n < MAPPLE_SURFACE_MAX_ID - 1; n++) {
		uint32_t id = mapple_surface_next_id;
		mapple_surface_next_id = id + 1 < MAPPLE_SURFACE_MAX_ID ? id + 1 : 1;
		if (mapple_surface_table[id] == NULL) {
			s->id = id;
			s->retain();
			mapple_surface_table[id] = s;
			mapple_surface_users[id] = 1;
			*(uint64_t *)s->state->getBytesNoCopy() = id;   /* registry id */
			return true;
		}
	}
	return false;
}

/* Add a user to a surface that may have left the table (port lookups). */
static bool
mapple_surface_get(MappleSurface *s)
{
	bool ok = true;
	IOLockLock(mapple_surface_lock);
	if (mapple_surface_table[s->id] == s) {
		mapple_surface_users[s->id]++;
	} else if (mapple_surface_table[s->id] == NULL) {
		s->retain();
		mapple_surface_table[s->id] = s;
		mapple_surface_users[s->id] = 1;
	} else {
		ok = false;
	}
	IOLockUnlock(mapple_surface_lock);
	return ok;
}

/* Drop a user; the last one removes the surface from the table. */
static void
mapple_surface_put(MappleSurface *s)
{
	bool drop = false;
	IOLockLock(mapple_surface_lock);
	if (mapple_surface_table[s->id] == s && --mapple_surface_users[s->id] == 0) {
		mapple_surface_table[s->id] = NULL;
		drop = true;
	}
	IOLockUnlock(mapple_surface_lock);
	if (drop) {
		s->release();
	}
}

MappleSurface *
mapple_surface_copy(uint32_t id)
{
	MappleSurface *s = NULL;
	if (id == 0 || id >= MAPPLE_SURFACE_MAX_ID || mapple_surface_lock == NULL) {
		return NULL;
	}
	IOLockLock(mapple_surface_lock);
	s = mapple_surface_table[id];
	if (s != NULL) {
		s->retain();
	}
	IOLockUnlock(mapple_surface_lock);
	return s;
}

/* The table lock; the framebuffer may publish a surface before the root starts. */
static void
mapple_surface_init_lock(void)
{
	if (mapple_surface_lock == NULL) {
		IOLock *lock = IOLockAlloc();
		if (!OSCompareAndSwapPtr(NULL, lock, &mapple_surface_lock)) {
			IOLockFree(lock);
		}
	}
}

static MappleSurface *
mapple_surface_alloc(void)
{
	mapple_surface_init_lock();
	MappleSurface *s = OSTypeAlloc(MappleSurface);
	if (s == NULL || !s->init()) {
		OSSafeReleaseNULL(s);
		return NULL;
	}
	s->poolSlot = -1;
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
mapple_surface_publish(MappleSurface *s)
{
	bool ok;
	IOLockLock(mapple_surface_lock);
	ok = mapple_surface_insert_locked(s);
	IOLockUnlock(mapple_surface_lock);
	return ok;
}

MappleSurface *
mapple_surface_create_physical(uint64_t phys, uint32_t width, uint32_t height,
    uint32_t bytesPerRow, uint32_t pixelFormat)
{
	MappleSurface *s = mapple_surface_alloc();
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
	if (s->memory == NULL || !mapple_surface_publish(s)) {
		s->release();
		return NULL;
	}
	return s;
}

mach_port_name_t
mapple_surface_port_in_task(MappleSurface *s, task_t task)
{
	mach_port_name_t name = 0;
	IOUserClient::copyPortNameForObjectInTask(task, s, &name);
	return name;
}

static uint32_t
mapple_dict_u32(OSDictionary *dict, const char *key, uint32_t def)
{
	OSNumber *n = OSDynamicCast(OSNumber, dict->getObject(key));
	return n != NULL ? n->unsigned32BitValue() : def;
}

static uint32_t
mapple_div_up(uint32_t a, uint32_t b)
{
	return b > 1 ? (a + b - 1) / b : a;
}

static uint32_t
mapple_row_bytes(uint32_t width, uint32_t ew, uint32_t bpe)
{
	uint64_t row = (uint64_t)mapple_div_up(width, ew) * bpe;
	return (uint32_t)((row + MAPPLE_SURFACE_ROW_ALIGN - 1) & ~(uint64_t)(MAPPLE_SURFACE_ROW_ALIGN - 1));
}

/* Geometry from the creation properties (IOSurface.h keys); *needed: the bytes the layout covers. */
static IOReturn
mapple_surface_layout(MappleSurface *s, OSDictionary *props, uint64_t *needed)
{
	uint64_t end;

	s->width = mapple_dict_u32(props, "IOSurfaceWidth", 0);
	s->height = mapple_dict_u32(props, "IOSurfaceHeight", 0);
	s->bytesPerElement = (uint16_t)mapple_dict_u32(props, "IOSurfaceBytesPerElement", 4);
	s->elementWidth = (uint8_t)mapple_dict_u32(props, "IOSurfaceElementWidth", 1);
	s->elementHeight = (uint8_t)mapple_dict_u32(props, "IOSurfaceElementHeight", 1);
	s->pixelFormat = mapple_dict_u32(props, "IOSurfacePixelFormat", 0);
	s->cacheMode = mapple_dict_u32(props, "IOSurfaceCacheMode", 0);
	s->offset = mapple_dict_u32(props, "IOSurfaceOffset", 0);
	if (s->width > MAPPLE_SURFACE_MAX_DIM || s->height > MAPPLE_SURFACE_MAX_DIM ||
	    s->bytesPerElement > 64 || s->elementWidth == 0 || s->elementHeight == 0) {
		return kIOReturnBadArgument;
	}
	s->bytesPerRow = mapple_dict_u32(props, "IOSurfaceBytesPerRow",
	    mapple_row_bytes(s->width, s->elementWidth, s->bytesPerElement));
	if (s->bytesPerRow < (uint64_t)mapple_div_up(s->width, s->elementWidth) * s->bytesPerElement) {
		return kIOReturnBadArgument;
	}
	end = (uint64_t)s->offset + (uint64_t)s->bytesPerRow * mapple_div_up(s->height, s->elementHeight);

	OSArray *planes = OSDynamicCast(OSArray, props->getObject("IOSurfacePlaneInfo"));
	s->planeCount = 0;
	if (planes != NULL) {
		uint64_t next = s->offset;
		if (planes->getCount() > MAPPLE_SURFACE_MAX_PLANES) {
			return kIOReturnBadArgument;
		}
		for (unsigned int i = 0; i < planes->getCount(); i++) {
			OSDictionary *pd = OSDynamicCast(OSDictionary, planes->getObject(i));
			struct mapple_surface_plane *p = &s->planes[i];
			if (pd == NULL) {
				return kIOReturnBadArgument;
			}
			p->width = mapple_dict_u32(pd, "IOSurfacePlaneWidth", s->width);
			p->height = mapple_dict_u32(pd, "IOSurfacePlaneHeight", s->height);
			p->bytesPerElement = (uint16_t)mapple_dict_u32(pd, "IOSurfacePlaneBytesPerElement", 1);
			p->elementWidth = (uint8_t)mapple_dict_u32(pd, "IOSurfacePlaneElementWidth", 1);
			p->elementHeight = (uint8_t)mapple_dict_u32(pd, "IOSurfacePlaneElementHeight", 1);
			if (p->width > MAPPLE_SURFACE_MAX_DIM || p->height > MAPPLE_SURFACE_MAX_DIM ||
			    p->bytesPerElement > 64 || p->elementWidth == 0 || p->elementHeight == 0) {
				return kIOReturnBadArgument;
			}
			p->bytesPerRow = mapple_dict_u32(pd, "IOSurfacePlaneBytesPerRow",
			    mapple_row_bytes(p->width, p->elementWidth, p->bytesPerElement));
			p->offset = mapple_dict_u32(pd, "IOSurfacePlaneOffset", (uint32_t)next);
			p->size = mapple_dict_u32(pd, "IOSurfacePlaneSize",
			    p->bytesPerRow * mapple_div_up(p->height, p->elementHeight));
			next = round_page((uint64_t)p->offset + p->size);
			if ((uint64_t)p->offset + p->size > end) {
				end = (uint64_t)p->offset + p->size;
			}
		}
		s->planeCount = planes->getCount();
	}

	uint64_t alloc = mapple_dict_u32(props, "IOSurfaceAllocSize", 0);
	/*
	 * 0184 (P6 of gpu/docs/ANALISI-PRESTAZIONI-2026-11-06.md): a client pitch that is not a multiple of 64 (CA's
	 * home snapshot 1125x2436 with 4512, LA08 230x186 with 480) cannot be imported by the GPU, which copied such
	 * surfaces in and out at every use (10.7 MB each way, 23 ms stalls on the GPU queue). Single-plane surfaces get
	 * the pitch rounded up: clients read it back from the lock result (+0x28; CA, ImageIO, IOSurface.framework).
	 * Not when an explicit IOSurfaceAllocSize would no longer hold the rows (the client laid the memory out
	 * itself), nor with an offset or a parent. Off by default: it garbled the home screen's icons.
	 * mp_surface_pitch64=1: on; 2 (0187, to try): also those, the allocation grown to the rounded rows (CA's
	 * 1125x2436 snapshot and 787x1705 switcher surfaces declare exactly pitch x height and stayed unaligned: correct
	 * only if their writers read the pitch back).
	 */
	if (mapple_surface_pitch64 < 0) {
		int on = 0;
		PE_parse_boot_argn_alias("mp_surface_pitch64", "vivasurfpitch", &on, sizeof(on));
		mapple_surface_pitch64 = on;
	}
	if (mapple_surface_pitch64 && planes == NULL && s->offset == 0 && s->parentID == 0 &&
	    props->getObject("IOSurfaceAddress") == NULL &&
	    (s->bytesPerRow & (MAPPLE_SURFACE_ROW_ALIGN - 1)) != 0 && props->getObject("IOSurfaceBytesPerRow") != NULL) {
		uint64_t row = ((uint64_t)s->bytesPerRow + MAPPLE_SURFACE_ROW_ALIGN - 1) & ~(uint64_t)(MAPPLE_SURFACE_ROW_ALIGN - 1);
		uint64_t rend = row * mapple_div_up(s->height, s->elementHeight);
		bool ok = row <= UINT32_MAX && (alloc == 0 || rend <= alloc || mapple_surface_pitch64 == 2);
		static volatile SInt32 logged;
		if (OSIncrementAtomic(&logged) < 16) {
			IOLog("SURF: %ux%u fmt 0x%x pitch %u %s %llu%s\n", s->width, s->height, s->pixelFormat,
			    s->bytesPerRow, ok ? "->" : "kept (alloc size", ok ? row : alloc, ok ? "" : ")");
		}
		if (ok) {
			s->bytesPerRow = (uint32_t)row;
			end = rend;
			OSIncrementAtomic64((volatile SInt64 *)&mapple_surface_pitch_rounds);
		}
	}
	*needed = end;
	if (alloc < end) {
		alloc = end;
	}
	alloc = round_page(alloc);
	if (alloc == 0 || alloc > MAPPLE_SURFACE_MAX_ALLOC) {
		return kIOReturnNoMemory;
	}
	s->allocSize = (uint32_t)alloc;
	return kIOReturnSuccess;
}

/* Per-client mapping of one surface. */
class MappleSurfaceRef : public OSObject {
	OSDeclareDefaultStructors(MappleSurfaceRef);

public:
	MappleSurface *surface;
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
mapple_surface_use(MappleSurface *s, uint32_t category, int delta)
{
	volatile SInt32 *c = (volatile SInt32 *)((uint8_t *)s->state->getBytesNoCopy() + MAPPLE_SURFACE_STATE_USE +
	    4 * (category & 1));
	OSAddAtomic(delta, c);
}

OSDefineMetaClassAndStructors(MappleSurfaceRef, OSObject);

void
MappleSurfaceRef::free()
{
	OSSafeReleaseNULL(data);
	OSSafeReleaseNULL(state);
	if (surface != NULL) {
		for (uint32_t c = 0; c < 2; c++) {
			if (use[c] != 0) {
				mapple_surface_use(surface, c, -(int)use[c]);     /* the client went away holding it */
			}
		}
		mapple_surface_put(surface);
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
	mapple_surface_init_lock();
	setProperty(kIOUserClientClassKey, "IOSurfaceRootUserClient");
	registerService();
	IOLog("SURF: IOCoreSurfaceRoot registered\n");
	return true;
}

class IOSurfaceRootUserClient : public IOUserClient {
	OSDeclareDefaultStructors(IOSurfaceRootUserClient);

	task_t   fTask;
	IOLock  *fLock;
	OSArray *fRefs;

	MappleSurfaceRef *findRef(uint32_t id, unsigned int *index);
	IOReturn addRef(MappleSurface *s, MappleSurfaceRef **refOut, bool *existing, bool own = false);
	IOReturn returnSurface(MappleSurface *s, IOExternalMethodArguments *args, bool own = false);
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

static uint64_t mapple_surf_logged;      /* selectors / traps already logged (< 64) */

static void
mapple_surf_log_once(const char *what, uint32_t n)
{
	uint32_t bit = (what[0] == 't' ? 48 : 0) + n;
	if (bit >= 64 || (mapple_surf_logged & (1ULL << bit))) {
		return;
	}
	mapple_surf_logged |= 1ULL << bit;
	char name[32];
	proc_selfname(name, sizeof(name));
	IOLog("SURF: %s %s %u\n", name, what, n);
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
MappleSurfaceRef *
IOSurfaceRootUserClient::findRef(uint32_t id, unsigned int *index)
{
	for (unsigned int i = 0; i < fRefs->getCount(); i++) {
		MappleSurfaceRef *ref = (MappleSurfaceRef *)fRefs->getObject(i);
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
IOSurfaceRootUserClient::addRef(MappleSurface *s, MappleSurfaceRef **refOut, bool *existing, bool own)
{
	*existing = false;
	IOLockLock(fLock);
	MappleSurfaceRef *ref = findRef(s->id, NULL);
	if (ref != NULL && ref->surface == s) {
		IOLockUnlock(fLock);
		mapple_surface_put(s);
		*refOut = ref;
		*existing = true;
		return kIOReturnSuccess;
	}
	IOLockUnlock(fLock);

	ref = OSTypeAlloc(MappleSurfaceRef);
	if (ref == NULL || !ref->init()) {
		OSSafeReleaseNULL(ref);
		mapple_surface_put(s);
		return kIOReturnNoMemory;
	}
	s->retain();
	ref->surface = s;
	ref->count = 1;
	/* own: the creator of a client surface (IOSurfaceAddress) uses its own address, nothing mapped */
	ref->data = own ? NULL : s->memory->createMappingInTask(fTask, 0, kIOMapAnywhere | s->mapCache);
	ref->state = s->state->createMappingInTask(fTask, 0, kIOMapAnywhere | kIOMapReadOnly);
	if ((ref->data == NULL && !own) || ref->state == NULL) {
		ref->release();
		return kIOReturnVMError;
	}
	IOLockLock(fLock);
	MappleSurfaceRef *raced = findRef(s->id, NULL);
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
mapple_surface_fill_result(MappleSurface *s, MappleSurfaceRef *ref, uint8_t *out)
{
	bzero(out, MAPPLE_SURFACE_RESULT_SIZE);
	put64(out, 0x00, ref->data != NULL ? ref->data->getAddress() : s->clientAddr);
	put64(out, 0x08, ref->state->getAddress());
	put64(out, 0x10, ref->state->getAddress() + MAPPLE_SURFACE_TIMESTAMPS);
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
	static int planeRowOn = -1;
	if (planeRowOn < 0) {
		int v = 1;
		PE_parse_boot_argn("mp_surface_bpr", &v, sizeof(v));
		planeRowOn = v != 0;
	}
	for (uint32_t i = 0; i < s->planeCount; i++) {
		uint8_t *p = out + 0x68 + i * 0x80;
		const struct mapple_surface_plane *pl = &s->planes[i];
		put32(p, 0x00, pl->width);
		put32(p, 0x04, pl->height);
		put32(p, 0x08, pl->bytesPerRow);
		put32(p, 0x0c, pl->offset);
		/* IOSurfaceGetBytesPerRowOfPlane reads +0x10 (0 left there broke every planar surface);
		 * mp_surface_bpr=0: only +0x08, as before */
		if (planeRowOn) {
			put32(p, 0x10, pl->bytesPerRow);
		}
		put32(p, 0x14, pl->size);
		memcpy(p + 0x18, &pl->bytesPerElement, 2);
		p[0x1a] = pl->elementWidth;
		p[0x1b] = pl->elementHeight;
	}
}

static IOReturn
mapple_struct_out(IOExternalMethodArguments *args, const void *buf, uint32_t len)
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
IOSurfaceRootUserClient::returnSurface(MappleSurface *s, IOExternalMethodArguments *args, bool own)
{
	MappleSurfaceRef *ref = NULL;
	bool existing = false;
	IOReturn ret = addRef(s, &ref, &existing, own);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	uint8_t *out = (uint8_t *)IOMallocZeroData(MAPPLE_SURFACE_RESULT_SIZE);
	if (out == NULL) {
		ret = kIOReturnNoMemory;
	} else {
		mapple_surface_fill_result(s, ref, out);
		if (existing) {
			put64(out, 0x00, ~0ULL);        /* "already yours": use the cached client */
		}
		ret = mapple_struct_out(args, out, MAPPLE_SURFACE_RESULT_SIZE);
		IOFreeData(out, MAPPLE_SURFACE_RESULT_SIZE);
	}
	if (ret != kIOReturnSuccess && !existing) {
		/* The caller never learns the id: undo this create/lookup. */
		trapRelease((void *)(uintptr_t)s->id, NULL, NULL, NULL, NULL, NULL);
	}
	return ret;
}

uint32_t
mapple_surface_pool_init(uint32_t w, uint32_t h, uint32_t count)
{
	static volatile UInt32 busy;
	count = count > MAPPLE_SURFACE_POOL_MAX ? MAPPLE_SURFACE_POOL_MAX : count;
	/* room for any 64-byte aligned pitch up to 256 bytes wider than the row */
	uint32_t size = (uint32_t)round_page((uint64_t)((w * 4 + 255) & ~255u) * h);
	uint32_t have = mapple_surface_pool_count, made = 0;
	if ((have != 0 && mapple_surface_pool_size != size) || !OSCompareAndSwap(0, 1, &busy)) {
		return have;            /* another size already, or another caller is filling it */
	}
	for (uint32_t i = have; i < count; i++) {
		IOBufferMemoryDescriptor *mem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
		    kIODirectionInOut | kIOMemoryKernelUserShared | kIOMemoryPhysicallyContiguous, size, 0x00000000ffffc000ull);
		if (mem == NULL) {
			break;
		}
		mapple_surface_pool[i] = mem;
		mapple_surface_pool_rc[i] = mem->getRetainCount();      /* the pool's own + the kernel mapping's */
		mapple_surface_pool_size = size;
		OSSynchronizeIO();
		mapple_surface_pool_count = i + 1;
		made++;
	}
	busy = 0;
	IOLog("SURF: display page pool: %u buffers of %u KiB for %ux%u (%u new, mp_fb_pool=%u)\n",
	    have + made, size >> 10, w, h, made, count);
	return have + made;
}

void
mapple_surface_pool_stats(uint32_t *buffers, uint32_t *inUse, uint64_t *fromPool, uint64_t *fromContig,
    uint64_t *sparse, uint32_t *poolKiB)
{
	*buffers = mapple_surface_pool_count;
	*inUse = 0;
	for (uint32_t i = 0; i < *buffers; i++) {
		*inUse += mapple_surface_pool_used[i] != 0;
	}
	*fromPool = mapple_surface_from_pool;
	*fromContig = mapple_surface_from_contig;
	*sparse = mapple_surface_sparse;
	*poolKiB = mapple_surface_pool_size >> 10;
}

uint64_t
mapple_surface_pitch_rounded(void)
{
	return mapple_surface_pitch_rounds;
}

/* "disp surfpitch N": for surfaces created from now on; set < 0 only reads */
int
mapple_surface_pitch_mode(int set)
{
	if (set >= 0 && set <= 2) {
		mapple_surface_pitch64 = set;
	}
	return mapple_surface_pitch64;
}

/*
 * 0184: a free pool buffer for `size` bytes (retained, slot in *slot), or NULL. A buffer someone else still
 * holds (a mapping or an import that outlived its surface) is skipped.
 */
static IOBufferMemoryDescriptor *
mapple_surface_pool_take(uint32_t size, int8_t *slot)
{
	uint32_t n = mapple_surface_pool_count;
	if (size > mapple_surface_pool_size) {
		return NULL;
	}
	for (uint32_t i = 0; i < n; i++) {
		if (!OSCompareAndSwap(0, 1, &mapple_surface_pool_used[i])) {
			continue;
		}
		IOBufferMemoryDescriptor *mem = mapple_surface_pool[i];
		if (mem->getRetainCount() != mapple_surface_pool_rc[i]) {
			mapple_surface_pool_used[i] = 0;
			continue;
		}
		mem->retain();
		*slot = (int8_t)i;
		return mem;
	}
	return NULL;
}

/*
 * IOSurfaceAddress: the pixels stay in the creator's own memory. Its pages are wired for the surface's life and mapped
 * as they are into every other client and the GPU; the creator keeps using its address. A range that is not page
 * aligned, or shorter than the layout, is refused (CoreImage then renders into memory of its own).
 */
static IOReturn
mapple_surface_wrap(MappleSurface *s, task_t task, uint64_t addr, uint64_t given, uint64_t needed)
{
	uint64_t len = mapple_surface_address_length(addr, given, needed, page_size, MAPPLE_SURFACE_MAX_ALLOC);
	if (len == 0) {
		IOLog("SURF: IOSurfaceAddress 0x%llx refused: %llu bytes given, %llu needed (page-aligned range)\n",
		    addr, given, needed);
		return kIOReturnBadArgument;
	}
	IOMemoryDescriptor *md = IOMemoryDescriptor::withAddressRange(addr, len, kIODirectionInOut, task);
	if (md == NULL) {
		return kIOReturnNoMemory;
	}
	if (md->prepare(kIODirectionInOut) != kIOReturnSuccess) {
		IOLog("SURF: IOSurfaceAddress 0x%llx: %llu bytes not wired (not all mapped)\n", addr, len);
		md->release();
		return kIOReturnVMError;
	}
	s->memory = md;
	s->client = true;
	s->clientAddr = addr;
	s->allocSize = (uint32_t)len;
	s->mapCache = kIOMapDefaultCache;
	s->cpuDirty = 1;
	if (s->kernelAddress() == 0) {          /* made now, not under the GPU's gate at the first clean */
		return kIOReturnVMError;
	}
	return mapple_surface_publish(s) ? kIOReturnSuccess : kIOReturnNoResources;
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
		if (len == 0 || len > MAPPLE_SURFACE_MAX_PROPS) {
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
		IOLog("SURF: create: bad property dictionary (%u bytes)\n", len);
		return kIOReturnBadArgument;
	}

	MappleSurface *s = mapple_surface_alloc();
	if (s == NULL) {
		props->release();
		return kIOReturnNoMemory;
	}
	s->parentID = args->scalarInputCount > 0 ? (uint32_t)args->scalarInput[0] : 0;
	uint64_t needed = 0;
	ret = mapple_surface_layout(s, props, &needed);
	static int clientOn = -1;
	if (clientOn < 0) {
		int v = 1;
		PE_parse_boot_argn("mp_surface_client", &v, sizeof(v));
		clientOn = v != 0;
	}
	/* mp_surface_client=0: IOSurfaceAddress ignored, as before (the surface gets memory of its own) */
	OSNumber *address = clientOn ? OSDynamicCast(OSNumber, props->getObject("IOSurfaceAddress")) : NULL;
	if (ret == kIOReturnSuccess && address != NULL) {
		ret = mapple_surface_wrap(s, fTask, address->unsigned64BitValue(),
		    mapple_dict_u32(props, "IOSurfaceAllocSize", 0), needed);
	} else if (ret == kIOReturnSuccess) {
		IOBufferMemoryDescriptor *mem = NULL;
		if ((uint64_t)OSAddAtomic64(s->allocSize, (volatile SInt64 *)&mapple_surface_wired) +
		    s->allocSize <= MAPPLE_SURFACE_WIRED_MAX) {
			bool display = mapple_surface_scanout_w != 0 && s->width == mapple_surface_scanout_w &&
			    s->height == mapple_surface_scanout_h && s->bytesPerElement == 4 && s->planeCount == 0;
			IOOptionBits wc = mapple_gpu_surf_uncached ? kIOMapWriteCombineCache : 0;
			if (display && !wc) {
				/* 0184: the boot-time pool first (fragmentation made the allocation below fail for 60 % of pages) */
				mem = mapple_surface_pool_take(s->allocSize, &s->poolSlot);
				if (mem != NULL) {
					s->contig = true;
					mapple_surface_from_pool++;
				}
			}
			if (mem == NULL && display && mapple_surface_contig + s->allocSize <= MAPPLE_SURFACE_CONTIG_MAX) {
				/* the display's DMA has no IOMMU here: one physical range below 4 GiB, 16 KiB aligned */
				mem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, kIODirectionInOut |
				    kIOMemoryKernelUserShared | kIOMemoryPhysicallyContiguous | wc, s->allocSize, 0x00000000ffffc000ull);
				if (mem != NULL) {
					OSAddAtomic64(s->allocSize, (volatile SInt64 *)&mapple_surface_contig);
					s->contig = true;
					mapple_surface_from_contig++;
				}
			}
			if (mem == NULL) {
				mapple_surface_sparse += display;
				mem = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,
				    kIODirectionInOut | kIOMemoryKernelUserShared | wc, s->allocSize, page_size);
			}
			if (mem != NULL && wc) {
				s->mapCache = kIOMapWriteCombineCache;
			}
		}
		if (mem == NULL) {
			OSAddAtomic64(-(SInt64)s->allocSize, (volatile SInt64 *)&mapple_surface_wired);
			ret = kIOReturnNoMemory;
		} else {
			bzero(mem->getBytesNoCopy(), s->allocSize);
			s->memory = mem;
			if (mapple_surface_footprint()) {
				s->owner = current_task();
				task_reference(s->owner);
				mapple_task_graphics_footprint(s->owner, (int64_t)s->allocSize);
			}
			if (s->mapCache != kIOMapWriteCombineCache) {
				s->cpuDirty = 1;        /* the zeros may still be in the CPU caches */
				s->mapCache = kIOMapDefaultCache;
			}
			if (!mapple_surface_publish(s)) {
				ret = kIOReturnNoResources;
			}
		}
	}
	if (ret != kIOReturnSuccess || mapple_surface_creates_logged < 64) {
		char name[32];
		proc_selfname(name, sizeof(name));
		mapple_surface_creates_logged++;
		IOLog("SURF: %s create id %u %ux%u fmt 0x%x bpe %u bpr %u planes %u size %u keys %u -> 0x%x\n",
		    name, s->id, s->width, s->height, s->pixelFormat, s->bytesPerElement,
		    s->bytesPerRow, s->planeCount, s->allocSize, props->getCount(), ret);
	}
	props->release();
	if (ret != kIOReturnSuccess) {
		s->release();
		return ret;
	}
	ret = returnSurface(s, args, s->client);   /* takes the creator's user count */
	s->release();
	return ret;
}

static MappleSurface *mapple_surface_held(IOLock *lock, OSArray *refs, uint32_t id);

/* The struct input, inband or out of line (> 4096 bytes); IOFreeData(*buf, *len) when done. */
static IOReturn
mapple_struct_in(IOExternalMethodArguments *args, uint32_t max, uint8_t **buf, uint32_t *len)
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
mapple_surface_values_changed(MappleSurface *s)
{
	uint8_t *st = (uint8_t *)s->state->getBytesNoCopy();
	uint32_t gen, size = 1;
	memcpy(&gen, st + MAPPLE_SURFACE_STATE_VALUE_GEN, 4);
	gen = (gen + 2) | 1;
	OSSerialize *ser = OSSerialize::binaryWithCapacity(1024);
	if (ser != NULL && s->values != NULL && s->values->serialize(ser)) {
		size = ser->getLength();
	}
	OSSafeReleaseNULL(ser);
	memcpy(st + MAPPLE_SURFACE_STATE_VALUE_SIZE, &size, 4);
	__asm__ volatile ("dmb ish" : : : "memory");    /* size before the generation clients compare */
	memcpy(st + MAPPLE_SURFACE_STATE_VALUE_GEN, &gen, 4);
	return gen;
}

/* Bulk attachment fields: mask bit -> offset and length in the 0x70-byte block. */
static const struct { uint8_t off, len; } mapple_surface_bulk_fields[15] = {
	{ 0x00, 32 }, { 0x20, 16 }, { 0x30, 8 }, { 0x38, 1 }, { 0x39, 1 }, { 0x3a, 1 }, { 0x3b, 1 }, { 0x3c, 1 },
	{ 0x3d, 1 }, { 0x3e, 1 }, { 0x3f, 1 }, { 0x40, 24 }, { 0x58, 4 }, { 0x5c, 8 }, { 0x64, 1 },
};

/* IOSurfaceSetValue(s) / RemoveValue / CopyValue / bulk attachments (selectors 9, 11, 10, 27). */
IOReturn
IOSurfaceRootUserClient::values(uint32_t selector, IOExternalMethodArguments *args)
{
	uint8_t *in = NULL;
	uint32_t len = 0;
	IOReturn ret = mapple_struct_in(args, MAPPLE_SURFACE_MAX_VALUES + 16, &in, &len);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	uint32_t id;
	memcpy(&id, in + (selector == kMappleSurfaceSetBulk ? 0x80 : 0), 4);
	MappleSurface *s = (selector == kMappleSurfaceSetBulk ? len >= 0x84 : len >= 8) ?
	    mapple_surface_held(fLock, fRefs, id) : NULL;
	if (s == NULL) {
		IOFreeData(in, len);
		return kIOReturnNotFound;
	}
	uint32_t gen = 0;
	uint8_t *st = (uint8_t *)s->state->getBytesNoCopy();
	mapple_surf_log_once("sel", selector);

	switch (selector) {
	case kMappleSurfaceSetValue: {
		OSObject *obj = OSUnserializeXML((const char *)in + 8, len - 8);
		OSArray *arr = OSDynamicCast(OSArray, obj);
		OSDictionary *merge = arr && arr->getCount() == 1 ? OSDynamicCast(OSDictionary, arr->getObject(0)) : NULL;
		OSString *key = arr && arr->getCount() == 2 ? OSDynamicCast(OSString, arr->getObject(1)) : NULL;
		if (merge == NULL && key == NULL) {
			IOLog("SURF: SetValue on %u: unexpected payload (%u bytes)\n", id, len);
			ret = kIOReturnBadArgument;
		} else {
			IOLockLock(mapple_surface_lock);
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
				gen = mapple_surface_values_changed(s);
				uint32_t size;
				memcpy(&size, st + MAPPLE_SURFACE_STATE_VALUE_SIZE, 4);
				if (size > MAPPLE_SURFACE_MAX_VALUES) {      /* keep it bounded: undo */
					if (merge != NULL) {
						s->values->flushCollection();
					} else {
						s->values->removeObject(key);
					}
					gen = mapple_surface_values_changed(s);
					ret = kIOReturnNoMemory;
				}
			}
			IOLockUnlock(mapple_surface_lock);
		}
		OSSafeReleaseNULL(obj);
		break;
	}
	case kMappleSurfaceRemoveValue: {
		char *key = (char *)in + 8;
		in[len - 1] = 0;
		IOLockLock(mapple_surface_lock);
		if (s->values != NULL) {
			if (len <= 9 || key[0] == 0) {
				s->values->flushCollection();
			} else {
				s->values->removeObject(key);
			}
		}
		gen = mapple_surface_values_changed(s);  /* size stays >= 1: other clients must notice */
		IOLockUnlock(mapple_surface_lock);
		break;
	}
	case kMappleSurfaceCopyValues: {
		IOLockLock(mapple_surface_lock);
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
				memcpy(out + 4, st + MAPPLE_SURFACE_STATE_VALUE_GEN, 4);
				memcpy(out + 8, ser->text(), n);
				IOMemoryDescriptor *md = args->structureOutputDescriptor;   /* >= 16 KiB: out of line */
				if (md != NULL && md->prepare() != kIOReturnSuccess) {
					ret = kIOReturnVMError;
				} else {
					ret = mapple_struct_out(args, out, total);   /* kIOReturnNoSpace: the client retries bigger */
					if (md != NULL) {
						md->complete();
					}
				}
				IOFreeData(out, total);
			}
		}
		OSSafeReleaseNULL(ser);
		OSSafeReleaseNULL(empty);
		IOLockUnlock(mapple_surface_lock);
		break;
	}
	case kMappleSurfaceSetBulk: {
		uint64_t size, mask;
		memcpy(&size, in + 0x70, 8);
		memcpy(&mask, in + 0x78, 8);
		IOLockLock(mapple_surface_lock);
		for (uint32_t bit = 0; bit < 15; bit++) {
			uint32_t off = mapple_surface_bulk_fields[bit].off, n = mapple_surface_bulk_fields[bit].len;
			if ((mask & (1ull << bit)) && off + n <= size && off + n <= MAPPLE_SURFACE_BULK_SIZE) {
				memcpy(st + MAPPLE_SURFACE_STATE_BULK + off, in + off, n);
			}
		}
		uint32_t vsize;
		memcpy(&vsize, st + MAPPLE_SURFACE_STATE_VALUE_SIZE, 4);
		if (vsize == 0) {
			mapple_surface_values_changed(s);        /* CopyAllValues returns nothing while the size is 0 */
		}
		IOLockUnlock(mapple_surface_lock);
		break;
	}
	}
	IOFreeData(in, len);
	s->release();
	if (selector == kMappleSurfaceSetValue || selector == kMappleSurfaceRemoveValue) {
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
	case kMappleSurfaceLimits: {
		uint8_t limits[0x18] = { 0 };
		put32(limits, 0x0, MAPPLE_SURFACE_ROW_ALIGN - 1);
		put32(limits, 0x4, MAPPLE_SURFACE_ROW_ALIGN - 1);
		put32(limits, 0xc, MAPPLE_SURFACE_MAX_DIM);
		put32(limits, 0x10, MAPPLE_SURFACE_MAX_DIM);
		limits[0x16] = 1;
		mapple_surf_log_once("sel", selector);
		return mapple_struct_out(args, limits, sizeof(limits));
	}
	case kMappleSurfaceCreate:
		return create(args);
	case kMappleSurfaceSetValue:
	case kMappleSurfaceCopyValues:
	case kMappleSurfaceRemoveValue:
	case kMappleSurfaceSetBulk:
		return values(selector, args);
	case kMappleSurfaceLookup: {
		MappleSurface *s = mapple_surface_copy((uint32_t)arg0);
		IOReturn ret = kIOReturnNotFound;
		if (s != NULL) {
			ret = mapple_surface_get(s) ? returnSurface(s, args) : kIOReturnNotFound;
			s->release();
		}
		mapple_surf_log_once("sel", selector);
		return ret;
	}
	case kMappleSurfaceLookupFromPort: {
		OSObject *obj = NULL;
		IOReturn ret = IOUserClient::copyObjectForPortNameInTask(fTask, (mach_port_name_t)arg0, &obj);
		MappleSurface *s = OSDynamicCast(MappleSurface, obj);
		if (ret == kIOReturnSuccess) {
			ret = (s != NULL && mapple_surface_get(s)) ? returnSurface(s, args) : kIOReturnBadArgument;
		}
		OSSafeReleaseNULL(obj);
		mapple_surf_log_once("sel", selector);
		return ret;
	}
	case kMappleSurfaceCreatePort: {
		MappleSurface *s = mapple_surface_copy((uint32_t)arg0);
		if (s == NULL || args->scalarOutputCount < 1) {
			OSSafeReleaseNULL(s);
			return kIOReturnNotFound;
		}
		args->scalarOutput[0] = mapple_surface_port_in_task(s, fTask);
		args->scalarOutputCount = 1;
		s->release();
		mapple_surf_log_once("sel", selector);
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
		mapple_surf_log_once("sel", selector);
		return kIOReturnSuccess;
	}
}

IOReturn
IOSurfaceRootUserClient::useCount(uintptr_t id, uintptr_t category, int delta)
{
	IOLockLock(fLock);
	MappleSurfaceRef *ref = findRef((uint32_t)id, NULL);
	uint32_t c = (uint32_t)category & 1;
	if (ref != NULL && (delta > 0 || ref->use[c] != 0)) {
		ref->use[c] += delta > 0 ? 1 : -1;
		mapple_surface_use(ref->surface, c, delta > 0 ? 1 : -1);
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
mapple_surface_seed_out(MappleSurface *s, void *seedOut, bool bump)
{
	uint32_t seed = bump ? OSIncrementAtomic((volatile SInt32 *)mapple_surface_seed(s)) + 1 :
	    *mapple_surface_seed(s);
	if (seedOut != NULL && copyout(&seed, (user_addr_t)seedOut, sizeof(seed)) != 0) {
		return kIOReturnBadArgument;
	}
	return kIOReturnSuccess;
}

/* Retained surface this client created or looked up, or NULL. */
static MappleSurface *
mapple_surface_held(IOLock *lock, OSArray *refs, uint32_t id)
{
	MappleSurface *s = NULL;
	IOLockLock(lock);
	for (unsigned int i = 0; i < refs->getCount(); i++) {
		MappleSurfaceRef *ref = (MappleSurfaceRef *)refs->getObject(i);
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
	MappleSurface *s = mapple_surface_held(fLock, fRefs, (uint32_t)(uintptr_t)id);
	if (s == NULL) {
		return kIOReturnNotFound;
	}
	/* like Apple's: the lock waits for the GPU jobs pending on the surface (AvoidSync: fail instead) */
	if (!mapple_gpu_surface_idle(s, NULL)) {
		if ((uintptr_t)options & kIOSurfaceLockAvoidSync) {
			s->release();
			return kIOReturnCannotLock;
		}
		mapple_gpu_surface_wait(s, MAPPLE_SURFACE_LOCK_WAIT_MS);
	}
	mapple_surface_before_cpu_access(s);
	if (!((uintptr_t)options & kIOSurfaceLockReadOnly) && mapple_surface_cacheable(s)) {
		OSCompareAndSwap(0, 1, &s->cpuDirty);
	}
	IOReturn ret = mapple_surface_seed_out(s, seedOut, false);
	s->release();
	return ret;
}

IOReturn
IOSurfaceRootUserClient::trapUnlock(void *id, void *options, void *seedOut,
    void *, void *, void *)
{
	MappleSurface *s = mapple_surface_held(fLock, fRefs, (uint32_t)(uintptr_t)id);
	if (s == NULL) {
		return kIOReturnNotFound;
	}
	if (!((uintptr_t)options & kIOSurfaceLockReadOnly) && mapple_surface_cacheable(s)) {
		OSCompareAndSwap(0, 1, &s->cpuDirty);   /* the writes made while locked */
	}
	IOReturn ret = mapple_surface_seed_out(s, seedOut,
	    !((uintptr_t)options & kIOSurfaceLockReadOnly));
	s->release();
	return ret;
}

IOReturn
IOSurfaceRootUserClient::trapRelease(void *id, void *, void *, void *, void *, void *)
{
	unsigned int index;
	IOLockLock(fLock);
	MappleSurfaceRef *ref = findRef((uint32_t)(uintptr_t)id, &index);
	if (ref != NULL && --ref->count == 0) {
		fRefs->removeObject(index);     /* unmaps and drops the user count */
	}
	IOLockUnlock(fLock);
	return ref != NULL ? kIOReturnSuccess : kIOReturnNotFound;
}

static IOExternalTrap mapple_surface_traps[] = {
	[kMappleSurfaceTrapIncrementUse]  = { NULL, (IOTrap) & IOSurfaceRootUserClient::trapUseCount },
	[kMappleSurfaceTrapDecrementUse]  = { NULL, (IOTrap) & IOSurfaceRootUserClient::trapUseCountDown },
	[kMappleSurfaceTrapLock]          = { NULL, (IOTrap) & IOSurfaceRootUserClient::trapLock },
	[kMappleSurfaceTrapUnlock]        = { NULL, (IOTrap) & IOSurfaceRootUserClient::trapUnlock },
	[kMappleSurfaceTrapRelease]       = { NULL, (IOTrap) & IOSurfaceRootUserClient::trapRelease },
	[kMappleSurfaceTrapReleaseFailed] = { NULL, (IOTrap) & IOSurfaceRootUserClient::trapRelease },
};

IOExternalTrap *
IOSurfaceRootUserClient::getTargetAndTrapForIndex(IOService **targetP, UInt32 index)
{
	if (index >= sizeof(mapple_surface_traps) / sizeof(mapple_surface_traps[0])) {
		mapple_surf_log_once("trap", index);
		return NULL;
	}
	mapple_surf_log_once("trap", index);
	*targetP = this;
	return &mapple_surface_traps[index];
}

/* ---- black Metal frames: the experiments of one boot (MappleSurface.h) ---- */

int mapple_gpu_surf_maint;
int mapple_gpu_surf_uncached;

#if DEVELOPMENT || DEBUG
#include <sys/sysctl.h>
#include <sys/kauth.h>

extern "C" uint64_t mapple_gpu_surface_pa(MappleSurface *surface);         /* MappleGPU.cpp */

/* Read by the Metal driver at each texture it makes: render targets linear, IOSurfaces copied instead of imported. */
static int mapple_gpu_linear, mapple_gpu_surf_copy;
SYSCTL_INT(_kern, OID_AUTO, mapple_gpu_linear, CTLFLAG_RW | CTLFLAG_LOCKED, &mapple_gpu_linear, 0,
    "Metal driver: every render target linear (no AFBC or tiling)");
SYSCTL_INT(_kern, OID_AUTO, mapple_gpu_surf_copy, CTLFLAG_RW | CTLFLAG_LOCKED, &mapple_gpu_surf_copy, 0,
    "Metal driver: IOSurfaces copied in and out, never imported");
SYSCTL_INT(_kern, OID_AUTO, mapple_gpu_surf_maint, CTLFLAG_RW | CTLFLAG_LOCKED, &mapple_gpu_surf_maint, 0,
    "after a GPU job a surface's CPU lines are dropped: 0 at the next CPU access, 1 at once, 2 never");
SYSCTL_INT(_kern, OID_AUTO, mapple_gpu_surf_uncached, CTLFLAG_RW | CTLFLAG_LOCKED, &mapple_gpu_surf_uncached, 0,
    "new IOSurfaces write-combined");

/* GPU memory (read by Mesa and the Metal driver; 1 when absent): Mesa's BO cache also drops buffers unused for 2 s when
 * it hands one out and at each submit; the driver empties that cache at a memory pressure notification. */
static int mapple_gpu_bocache = 1, mapple_gpu_pressure = 1;
SYSCTL_INT(_kern, OID_AUTO, mapple_gpu_bocache, CTLFLAG_RW | CTLFLAG_LOCKED, &mapple_gpu_bocache, 0,
    "Mesa's BO cache: stale buffers dropped at fetch and submit too (0: only when one is put back, as upstream)");
SYSCTL_INT(_kern, OID_AUTO, mapple_gpu_pressure, CTLFLAG_RW | CTLFLAG_LOCKED, &mapple_gpu_pressure, 0,
    "the Metal driver empties Mesa's BO cache at a memory pressure notification");

/* 16 bytes at `pa`, through an uncached mapping: what is in DRAM, whatever the CPU caches hold. */
static void
mapple_surface_dram(uint64_t pa, uint8_t *out)
{
	bzero(out, 16);
	IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(pa & ~15ull, 16, kIODirectionIn);
	IOMemoryMap *map = md != NULL ? md->map(kIOMapInhibitCache | kIOMapReadOnly) : NULL;
	if (map != NULL) {
		const volatile uint8_t *p = (const volatile uint8_t *)map->getVirtualAddress();
		for (int i = 0; i < 16; i++) {
			out[i] = p[i];
		}
	}
	OSSafeReleaseNULL(map);
	OSSafeReleaseNULL(md);
}

static int
mapple_surface_hex(char *out, size_t len, const char *what, const uint8_t *b)
{
	return snprintf(out, len, " %s %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x", what,
	           b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
}

/*
 * kern.mapple_gpu_surf_debug: newp = a surface id (4 bytes); oldp = its pages and its first bytes, at the start and in
 * the middle, as DRAM holds them (uncached), as the framebuffer's copy reads them (the kernel's mapping, before any
 * maintenance) and after a clean + invalidate. Root only.
 */
static int
mapple_surface_debug SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	uint32_t id = 0;
	if (req->newptr == USER_ADDR_NULL || req->newlen != sizeof(id) || !kauth_cred_issuser(kauth_cred_get())) {
		return EINVAL;
	}
	int error = SYSCTL_IN(req, &id, sizeof(id));
	if (error) {
		return error;
	}
	MappleSurface *s = mapple_surface_copy(id);
	if (s == NULL) {
		return ENOENT;
	}
	char out[1024];
	size_t o = 0;
	o += snprintf(out + o, sizeof(out) - o, "surface %u %ux%u bpr %u alloc %u %s%s, cpuDirty %u gpuDirty %u, %llu GPU jobs\n",
	    s->id, s->width, s->height, s->bytesPerRow, s->allocSize, s->contig ? "contiguous" : "sparse",
	    s->mapCache == kIOMapWriteCombineCache ? " write-combined" : " cached", s->cpuDirty, s->gpuDirty, s->gpuJobs);
	uint64_t mid = (uint64_t)(s->height / 2) * s->bytesPerRow;
	uint64_t at[2] = { s->offset, s->offset + mid };
	uint64_t gpa = mapple_gpu_surface_pa(s);
	for (int k = 0; k < 2 && s->memory != NULL; k++) {
		IOByteCount seg = 0;
		uint64_t pa = s->memory->getPhysicalSegment(at[k], &seg, kIOMemoryMapperNone);
		uint8_t dram[16], cached[16], after[16];
		mapple_surface_dram(pa, dram);
		uintptr_t kva = s->kernelAddress();
		if (kva != 0) {
			memcpy(cached, (const void *)(kva + at[k]), 16);
			for (uintptr_t line = (kva + at[k]) & ~(uintptr_t)63; line < kva + at[k] + 16; line += 64) {
				__asm__ volatile ("dc civac, %0" : : "r"(line) : "memory");
			}
			__asm__ volatile ("dsb sy" : : : "memory");
			memcpy(after, (const void *)(kva + at[k]), 16);
		} else {
			bzero(cached, 16);
			bzero(after, 16);
		}
		o += snprintf(out + o, sizeof(out) - o, "%s +0x%llx: pa 0x%llx", k ? "middle" : "start", at[k], pa);
		if (k == 0) {
			o += snprintf(out + o, sizeof(out) - o, ", GPU va 0x%llx -> pa 0x%llx (%s)", s->gpuVA, gpa,
			    gpa == 0 ? "no live import" : gpa == pa ? "same" : "DIFFERENT");
		}
		o += snprintf(out + o, sizeof(out) - o, "\n ");
		o += mapple_surface_hex(out + o, sizeof(out) - o, "dram", dram);
		o += mapple_surface_hex(out + o, sizeof(out) - o, "| cached", cached);
		o += mapple_surface_hex(out + o, sizeof(out) - o, "| after civac", after);
		o += snprintf(out + o, sizeof(out) - o, "\n");
	}
	s->release();
	return SYSCTL_OUT(req, out, strnlen(out, sizeof(out)) + 1);
}
SYSCTL_PROC(_kern, OID_AUTO, mapple_gpu_surf_debug, CTLTYPE_OPAQUE | CTLFLAG_RW | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    0, 0, mapple_surface_debug, "-", "an IOSurface's pages and first bytes: DRAM, the CPU's view, after civac");

extern "C" int task_pid(task_t task);

/* Every live surface: id, size, format, users, the task that made it (and whether it still runs), GPU jobs; totals. */
static int
mapple_surface_list(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	const size_t cap = 160 * MAPPLE_SURFACE_MAX_ID;
	char *out = (char *)IOMallocData(cap);
	size_t o = 0;
	uint64_t n = 0, bytes = 0, gone = 0, gone_bytes = 0;

	if (out == NULL) {
		return ENOMEM;
	}
	IOLockLock(mapple_surface_lock);
	for (uint32_t id = 1; id < MAPPLE_SURFACE_MAX_ID; id++) {
		MappleSurface *s = mapple_surface_table[id];
		if (s == NULL) {
			continue;
		}
		int pid = s->owner ? task_pid(s->owner) : -1;
		char name[32] = "-";
		bool alive = false;
		if (pid > 0) {
			proc_t p = proc_find(pid);
			if (p != NULL) {
				alive = true;
				proc_rele(p);
			}
			proc_name(pid, name, sizeof(name));
		}
		const uint32_t f = s->pixelFormat;
		const char fmt[5] = { (char)(f >> 24), (char)(f >> 16), (char)(f >> 8), (char)f, 0 };
		n++;
		bytes += s->allocSize;
		if (pid > 0 && !alive) {
			gone++;
			gone_bytes += s->allocSize;
		}
		if (o < cap - 160) {
			o += snprintf(out + o, cap - o, "%u %ux%u '%s' %u KiB, %u users, by %s[%d]%s, %llu GPU jobs%s\n", id,
			    s->width, s->height, fmt, s->allocSize >> 10, mapple_surface_users[id], name, pid,
			    pid > 0 && !alive ? " (gone)" : "", s->gpuJobs, s->poolSlot >= 0 ? ", display pool" : "");
		}
	}
	IOLockUnlock(mapple_surface_lock);
	o += snprintf(out + o, cap - o, "%llu surfaces, %llu KiB; %llu of them, %llu KiB, made by tasks that are gone\n", n,
	    bytes >> 10, gone, gone_bytes >> 10);
	int err = SYSCTL_OUT(req, out, o + 1);
	IOFreeData(out, cap);
	return err;
}
SYSCTL_PROC(_kern, OID_AUTO, mapple_surf_list, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    0, 0, mapple_surface_list, "A", "every live IOSurface: size, format, users, maker, GPU jobs; totals");
#endif /* DEVELOPMENT || DEBUG */
#endif /* MT6781 */
