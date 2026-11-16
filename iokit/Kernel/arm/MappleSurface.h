/*
 * Built-in IOSurface for MT6781 (see MappleSurface.cpp).
 *
 * Shared with MappleFramebuffer.cpp: the framebuffer looks surfaces up by
 * id when a swap names them and publishes the panel memory itself as the
 * default framebuffer surface.
 */
#ifndef _MAPPLE_SURFACE_H
#define _MAPPLE_SURFACE_H

#include <pexpert/arm64/board_config.h>

#if defined(MT6781)
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <libkern/c++/OSObject.h>
#include <mach/mach_types.h>

#define MAPPLE_SURFACE_MAX_PLANES        20

struct mapple_surface_plane {
	uint32_t width, height, bytesPerRow, offset, size;
	uint16_t bytesPerElement;
	uint8_t  elementWidth, elementHeight;
};

class MappleSurface : public OSObject {
	OSDeclareDefaultStructors(MappleSurface);

public:
	uint32_t id;
	uint32_t width, height, bytesPerRow, offset, pixelFormat, allocSize;
	uint16_t bytesPerElement;
	uint8_t  elementWidth, elementHeight;
	uint32_t cacheMode, planeCount, parentID;
	struct mapple_surface_plane planes[MAPPLE_SURFACE_MAX_PLANES];

	IOMemoryDescriptor       *memory;       /* pixels (buffer or physical) */
	IOOptionBits              mapCache;     /* cache mode for user mappings */
	IOBufferMemoryDescriptor *state;        /* shared state page, read-only for users */
	IOMemoryMap              *kernelMap;    /* lazily created for kernel copies */

	/*
	 * CPU cache state for devices that read and write the pixels without snooping the CPU caches (the Mapple
	 * GPU, the display). CPU access goes through IOSurfaceLock/Unlock: a write lock marks the CPU lines dirty,
	 * any lock first drops the lines a device made stale. Only buffer surfaces (cacheable) track this.
	 */
	volatile UInt32           cpuDirty;     /* the CPU may hold dirty lines (write lock since the last clean) */
	volatile UInt32           gpuDirty;     /* a device wrote the memory since the CPU last dropped its lines */

	OSDictionary             *values;       /* IOSurfaceSetValue (non-bulk), under the surface table lock */
	bool                      contig;       /* physically contiguous below 4 GiB (scanout candidate) */
	bool                      client;       /* IOSurfaceAddress: the creator's own pages (wired), cacheable */
	uint64_t                  clientAddr;   /* their address in the creator */
	int8_t                    poolSlot;     /* 0184: buffer of the display page pool, -1 = none */

	/*
	 * Mapple GPU (MappleGPU.cpp, under its gate): fence of the last GPU job that used the surface, from any
	 * connection (every job also waits for the one before on the surface, so this one signals last), and how
	 * many jobs used it so far.
	 */
	OSObject                 *gpuFence;
	uint64_t                  gpuJobs;
	/*
	 * K0i: while gpuFence is a pending promise (SURFACE_PROMISE), the fence it replaced: the promise owner's own
	 * jobs on the surface wait for this one, not for the promise they will resolve.
	 */
	OSObject                 *gpuPrev;
	/* the task that made it (referenced) while its pixels count in that task's footprint (mp_gpu_footprint) */
	task_t                    owner;
	/* the last GPU import (kern.mapple_gpu_surf_debug): its VM (compared only, never dereferenced here) and address */
	void                     *gpuVM;
	uint64_t                  gpuVA;

	bool init() APPLE_KEXT_OVERRIDE { return OSObject::init(); }
	void free() APPLE_KEXT_OVERRIDE;

	/* Kernel address of the pixels (mapped on first use), or 0. */
	uintptr_t kernelAddress();
};

/*
 * Black Metal frames, one boot for every case (sysctls in DEVELOPMENT/DEBUG kernels): after a GPU job the surface's CPU
 * lines are dropped at the next CPU access (kern.mapple_gpu_surf_maint=0), at once (1) or never (2); new surfaces
 * are write-combined (kern.mapple_gpu_surf_uncached=1).
 */
extern int mapple_gpu_surf_maint;
extern int mapple_gpu_surf_uncached;

/* Retained surface for an id, or NULL. */
MappleSurface *mapple_surface_copy(uint32_t id);

/*
 * Publish physical memory (the panel framebuffer) as a surface. Returns a
 * retained surface that stays in the id table until released by the caller
 * and every user.
 */
MappleSurface *mapple_surface_create_physical(uint64_t phys, uint32_t width,
    uint32_t height, uint32_t bytesPerRow, uint32_t pixelFormat);

/*
 * Cache maintenance around device access (see cpuDirty/gpuDirty). Each returns the bytes cleaned (0: nothing
 * to do or not a cacheable surface).
 *   before_device_read: a device is about to read: clean the CPU's dirty lines to memory
 *   before_cpu_access:  the CPU is about to read: drop lines older than what a device wrote
 *   device_wrote:       a device finished writing it
 */
uint64_t mapple_surface_before_device_read(MappleSurface *surface);
uint64_t mapple_surface_before_cpu_access(MappleSurface *surface);
void mapple_surface_device_wrote(MappleSurface *surface);
/* new seed (contents changed outside a lock); cpu_wrote = CPU-dirty + new seed (the kernel wrote it, MTKScaler) */
void mapple_surface_seed_bump(MappleSurface *surface);
void mapple_surface_cpu_wrote(MappleSurface *surface);

/*
 * Mapple GPU (MappleGPU.cpp), any thread but the GPU's work loop: no GPU job using the surface is pending (true when
 * the GPU driver is not running); *jobs (optional) = GPU jobs that used it so far. surface_wait sleeps until it
 * is idle, at most timeoutMs (kIOReturnTimeout).
 */
bool mapple_gpu_surface_idle(MappleSurface *surface, uint64_t *jobs);
IOReturn mapple_gpu_surface_wait(MappleSurface *surface, uint32_t timeoutMs);

/*
 * Display (MappleFramebuffer.cpp), from the GPU's work loop: a GPU job that used IOSurfaces completed. If a swap is
 * waiting to be shown, AppleCLCD tries it at once on its own work loop instead of at the next vsync tick. Cheap,
 * never blocks.
 */
void mapple_fb_gpu_done(void);

/*
 * 0137: vsync ticks at which the display had a frame queued but the GPU had not finished it (K0c). The GPU's DVFS
 * governor goes to its fastest OPP when this moves: with SwapWait (0133) every frame of GPU latency is a frame less
 * per second, and the load average of bursty frames kept the clock low.
 */
extern volatile uint32_t mapple_fb_gpu_waits;

/*
 * Size of the surfaces the display may scan out directly (OVL, MappleFramebuffer.cpp; 0 = none): such
 * 4-byte single-plane surfaces are allocated physically contiguous below 4 GiB when possible.
 */
extern uint32_t mapple_surface_scanout_w, mapple_surface_scanout_h;

/*
 * 0184 (P2 of gpu/docs/ANALISI-PRESTAZIONI-2026-11-06.md): `count` (mp_fb_pool, default 4, at most 8) physically
 * contiguous buffers below 4 GiB for w x h 4-byte surfaces, taken once at boot before memory fragments. Display-sized
 * surfaces (mapple_surface_scanout_w/h) get one of them first, a fresh contiguous allocation second, sparse memory
 * last; a buffer goes back to the pool when its surface is freed. Called by AppleCLCD when scanout is on; returns
 * the buffers the pool has (it only grows). Stats: pool buffers, in use, and surfaces of display size created from
 * the pool, from a contiguous allocation, sparse; *poolKiB = size of one buffer.
 */
uint32_t mapple_surface_pool_init(uint32_t w, uint32_t h, uint32_t count);
void mapple_surface_pool_stats(uint32_t *buffers, uint32_t *inUse, uint64_t *fromPool, uint64_t *fromContig,
    uint64_t *sparse, uint32_t *poolKiB);

/*
 * 0184 (P6): client bytes-per-row values rounded up to 64 (the GPU imports only 64-byte aligned pitches; others were
 * copied in and out at every use). Boot-arg mp_surface_pitch64=0: as the client asked. Count of rounded surfaces.
 */
uint64_t mapple_surface_pitch_rounded(void);
/* mp_surface_pitch64 now (0 never, 1 default, 2 also with an explicit alloc size), -1 = boot-arg not read yet */
int mapple_surface_pitch_mode(int set);

/* Send right for the surface in task (IOSurfaceLookupFromMachPort). */
mach_port_name_t mapple_surface_port_in_task(MappleSurface *surface, task_t task);

#endif /* MT6781 */
#endif /* _MAPPLE_SURFACE_H */
