/*
 * Built-in IOSurface for SWEET (see SweetSurface.cpp).
 *
 * Shared with SweetFramebuffer.cpp: the framebuffer looks surfaces up by
 * id when a swap names them and publishes the panel memory itself as the
 * default framebuffer surface.
 */
#ifndef _SWEET_SURFACE_H
#define _SWEET_SURFACE_H

#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <libkern/c++/OSObject.h>
#include <mach/mach_types.h>

#define SWEET_SURFACE_MAX_PLANES        20

struct sweet_surface_plane {
	uint32_t width, height, bytesPerRow, offset, size;
	uint16_t bytesPerElement;
	uint8_t  elementWidth, elementHeight;
};

class SweetSurface : public OSObject {
	OSDeclareDefaultStructors(SweetSurface);

public:
	uint32_t id;
	uint32_t width, height, bytesPerRow, offset, pixelFormat, allocSize;
	uint16_t bytesPerElement;
	uint8_t  elementWidth, elementHeight;
	uint32_t cacheMode, planeCount, parentID;
	struct sweet_surface_plane planes[SWEET_SURFACE_MAX_PLANES];

	IOMemoryDescriptor       *memory;       /* pixels (buffer or physical) */
	IOOptionBits              mapCache;     /* cache mode for user mappings */
	IOBufferMemoryDescriptor *state;        /* shared state page, read-only for users */
	IOMemoryMap              *kernelMap;    /* lazily created for kernel copies */

	/*
	 * CPU cache state for devices that read and write the pixels without snooping the CPU caches (the viva
	 * GPU, the display). CPU access goes through IOSurfaceLock/Unlock: a write lock marks the CPU lines dirty,
	 * any lock first drops the lines a device made stale. Only buffer surfaces (cacheable) track this.
	 */
	volatile UInt32           cpuDirty;     /* the CPU may hold dirty lines (write lock since the last clean) */
	volatile UInt32           gpuDirty;     /* a device wrote the memory since the CPU last dropped its lines */

	OSDictionary             *values;       /* IOSurfaceSetValue (non-bulk), under the surface table lock */
	bool                      contig;       /* physically contiguous below 4 GiB (scanout candidate) */

	/*
	 * viva GPU (VivaGPU.cpp, under its gate): fence of the last GPU job that used the surface, from any
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

	bool init() APPLE_KEXT_OVERRIDE { return OSObject::init(); }
	void free() APPLE_KEXT_OVERRIDE;

	/* Kernel address of the pixels (mapped on first use), or 0. */
	uintptr_t kernelAddress();
};

/* Retained surface for an id, or NULL. */
SweetSurface *sweet_surface_copy(uint32_t id);

/*
 * Publish physical memory (the panel framebuffer) as a surface. Returns a
 * retained surface that stays in the id table until released by the caller
 * and every user.
 */
SweetSurface *sweet_surface_create_physical(uint64_t phys, uint32_t width,
    uint32_t height, uint32_t bytesPerRow, uint32_t pixelFormat);

/*
 * Cache maintenance around device access (see cpuDirty/gpuDirty). Each returns the bytes cleaned (0: nothing
 * to do or not a cacheable surface).
 *   before_device_read: a device is about to read: clean the CPU's dirty lines to memory
 *   before_cpu_access:  the CPU is about to read: drop lines older than what a device wrote
 *   device_wrote:       a device finished writing it
 */
uint64_t sweet_surface_before_device_read(SweetSurface *surface);
uint64_t sweet_surface_before_cpu_access(SweetSurface *surface);
void sweet_surface_device_wrote(SweetSurface *surface);

/*
 * viva GPU (VivaGPU.cpp), any thread but the GPU's work loop: no GPU job using the surface is pending (true when
 * the GPU driver is not running); *jobs (optional) = GPU jobs that used it so far. surface_wait sleeps until it
 * is idle, at most timeoutMs (kIOReturnTimeout).
 */
bool viva_gpu_surface_idle(SweetSurface *surface, uint64_t *jobs);
IOReturn viva_gpu_surface_wait(SweetSurface *surface, uint32_t timeoutMs);

/*
 * Display (SweetFramebuffer.cpp), from the GPU's work loop: a GPU job that used IOSurfaces completed. If a swap is
 * waiting to be shown, AppleCLCD tries it at once on its own work loop instead of at the next vsync tick. Cheap,
 * never blocks.
 */
void sweet_fb_gpu_done(void);

/*
 * 0137: vsync ticks at which the display had a frame queued but the GPU had not finished it (K0c). The GPU's DVFS
 * governor goes to its fastest OPP when this moves: with SwapWait (0133) every frame of GPU latency is a frame less
 * per second, and the load average of bursty frames kept the clock low.
 */
extern volatile uint32_t sweet_fb_gpu_waits;

/*
 * Size of the surfaces the display may scan out directly (viva OVL, SweetFramebuffer.cpp; 0 = none): such
 * 4-byte single-plane surfaces are allocated physically contiguous below 4 GiB when possible.
 */
extern uint32_t sweet_surface_scanout_w, sweet_surface_scanout_h;

/* Send right for the surface in task (IOSurfaceLookupFromMachPort). */
mach_port_name_t sweet_surface_port_in_task(SweetSurface *surface, task_t task);

#endif /* SWEET */
#endif /* _SWEET_SURFACE_H */
