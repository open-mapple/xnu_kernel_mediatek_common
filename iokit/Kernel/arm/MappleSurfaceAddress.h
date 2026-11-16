/*
 * IOSurfaceAddress: a surface over memory of the creating task's own (CoreImage asks for one for every bitmap or
 * CGImage it renders into). No kernel headers here: tools/surface_address_test.c checks it on the host.
 */
#ifndef _MAPPLE_SURFACE_ADDRESS_H
#define _MAPPLE_SURFACE_ADDRESS_H

#include <stdint.h>

/*
 * The length of the caller's range to wire, or 0 to refuse it: the address page aligned, the caller's length (its
 * IOSurfaceAllocSize, else what the layout needs) holding every byte of the layout (rows, planes, offset), no wrap.
 * The length is rounded up to whole pages: the rest of the last page is the caller's too.
 */
static inline uint64_t
mapple_surface_address_length(uint64_t addr, uint64_t given, uint64_t needed, uint64_t page, uint64_t max)
{
	uint64_t len = given != 0 ? given : needed;
	if (addr == 0 || (addr & (page - 1)) != 0 || needed == 0 || len < needed || len > max) {
		return 0;
	}
	len = (len + page - 1) & ~(page - 1);
	return addr + len > addr ? len : 0;
}

#endif
