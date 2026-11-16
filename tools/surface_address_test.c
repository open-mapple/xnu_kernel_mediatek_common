/*
 * iokit/Kernel/arm/MappleSurfaceAddress.h on this machine: which IOSurfaceAddress ranges a surface takes.
 *   cc -Wall tools/surface_address_test.c -o surface_address_test && ./surface_address_test
 */
#include <assert.h>
#include <stdio.h>
#include "../iokit/Kernel/arm/MappleSurfaceAddress.h"

#define PAGE 16384ull
#define MAX  (256ull << 20)

int
main(void)
{
	/* CoreImage's 64x64 BGRA bitmap: one page, its own size */
	assert(mapple_surface_address_length(0x104c5c000, 16384, 16384, PAGE, MAX) == 16384);
	/* no IOSurfaceAllocSize: what the rows need, rounded up to the page */
	assert(mapple_surface_address_length(0x104c5c000, 0, 64 * 4 * 65, PAGE, MAX) == 2 * PAGE);
	/* a larger AllocSize than the rows: the caller's length */
	assert(mapple_surface_address_length(0x104c5c000, 3 * PAGE, PAGE, PAGE, MAX) == 3 * PAGE);
	/* refused: not page aligned, null, the caller's length short of the rows (bytes per row x height or a plane),
	 * nothing needed, over the largest surface, the range wrapping the address space */
	assert(mapple_surface_address_length(0x104c5c010, 16384, 16384, PAGE, MAX) == 0);
	assert(mapple_surface_address_length(0, 16384, 16384, PAGE, MAX) == 0);
	assert(mapple_surface_address_length(0x104c5c000, 4608 * 2435, 4608 * 2436, PAGE, MAX) == 0);
	assert(mapple_surface_address_length(0x104c5c000, 16384, 0, PAGE, MAX) == 0);
	assert(mapple_surface_address_length(0x104c5c000, MAX + PAGE, PAGE, PAGE, MAX) == 0);
	assert(mapple_surface_address_length(0xffffffffffffc000ull, PAGE, PAGE, PAGE, MAX) == 0);
	printf("surface_address_test: ok\n");
	return 0;
}
