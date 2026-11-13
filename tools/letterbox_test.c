/*
 * iokit/Kernel/arm/MappleLetterbox.h on this machine: the picture on the 1080x2412 panel, the hole in the strip, the
 * sample positions, touches at the corners and the centre, and the pixel blend.
 *   cc -Wall tools/letterbox_test.c -o letterbox_test && ./letterbox_test
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../iokit/Kernel/arm/MappleLetterbox.h"

#define PW 1080
#define PH 2412
#define LW 1125
#define LH 2436

/* logical position (16.16) a touch at panel x, y lands on */
static void
touch(struct mapple_lb r, int32_t x, int32_t y, int64_t *lx, int64_t *ly)
{
	*lx = (int64_t)mapple_lb_norm(mapple_lb_clamp(x, r.x, r.w), r.w) * LW;
	*ly = (int64_t)mapple_lb_norm(mapple_lb_clamp(y, r.y, r.h), r.h) * LH;
}

int
main(void)
{
	struct mapple_lb r = mapple_lb_rect(PW, PH, LW, LH, 0);
	assert(r.w == 0);

	/* the true aspect at full width: 1080x2339, 73 rows of strip, too shallow for the hole */
	r = mapple_lb_rect(PW, PH, LW, LH, 2);
	assert(r.w == 1080 && r.h == 2339 && r.x == 0 && r.y == 73);
	assert(MAPPLE_LB_HOLE_Y1 > r.y);

	/* the default: the strip down to the hole's bottom */
	r = mapple_lb_rect(PW, PH, LW, LH, 1);
	printf("picture %dx%d at %d,%d, strip %d rows, hole x %d..%d y 0..%d\n", r.w, r.h, r.x, r.y, r.y, MAPPLE_LB_HOLE_X0,
	    MAPPLE_LB_HOLE_X1, MAPPLE_LB_HOLE_Y1);
	assert(r.w == 1058 && r.h == 2292 && r.x == 11 && r.y == 120);
	assert(r.y >= MAPPLE_LB_HOLE_Y1 && r.y + r.h == PH && r.x == (PW - r.w) / 2);
	/* the aspect kept to within half a pixel */
	assert(abs(r.w * LH - r.h * LW) * 2 <= LH);

	/* a strip given in rows, and one that asks for too much */
	r = mapple_lb_rect(PW, PH, LW, LH, 105);
	assert(r.y == 105 && r.h == 2307);
	r = mapple_lb_rect(PW, PH, LW, LH, 100000);
	assert(r.y == PH / 2);

	r = mapple_lb_rect(PW, PH, LW, LH, 1);
	/* sample positions: inside the display, increasing, the first and last a fraction of a pixel in */
	for (int32_t i = 0; i < r.w; i++) {
		uint32_t u = mapple_lb_src(i, r.w, LW);
		assert((u >> 16) + 1 < LW && (i == 0 || u > mapple_lb_src(i - 1, r.w, LW)));
	}
	for (int32_t j = 0; j < r.h; j++) {
		uint32_t v = mapple_lb_src(j, r.h, LH);
		assert((v >> 16) + 1 < LH && (j == 0 || v > mapple_lb_src(j - 1, r.h, LH)));
	}
	assert(mapple_lb_src(0, r.w, LW) < 0x10000 && mapple_lb_src(r.w - 1, r.w, LW) > (uint32_t)(LW - 2) << 16);

	/* touches: a picture pixel lands where the framebuffer sampled it (centres line up, to 1/65536) */
	int64_t lx, ly;
	for (int32_t i = 0; i < r.w; i += 7) {
		touch(r, r.x + i, r.y, &lx, &ly);
		int64_t want = (int64_t)mapple_lb_src(i, r.w, LW) + 0x8000;
		assert(lx - want >= -LW && lx - want <= LW);
	}
	/* corners: the picture's corners are the display's, the strip and the bars clamp to the picture's edge */
	touch(r, r.x, r.y, &lx, &ly);
	assert(lx >> 16 == 0 && ly >> 16 == 0);
	touch(r, 0, 0, &lx, &ly);
	assert(lx >> 16 == 0 && ly >> 16 == 0);
	touch(r, MAPPLE_LB_HOLE_X1, MAPPLE_LB_HOLE_Y1 - 1, &lx, &ly);
	assert(ly >> 16 == 0);
	touch(r, PW - 1, PH - 1, &lx, &ly);
	assert(lx >> 16 == LW - 1 && ly >> 16 == LH - 1);
	touch(r, PW - 1, 0, &lx, &ly);
	assert(lx >> 16 == LW - 1 && ly >> 16 == 0);
	touch(r, 0, PH - 1, &lx, &ly);
	assert(lx >> 16 == 0 && ly >> 16 == LH - 1);
	/* the centre of the picture is the centre of the display */
	touch(r, r.x + r.w / 2, r.y + r.h / 2, &lx, &ly);
	assert(llabs(lx - (LW << 15)) <= 0x10000 && llabs(ly - (LH << 15)) <= 0x10000);

	/* the blend: the ends at weight 0 and 256, each lane rounded on its own, no carry between lanes */
	for (uint32_t a = 0; a < 256; a += 5) {
		for (uint32_t b = 0; b < 256; b += 3) {
			for (uint32_t w = 0; w <= 256; w += 17) {
				uint32_t pa = a | (255 - a) << 8 | b << 16 | (a ^ b) << 24;
				uint32_t pb = b | (255 - b) << 8 | a << 16 | (255 - (a ^ b)) << 24;
				uint32_t m = mapple_lb_pack(mapple_lb_mix(mapple_lb_spread(pa), mapple_lb_spread(pb), w));
				for (int c = 0; c < 32; c += 8) {
					uint32_t ca = (pa >> c) & 0xff, cb = (pb >> c) & 0xff;
					assert(((m >> c) & 0xff) == (ca * (256 - w) + cb * w + 128) >> 8);
				}
			}
		}
	}
	assert(mapple_lb_pack(mapple_lb_spread(0xff000000u)) == 0xff000000u);
	assert(mapple_lb_pack(mapple_lb_mix(mapple_lb_spread(0xffffffffu), mapple_lb_spread(0xffffffffu), 128)) ==
	    0xffffffffu);
	printf("letterbox: all checks passed\n");
	return 0;
}
