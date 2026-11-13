/*
 * Boot-arg mp_fb_letterbox: the presented display (1125x2436, the iPhone X) scaled at its own aspect onto the panel,
 * bottom-aligned and centred, under a black strip that holds the camera hole. 0 (default) = the centred crop, 1 = a
 * strip down to the hole's bottom, N > 1 = a strip of N rows. The strip is never shallower than the true aspect
 * leaves at the panel's full width (73 rows on 1080x2412). MappleFramebuffer.cpp draws the picture (mp_fb_rsz=2: on
 * the display pipe's resizer, below), MappleHID.cpp maps touches onto it; tools/letterbox_test.c checks the arithmetic.
 */
#ifndef _MAPPLE_LETTERBOX_H_
#define _MAPPLE_LETTERBOX_H_

#include <stdint.h>

/*
 * The hole as Android cuts it out (config_mainBuiltInDisplayCutout "M -160,0 L -180,0 L -180,40 L -130,40 L -130,0 Z
 * @dp", x from the panel's centre) at ro.sf.lcd_density 480, 3 pixels a dp: x 0..150, y 0..120 on the 1080-wide panel.
 */
#define MAPPLE_LB_HOLE_X0       0
#define MAPPLE_LB_HOLE_X1       150
#define MAPPLE_LB_HOLE_Y1       120

/* the picture on the panel, in panel pixels; w == 0: letterbox off */
struct mapple_lb {
	int32_t x, y, w, h;
};

static inline struct mapple_lb
mapple_lb_rect(int32_t pw, int32_t ph, int32_t lw, int32_t lh, uint32_t arg)
{
	struct mapple_lb r = { 0, 0, 0, 0 };
	int32_t strip = arg == 1 ? MAPPLE_LB_HOLE_Y1 : (int32_t)(arg > (uint32_t)ph / 2 ? (uint32_t)ph / 2 : arg);

	if (arg == 0 || pw <= 0 || ph <= 0 || lw <= 0 || lh <= 0) {
		return r;
	}
	r.h = ph - strip;
	r.w = (int32_t)(((int64_t)r.h * lw + lh / 2) / lh);
	if (r.w > pw) {
		r.w = pw;
		r.h = (int32_t)(((int64_t)pw * lh + lw / 2) / lw);
	}
	r.x = (pw - r.w) / 2;
	r.y = ph - r.h;
	return r;
}

/* Logical position (16.16) that picture pixel i of n samples, n pixels showing l logical ones: centres line up. */
static inline uint32_t
mapple_lb_src(int32_t i, int32_t n, int32_t l)
{
	int64_t v = (((int64_t)(2 * i + 1) * l) << 15) / n - 0x8000;

	return v < 0 ? 0 : (uint32_t)v;
}

/* Touches: a panel coordinate as a pixel of the picture [o, o + n); the black strip and bars go to its nearest edge. */
static inline int32_t
mapple_lb_clamp(int32_t p, int32_t o, int32_t n)
{
	return p < o ? 0 : p >= o + n ? n - 1 : p - o;
}

/* picture pixel p of n as a fraction of the display, 16.16 (the digitizer's normalized coordinate) */
static inline int32_t
mapple_lb_norm(int32_t p, int32_t n)
{
	return (int32_t)(((int64_t)(2 * p + 1) << 15) / n);
}

/* BGRA pixels as four 16-bit lanes, blended with an 8-bit weight (0..256 towards b) without carries between lanes. */
#define MAPPLE_LB_LANES         0x00ff00ff00ff00ffull

static inline uint64_t
mapple_lb_spread(uint32_t p)
{
	return (p & 0x00ff00ffu) | (uint64_t)((p >> 8) & 0x00ff00ffu) << 32;
}

static inline uint64_t
mapple_lb_mix(uint64_t a, uint64_t b, uint32_t w)
{
	return ((a * (256 - w) + b * w + 0x0080008000800080ull) >> 8) & MAPPLE_LB_LANES;
}

static inline uint32_t
mapple_lb_pack(uint64_t v)
{
	return (uint32_t)(v & 0x00ff00ffu) | (uint32_t)((v >> 32) & 0x00ff00ffu) << 8;
}

/*
 * Boot-arg mp_fb_rsz=2: the display pipe's resizer (DISP_RSZ0, between OVL0 and RDMA0) scales a frame the OVL composes
 * (the presented display at lx, ly over black) onto the whole panel, so the picture lands on mapple_lb_rect's. The
 * input position of output pixel k is off + k * step, in 1/32768 of an input pixel between pixel centres (MediaTek's
 * mtk_rsz_calc_tile_params); it matches mapple_lb_src at the picture's first pixel.
 */
#define MAPPLE_RSZ_UNIT         32768

struct mapple_rsz {
	int32_t in_w, in_h;             /* the OVL's frame */
	int32_t lx, ly;                 /* the display in it */
	uint32_t step_x, step_y, off_x, off_y;
};

/* One axis: the picture's n pixels at `at` of `out` show l logical ones. */
static inline uint32_t
mapple_rsz_axis(int32_t at, int32_t n, int32_t l, int32_t out, int32_t *pos, int32_t *in, uint32_t *off)
{
	int64_t u = MAPPLE_RSZ_UNIT;
	int64_t step = ((int64_t)l * u * 2 + n) / (2 * n);
	int64_t c0 = (int64_t)l * u / (2 * n) - u / 2;
	int64_t need = (int64_t)at * step - c0;
	int64_t p = need <= 0 ? 0 : (need + u - 1) / u;
	int64_t o = p * u + c0 - (int64_t)at * step;

	*pos = (int32_t)p;
	*off = (uint32_t)o;
	*in = (int32_t)((o + (int64_t)(out - 1) * step + u - 1) / u + 1);
	return (uint32_t)step;
}

static inline struct mapple_rsz
mapple_rsz_config(int32_t pw, int32_t ph, int32_t lw, int32_t lh, struct mapple_lb r)
{
	struct mapple_rsz c;

	c.step_x = mapple_rsz_axis(r.x, r.w, lw, pw, &c.lx, &c.in_w, &c.off_x);
	c.step_y = mapple_rsz_axis(r.y, r.h, lh, ph, &c.ly, &c.in_h, &c.off_y);
	return c;
}

#ifdef KERNEL
#include <pexpert/pexpert.h>
#include <pexpert/device_tree.h>

/*
 * The resizer's mode for MappleFramebuffer and MappleHID: boot-arg mp_fb_rsz if given; else 2 when the device tree's
 * /chosen has "mapple-resize" (mkadt --resize, device.conf's resize), 0 on a safe boot or without it.
 */
static inline uint32_t
mapple_rsz_mode(void)
{
	uint32_t v = 0, record = 1;
	DTEntry e;
	void const *p = NULL;
	unsigned int sz = 0;
	if (PE_parse_boot_argn("mp_fb_rsz", &v, sizeof(v))) {
		return v;
	}
	(void)PE_parse_boot_argn("mp_bootrec", &record, sizeof(record));
	if (record != 0 && SecureDTLookupEntry(NULL, "/chosen", &e) == kSuccess &&
	    SecureDTGetProperty(e, "mapple-resize", &p, &sz) == kSuccess && sz >= 4 && *(const uint32_t *)p != 0) {
		return 2;
	}
	return 0;
}
#endif

#endif /* _MAPPLE_LETTERBOX_H_ */
