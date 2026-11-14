/*
 * iokit/Kernel/arm/hid/MappleSPU.h on this machine: floats built from fixed point, the CMA, wake and LastOrientation
 * messages at locationd's offsets, the integer square root and atan2, and the attitude filter (still, tilted,
 * turning, heading). Run by tools/scp-hub-test.sh.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../iokit/Kernel/arm/hid/MappleSPU.h"

static int failures;

#define CHECK(c) do { if (!(c)) { failures++; printf("FAILED line %d: %s\n", __LINE__, #c); } } while (0)

static float
f32(const uint8_t *p)
{
	uint32_t u = (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
	float f;
	memcpy(&f, &u, 4);
	return f;
}

static uint32_t
u32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static float
bits(uint32_t u)
{
	float f;
	memcpy(&f, &u, 4);
	return f;
}

static int
close_to(double a, double b, double tol)
{
	return fabs(a - b) <= tol;
}

static void
test_floats(void)
{
	CHECK(spu_f32(0, 16) == 0);
	CHECK(spu_f32(65536, 16) == 0x3f800000u);
	CHECK(spu_f32(-163840, 16) == 0xc0200000u);             /* -2.5 */
	CHECK(spu_f32(1 << 29, 30) == 0x3f000000u);              /* 0.5 */
	CHECK(close_to(bits(spu_f32(12345678, 16)), 12345678 / 65536.0, 0.001));
	CHECK(close_to(bits(spu_f32(-3, 16)), -3 / 65536.0, 1e-9));
}

static void
test_messages(void)
{
	uint8_t m[128];
	struct spu_steps s = { 1234, 3, 2u << 16, 5000000, 70 };

	spu_cma_steps(m, &s);
	CHECK(m[0] == 4 && m[1] == 70 && u32(m + 4) == 1234 && m[0x24] == 3);
	CHECK(close_to(f32(m + 0x28), 2.0, 1e-4));                     /* cadence */
	CHECK(close_to(f32(m + 0x1c), 100.0 / (2 * 70), 1e-3));        /* pace s/m */
	CHECK(close_to(f32(m + 0x20), 5.0, 1e-3));                     /* active time */
	CHECK(u32(m + 0x30) == 5000000 && u32(m + 0x08) == 0 && m[63] == 0);

	CHECK(spu_activity(0, 0) == CMA_STATIC);
	CHECK(spu_activity(2u << 16, 1000000) == CMA_WALKING);
	CHECK(spu_activity(3u << 16, 1000000) == CMA_RUNNING);
	CHECK(spu_activity(2u << 16, 5000000) == CMA_STATIC);

	spu_cma_motion(m, CMA_WALKING, 2, 7, 9000000, 4000000);
	CHECK(m[0] == 3 && m[1] == 2 && u32(m + 4) == 0x10 && m[0x0a] == 1 && u32(m + 0x0c) == 7);
	CHECK(u32(m + 0x10) == 9000000 && u32(m + 0x18) == 4000000 && u32(m + 0x20) == 0x10 && m[0x24] == 2 && m[118] == 0);

	static const uint8_t wake[8] = { 6, 1, 1, 1, 1, 0, 1, 0 }, dismiss[8] = { 6, 2, 1, 1, 1, 1, 1, 0 };
	spu_wake(m, SPU_WAKE_DETECTED, false);
	CHECK(memcmp(m, wake, 8) == 0 && u32(m + 20) == 0);
	spu_wake(m, SPU_WAKE_DISMISSED, true);
	CHECK(memcmp(m, dismiss, 8) == 0);

	spu_orientation_reply(m, 0x67, 0xabcdef);
	CHECK(m[0] == 2 && m[1] == 3 && u32(m + 2) == 0xabcdef && u32(m + 6) == 0);
}

static void
test_maths(void)
{
	CHECK(spu_isqrt(0) == 0 && spu_isqrt(1) == 1 && spu_isqrt(99) == 9 && spu_isqrt(1ull << 60) == 1ull << 30);
	CHECK(spu_isqrt(0xffffffffffffffffull) == 0xffffffffull);
	const double cases[][2] = { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -1, 0 }, { -1, 1 },
		{ 3, 4 }, { -7, 2 }, { 0.001, 1000 } };
	for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		double y = cases[i][0], x = cases[i][1];
		double got = spu_atan2_deg((int64_t)(y * 1e6), (int64_t)(x * 1e6)) / 65536.0;
		double want = atan2(y, x) * 180 / M_PI;
		if (want == 180 && got < 0) {
			got += 360;
		}
		CHECK(close_to(got, want, 0.01));
	}
}

static double
deg_between_up(const struct spu_fusion *f, double x, double y, double z)
{
	int32_t up[3];
	spu_up(f, up);
	double ux = up[0] / (double)Q30, uy = up[1] / (double)Q30, uz = up[2] / (double)Q30;
	double n = sqrt(x * x + y * y + z * z);
	double c = (ux * x + uy * y + uz * z) / n;
	return acos(c > 1 ? 1 : c < -1 ? -1 : c) * 180 / M_PI;
}

static void
test_fusion(void)
{
	struct spu_fusion f = { { 0 }, false };
	const int32_t still[3] = { 0, 0, 0 }, flat[3] = { 0, 0, -65536 };
	const int32_t north[3] = { 0, 1310720, -2621440 };     /* 20 uT along y, 40 uT down: y points north */

	for (int i = 0; i < 200; i++) {
		spu_fuse(&f, still, flat, NULL, 10000);
	}
	CHECK(deg_between_up(&f, 0, 0, 1) < 0.5);             /* yaw is arbitrary without a field */

	/* upright in portrait: iOS reads y = -1 g; the filter's up converges onto +y */
	const int32_t upright[3] = { 0, -65536, 0 };
	for (int i = 0; i < 1000; i++) {
		spu_fuse(&f, still, upright, NULL, 10000);
	}
	CHECK(deg_between_up(&f, 0, 1, 0) < 1.0);

	struct spu_motion o;
	spu_motion_from(&f, still, upright, NULL, &o);
	CHECK(abs(o.user[0]) < 700 && abs(o.user[1]) < 700 && abs(o.user[2]) < 700);     /* < 0.01 g left */
	CHECK(o.heading == -(1 << 16));

	/* starting face down: up is -z at once */
	struct spu_fusion d = { { 0 }, false };
	const int32_t down[3] = { 0, 0, 65536 };
	spu_fuse(&d, still, down, NULL, 10000);
	CHECK(deg_between_up(&d, 0, 0, -1) < 1.0);

	/* flat again, turning about z at 1 rad/s for 1 s with no correction about z: 57.3 degrees */
	struct spu_fusion t = { { 0 }, false };
	const int32_t spin[3] = { 0, 0, 65536 };
	spu_fuse(&t, still, flat, NULL, 10000);
	double yaw0 = 2 * atan2(t.q[3] / (double)Q30, t.q[0] / (double)Q30) * 180 / M_PI;
	for (int i = 0; i < 100; i++) {
		spu_fuse(&t, spin, flat, NULL, 10000);
	}
	double yaw = 2 * atan2(t.q[3] / (double)Q30, t.q[0] / (double)Q30) * 180 / M_PI - yaw0;
	CHECK(close_to(yaw, 57.2958, 0.5));

	/* heading: flat, y towards magnetic north reads 0; turned 90 degrees clockwise (y towards east) reads 90 */
	struct spu_fusion h = { { 0 }, false };
	for (int i = 0; i < 100; i++) {
		spu_fuse(&h, still, flat, north, 10000);
	}
	double hd = spu_heading(&h) / 65536.0;
	CHECK(close_to(hd, 0, 1.0) || close_to(hd, 360, 1.0));
	const int32_t east[3] = { -1310720, 0, -2621440 };      /* north now along -x */
	struct spu_fusion h2 = { { 0 }, false };
	for (int i = 0; i < 100; i++) {
		spu_fuse(&h2, still, flat, east, 10000);
	}
	CHECK(close_to(spu_heading(&h2) / 65536.0, 90, 1.0));

	uint8_t m[SPU_DM9_LEN];
	spu_motion_from(&h, still, flat, north, &o);
	spu_dm9(m, &o);
	double qn = 0;
	for (int i = 0; i < 4; i++) {
		qn += f32(m + 0x1c + 4 * i) * f32(m + 0x1c + 4 * i);
	}
	CHECK(m[0] == 2 && close_to(qn, 1.0, 1e-3) && (close_to(f32(m + 0x44), 0, 1) || close_to(f32(m + 0x44), 360, 1)));
	CHECK(close_to(f32(m + 0x14), 20.0, 1e-3) && close_to(f32(m + 0x18), -40.0, 1e-3));
	spu_dm(m, 4, &o);
	CHECK(m[0] == 4 && close_to(f32(m + 0x28), 0, 0.01) && close_to(f32(m + 0x28 + 8), 0, 0.01));
}

int
main(void)
{
	test_floats();
	test_messages();
	test_maths();
	test_fusion();
	printf("spu messages: %s\n", failures ? "FAILED" : "all passed");
	return failures != 0;
}
