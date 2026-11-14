/*
 * What iOS's locationd reads from the iPhone X's always-on processor (SPU) beyond plain HID events, built from the
 * sensor hub's data: the pure parts of MappleSPU.cpp, checked on the Mac by tools/scp-hub-test.sh. Formats from
 * iOS 16.7.16's locationd (the CMA, gesture, device-motion and LastOrientation reports; byte layouts as it parses them).
 *
 * The kernel cannot use the FPU: attitude is fused in fixed point (quaternion Q30, rates and vectors Q16) and floats
 * for the reports are built bit by bit.
 */
#ifndef _MAPPLE_SPU_H_
#define _MAPPLE_SPU_H_

#include <stdbool.h>
#include <stdint.h>

/* IEEE-754 single from a fixed-point value with frac fractional bits (truncated) */
static inline uint32_t
spu_f32(int64_t v, int frac)
{
	uint32_t sign = v < 0 ? 0x80000000u : 0;
	uint64_t u = v < 0 ? (uint64_t)-v : (uint64_t)v;

	if (u == 0) {
		return 0;
	}
	int msb = 63 - __builtin_clzll(u);
	uint32_t exp = (uint32_t)(msb - frac + 127);
	uint32_t man = (uint32_t)((u << (63 - msb)) >> 40) & 0x7fffffu;
	return sign | exp << 23 | man;
}

static inline void
spu_put16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static inline void
spu_put32(uint8_t *p, uint32_t v)
{
	spu_put16(p, (uint16_t)v);
	spu_put16(p + 2, (uint16_t)(v >> 16));
}

static inline void
spu_put64(uint8_t *p, uint64_t v)
{
	spu_put32(p, (uint32_t)v);
	spu_put32(p + 4, (uint32_t)(v >> 32));
}

static inline void
spu_zero(uint8_t *p, unsigned n)
{
	for (unsigned i = 0; i < n; i++) {
		p[i] = 0;
	}
}

/* ---- CMA (0xFF0C/1): steps and activity, as CMMotionCoprocessorReply messages ---- */

#define CMA_MOTION_STATE        0x03
#define CMA_MOTION_STATE_LEN    119
#define CMA_STEP_COUNT          0x04
#define CMA_STEP_COUNT_LEN      64
#define CMA_CONFIGURE           0x06    /* a command: +1 enableActivity, +2 enableStepCounting */

#define CMA_STATIC              0x02
#define CMA_WALKING             0x10
#define CMA_RUNNING             0x40

struct spu_steps {
	uint32_t count;         /* since boot */
	uint32_t delta;         /* since the last message */
	uint32_t cadence_q16;   /* steps a second */
	uint32_t bout_us;       /* since the first step of this walk */
	uint8_t stride_cm;
};

static inline void
spu_cma_steps(uint8_t m[CMA_STEP_COUNT_LEN], const struct spu_steps *s)
{
	spu_zero(m, CMA_STEP_COUNT_LEN);
	m[0] = CMA_STEP_COUNT;
	m[1] = s->stride_cm;
	spu_put32(m + 0x04, s->count);
	/* pace: s/m = 100 / (cadence * stride cm) */
	uint64_t speed_cm_q16 = (uint64_t)s->cadence_q16 * s->stride_cm;
	spu_put32(m + 0x1c, speed_cm_q16 ? spu_f32((int64_t)((100ull << 32) / speed_cm_q16), 16) : 0);
	spu_put32(m + 0x20, spu_f32((int64_t)s->bout_us * 65536 / 1000000, 16));        /* active time, s */
	m[0x24] = (uint8_t)(s->delta > 255 ? 255 : s->delta);
	spu_put32(m + 0x28, spu_f32(s->cadence_q16, 16));
	spu_put32(m + 0x30, s->bout_us);
}

/* walking from a cadence, running at 2.5 steps/s and more; static once no step came for 4 s */
static inline uint32_t
spu_activity(uint32_t cadence_q16, uint64_t since_step_us)
{
	if (since_step_us > 4000000 || cadence_q16 == 0) {
		return CMA_STATIC;
	}
	return cadence_q16 >= (5u << 15) ? CMA_RUNNING : CMA_WALKING;
}

static inline void
spu_cma_motion(uint8_t m[CMA_MOTION_STATE_LEN], uint32_t motion, uint8_t conf, uint32_t seq, uint64_t now_us,
    uint64_t start_us)
{
	spu_zero(m, CMA_MOTION_STATE_LEN);
	m[0] = CMA_MOTION_STATE;
	m[1] = conf;
	spu_put32(m + 0x04, motion);
	m[0x0a] = motion == CMA_WALKING || motion == CMA_RUNNING;
	spu_put32(m + 0x0c, seq);
	spu_put64(m + 0x10, now_us);
	spu_put64(m + 0x18, start_us);
	spu_put32(m + 0x20, motion);
	m[0x24] = conf;
}

/*
 * ---- gesture (0xFF0C/3): raise to wake, report type 6 as SpringBoard's CoreMotion reads it ----
 * +1 event (1 detected, 2 dismissed), +2 start pose (1 face up), +3 view pose (1 nominal), +4 mode (1 normal),
 * +5 host awake, +6 orientation; the rest unused. The event's own time is what SpringBoard takes.
 */

#define SPU_WAKE_LEN            24
#define SPU_WAKE_DETECTED       1
#define SPU_WAKE_DISMISSED      2

static inline void
spu_wake(uint8_t m[SPU_WAKE_LEN], uint8_t event, bool host_awake)
{
	spu_zero(m, SPU_WAKE_LEN);
	m[0] = 6;
	m[1] = event;
	m[2] = 1;
	m[3] = 1;
	m[4] = 1;
	m[5] = host_awake;
	m[6] = 1;
}

/* ---- orientation (0x20/0x8A): LastOrientation's answer to [0x02] ---- */

#define SPU_ORIENT_REPLY_LEN    10

static inline void
spu_orientation_reply(uint8_t m[SPU_ORIENT_REPLY_LEN], uint32_t usage, uint64_t when)
{
	m[0] = 0x02;
	m[1] = (uint8_t)(usage >= 0x64 ? usage - 0x64 : 0);
	spu_put64(m + 2, when);
}

/* ---- attitude: a Mahony filter in fixed point ---- */

#define Q30                     (1 << 30)

static inline uint64_t
spu_isqrt(uint64_t x)
{
	uint64_t r = 0, b = 1ull << 62;

	while (b > x) {
		b >>= 2;
	}
	while (b) {
		if (x >= r + b) {
			x -= r + b;
			r = (r >> 1) + b;
		} else {
			r >>= 1;
		}
		b >>= 2;
	}
	return r;
}

/* v scaled to length 1 in Q30; false for a zero vector */
static inline bool
spu_unit(const int32_t in[3], int32_t out[3])
{
	uint64_t n = 0;
	for (int i = 0; i < 3; i++) {
		n += (uint64_t)((int64_t)in[i] * in[i]);
	}
	uint64_t len = spu_isqrt(n);
	if (len == 0) {
		return false;
	}
	for (int i = 0; i < 3; i++) {
		out[i] = (int32_t)(((int64_t)in[i] << 30) / (int64_t)len);
	}
	return true;
}

static inline int32_t
q30mul(int32_t a, int32_t b)
{
	return (int32_t)(((int64_t)a * b) >> 30);
}

/* angle of (x, y) in degrees Q16, -180..180, by CORDIC */
static inline int32_t
spu_atan2_deg(int64_t y, int64_t x)
{
	static const int32_t atans[16] = {     /* atan(2^-i) in degrees Q16 */
		2949120, 1740967, 919879, 466945, 234379, 117304, 58666, 29335,
		14668, 7334, 3667, 1833, 917, 458, 229, 115,
	};
	int32_t angle = 0;

	if (x == 0 && y == 0) {
		return 0;
	}
	while ((x > (1ll << 40) || x < -(1ll << 40)) || (y > (1ll << 40) || y < -(1ll << 40))) {
		x >>= 1;
		y >>= 1;
	}
	if (x < 0) {
		angle = y >= 0 ? 180 << 16 : -(180 << 16);
		x = -x;
		y = -y;
	}
	for (int i = 0; i < 16; i++) {
		int64_t nx;
		if (y > 0) {
			nx = x + (y >> i);
			y -= x >> i;
			angle += atans[i];
		} else {
			nx = x - (y >> i);
			y += x >> i;
			angle -= atans[i];
		}
		x = nx;
	}
	if (angle > (180 << 16)) {
		angle -= 360 << 16;
	} else if (angle < -(180 << 16)) {
		angle += 360 << 16;
	}
	return angle;
}

struct spu_fusion {
	int32_t q[4];           /* w, x, y, z in Q30: the device's attitude in a z-up frame */
	bool started;
};

/* gravity in the device frame as the attitude has it (Q30, pointing up: what an accelerometer's reaction reads) */
static inline void
spu_up(const struct spu_fusion *f, int32_t v[3])
{
	const int32_t *q = f->q;
	v[0] = 2 * (q30mul(q[1], q[3]) - q30mul(q[0], q[2]));
	v[1] = 2 * (q30mul(q[0], q[1]) + q30mul(q[2], q[3]));
	v[2] = q30mul(q[0], q[0]) - q30mul(q[1], q[1]) - q30mul(q[2], q[2]) + q30mul(q[3], q[3]);
}

/* a x b in Q30 */
static inline void
spu_cross(const int32_t a[3], const int32_t b[3], int32_t o[3])
{
	o[0] = q30mul(a[1], b[2]) - q30mul(a[2], b[1]);
	o[1] = q30mul(a[2], b[0]) - q30mul(a[0], b[2]);
	o[2] = q30mul(a[0], b[1]) - q30mul(a[1], b[0]);
}

/*
 * The attitude straight from one sample (TRIAD): up from gravity, west = up x field, north = west x up; without a
 * field (or one along gravity) any horizontal direction does for north. R's rows are north, west, up in device
 * coordinates; the quaternion from R by the largest of its four candidates.
 */
static inline bool
spu_start(struct spu_fusion *f, const int32_t accel[3], const int32_t *mag)
{
	int32_t react[3] = { -accel[0], -accel[1], -accel[2] }, u[3], m[3], wq[3], w[3], n[3];

	if (!spu_unit(react, u)) {
		return false;
	}
	if (mag && spu_unit(mag, m)) {
		spu_cross(u, m, wq);
	}
	if (!mag || !spu_unit(wq, w)) {
		const int32_t y[3] = { 0, Q30, 0 }, x[3] = { Q30, 0, 0 };
		spu_cross(u, (u[1] > Q30 / 10 * 9 || u[1] < -Q30 / 10 * 9) ? x : y, wq);
		if (!spu_unit(wq, w)) {
			return false;
		}
	}
	spu_cross(w, u, n);
	const int32_t *R[3] = { n, w, u };
	int64_t t = (int64_t)R[0][0] + R[1][1] + R[2][2], qq[4];
	int32_t *q = f->q;
	if (t > 0) {
		int64_t s = (int64_t)spu_isqrt((uint64_t)(t + Q30) << 30) * 2;        /* 4w */
		qq[0] = s / 4;
		qq[1] = (((int64_t)R[2][1] - R[1][2]) << 30) / s;
		qq[2] = (((int64_t)R[0][2] - R[2][0]) << 30) / s;
		qq[3] = (((int64_t)R[1][0] - R[0][1]) << 30) / s;
	} else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
		int64_t s = (int64_t)spu_isqrt((uint64_t)(Q30 + (int64_t)R[0][0] - R[1][1] - R[2][2]) << 30) * 2;   /* 4x */
		qq[0] = (((int64_t)R[2][1] - R[1][2]) << 30) / s;
		qq[1] = s / 4;
		qq[2] = (((int64_t)R[0][1] + R[1][0]) << 30) / s;
		qq[3] = (((int64_t)R[0][2] + R[2][0]) << 30) / s;
	} else if (R[1][1] > R[2][2]) {
		int64_t s = (int64_t)spu_isqrt((uint64_t)(Q30 + (int64_t)R[1][1] - R[0][0] - R[2][2]) << 30) * 2;   /* 4y */
		qq[0] = (((int64_t)R[0][2] - R[2][0]) << 30) / s;
		qq[1] = (((int64_t)R[0][1] + R[1][0]) << 30) / s;
		qq[2] = s / 4;
		qq[3] = (((int64_t)R[1][2] + R[2][1]) << 30) / s;
	} else {
		int64_t s = (int64_t)spu_isqrt((uint64_t)(Q30 + (int64_t)R[2][2] - R[0][0] - R[1][1]) << 30) * 2;   /* 4z */
		qq[0] = (((int64_t)R[1][0] - R[0][1]) << 30) / s;
		qq[1] = (((int64_t)R[0][2] + R[2][0]) << 30) / s;
		qq[2] = (((int64_t)R[1][2] + R[2][1]) << 30) / s;
		qq[3] = s / 4;
	}
	for (int i = 0; i < 4; i++) {
		q[i] = (int32_t)qq[i];
	}
	f->started = true;
	return true;
}

/*
 * One step: gyro in rad/s Q16, accel in iOS's g Q16 (gravity, so negated here into the reaction Mahony expects), mag
 * in uT Q16 or NULL; dt in us. Proportional correction only (Kp 2), from gravity and, with a field, from its
 * horizontal direction. The first sample sets the attitude outright (spu_start).
 */
static inline void
spu_fuse(struct spu_fusion *f, const int32_t gyro[3], const int32_t accel[3], const int32_t *mag, uint32_t dt_us)
{
	int32_t a[3], up[3], e[3] = { 0, 0, 0 }, react[3] = { -accel[0], -accel[1], -accel[2] };
	int32_t *q = f->q;

	if (!f->started && !spu_start(f, accel, mag)) {
		return;
	}
	if (spu_unit(react, a)) {
		spu_up(f, up);
		e[0] = q30mul(a[1], up[2]) - q30mul(a[2], up[1]);
		e[1] = q30mul(a[2], up[0]) - q30mul(a[0], up[2]);
		e[2] = q30mul(a[0], up[1]) - q30mul(a[1], up[0]);
	}
	int32_t m[3];
	if (mag && spu_unit(mag, m)) {
		/* the field into the earth frame (h = q m q*), its horizontal part on x, back into the device frame */
		int32_t w = q[0], x = q[1], y = q[2], z = q[3];
		int64_t hx = 2 * ((int64_t)m[0] * (Q30 / 2 - q30mul(y, y) - q30mul(z, z)) + (int64_t)m[1] * (q30mul(x, y) - q30mul(w, z)) +
		    (int64_t)m[2] * (q30mul(x, z) + q30mul(w, y))) >> 30;
		int64_t hy = 2 * ((int64_t)m[0] * (q30mul(x, y) + q30mul(w, z)) + (int64_t)m[1] * (Q30 / 2 - q30mul(x, x) - q30mul(z, z)) +
		    (int64_t)m[2] * (q30mul(y, z) - q30mul(w, x))) >> 30;
		int64_t hz = 2 * ((int64_t)m[0] * (q30mul(x, z) - q30mul(w, y)) + (int64_t)m[1] * (q30mul(y, z) + q30mul(w, x)) +
		    (int64_t)m[2] * (Q30 / 2 - q30mul(x, x) - q30mul(y, y))) >> 30;
		int32_t bx = (int32_t)spu_isqrt((uint64_t)(hx * hx + hy * hy)), bz = (int32_t)hz;
		int32_t wv[3] = {
			2 * (q30mul(bx, Q30 / 2 - q30mul(y, y) - q30mul(z, z)) + q30mul(bz, q30mul(x, z) - q30mul(w, y))),
			2 * (q30mul(bx, q30mul(x, y) - q30mul(w, z)) + q30mul(bz, q30mul(w, x) + q30mul(y, z))),
			2 * (q30mul(bx, q30mul(w, y) + q30mul(x, z)) + q30mul(bz, Q30 / 2 - q30mul(x, x) - q30mul(y, y))),
		};
		e[0] += q30mul(m[1], wv[2]) - q30mul(m[2], wv[1]);
		e[1] += q30mul(m[2], wv[0]) - q30mul(m[0], wv[2]);
		e[2] += q30mul(m[0], wv[1]) - q30mul(m[1], wv[0]);
	}
	/* rates with the correction (Kp 2, e Q30 -> Q16), then dq = q (x) (0, g) / 2 dt */
	int64_t g[3];
	for (int i = 0; i < 3; i++) {
		g[i] = (int64_t)gyro[i] + ((int64_t)e[i] >> 13);
	}
	int64_t h = (int64_t)dt_us * 65536 / 2000000;          /* dt / 2 in Q16 s */
	int64_t gx = g[0] * h >> 16, gy = g[1] * h >> 16, gz = g[2] * h >> 16; /* Q16 rad */
	int32_t w = q[0], x = q[1], y = q[2], z = q[3];
	int64_t nq[4] = {
		w + ((-x * gx - y * gy - z * gz) >> 16),
		x + ((w * gx + y * gz - z * gy) >> 16),
		y + ((w * gy - x * gz + z * gx) >> 16),
		z + ((w * gz + x * gy - y * gx) >> 16),
	};
	uint64_t n = 0;
	for (int i = 0; i < 4; i++) {
		n += (uint64_t)((nq[i] >> 8) * (nq[i] >> 8));
	}
	uint64_t len = spu_isqrt(n) << 8;
	for (int i = 0; len && i < 4; i++) {
		q[i] = (int32_t)((nq[i] << 30) / (int64_t)len);
	}
}

/*
 * Heading in degrees Q16, 0-360 clockwise from magnetic north: where the device's top (+y) points. With the field
 * in the filter its earth frame has x on magnetic north, y west, z up; +y's earth coordinates are R's second column.
 */
static inline int32_t
spu_heading(const struct spu_fusion *f)
{
	int32_t w = f->q[0], x = f->q[1], y = f->q[2], z = f->q[3];
	int64_t north = 2 * ((int64_t)q30mul(x, y) - q30mul(w, z));
	int64_t west = Q30 - 2 * ((int64_t)q30mul(x, x) + q30mul(z, z));
	int32_t d = spu_atan2_deg(-west, north);
	return d < 0 ? d + (360 << 16) : d;
}

/* ---- device motion (0xFF0C/4 type 2, /5 type 3, /7 type 4) ---- */

#define SPU_DM9_LEN             88
#define SPU_DM_LEN              52

struct spu_motion {
	int32_t q[4];           /* w, x, y, z Q30 */
	int32_t rate[3];        /* rad/s Q16 */
	int32_t user[3];        /* g Q16, gravity taken out */
	int32_t mag[3];         /* uT Q16 */
	int32_t heading;        /* degrees Q16, -1 << 16 = none */
};

static inline void
spu_motion_from(const struct spu_fusion *f, const int32_t gyro[3], const int32_t accel[3], const int32_t *mag,
    struct spu_motion *o)
{
	int32_t up[3];

	spu_up(f, up);
	for (int i = 0; i < 4; i++) {
		o->q[i] = f->q[i];
	}
	for (int i = 0; i < 3; i++) {
		o->rate[i] = gyro[i];
		o->user[i] = accel[i] + (up[i] >> 14);  /* iOS gravity is -up */
		o->mag[i] = mag ? mag[i] : 0;
	}
	o->heading = mag ? spu_heading(f) : -(1 << 16);
}

static inline void
spu_vec_f32(uint8_t *p, const int32_t *v, int n, int frac)
{
	for (int i = 0; i < n; i++) {
		spu_put32(p + 4 * i, spu_f32(v[i], frac));
	}
}

/* quaternion as x, y, z, w */
static inline void
spu_quat_f32(uint8_t *p, const int32_t q[4])
{
	const int32_t xyzw[4] = { q[1], q[2], q[3], q[0] };
	spu_vec_f32(p, xyzw, 4, 30);
}

static inline void
spu_dm9(uint8_t m[SPU_DM9_LEN], const struct spu_motion *o)
{
	spu_zero(m, SPU_DM9_LEN);
	m[0] = 2;
	m[1] = 2;                       /* calibration level: medium */
	spu_vec_f32(m + 0x10, o->mag, 3, 16);
	spu_quat_f32(m + 0x1c, o->q);
	spu_vec_f32(m + 0x2c, o->rate, 3, 16);
	spu_vec_f32(m + 0x38, o->user, 3, 16);
	spu_put32(m + 0x44, spu_f32(o->heading, 16));
	/* +0x48 accuracy, +0x4c the field's bias: 0 */
}

static inline void
spu_dm(uint8_t m[SPU_DM_LEN], uint8_t type, const struct spu_motion *o)
{
	spu_zero(m, SPU_DM_LEN);
	m[0] = type;
	spu_quat_f32(m + 0x0c, o->q);
	spu_vec_f32(m + 0x1c, o->rate, 3, 16);
	spu_vec_f32(m + 0x28, o->user, 3, 16);
}

#endif /* _MAPPLE_SPU_H_ */
