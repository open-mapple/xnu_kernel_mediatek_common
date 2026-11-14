/*
 * MediaTek's SCP sensor hub, as Android's sensors-1.0 client talks to it: the parts with no hardware in them,
 * shared by bsd/dev/mtk_scp.c and tools/scp_hub_test.c. Facts from the realme 8i's kernel (sensors-1.0/sensorHub:
 * inc_v1/SCP_sensorHub.h, SCP_nanoHub/SCP_nanoHub.c, nanohub/comms.c; hwmon/include/hwmsensor.h).
 *
 * Two channels: IPI 8 carries 48-byte requests and notifications; IPI 24 carries nanohub packets whose payload is a
 * ConfigCmd (enable, rate, latency, flush). The hub writes 44-byte records into a ring in our DRAM and says how far
 * it got in a notification, which also pairs its own clock with the system counter.
 */
#ifndef _MTK_SCP_HUB_H_
#define _MTK_SCP_HUB_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SCP_IPI_LOGGER_INIT     4
#define SCP_IPI_SENSOR          8
#define SCP_IPI_READY           12
#define SCP_IPI_CHRE            24
#define SCP_IPI_CHREX           25
#define SCP_IPI_ERROR_INFO      31
#define SCP_IPI_PAYLOAD         272
#define SCP_SRAM_IMAGE          0xC0000u        /* what READY must report */

/* IPI 8: {u8 sensor, u8 action, u8 event/error, u8 0, 44 bytes} */
#define SH_MSG                  48
#define SH_GET_DATA             2
#define SH_SET_CONFIG           4
#define SH_NOTIFY               6
#define SH_SET_TIMESTAMP        8
#define SH_EV_INIT_DONE         0
#define SH_EV_FIFO_FULL         1
#define SH_EV_DIRECT_PUSH       4

/* the ring: {u32 rp, u32 wp, u32 size, u32 0}, records from +16 */
#define SH_RING_BYTES           0x100000u
#define SH_RING_HDR             16u
#define SH_REC                  44u
#define SH_RING_DATA            ((SH_RING_BYTES - SH_RING_HDR) / SH_REC * SH_REC)

/* Android sensor types (ConfigCmd); records carry type - 1 */
#define SH_ACCEL                1
#define SH_MAG                  2
#define SH_GYRO                 4
#define SH_LIGHT                5
#define SH_PRESSURE             6
#define SH_PROX                 8
#define SH_STEP_DETECTOR        18
#define SH_STEP_COUNTER         19
#define SH_PICKUP               25
#define SH_DEVICE_ORIENTATION   27

#define SH_FLUSH_DONE           1

/* nanohub */
#define NH_SYNC                 0x31
#define NH_ACK                  0
#define NH_NACK                 1
#define NH_BUSY                 2
#define NH_UNMASK_INTR          0x1082          /* u8 interrupt (2: non-wakeup), answer 1 */
#define NH_WRITE                0x1091
#define NH_HDR                  10              /* sync, seq, reason, len */
#define NH_MAX                  (NH_HDR + 255 + 4)
#define NH_EVT_CONFIG           0x300
#define NH_CMD_DISABLE          0
#define NH_CMD_ENABLE           1
#define NH_CMD_FLUSH            2
#define NH_CONFIG               20
#define NH_RATE_ONCHANGE        0xFFFFFF01u

static inline void
sh_put32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static inline void
sh_put64(uint8_t *p, uint64_t v)
{
	sh_put32(p, (uint32_t)v);
	sh_put32(p + 4, (uint32_t)(v >> 32));
}

static inline uint32_t
sh_get32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static inline uint64_t
sh_get64(const uint8_t *p)
{
	return (uint64_t)sh_get32(p) | (uint64_t)sh_get32(p + 4) << 32;
}

/* SET_CONFIG (the ring's address, low 32 bits) or SET_TIMESTAMP: ns and the system counter, read together */
static inline void
sh_msg_time(uint8_t m[SH_MSG], uint8_t action, uint32_t ring_pa, uint64_t ns, uint64_t counter)
{
	for (int i = 0; i < SH_MSG; i++) {
		m[i] = 0;
	}
	m[1] = action;
	sh_put32(m + 4, ring_pa);
	sh_put64(m + 16, ns);
	sh_put64(m + 24, counter);
}

struct sh_notify {
	uint8_t event;
	uint32_t wp;            /* byte offset into the ring's data */
	uint64_t scp_ns;        /* the hub's clock ... */
	uint64_t counter;       /* ... and the system counter at that moment */
};

static inline bool
sh_parse_notify(const uint8_t *m, uint32_t len, struct sh_notify *n)
{
	if (len < 32 || m[1] != SH_NOTIFY) {
		return false;
	}
	n->event = m[2];
	n->wp = sh_get32(m + 8);
	n->scp_ns = sh_get64(m + 16);
	n->counter = sh_get64(m + 24);
	return true;
}

/* CRC-32, polynomial 0x04C11DB7 MSB first, over little-endian words, the last one zero-padded; no final xor */
static inline uint32_t
nh_crc(const uint8_t *p, size_t n, uint32_t crc)
{
	for (size_t i = 0; i < n; i += 4) {
		uint32_t w = 0;
		for (size_t j = 0; j < 4 && i + j < n; j++) {
			w |= (uint32_t)p[i + j] << (8 * j);
		}
		crc ^= w;
		for (int b = 0; b < 32; b++) {
			crc = (crc << 1) ^ ((crc & 0x80000000u) ? 0x04C11DB7u : 0);
		}
	}
	return crc;
}

/* A packet into out (NH_MAX bytes); its length. */
static inline size_t
nh_packet(uint8_t *out, uint32_t seq, uint32_t reason, const uint8_t *data, uint8_t len)
{
	out[0] = NH_SYNC;
	sh_put32(out + 1, seq);
	sh_put32(out + 5, reason);
	out[9] = len;
	for (unsigned i = 0; i < len; i++) {
		out[NH_HDR + i] = data[i];
	}
	sh_put32(out + NH_HDR + len, nh_crc(out, NH_HDR + len, ~0u));
	return NH_HDR + len + 4;
}

struct nh_reply {
	uint32_t seq, reason;
	uint8_t len;
	const uint8_t *data;
};

static inline bool
nh_parse(const uint8_t *in, size_t n, struct nh_reply *r)
{
	if (n < NH_HDR + 4 || in[0] != NH_SYNC || n < (size_t)NH_HDR + in[9] + 4) {
		return false;
	}
	r->seq = sh_get32(in + 1);
	r->reason = sh_get32(in + 5);
	r->len = in[9];
	r->data = in + NH_HDR;
	return sh_get32(in + NH_HDR + r->len) == nh_crc(in, NH_HDR + r->len, ~0u);
}

/* ConfigCmd: {u32 0x300, u64 latency ns, u32 rate (Hz in Q10), u8 type, u8 cmd, u16 flags} */
static inline void
nh_config(uint8_t c[NH_CONFIG], uint8_t type, uint8_t cmd, uint32_t rate, uint64_t latency_ns)
{
	sh_put32(c, NH_EVT_CONFIG);
	sh_put64(c + 4, latency_ns);
	sh_put32(c + 12, rate);
	c[16] = type;
	c[17] = cmd;
	c[18] = c[19] = 0;
}

/* microseconds between samples -> rate in Hz Q10 (1024e9 / period ns) */
static inline uint32_t
nh_rate(uint64_t period_us)
{
	return period_us ? (uint32_t)(1024000000ull / period_us) : 0;
}

/* One record: {u8 type - 1, u8 flush action, 2, u64 hub ns @4, 32 bytes @12}. */
struct sh_record {
	uint8_t type;           /* Android type */
	uint8_t action;
	uint64_t scp_ns;
	int32_t d[8];
};

static inline void
sh_parse_record(const uint8_t *p, struct sh_record *r)
{
	r->type = (uint8_t)(p[0] + 1);
	r->action = p[1];
	r->scp_ns = sh_get64(p + 4);
	for (int i = 0; i < 8; i++) {
		r->d[i] = (int32_t)sh_get32(p + 12 + 4 * i);
	}
}

/* Records from rp up to wp (byte offsets into the ring's data; its end is the same as 0): how many, wrapping. */
static inline uint32_t
sh_ring_count(uint32_t rp, uint32_t wp)
{
	rp = rp == SH_RING_DATA ? 0 : rp;
	wp = wp == SH_RING_DATA ? 0 : wp;
	if (rp > SH_RING_DATA || wp > SH_RING_DATA || rp % SH_REC || wp % SH_REC) {
		return 0;
	}
	return ((wp + SH_RING_DATA - rp) % SH_RING_DATA) / SH_REC;
}

/*
 * More than half the ring between rp and wp is the hub having lapped us (it writes faster than we read): those
 * records are partly overwritten, so they are skipped, not read.
 */
static inline bool
sh_ring_overrun(uint32_t count)
{
	return count > SH_RING_DATA / SH_REC / 2;
}

/* the fastest a continuous sensor runs (100 Hz, CoreMotion's own ceiling); iOS asks for 1 us at times */
#define SH_MIN_PERIOD_US        10000u

static inline uint32_t
sh_period(uint8_t type, uint32_t period_us)
{
	bool continuous = type == SH_ACCEL || type == SH_GYRO || type == SH_MAG || type == SH_PRESSURE;
	return continuous && period_us && period_us < SH_MIN_PERIOD_US ? SH_MIN_PERIOD_US : period_us;
}

static inline uint32_t
sh_ring_next(uint32_t off)
{
	off = (off == SH_RING_DATA ? 0 : off) + SH_REC;
	return off >= SH_RING_DATA ? 0 : off;
}

/*
 * Time. The hub stamps records with its own ns; each notification pairs that clock with the system counter (13 MHz
 * on MT6781, mach_absolute_time's). offset = counter - ticks(scp_ns); a record's mach time = ticks(its ns) + offset.
 */
static inline uint64_t
sh_ns_to_ticks(uint64_t ns, uint64_t hz)
{
	return ns / 1000000000ull * hz + ns % 1000000000ull * hz / 1000000000ull;
}

static inline int64_t
sh_time_offset(const struct sh_notify *n, uint64_t hz)
{
	return (int64_t)(n->counter - sh_ns_to_ticks(n->scp_ns, hz));
}

static inline uint64_t
sh_record_ticks(uint64_t scp_ns, int64_t offset, uint64_t hz)
{
	return (uint64_t)((int64_t)sh_ns_to_ticks(scp_ns, hz) + offset);
}

/*
 * Axes: the hub's x, y, z into the frame iOS uses (x right, y up the screen, z out of it). One byte per output
 * axis: 1-3 the source axis, negative to flip it. Fixed point 16.16 out (IOFixed).
 */
static inline void
sh_axes(const int8_t map[3], const int32_t in[3], int64_t out[3])
{
	for (int i = 0; i < 3; i++) {
		int a = map[i] < 0 ? -map[i] : map[i];
		int64_t v = (a >= 1 && a <= 3) ? in[a - 1] : 0;
		out[i] = map[i] < 0 ? -v : v;
	}
}

/* value / div in 16.16, rounded to nearest; num scales first (e.g. -1000 for m/s^2 x1000 -> g with the sign flip) */
static inline int32_t
sh_fixed(int64_t value, int64_t num, int64_t div)
{
	int64_t x = value * num * 65536;
	return (int32_t)((x >= 0 ? x + div / 2 : x - div / 2) / div);
}

/*
 * Units into iOS's. Android's accelerometer reports the reaction to gravity (+9.8 m/s^2 up, lying flat), iOS
 * gravity itself (-1 g): the sign flips. accel: m/s^2 x1000 -> g; gyro: deg/s x131000 (raw / 7505747 = rad/s)
 * -> rad/s; mag: uT x100 -> uT; pressure: hPa x100 -> hPa.
 */
#define SH_G_MILLI              9806650ll       /* standard gravity in m/s^2 x1000, x1000 again for precision */
static inline int32_t sh_accel_g(int32_t v)      { return sh_fixed(v, -1000, SH_G_MILLI); }
static inline int32_t sh_gyro_rads(int32_t v)    { return sh_fixed(v, 1, 7505747); }
static inline int32_t sh_mag_ut(int32_t v)       { return sh_fixed(v, 1, 100); }
static inline int32_t sh_pressure_hpa(int32_t v) { return sh_fixed(v, 1, 100); }

/*
 * Device orientation from gravity in iOS's frame (accel in g, 16.16), as iOS names it: HID usages
 * 0x64-0x6A on the Apple vendor motion page (locationd takes 100-106).
 * Face up/down when |z| is over 0.8 g; else the larger of |x| and |y|, once it leads the other by 0.25 g.
 * Otherwise the last answer stands (hysteresis), starting from ambiguous.
 */
#define SH_ORIENT_AMBIGUOUS     0x64
#define SH_ORIENT_PORTRAIT      0x65
#define SH_ORIENT_UPSIDE_DOWN   0x66
#define SH_ORIENT_LANDSCAPE_L   0x67
#define SH_ORIENT_LANDSCAPE_R   0x68
#define SH_ORIENT_FACE_UP       0x69
#define SH_ORIENT_FACE_DOWN     0x6A

static inline uint32_t
sh_orientation(int32_t x, int32_t y, int32_t z, uint32_t last)
{
	int32_t ax = x < 0 ? -x : x, ay = y < 0 ? -y : y, az = z < 0 ? -z : z;

	if (az > 52429) {                       /* 0.8 g */
		return z < 0 ? SH_ORIENT_FACE_UP : SH_ORIENT_FACE_DOWN;
	}
	if (ay > ax + 16384) {                  /* 0.25 g */
		return y < 0 ? SH_ORIENT_PORTRAIT : SH_ORIENT_UPSIDE_DOWN;
	}
	if (ax > ay + 16384) {
		return x < 0 ? SH_ORIENT_LANDSCAPE_L : SH_ORIENT_LANDSCAPE_R;
	}
	return last ? last : SH_ORIENT_AMBIGUOUS;
}

#endif /* _MTK_SCP_HUB_H_ */
