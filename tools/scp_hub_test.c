/*
 * bsd/dev/mtk_scp_hub.h on this machine: the nanohub CRC against a nibble-table one and a known value, packets both
 * ways, the ConfigCmd and IPI 8 layouts, the ring walk across its end, the time mapping, units, axes and orientation.
 * Run by tools/scp-hub-test.sh.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../bsd/dev/mtk_scp_hub.h"

static int failures;

#define CHECK(c) do { if (!(c)) { failures++; printf("FAILED line %d: %s\n", __LINE__, #c); } } while (0)

/* the same CRC four bits at a time, as a table */
static uint32_t
crc_nibbles(const uint8_t *p, size_t n)
{
	uint32_t table[16], crc = ~0u;

	for (uint32_t i = 0; i < 16; i++) {
		uint32_t c = i << 28;
		for (int b = 0; b < 4; b++) {
			c = (c << 1) ^ ((c & 0x80000000u) ? 0x04C11DB7u : 0);
		}
		table[i] = c;
	}
	for (size_t i = 0; i < n; i += 4) {
		uint32_t w = 0;
		for (size_t j = 0; j < 4 && i + j < n; j++) {
			w |= (uint32_t)p[i + j] << (8 * j);
		}
		crc ^= w;
		for (int k = 0; k < 8; k++) {
			crc = (crc << 4) ^ table[crc >> 28];
		}
	}
	return crc;
}

static void
test_crc(void)
{
	uint8_t zero[4] = { 0 }, buf[300];

	CHECK(nh_crc(zero, 4, ~0u) == 0xC704DD7Bu);     /* the STM32-style CRC of one zero word */
	srand(1);
	for (size_t n = 1; n < sizeof(buf); n += 7) {
		for (size_t i = 0; i < n; i++) {
			buf[i] = (uint8_t)rand();
		}
		CHECK(nh_crc(buf, n, ~0u) == crc_nibbles(buf, n));
	}
}

static void
test_packets(void)
{
	uint8_t cfg[NH_CONFIG], pkt[NH_MAX];
	struct nh_reply r;

	nh_config(cfg, SH_ACCEL, NH_CMD_ENABLE, nh_rate(20000), 20000000ull);
	CHECK(sh_get32(cfg) == 0x300);
	CHECK(sh_get64(cfg + 4) == 20000000ull);
	CHECK(sh_get32(cfg + 12) == 51200);             /* 50 Hz in Q10 */
	CHECK(cfg[16] == 1 && cfg[17] == 1 && cfg[18] == 0 && cfg[19] == 0);

	size_t n = nh_packet(pkt, 7, NH_WRITE, cfg, NH_CONFIG);
	CHECK(n == 10 + 20 + 4);
	CHECK(pkt[0] == 0x31 && sh_get32(pkt + 1) == 7 && sh_get32(pkt + 5) == 0x1091 && pkt[9] == 20);
	CHECK(nh_parse(pkt, n, &r) && r.seq == 7 && r.reason == NH_WRITE && r.len == 20 && memcmp(r.data, cfg, 20) == 0);
	pkt[12] ^= 1;
	CHECK(!nh_parse(pkt, n, &r));
	CHECK(!nh_parse(pkt, 13, &r));

	uint8_t one = 1;
	n = nh_packet(pkt, 8, NH_ACK, NULL, 0);
	CHECK(n == 14 && nh_parse(pkt, n, &r) && r.reason == NH_ACK && r.len == 0);
	n = nh_packet(pkt, 8, NH_WRITE, &one, 1);
	CHECK(nh_parse(pkt, n, &r) && r.len == 1 && r.data[0] == 1);
}

static void
test_ipi8(void)
{
	uint8_t m[SH_MSG];
	struct sh_notify nt;

	sh_msg_time(m, SH_SET_CONFIG, 0x4e000000u, 123456789ull, 0x1122334455ull);
	CHECK(m[0] == 0 && m[1] == 4 && m[2] == 0 && sh_get32(m + 4) == 0x4e000000u && sh_get32(m + 8) == 0);
	CHECK(sh_get64(m + 16) == 123456789ull && sh_get64(m + 24) == 0x1122334455ull);

	memset(m, 0, sizeof(m));
	m[1] = SH_NOTIFY;
	m[2] = SH_EV_DIRECT_PUSH;
	sh_put32(m + 8, 44 * 5);
	sh_put64(m + 16, 5000000000ull);
	sh_put64(m + 24, 1000000000ull);
	CHECK(sh_parse_notify(m, SH_MSG, &nt) && nt.event == 4 && nt.wp == 220 && nt.scp_ns == 5000000000ull &&
	    nt.counter == 1000000000ull);
	CHECK(!sh_parse_notify(m, 16, &nt));
	m[1] = SH_GET_DATA;
	CHECK(!sh_parse_notify(m, SH_MSG, &nt));
}

static void
test_records(void)
{
	uint8_t p[SH_REC] = { 0 };
	struct sh_record r;

	p[0] = SH_ACCEL - 1;
	p[1] = SH_FLUSH_DONE;
	sh_put64(p + 4, 0x0102030405060708ull);
	sh_put32(p + 12, (uint32_t)-76);
	sh_put32(p + 16, (uint32_t)-38);
	sh_put32(p + 20, 9868);
	sh_put32(p + 40, 3);
	sh_parse_record(p, &r);
	CHECK(r.type == SH_ACCEL && r.action == 1 && r.scp_ns == 0x0102030405060708ull);
	CHECK(r.d[0] == -76 && r.d[1] == -38 && r.d[2] == 9868 && r.d[7] == 3);
}

static void
test_ring(void)
{
	CHECK(SH_RING_DATA == 0xFFFC8u);
	CHECK(sh_ring_count(0, 0) == 0);
	CHECK(sh_ring_count(0, 44 * 3) == 3);
	CHECK(sh_ring_count(SH_RING_DATA - 44, 44) == 2);       /* across the end */
	CHECK(sh_ring_count(SH_RING_DATA, 88) == 2);            /* the end is 0 */
	CHECK(sh_ring_count(44, SH_RING_DATA) == SH_RING_DATA / 44 - 1);
	CHECK(sh_ring_count(3, 44) == 0);                       /* not on a record */
	CHECK(sh_ring_count(0, SH_RING_DATA + 44) == 0);
	CHECK(sh_ring_next(SH_RING_DATA - 44) == 0);
	CHECK(sh_ring_next(SH_RING_DATA) == 44);
	CHECK(sh_ring_next(0) == 44);

	uint32_t rp = SH_RING_DATA - 88, wp = 132, seen = 0;
	for (uint32_t n = sh_ring_count(rp, wp); n; n--) {
		rp = sh_ring_next(rp);
		seen++;
	}
	CHECK(seen == 5 && rp == wp);

	/* the hub a little past us is fine; a wp just behind rp is stale */
	CHECK(!sh_ring_stale(sh_ring_count(0, 44 * 100)));
	CHECK(sh_ring_stale(sh_ring_count(44 * 10, 44 * 9)));
	CHECK(sh_ring_stale(sh_ring_count(275704, 275660)));    /* run 42's: rp one record ahead */
	CHECK(!sh_ring_stale(sh_ring_count(44 * 10, 44 * 10)));
	CHECK(!sh_ring_stale(sh_ring_count(SH_RING_DATA - 44, 88)));  /* across the end, ahead */
}

static void
test_periods(void)
{
	CHECK(sh_period(SH_ACCEL, 1) == 10000 && sh_period(SH_GYRO, 9999) == 10000 && sh_period(SH_MAG, 16667) == 16667);
	CHECK(sh_period(SH_ACCEL, 0) == 0);                    /* off stays off */
	CHECK(sh_period(SH_LIGHT, 1000) == 1000 && sh_period(SH_PICKUP, 1) == 1);
}

static void
test_time(void)
{
	const uint64_t hz = 13000000;
	struct sh_notify n = { SH_EV_DIRECT_PUSH, 0, 2000000000ull, 91000000ull };   /* hub 2 s = counter 7 s */

	CHECK(sh_ns_to_ticks(1000000000ull, hz) == hz);
	CHECK(sh_ns_to_ticks(1500, hz) == 19);
	CHECK(sh_ns_to_ticks(1000000000000000000ull, hz) == 13000000000000000ull);  /* no overflow at 31 years */
	int64_t off = sh_time_offset(&n, hz);
	CHECK(off == 65000000);
	CHECK(sh_record_ticks(2000000000ull, off, hz) == 91000000ull);
	CHECK(sh_record_ticks(1980000000ull, off, hz) == 91000000ull - 260000);      /* 20 ms earlier */
	n.counter = 1000;                                                           /* hub clock ahead of the counter */
	CHECK(sh_record_ticks(2000000000ull, sh_time_offset(&n, hz), hz) == 1000);
}

static int
near(int32_t got, double want)
{
	double g = got / 65536.0;
	return g - want < 0.0001 && want - g < 0.0001;
}

static void
test_units(void)
{
	CHECK(near(sh_accel_g(9807), -1.0000));         /* lying flat: Android +9.8 m/s^2, iOS -1 g */
	CHECK(near(sh_accel_g(-4903), 0.5));
	CHECK(near(sh_accel_g(0), 0));
	CHECK(near(sh_gyro_rads(7505747), 1.0));
	CHECK(near(sh_gyro_rads(-131000), -0.017453));  /* -1 deg/s */
	CHECK(near(sh_mag_ut(-4250), -42.5));
	CHECK(near(sh_pressure_hpa(101325), 1013.25));

	const int8_t ident[3] = { 1, 2, 3 }, swap[3] = { -2, 1, 3 }, bad[3] = { 0, 4, -3 };
	const int32_t in[3] = { 10, 20, 30 };
	int64_t out[3];
	sh_axes(ident, in, out);
	CHECK(out[0] == 10 && out[1] == 20 && out[2] == 30);
	sh_axes(swap, in, out);
	CHECK(out[0] == -20 && out[1] == 10 && out[2] == 30);
	sh_axes(bad, in, out);
	CHECK(out[0] == 0 && out[1] == 0 && out[2] == -30);
}

static void
test_orientation(void)
{
	const int32_t g = 65536;

	CHECK(sh_orientation(0, -g, 0, 0) == SH_ORIENT_PORTRAIT);
	CHECK(sh_orientation(0, g, 0, 0) == SH_ORIENT_UPSIDE_DOWN);
	CHECK(sh_orientation(-g, 0, 0, 0) == SH_ORIENT_LANDSCAPE_L);   /* rotated left: its right side up */
	CHECK(sh_orientation(g, 0, 0, 0) == SH_ORIENT_LANDSCAPE_R);
	CHECK(sh_orientation(0, 0, -g, 0) == SH_ORIENT_FACE_UP);
	CHECK(sh_orientation(0, 0, g, 0) == SH_ORIENT_FACE_DOWN);
	CHECK(sh_orientation(-g / 2, -g / 2, g / 2, 0) == SH_ORIENT_AMBIGUOUS);
	/* 45 degrees between portrait and landscape keeps the last answer */
	CHECK(sh_orientation(-46341, -46341, 0, SH_ORIENT_PORTRAIT) == SH_ORIENT_PORTRAIT);
	CHECK(sh_orientation(-46341, -46341, 0, SH_ORIENT_LANDSCAPE_L) == SH_ORIENT_LANDSCAPE_L);
	/* debounced: a change counts after three samples in a row; a one-sample flick does not */
	struct sh_orient d = { 0, 0, 0 };
	uint64_t t = 0;
	CHECK(sh_orient_step(&d, 0, -g, 0, t) == 0 && sh_orient_step(&d, 0, -g, 0, t += 490000) == 0);
	CHECK(sh_orient_step(&d, 0, -g, 0, t += 10000) == SH_ORIENT_PORTRAIT);
	CHECK(sh_orient_step(&d, -g, 0, 0, t += 10000) == 0);           /* a flick to landscape */
	CHECK(sh_orient_step(&d, 0, -g, 0, t += 10000) == 0 && sh_orient_step(&d, 0, -g, 0, t += 600000) == 0);
	CHECK(d.shown == SH_ORIENT_PORTRAIT);
	CHECK(sh_orient_step(&d, -g, 0, 0, t += 10000) == 0 && sh_orient_step(&d, -g, 0, 0, t += 400000) == 0);
	CHECK(sh_orient_step(&d, -g, 0, 0, t += 100000) == SH_ORIENT_LANDSCAPE_L);
	CHECK(sh_orient_step(&d, -g, 0, 0, t += 10000) == 0);           /* no repeat */
	/* landscape tilted back about 37 degrees, z jittering across 0.8 g at 100 Hz (run 43): no change reported */
	int changes = 0;
	for (int i = 0; i < 200; i++) {
		changes += sh_orient_step(&d, -37450, 0, (i & 4) ? -53740 : -51118, t += 10000) != 0;
	}
	CHECK(changes == 1 && d.shown == SH_ORIENT_FACE_UP);            /* face up from the first crossing on, once */
	CHECK(sh_orientation(-37450, 0, -45875, SH_ORIENT_FACE_UP) == SH_ORIENT_FACE_UP);      /* 0.7 g: still face up */
	CHECK(sh_orientation(-g, 0, -39322, SH_ORIENT_FACE_UP) == SH_ORIENT_LANDSCAPE_L);      /* 0.6 g: out */
	/* tilted 30 degrees back in portrait: still portrait */
	CHECK(sh_orientation(0, -56756, -g / 2, SH_ORIENT_FACE_UP) == SH_ORIENT_PORTRAIT);
}

int
main(void)
{
	test_crc();
	test_packets();
	test_ipi8();
	test_records();
	test_ring();
	test_periods();
	test_time();
	test_units();
	test_orientation();
	printf("scp hub: %s\n", failures ? "FAILED" : "all passed");
	return failures != 0;
}
