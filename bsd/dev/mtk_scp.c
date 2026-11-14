/*
 * The sensor hub on MediaTek's SCP (osfmk/arm64/mt6781_scp.c has the registers): its start, the IPI messages, the
 * nanohub link that turns sensors on and off, and the ring in our DRAM that the hub writes records into. Records go
 * to iokit/Kernel/arm/hid/MappleSPU.cpp's HID services. Facts from the realme 8i's kernel (scp/cm4,
 * sensors-1.0/sensorHub); the protocol's pure parts are in mtk_scp_hub.h.
 *
 * One thread owns the SCP: it polls for messages every 2 ms, answers them, and applies what the HID services ask
 * for (mtk_scp_sensor_set only records it).
 *
 * Boot-arg mp_sensor_steps (default 0), each step with the ones before it:
 * 1 status, read only: scp_sel's clock, cfg+0, the PC twice, LR, sleep, to host, the general registers, LK's region
 *   info in the SCP's SRAM.
 * 2 the IPI handshake: scp_sel to clk26m (only with mp_scp_clk26m=1, and only if it is not there), the start
 *   (cfg+0 = 1) when LK's region info is there and its DRAM lies in a memory hole; READY (30 s), the hub's
 *   INIT_DONE, the ring (1 MiB below 2 GiB, taken at boot) given with SET_CONFIG, the non-wakeup interrupt
 *   unmasked (mp_scp_unmask=0 skips it; again whenever sensors are on and 5 s pass with no record),
 *   SET_TIMESTAMP every 10 s. The SCP's
 *   first 64 messages are logged.
 * 3 the accelerometer and its HID service (one sample a second logged for the first 30 s). Once the hub is up, a
 *   status line (the ring's header, what was pushed, records by Android type) a second for 30 s, then every 10 s
 *   while records come and every minute regardless; at 3 and 6 s the hub's own latest accel record (GET_DATA).
 * 4 the plain sensors and their HID services.
 * 5 and what locationd reads from the SPU beyond them: steps and activity, raise to wake, device motion.
 */
#if defined(ARM64_BOARD_CONFIG_MT6781)
#include <stdbool.h>
#include <stdint.h>
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_time.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include "mtk_scp_hub.h"

extern bool mt6781_scp_map(void);
extern uint32_t mt6781_scp_cfg(uint32_t off);
extern uint32_t mt6781_scp_sram(uint32_t off);
extern uint32_t mt6781_scp_clock(void);
extern bool mt6781_scp_clock_26m(void);
extern bool mt6781_scp_run(void);
extern bool mt6781_scp_send(uint32_t id, const void *data, uint32_t len);
extern bool mt6781_scp_recv(uint32_t *id, void *data, uint32_t *len, uint32_t max);
extern bool mapple_pa_in_hole(uint64_t pa);                                     /* osfmk/arm64/arm_vm_init.c */
extern volatile uint8_t *mapple_spu_ring(uint64_t *pa);                         /* MappleSPU.cpp */
extern void mapple_spu_start(uint32_t steps);                                   /* MappleSPU.cpp */
extern void mapple_spu_record(const struct sh_record *r, uint64_t ticks);       /* MappleSPU.cpp */
extern void mapple_spu_tick(void);                                              /* MappleSPU.cpp */
extern kern_return_t thread_terminate(thread_t thread);

void mtk_scp_start(void);
void mtk_scp_sensor_set(uint8_t type, uint32_t period_us, uint32_t latency_us);
void mtk_scp_sensor_rearm(uint8_t type);

#define REGION_INFO     0x400           /* LK's: loader, firmware, DRAM (start, size each), ... */
#define SAID_MAX        64

static uint32_t steps;
static volatile uint8_t *ring;          /* write-combined: what the hub writes is read as it lands */
static uint64_t ring_pa;
static uint32_t said;
static bool ready, hub_init, hub_up;
static uint64_t hz, last_stamp;
static int64_t offset;                  /* system counter - hub ticks (mtk_scp_hub.h) */
static int64_t mach_minus_counter;
static uint32_t nh_seq;
static uint32_t notifies, records, last_wp, bad_wp;      /* what the hub has pushed, for the status lines */
static uint32_t by_type[SH_DEVICE_ORIENTATION + 1], overruns;
static uint8_t last_event;

/* what the HID services want, per Android type; applied by the thread */
static LCK_GRP_DECLARE(scp_grp, "mp_scp");
static LCK_SPIN_DECLARE(want_lock, &scp_grp);
static struct {
	uint32_t period_us, latency_us;
	bool dirty;
} want[SH_DEVICE_ORIENTATION + 1];

/* the IPI 8 reply being waited for, and the nanohub packet */
static uint8_t sh_reply[SH_MSG], nh_buf[NH_MAX];
static bool sh_waiting, sh_got, nh_waiting, nh_ack, nh_got;
static uint8_t sh_wait_action;
static uint32_t nh_wait_seq, nh_len;

static uint64_t
counter_now(void)
{
	return __builtin_arm_rsr64("CNTVCT_EL0");
}

static void
say(uint32_t id, const uint8_t *m, uint32_t len)
{
	if (said++ < SAID_MAX) {
		printf("scp: id %u, %u bytes: %02x%02x%02x%02x %08x %08x %08x %08x %08x %08x\n", id, len, m[0], m[1], m[2],
		    m[3], sh_get32(m + 4), sh_get32(m + 8), sh_get32(m + 12), sh_get32(m + 16), sh_get32(m + 20),
		    sh_get32(m + 24));
	}
}

/* the ring's own header: the hub moves wp as it writes (the Hot 20 read its samples from it) */
static uint32_t
ring_word(uint32_t off)
{
	return *(volatile uint32_t *)(ring + off);
}

/*
 * Records up to wp (from a notification, or the header while no notification has come: then stamped on arrival, as
 * there is no clock pairing); a wp that is not a record boundary is counted, not used.
 */
static void
drain(uint32_t wp)
{
	uint64_t arrival = notifies ? 0 : mach_absolute_time();
	uint32_t rp = ring_word(0);
	uint32_t n = sh_ring_count(rp, wp);
	uint8_t rec[SH_REC];
	struct sh_record r;

	if (n == 0 && wp != rp && (wp % SH_REC || wp > SH_RING_DATA)) {
		if (bad_wp++ < 4) {
			printf("scp: ring wp %u not a record boundary (rp %u, header wp %u)\n", wp, rp, ring_word(4));
		}
		return;
	}
	if (sh_ring_overrun(n)) {
		if (overruns++ < 4) {
			printf("scp: ring overrun (rp %u, wp %u: %u records behind), skipped\n", rp, wp, n);
		}
		*(volatile uint32_t *)ring = wp == SH_RING_DATA ? 0 : wp;
		return;
	}
	records += n;
	for (; n; n--) {
		for (uint32_t i = 0; i < SH_REC; i++) {
			rec[i] = ring[SH_RING_HDR + rp + i];
		}
		sh_parse_record(rec, &r);
		if (r.type <= SH_DEVICE_ORIENTATION) {
			by_type[r.type]++;
		}
		mapple_spu_record(&r, arrival ? arrival : (uint64_t)((int64_t)sh_record_ticks(r.scp_ns, offset, hz) + mach_minus_counter));
		rp = sh_ring_next(rp);
	}
	*(volatile uint32_t *)ring = wp == SH_RING_DATA ? 0 : wp;
}

/* One message from the SCP, if one waits; true if there was one. */
static bool
pump(void)
{
	uint8_t m[SCP_IPI_PAYLOAD];
	uint32_t id, len, done;
	struct sh_notify nt;
	struct nh_reply r;

	if (!mt6781_scp_recv(&id, m, &len, sizeof(m))) {
		return false;
	}
	switch (id) {
	case SCP_IPI_READY:
		say(id, m, len);
		printf("scp: ready, SRAM image %#x%s\n", sh_get32(m), sh_get32(m) == SCP_SRAM_IMAGE ? "" : " (not 0xc0000)");
		ready = true;
		break;
	case SCP_IPI_SENSOR:
		if (sh_parse_notify(m, len, &nt)) {
			if (nt.event == SH_EV_INIT_DONE) {
				say(id, m, len);
				hub_init = true;
			} else if (hub_up && (nt.event == SH_EV_DIRECT_PUSH || nt.event == SH_EV_FIFO_FULL)) {
				if (notifies++ < 4) {
					say(id, m, len);
				}
				last_wp = nt.wp;
				last_event = nt.event;
				offset = sh_time_offset(&nt, hz);
				drain(nt.wp);
			} else {
				say(id, m, len);
			}
		} else if (sh_waiting && len >= 4 && m[1] == sh_wait_action) {
			for (uint32_t i = 0; i < SH_MSG; i++) {
				sh_reply[i] = i < len ? m[i] : 0;
			}
			sh_got = true;
		} else {
			say(id, m, len);
		}
		break;
	case SCP_IPI_CHRE:
		done = len;
		if (nh_parse(m, len, &r) && nh_waiting && r.seq == nh_wait_seq) {
			if (r.reason == NH_ACK && r.len == 0 && !nh_ack) {
				nh_ack = true;
			} else {
				nh_len = r.len;
				for (uint32_t i = 0; i < r.len; i++) {
					nh_buf[i] = r.data[i];
				}
				nh_got = r.reason != NH_NACK && r.reason != NH_BUSY;
				nh_waiting = !nh_got && r.reason == NH_BUSY;
			}
		} else {
			say(id, m, len);
		}
		mt6781_scp_send(SCP_IPI_CHREX, &done, sizeof(done));
		break;
	default:
		say(id, m, len);
		break;
	}
	return true;
}

static void
pause_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

/* Wait up to ms for *flag, answering everything else meanwhile. */
static bool
wait_for(volatile bool *flag, uint32_t ms)
{
	for (uint32_t t = 0; !*flag && t < ms; t += 2) {
		while (pump() && !*flag) {
		}
		if (!*flag) {
			pause_ms(2);
		}
	}
	return *flag;
}

/* IPI 8 request and its reply (same action, error byte 0); 500 ms as Android waits */
static bool
sh_request(const uint8_t m[SH_MSG])
{
	sh_waiting = true;
	sh_got = false;
	sh_wait_action = m[1];
	for (int i = 0; i < 100 && !mt6781_scp_send(SCP_IPI_SENSOR, m, SH_MSG); i++) {
		pause_ms(1);
	}
	bool ok = wait_for(&sh_got, 500) && sh_reply[2] == 0;
	sh_waiting = false;
	if (!ok) {
		printf("scp: hub action %u: %s (error %u)\n", m[1], sh_got ? "refused" : "no answer", sh_reply[2]);
	}
	return ok;
}

static void
stamp(void)
{
	uint8_t m[SH_MSG];
	uint64_t c = counter_now(), ns;

	mach_minus_counter = (int64_t)(mach_absolute_time() - c);       /* moves across sleep */
	absolutetime_to_nanoseconds((uint64_t)((int64_t)c + mach_minus_counter), &ns);
	sh_msg_time(m, SH_SET_TIMESTAMP, 0, ns, c);
	sh_request(m);
	last_stamp = c;
}

/* One nanohub transaction: the packet, its ACK, then the answer; its first byte (nonzero = taken), -1 if none. */
static int
nh_transact(uint32_t reason, const uint8_t *data, uint8_t len)
{
	uint8_t pkt[NH_MAX];

	for (int attempt = 0; attempt < 3; attempt++) {
		size_t n = nh_packet(pkt, ++nh_seq, reason, data, len);
		nh_wait_seq = nh_seq;
		nh_waiting = true;
		nh_ack = nh_got = false;
		if (mt6781_scp_send(SCP_IPI_CHRE, pkt, (uint32_t)n) && wait_for(&nh_got, 200) && nh_len >= 1 && nh_buf[0]) {
			nh_waiting = false;
			return nh_buf[0];
		}
		nh_waiting = false;
	}
	return nh_got && nh_len ? nh_buf[0] : -1;
}

/* A ConfigCmd through nanohub. */
static bool
nh_send_config(uint8_t type, uint32_t period_us, uint32_t latency_us)
{
	uint8_t c[NH_CONFIG];
	uint32_t rate;
	bool on = period_us != 0;

	if (type == SH_PICKUP) {
		rate = 0xFFFFFF02u;     /* one shot */
	} else if (type == SH_LIGHT || type == SH_PROX || type == SH_STEP_COUNTER || type == SH_STEP_DETECTOR ||
	    type == SH_DEVICE_ORIENTATION) {
		rate = NH_RATE_ONCHANGE;
	} else {
		rate = nh_rate(period_us);
	}
	nh_config(c, type, on ? NH_CMD_ENABLE : NH_CMD_DISABLE, on ? rate : 0, (uint64_t)latency_us * 1000);
	if (nh_transact(NH_WRITE, c, NH_CONFIG) > 0) {
		printf("scp: sensor %u %s (%u us, latency %u us)\n", type, on ? "on" : "off", period_us, latency_us);
		return true;
	}
	printf("scp: sensor %u %s: not taken (ack %d, answer %d, %u bytes, %#x)\n", type, on ? "on" : "off", nh_ack,
	    nh_got, nh_len, nh_len ? nh_buf[0] : 0);
	return false;
}

/* IPI 8 GET_DATA: the hub's latest record of one sensor, straight from it (data_unit_t at byte 4 of the answer) */
static void
get_data(uint8_t type)
{
	uint8_t m[SH_MSG] = { (uint8_t)(type - 1), SH_GET_DATA };

	if (sh_request(m)) {
		printf("scp: sensor %u now: handle %u, action %u, time %llu, %d %d %d\n", type, sh_reply[4], sh_reply[5],
		    sh_get64(sh_reply + 8), (int32_t)sh_get32(sh_reply + 16), (int32_t)sh_get32(sh_reply + 20),
		    (int32_t)sh_get32(sh_reply + 24));
	}
}

/* the non-wakeup interrupt, as Android's nanohub unmasks it on resume: the hub holds such data otherwise */
static uint32_t unmask = 1;

static int
unmask_nonwakeup(void)
{
	uint8_t intr = 2;
	return nh_transact(NH_UNMASK_INTR, &intr, 1);
}

/* After READY and INIT_DONE: the ring's header, its address, the time; then what is wanted. */
static void
hub_power_up(void)
{
	uint8_t m[SH_MSG];
	uint64_t c = counter_now(), ns;

	*(volatile uint32_t *)(ring + 0) = 0;
	*(volatile uint32_t *)(ring + 4) = 0;
	*(volatile uint32_t *)(ring + 8) = SH_RING_DATA;
	*(volatile uint32_t *)(ring + 12) = 0;
	__builtin_arm_dsb(0xf);
	absolutetime_to_nanoseconds((uint64_t)((int64_t)c + mach_minus_counter), &ns);
	sh_msg_time(m, SH_SET_CONFIG, (uint32_t)ring_pa, ns, c);
	sh_put32(m + 8, SH_RING_BYTES);         /* Android leaves it unset; the Hot 20's hub ran with it filled */
	hub_up = sh_request(m);
	printf("scp: hub %s, ring at %#llx\n", hub_up ? "up" : "did not take its ring", ring_pa);
	if (hub_up) {
		(void)PE_parse_boot_argn("mp_scp_unmask", &unmask, sizeof(unmask));
		if (unmask) {
			printf("scp: non-wakeup interrupt unmasked: %d\n", unmask_nonwakeup());
		}
		stamp();
		lck_spin_lock(&want_lock);
		for (unsigned t = 0; t <= SH_DEVICE_ORIENTATION; t++) {
			want[t].dirty = want[t].period_us != 0;
		}
		lck_spin_unlock(&want_lock);
	}
}

static bool
any_wanted(void)
{
	bool any = false;

	lck_spin_lock(&want_lock);
	for (unsigned t = 0; t <= SH_DEVICE_ORIENTATION; t++) {
		any |= want[t].period_us != 0 && t != SH_PICKUP;
	}
	lck_spin_unlock(&want_lock);
	return any;
}

static void
apply_wanted(void)
{
	for (unsigned t = 0; t <= SH_DEVICE_ORIENTATION; t++) {
		uint32_t p, l;
		lck_spin_lock(&want_lock);
		bool dirty = want[t].dirty;
		want[t].dirty = false;
		p = want[t].period_us;
		l = want[t].latency_us;
		lck_spin_unlock(&want_lock);
		if (dirty) {
			nh_send_config((uint8_t)t, p, l);
		}
	}
}

/* period 0 = off. Step 3 allows the accelerometer only. */
void
mtk_scp_sensor_set(uint8_t type, uint32_t period_us, uint32_t latency_us)
{
	if (type > SH_DEVICE_ORIENTATION || steps < 3 || (steps == 3 && type != SH_ACCEL)) {
		return;
	}
	period_us = sh_period(type, period_us);
	lck_spin_lock(&want_lock);
	if (want[type].period_us != period_us || want[type].latency_us != latency_us) {
		want[type].period_us = period_us;
		want[type].latency_us = latency_us;
		want[type].dirty = true;
	}
	lck_spin_unlock(&want_lock);
}

/* A one-shot sensor (pickup) disables itself when it fires: enabled again as it was. */
void
mtk_scp_sensor_rearm(uint8_t type)
{
	if (type > SH_DEVICE_ORIENTATION) {
		return;
	}
	lck_spin_lock(&want_lock);
	want[type].dirty = want[type].period_us != 0;
	lck_spin_unlock(&want_lock);
}

static void
status(const char *when)
{
	uint32_t clk = mt6781_scp_clock(), pc0 = mt6781_scp_cfg(0xb4);

	delay(1000);
	printf("scp: %s: scp_sel %u%s, run %#x, PC %#x then %#x, LR %#x, sleep %#x, to host %#x, GIPC %#x, general %#x "
	    "%#x %#x %#x %#x %#x %#x %#x\n", when, (clk >> 8) & 7, (clk & (1u << 15)) ? " powered down" : "", mt6781_scp_cfg(0),
	    pc0, mt6781_scp_cfg(0xb4), mt6781_scp_cfg(0xac), mt6781_scp_cfg(0x114), mt6781_scp_cfg(0x1c),
	    mt6781_scp_cfg(0x28), mt6781_scp_cfg(0x50), mt6781_scp_cfg(0x54), mt6781_scp_cfg(0x58), mt6781_scp_cfg(0x5c),
	    mt6781_scp_cfg(0x60), mt6781_scp_cfg(0x64), mt6781_scp_cfg(0x68), mt6781_scp_cfg(0x6c));
}

static void
worker(__unused void *arg, __unused wait_result_t w)
{
	uint32_t info[6], clk26m = 0;

	status("at boot");
	for (int i = 0; i < 6; i++) {
		info[i] = mt6781_scp_sram(REGION_INFO + 4 * i);
	}
	printf("scp: LK's region info: loader %#x+%#x, firmware %#x+%#x, DRAM %#x+%#x\n", info[0], info[1], info[2],
	    info[3], info[4], info[5]);
	if (steps < 2) {
		goto out;
	}
	/* the DRAM's size reads 0xffffffff on spaced: its start only */
	bool sane = info[0] && info[1] && info[2] && info[3] && info[4] && mapple_pa_in_hole(info[0]) &&
	    mapple_pa_in_hole((uint64_t)info[0] + info[1] - 1) && mapple_pa_in_hole(info[4]);
	if (!sane || ring == NULL || mt6781_scp_cfg(0) != 0) {
		printf("scp: not started (%s)\n", !sane ? "LK's region info missing or outside the memory holes" :
		    ring == NULL ? "no ring" : "already running");
		goto out;
	}
	if (steps >= 3) {
		/* the services before the hub: MobileGestalt caches "ambient-light-sensor" from whether one exists when it
		 * first looks; what they ask for waits for the hub */
		mapple_spu_start(steps);
	}
	(void)PE_parse_boot_argn("mp_scp_clk26m", &clk26m, sizeof(clk26m));
	if (mt6781_scp_clock() & (1u << 15)) {
		printf("scp: scp_sel powered down (CLK_CFG_0 %#x), left as it is\n", mt6781_scp_clock());
	}
	if ((mt6781_scp_clock() & 0x700u) != 0) {
		printf("scp: scp_sel not on clk26m%s\n", clk26m ? ", setting it" : " (mp_scp_clk26m=1 sets it)");
		if (clk26m && !mt6781_scp_clock_26m()) {
			printf("scp: scp_sel still %#x\n", mt6781_scp_clock());
		}
	}
	if (!mt6781_scp_run()) {
		goto out;
	}
	if (!wait_for(&ready, 30000)) {
		status("no READY in 30 s");
		goto out;
	}
	if (!wait_for(&hub_init, 10000)) {
		status("no INIT_DONE in 10 s");
		goto out;
	}
	hub_power_up();
	if (!hub_up) {
		goto out;
	}
	uint64_t up_at = counter_now(), quiet_since = up_at;
	uint32_t said_records = 0, quiet_records = 0, unmasks = 0;
	for (uint64_t tick = up_at;;) {
		while (pump()) {
		}
		apply_wanted();
		if (notifies == 0 && ring_word(4) != ring_word(0)) {
			drain(ring_word(4));
		}
		/* sensors on, no record for 5 s: unmasked again (some boots the hub holds its data despite the first) */
		if (records != quiet_records) {
			quiet_records = records;
			quiet_since = counter_now();
		} else if (unmask && counter_now() - quiet_since > 5 * hz && any_wanted()) {
			quiet_since = counter_now();
			if (unmasks++ < 10) {
				printf("scp: no records for 5 s, non-wakeup interrupt unmasked again: %d\n", unmask_nonwakeup());
			}
		}
		uint32_t secs = (uint32_t)((counter_now() - up_at) / hz);
		if (counter_now() - tick > hz && (secs < 30 || (secs % 10 == 0 && records != said_records) || secs % 60 == 0)) {
			char types[96] = "";
			size_t at = 0;
			for (unsigned t = 0; t <= SH_DEVICE_ORIENTATION && at < sizeof(types) - 12; t++) {
				if (by_type[t]) {
					at += (size_t)snprintf(types + at, sizeof(types) - at, " %u:%u", t, by_type[t]);
				}
			}
			said_records = records;
			printf("scp: %u s: ring rp %u wp %u size %u, %u notifications (last wp %u, event %u), %u records (by type%s), "
			    "%u bad wp, %u overruns; to host %#x, PC %#x\n", secs, ring_word(0), ring_word(4), ring_word(8), notifies,
			    last_wp, last_event, records, types, bad_wp, overruns, mt6781_scp_cfg(0x1c), mt6781_scp_cfg(0xb4));
			if (secs == 3 || secs == 6) {
				get_data(SH_ACCEL);
			}
		}
		if (counter_now() - last_stamp > 10 * hz) {
			stamp();
		}
		if (counter_now() - tick > hz) {
			tick = counter_now();
			mapple_spu_tick();
		}
		pause_ms(2);
	}
out:
	thread_terminate(current_thread());
}

/* At boot (MTKPwrap::start): the ring while memory below 2 GiB is still in one piece, then the thread. */
void
mtk_scp_start(void)
{
	thread_t t;

	(void)PE_parse_boot_argn("mp_sensor_steps", &steps, sizeof(steps));
	printf("scp: mp_sensor_steps=%u\n", steps);
	if (steps == 0 || !mt6781_scp_map()) {
		return;
	}
	hz = __builtin_arm_rsr64("CNTFRQ_EL0");
	mach_minus_counter = (int64_t)(mach_absolute_time() - counter_now());
	if (steps >= 2) {
		ring = mapple_spu_ring(&ring_pa);
		printf("scp: ring %s %#llx\n", ring ? "at" : "NOT TAKEN", ring_pa);
	}
	if (kernel_thread_start(worker, NULL, &t) == KERN_SUCCESS) {
		thread_deallocate(t);
	}
}
#endif /* ARM64_BOARD_CONFIG_MT6781 */
