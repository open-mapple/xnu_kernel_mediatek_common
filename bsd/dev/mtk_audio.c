/*
 * The realme 8i's speaker and vibration motor, behind the audio plug-in's /dev nodes (bsd/dev/mapple_audio.c).
 *
 * Speaker: AFE memif DL1 -> ADDA DL (osfmk/arm64/mt6781_afe.c) -> the MT6366 codec's line-out (left, from DAC R) ->
 * an analog amplifier on GPIO164: an SIA8109 (one-wire, mode 6) or an AW87319 (I2C3 0x58), told apart at run time.
 * Facts from the realme kernel (mt6358.c, mt6781-mt6366.c, sia81xx.c) and the AW87319 datasheet. Android's levels
 * are the ceiling, and the writes enforce it: MTKPwrap::writeAudio (codec, VIBR), mt6781_afe.c (clocks, AFE),
 * pa_write() here (the amplifier). The samples go through the speaker guard (bsd/dev/speaker_guard.c) last.
 *
 * Boot-arg mp_audio_steps (default 0), each step with the ones before it:
 *   0  status, read only: the codec's rails and IDs, the AFE's clocks and pins, the amplifier's pin, the motor's rail
 *   1  clocks, AFE, ADDA DL and the codec's supplies and digital path up; held mp_audio_secs, then all down
 *   2  the codec's line-out up and a 1 kHz tone through DL1, the amplifier off; held, then down
 *   3  the amplifier found and on, the tone through the guard; held, then down
 *   4  iOS plays: /dev/mapple-audio through the guard; the amplifier on while there is sound, off after 3 s without
 * mp_audio_below (default 160 = 20 dB, at most 90 dB) how far under Android's level the speaker stays (Darwin's volume
 * is under that); mp_audio_gain (default 0, at most 12 dB) the guard's gain; mp_audio_secs (default 5) for steps 1-3.
 *
 * Boot-arg mp_vib_steps (default 0): 0 status; 1 one 100 ms buzz at Android's VIBR voltage; 2 /dev/mapple-haptic runs
 * the motor while iOS's haptic waveform is loud enough: at least 30 ms (Android's vib_timer), at most 1 s at a time,
 * then 1 s off. Refused while VIBR_VOSEL is not Android's (a voltage write: its first run is a run of its own).
 */
#if defined(ARM64_BOARD_CONFIG_MT6781)
#include <stdbool.h>
#include <stdint.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_time.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <sys/mapple_audio.h>
#include "mtk_audio_level.h"

extern kern_return_t thread_terminate(thread_t thread);
void mtk_audio_start(void);                                                            /* MTKPwrap::start */
extern int mtk_pwrap_read(uint32_t addr, uint32_t *value);                             /* MTKPwrap.cpp */
extern int mtk_pwrap_write_audio(uint32_t addr, uint32_t mask, uint32_t value);
extern int mapple_gpio_set(uint32_t pin, int high);                                    /* MTKIO.cpp */
extern int mapple_i2c_xfer_bus(uint32_t bus, uint8_t addr, const uint8_t *wbuf, uint32_t wlen, uint8_t *rbuf, uint32_t rlen);
extern void mt6781_afe_status(void);                                                   /* osfmk/arm64/mt6781_afe.c */
extern void mt6781_gpio_pin(uint32_t pin, uint32_t *mode, bool *output, bool *high);
extern bool mt6781_afe_up(void);
extern bool mt6781_afe_dl1_start(volatile uint32_t **ring, uint32_t *bytes);
extern uint32_t mt6781_afe_dl1_position(void);
extern void mt6781_afe_down(void);

#define PA_PIN          164
#define PA_BUS          3
#define AW87319         0x58
#define SIA8109         0x28
#define SIA_MODE        6               /* the phone's owi_mode: the only one sent */
#define VIBR_VOSEL      0xff            /* Android's VIBR_ANA_CON0 [11:8] (AUDIO.md, the VIBR dump), as in MTKPwrap.cpp's
                                         * whitelist; 0xff: not known yet, the motor stays off */

static uint32_t steps, vib_steps, below_base = 160, gain, secs = 5;

static void
ms(uint32_t n)
{
	delay_for_interval(n, NSEC_PER_MSEC);
}

static uint32_t
pmic(uint32_t addr)
{
	uint32_t v = 0;

	(void)mtk_pwrap_read(addr, &v);
	return v;
}

/* A codec write; after the first failure the rest of a bring-up is skipped (failed stays set until checked). */
static bool failed;

static void
cw(uint32_t addr, uint32_t mask, uint32_t value)
{
	if (!failed && mtk_pwrap_write_audio(addr, mask, value) != 0) {
		failed = true;
		printf("audio: codec write %#06x mask %#x <- %#x FAILED: the rest is not done\n", addr, mask, value);
	}
}

/* A write of a power-down: always tried. */
static void
cwd(uint32_t addr, uint32_t mask, uint32_t value)
{
	(void)mtk_pwrap_write_audio(addr, mask, value);
}

/* ---- status ---- */

static void
status(void)
{
	uint32_t mode;
	bool out, high;

	printf("audio: pmic hwcid %04x swcid %04x; VAUD28 %s, VA12 %s; codec ids top %04x dig %04x enc %04x dec %04x zcd %04x\n",
	    pmic(0x0008), pmic(0x000a), pmic(0x1ad2) & 0x8000 ? "on" : "OFF", pmic(0x1aaa) & 0x8000 ? "on" : "OFF", pmic(0x2200),
	    pmic(0x2280), pmic(0x2380), pmic(0x2400), pmic(0x2480));
	printf("audio: codec ckpdn %04x top %04x afe %04x lo %04x dac %04x zcd gain %04x dcxo_cw14 %04x\n", pmic(0x220c),
	    pmic(0x2292), pmic(0x2288), pmic(0x2416), pmic(0x2408), pmic(0x248a), pmic(0x07ac));
	printf("audio: vibr en %04x op %04x status %04x vosel %u\n", pmic(0x1d08), pmic(0x1d0a), pmic(0x1d16), (pmic(0x1e44) >> 8) & 0xf);
	mt6781_gpio_pin(PA_PIN, &mode, &out, &high);
	printf("audio: amplifier pin %u: mode %u (0 = GPIO), %s, %s\n", PA_PIN, mode, out ? "output" : "input", high ? "high" : "low");
	mt6781_afe_status();
}

/* ---- the codec (mt6358.c's order) ---- */

static uint32_t dcxo_audio;     /* DCXO_CW14 bit 13 as found: put back at the end (the clock buffer driver's register) */

static void
lol_gain(uint32_t code)
{
	cw(0x248a, 0x001f, code);
	cw(0x248a, 0x0f80, code << 7);
}

/* Supplies, clocks and the digital path from the AFE (DAPM supplies, AIF_RX). */
static bool
codec_up(void)
{
	if (!(pmic(0x1ad2) & 0x8000) || !(pmic(0x1aaa) & 0x8000)) {
		printf("audio: codec: VAUD28 or VA12 is off: not started\n");
		return false;
	}
	failed = false;
	dcxo_audio = pmic(0x07ac) & 0x2000;
	cw(0x07ac, 0x2000, 0x2000);
	cw(0x2416, 0x0100, 0x0100);
	cw(0x2422, 0x0010, 0);
	cw(0x2394, 0x0002, 0);
	cw(0x2394, 0x0001, 0x0001);
	cw(0x220c, 0x0064, 0);
	cw(0x220c, 0x0002, 0);
	delay_for_interval(250, NSEC_PER_USEC);
	cw(0x2292, 0x0044, 0);
	cw(0x2292, 0x0080, 0);
	delay_for_interval(250, NSEC_PER_USEC);
	cw(0x2288, 0x0001, 0x0001);
	cw(0x00d8, 0x0fff, 0x0249);
	cw(0x0030, 0x0ff0, 0x0ff0);
	cw(0x229a, 0x000f, 0x0006);
	cw(0x2296, 0xffff, 0xcba1);
	cw(0x229a, 0x000f, 0x0003);
	cw(0x229a, 0x000f, 0x000b);
	cw(0x228a, 0x0001, 0x0001);
	printf("audio: codec: supplies and digital path %s\n", failed ? "FAILED" : "up");
	return !failed;
}

static void
codec_down(void)
{
	cwd(0x228a, 0x0001, 0);
	cwd(0x229a, 0x000f, 0);
	cwd(0x2296, 0xffff, 0xcba0);
	cwd(0x00d8, 0x01f8, 0);
	cwd(0x0088, 0x0f00, 0);
	cwd(0x0030, 0x0ff0, 0);
	cwd(0x2288, 0x0001, 0);
	delay_for_interval(250, NSEC_PER_USEC);
	cwd(0x2292, 0x0080, 0x0080);
	cwd(0x2292, 0x0044, 0x0044);
	delay_for_interval(250, NSEC_PER_USEC);
	cwd(0x220c, 0x0002, 0x0002);
	cwd(0x220c, 0x0064, 0x0064);
	cwd(0x2394, 0x0001, 0);
	cwd(0x2422, 0x0010, 0x0010);
	cwd(0x07ac, 0x2000, dcxo_audio);
	cwd(0x2416, 0x0100, 0);
	printf("audio: codec: down\n");
}

/* The line-out (mt_lo_event): charge pump, LDOs, zero-cross detector, at -10 dB, then to `code', DAC R into it. */
static bool
lineout_up(uint32_t code)
{
	failed = false;
	cw(0x240c, 0xffff, 0x4000);
	cw(0x223c, 0x0001, 0x0001);
	cw(0x223e, 0x003f, 0x002c);
	cw(0x223a, 0x0001, 0x0001);
	cw(0x2242, 0x0003, 0x0003);
	cw(0x2240, 0x0001, 0);
	delay_for_interval(250, NSEC_PER_USEC);
	cw(0x2424, 0x1055, 0x1055);
	cw(0x2426, 0x0001, 0x0001);
	delay_for_interval(100, NSEC_PER_USEC);
	cw(0x241e, 0x0007, 0);
	cw(0x2488, 0x07ff, 0x010b);
	cw(0x2416, 0x0010, 0x0010);
	lol_gain(LOL_QUIETEST);
	cw(0x2420, 0xffff, 0x0055);
	cw(0x241e, 0xff80, 0x4900);
	cw(0x2416, 0x0100, 0x0100);
	cw(0x241a, 0xff01, 0);
	cw(0x241c, 0xffff, 0);
	cw(0x2416, 0x0002, 0x0002);
	cw(0x2416, 0x0001, 0x0001);
	lol_gain(code);
	cw(0x2422, 0x0001, 0x0001);
	cw(0x2408, 0x0006, 0x0006);
	cw(0x241a, 0xff01, 0x0001);
	cw(0x2416, 0x000c, 0x0004);
	printf("audio: codec: line-out %s, gain code %u (%d dB)\n", failed ? "FAILED" : "up", code, 8 - (int)code);
	return !failed;
}

static void
lineout_down(void)
{
	cwd(0x2416, 0x000c, 0);
	cwd(0x2408, 0x0006, 0);
	cwd(0x2422, 0x0001, 0);
	cwd(0x248a, 0x001f, LOL_MUTE);
	cwd(0x248a, 0x0f80, LOL_MUTE << 7);
	cwd(0x2416, 0x0001, 0);
	cwd(0x2416, 0x0002, 0);
	cwd(0x241a, 0xff00, 0);
	cwd(0x241a, 0xff00, 0x0200);
	cwd(0x2420, 0x0100, 0x0100);
	cwd(0x241e, 0x0007, 0x0004);
	cwd(0x2488, 0x07ff, 0);
	cwd(0x2426, 0x0001, 0);
	cwd(0x2424, 0x1055, 0);
	cwd(0x2240, 0x0001, 0x0001);
	printf("audio: codec: line-out down\n");
}

/* ---- the amplifier ---- */

enum { PA_NONE, PA_AW87319, PA_SIA8109 };
static int pa_kind;
static bool pa_on;
static bool sia_i2c;            /* the SIA8109 answers on I2C (0x41 reads 0x09): Android's registers are written */

/* AW87319 registers and the values allowed (the datasheet's defaults, the gain from 18 dB up to its default 24 dB). */
static const struct {
	uint8_t reg, min, max;
} aw_regs[] = {
	{ 0x01, 0x00, 0x07 },   /* SYSCTRL: EN_SW, class D, boost */
	{ 0x02, 0x28, 0x28 },   /* BATSAFE */
	{ 0x03, 0x04, 0x05 },   /* boost: 7.75 or 8.0 V */
	{ 0x04, 0x04, 0x04 },   /* inductor current */
	{ 0x05, 0x09, 0x0d },   /* gain: 18 .. 24 dB */
	{ 0x06, 0x00, 0x03 },   /* AGC3 power limit: at most 0.8 W into 8 ohms */
	{ 0x07, 0x52, 0x52 }, { 0x08, 0x28, 0x28 }, { 0x09, 0x02, 0x02 },
};
#define AW_GAIN         0x09    /* 18 dB, the least: Android's own (odm aw87319_spk.bin) is not known yet */

/* SIA8109: Android's register values (sia8109_regs.c), the only ones written; 0x02 = 0xcc is its boost, 8.5 V. */
static const uint8_t sia_regs[14] = { 0x07, 0x80, 0xcc, 0x19, 0x26, 0x0a, 0x02, 0x28, 0x05, 0x8a, 0x00, 0x00, 0x00, 0x00 };

static bool
pa_read(uint8_t addr, uint8_t reg, uint8_t *value)
{
	return mapple_i2c_xfer_bus(PA_BUS, addr, &reg, 1, value, 1) == 0;
}

static bool
pa_write(uint8_t addr, uint8_t reg, uint8_t value)
{
	uint8_t w[2] = { reg, value }, now = 0;
	bool allowed = false;

	if (addr == AW87319) {
		for (unsigned i = 0; i < sizeof(aw_regs) / sizeof(aw_regs[0]); i++) {
			allowed |= aw_regs[i].reg == reg && value >= aw_regs[i].min && value <= aw_regs[i].max;
		}
	} else if (addr == SIA8109) {
		allowed = reg < sizeof(sia_regs) && value == sia_regs[reg];
	}
	if (!allowed) {
		printf("audio: amplifier: refused write %#x reg %#x <- %#x\n", addr, reg, value);
		return false;
	}
	if (mapple_i2c_xfer_bus(PA_BUS, addr, w, 2, NULL, 0) != 0 || !pa_read(addr, reg, &now) || now != value) {
		printf("audio: amplifier: %#x reg %#x <- %#x FAILED (reads %#x)\n", addr, reg, value, now);
		return false;
	}
	return true;
}

static bool
pin(int high)
{
	return mapple_gpio_set(PA_PIN, high) == 0;
}

/*
 * SIA8109 one-wire: the pin low 1 ms, then `mode' rising edges 1 us apart (sia81xx.c), the last one held high for as
 * long as it plays. Sent with interrupts off; a phase longer than 10 us (the chip's limit) and it is sent again.
 */
static bool
sia_mode(unsigned mode)
{
	uint64_t us, t, last, worst;

	nanoseconds_to_absolutetime(NSEC_PER_USEC, &us);
	for (int attempt = 0; attempt < 10; attempt++) {
		boolean_t was;

		if (!pin(0)) {
			return false;
		}
		ms(1);
		was = ml_set_interrupts_enabled(FALSE);
		worst = 0;
		last = mach_absolute_time();
		for (unsigned edge = 0; edge < 2 * mode - 1; edge++) {
			(void)pin(!(edge & 1));
			t = mach_absolute_time();
			worst = t - last > worst ? t - last : worst;
			last = t;
			if (edge < 2 * mode - 2) {
				while (mach_absolute_time() < last + us) {
				}
			}
		}
		ml_set_interrupts_enabled(was);
		if (worst <= 10 * us) {
			return true;
		}
		printf("audio: amplifier: one-wire phase of %llu ticks (over 10 us): sent again\n", worst);
	}
	(void)pin(0);
	return false;
}

/* Which amplifier: with its pin high, the AW87319 answers 0x9b at 0x58; an SIA8109 may answer 0x09 at 0x28 (0x41). */
static bool
pa_detect(void)
{
	uint32_t mode;
	bool out, high;
	uint8_t aw = 0, sia = 0;

	mt6781_gpio_pin(PA_PIN, &mode, &out, &high);
	if (mode != 0) {
		printf("audio: amplifier: pin %u is in mode %u, not GPIO: left alone\n", PA_PIN, mode);
		return false;
	}
	for (int i = 0; i < 40 && !pin(0); i++) {
		ms(250);
	}
	if (!pin(1)) {
		printf("audio: amplifier: pin %u cannot be driven (the device tree's output-pins)\n", PA_PIN);
		return false;
	}
	ms(2);
	{
		uint8_t reg = 0;
		int i;

		for (i = 0; i < 40 && mapple_i2c_xfer_bus(PA_BUS, AW87319, &reg, 1, &aw, 1) == -1; i++) {
			ms(250);
		}
		if (i == 40) {
			(void)pin(0);
			printf("audio: amplifier: no I2C3 driver (the device tree's i2c3): not started\n");
			return false;
		}
	}
	if (pa_read(AW87319, 0x00, &aw) && aw == 0x9b) {
		pa_kind = PA_AW87319;
	} else {
		pa_kind = PA_SIA8109;
		sia_i2c = pa_read(SIA8109, 0x41, &sia) && sia == 0x09;
	}
	(void)pin(0);
	ms(2);
	printf("audio: amplifier: %s (AW87319 id %#x, SIA8109 id %#x)\n", pa_kind == PA_AW87319 ? "AW87319" :
	    sia_i2c ? "SIA8109, on I2C too" : "taken for an SIA8109 on one-wire only", aw, sia);
	return true;
}

/* Off: whatever the I2C says, the pin goes low (shutdown for both); on stays true until the pin is known low. */
static void
pa_off(void)
{
	if (pa_kind == PA_AW87319) {
		(void)pa_write(AW87319, 0x01, 0x03);
	}
	if (pin(0)) {
		pa_on = false;
	} else {
		printf("audio: amplifier: its pin could NOT be set low\n");
	}
	ms(2);
	speaker_guard_attenuation(SPEAKER_GUARD_SILENT);
}

/* On, after the line-out (Android: the amplifier last, off first). */
static bool
pa_start(void)
{
	bool ok = true;

	if (pa_kind == PA_AW87319) {
		uint8_t id = 0;

		ok = pin(0);
		ms(2);
		ok = ok && pin(1);
		ms(2);
		ok = ok && pa_read(AW87319, 0x00, &id) && id == 0x9b && pa_write(AW87319, 0x01, 0x03) && pa_write(AW87319, 0x02, 0x28) &&
		    pa_write(AW87319, 0x03, 0x05) && pa_write(AW87319, 0x04, 0x04) && pa_write(AW87319, 0x05, AW_GAIN) &&
		    pa_write(AW87319, 0x06, 0x03) && pa_write(AW87319, 0x07, 0x52) && pa_write(AW87319, 0x08, 0x28) &&
		    pa_write(AW87319, 0x09, 0x02) && pa_write(AW87319, 0x01, 0x07);
		ms(40);
	} else if (pa_kind == PA_SIA8109) {
		ok = sia_mode(SIA_MODE);
		ms(45);
		for (uint8_t r = 0; ok && sia_i2c && r < sizeof(sia_regs); r++) {
			ok = pa_write(SIA8109, r, sia_regs[r]);
		}
	} else {
		ok = false;
	}
	pa_on = true;           /* (until pa_off() has its pin low) */
	if (!ok) {
		printf("audio: amplifier: did NOT start: off again\n");
		pa_off();
	}
	return ok;
}

/* ---- the feeder: frames into DL1's ring ahead of the hardware, through the volume and the guard ---- */

#define RING_FRAMES     8192
#define AHEAD           8192    /* bytes kept queued in DL1's ring: 43 ms */
#define CHUNK           512

static int16_t ring[2 * RING_FRAMES];
static uint32_t ring_head, ring_tail;   /* frames; head: the sink's, tail: the feeder's */
static volatile uint32_t *dl;
static uint32_t dl_bytes, dl_write, underruns;
static uint64_t last_sound;

/* /dev/mapple-audio's sink (the writer's thread, must not wait): into the ring, what fits. */
static uint32_t
sink(const int16_t *frames, uint32_t count)
{
	uint32_t head = ring_head, room = RING_FRAMES - (head - __atomic_load_n(&ring_tail, __ATOMIC_ACQUIRE));
	uint32_t n = count < room ? count : room;

	for (uint32_t i = 0; i < n; i++) {
		uint32_t at = (head + i) % RING_FRAMES;

		ring[2 * at] = frames[2 * i];
		ring[2 * at + 1] = frames[2 * i + 1];
	}
	__atomic_store_n(&ring_head, head + n, __ATOMIC_RELEASE);
	return n;
}

static const int16_t sine[48] = {       /* 1 kHz at 48 kHz, 6 dB under full scale */
	0, 2139, 4240, 6270, 8192, 9974, 11585, 12998, 14189, 15137, 15826, 16244, 16384, 16244, 15826, 15137,
	14189, 12998, 11585, 9974, 8192, 6270, 4240, 2139, 0, -2139, -4240, -6270, -8192, -9974, -11585, -12998,
	-14189, -15137, -15826, -16244, -16384, -16244, -15826, -15137, -14189, -12998, -11585, -9974, -8192, -6270, -4240, -2139,
};
static uint32_t sine_at;

/* `n' frames from the tone or from iOS (silence where there is none); true if any is not silence */
static bool
take(int16_t *frames, uint32_t n, bool tone)
{
	bool sound = false;

	if (tone) {
		for (uint32_t i = 0; i < n; i++, sine_at = (sine_at + 1) % 48) {
			frames[2 * i] = frames[2 * i + 1] = sine[sine_at];
		}
		return true;
	}
	uint32_t tail = ring_tail, have = __atomic_load_n(&ring_head, __ATOMIC_ACQUIRE) - tail, got = have < n ? have : n;

	for (uint32_t i = 0; i < got; i++) {
		uint32_t at = (tail + i) % RING_FRAMES;

		frames[2 * i] = ring[2 * at];
		frames[2 * i + 1] = ring[2 * at + 1];
		sound |= frames[2 * i] != 0 || frames[2 * i + 1] != 0;
	}
	__atomic_store_n(&ring_tail, tail + got, __ATOMIC_RELEASE);
	for (uint32_t i = 2 * got; i < 2 * n; i++) {
		frames[i] = 0;
	}
	return sound;
}

static uint32_t lol_now = LOL_QUIETEST;

/* One round: DL1's ring topped up to AHEAD bytes; the volume set where it changed. */
static void
feed(bool tone)
{
	static int16_t frames[2 * CHUNK];
	uint32_t pos = mt6781_afe_dl1_position(), queued = (dl_write - pos + dl_bytes) % dl_bytes, darwin = 0, gen;
	bool muted = false;
	uint64_t factor;
	int32_t digital;

	if (queued > AHEAD + 4096) {    /* the hardware has passed what was written */
		underruns++;
		dl_write = (pos + 1024) % dl_bytes;
		queued = 1024;
	}
	(void)mapple_audio_volume(&darwin, &muted, &gen);
	uint32_t below = below_base + (tone ? 0 : darwin), code = mtk_audio_lol_code(below);
	if (code != lol_now) {
		if (mtk_pwrap_write_audio(0x248a, 0x001f, code) == 0) {      /* LOL: the speaker's */
			lol_now = code;
		}
		(void)mtk_pwrap_write_audio(0x248a, 0x0f80, code << 7);
	}
	digital = (int32_t)below - (int32_t)(lol_now - LOL_ANDROID) * 8;   /* (the line-out as it is, if a change failed) */
	factor = speaker_guard_amplitude(digital > 0 ? (uint32_t)digital : 0);
	if (pa_on) {
		speaker_guard_attenuation((lol_now - LOL_ANDROID) * 8);
	}
	for (uint32_t left = (AHEAD - queued) / 4; left > 0;) {
		uint32_t n = left < CHUNK ? left : CHUNK;

		if (take(frames, n, tone) && !muted) {
			last_sound = mach_absolute_time();
		}
		for (uint32_t i = 0; i < 2 * n; i++) {
			frames[i] = muted ? 0 : (int16_t)((int64_t)frames[i] * (int64_t)factor >> 30);
		}
		speaker_guard_process(frames, n);
		for (uint32_t i = 0; i < n; i++) {
			dl[dl_write / 4] = (uint16_t)frames[2 * i] | (uint32_t)(uint16_t)frames[2 * i + 1] << 16;
			dl_write = (dl_write + 4) % dl_bytes;
		}
		left -= n;
	}
}

/* a Q30 amplitude gain in dB, "-3.5" */
static const char *
db(uint64_t g, char *text)
{
	int t = speaker_guard_decibels(g * g >> 30);

	snprintf(text, 8, "%s%d.%d", t < 0 ? "-" : "", (t < 0 ? -t : t) / 10, (t < 0 ? -t : t) % 10);
	return text;
}

static void
report(const char *when)
{
	struct speaker_guard_report r;
	char a[8], b[8];

	speaker_guard_take_report(&r);
	printf("audio: %s: DL1 at %u, %u underruns; guard: peak in %u out %u, gain %s dB (least %s), %u frames limited\n", when,
	    mt6781_afe_dl1_position(), underruns, r.peak_in, r.peak_out, db(r.gain, a), db(r.least_gain, b), r.limited_frames);
}

/* ---- the steps ---- */

static void
audio_thread(__unused void *arg, __unused wait_result_t w)
{
	bool afe = false, codec = false, lineout = false, dl1 = false;
	uint64_t second, end, retry = 0;
	unsigned pa_failures = 0;

	status();
	if (steps < 1) {
		goto out;
	}
	speaker_guard_configure(true, gain);
	afe = mt6781_afe_up();
	codec = afe && codec_up();
	if (codec && steps >= 2) {
		lineout = lineout_up(mtk_audio_lol_code(below_base));
		lol_now = mtk_audio_lol_code(below_base);
		dl1 = lineout && mt6781_afe_dl1_start(&dl, &dl_bytes);
		dl_write = 0;
	}
	if (dl1 && steps >= 3 && pa_detect()) {
		if (steps == 3) {
			(void)pa_start();
		}
	}
	status();
	nanoseconds_to_absolutetime(NSEC_PER_SEC, &second);
	if (steps == 4 && dl1 && pa_kind != PA_NONE) {
		/* iOS: for good. The amplifier on with the first sound, off after 3 s without. */
		mapple_audio_set_sink("the speaker", sink);
		for (;;) {
			feed(false);
			if (!pa_on && last_sound != 0 && mach_absolute_time() - last_sound < second && mach_absolute_time() >= retry &&
			    pa_failures < 3) {
				speaker_guard_attenuation((lol_now - LOL_ANDROID) * 8);
				if (pa_start()) {
					printf("audio: playing\n");
				} else if (++pa_failures < 3) {
					retry = mach_absolute_time() + 10 * second;
				} else {
					printf("audio: amplifier: three failed starts: the speaker stays off\n");
				}
			} else if (pa_on && mach_absolute_time() - last_sound > 3 * second) {
				pa_off();
				report("3 s of silence, the amplifier off");
			}
			ms(5);
		}
	}
	/* steps 1-3: held, then all down */
	end = mach_absolute_time() + secs * second;
	for (uint64_t next = mach_absolute_time() + second; mach_absolute_time() < end; ms(5)) {
		if (dl1) {
			if (pa_on) {
				speaker_guard_attenuation((lol_now - LOL_ANDROID) * 8);
			}
			feed(true);
		}
		if (mach_absolute_time() >= next) {
			next += second;
			if (dl1) {
				report(pa_on ? "the tone, the amplifier on" : "the tone, the amplifier off");
			}
		}
	}
	if (pa_on || pa_kind != PA_NONE) {
		pa_off();
	}
	if (lineout) {
		lineout_down();
	}
	if (codec) {
		codec_down();
	}
	if (afe) {
		mt6781_afe_down();
	}
	status();
out:
	printf("audio: mp_audio_steps=%u done\n", steps);
	thread_terminate(current_thread());
}

/* ---- the motor ---- */

static uint64_t vib_until;      /* mach time the motor may run to: /dev/mapple-haptic's sink sets it */

static void
vib_haptic(uint32_t peak, uint32_t frames)
{
	uint64_t now = mach_absolute_time(), length, least;

	if (peak < 2048) {      /* 24 dB under full scale: too quiet to be felt as more than the motor's start */
		return;
	}
	nanoseconds_to_absolutetime((uint64_t)frames * NSEC_PER_SEC / 48000, &length);
	nanoseconds_to_absolutetime(30 * NSEC_PER_MSEC, &least);
	length = length > least ? length : least;
	if (now + length > __atomic_load_n(&vib_until, __ATOMIC_RELAXED)) {
		__atomic_store_n(&vib_until, now + length, __ATOMIC_RELAXED);
	}
}

static bool
vib(bool on)
{
	return mtk_pwrap_write_audio(0x1d08, 0x0001, on ? 0x0001 : 0) == 0;
}

static void
vib_thread(__unused void *arg, __unused wait_result_t w)
{
	uint64_t second, since = 0, rest = 0;
	bool on = false;

	if (VIBR_VOSEL > 15) {
		printf("audio: vibr: mp_vib_steps=%u refused: Android's VIBR voltage is not known yet\n", vib_steps);
		thread_terminate(current_thread());
	}
	if (mtk_pwrap_write_audio(0x1e44, 0x0f00, VIBR_VOSEL << 8) != 0 || mtk_pwrap_write_audio(0x1d0a, 0x0001, 0x0001) != 0) {
		printf("audio: vibr: could not be set up\n");
		thread_terminate(current_thread());
	}
	if (vib_steps == 1) {
		printf("audio: vibr: one 100 ms buzz at vosel %u: %s\n", VIBR_VOSEL, vib(true) ? "on" : "FAILED");
		ms(100);
		vib(false);
		printf("audio: vibr: off; en %04x status %04x\n", pmic(0x1d08), pmic(0x1d16));
		thread_terminate(current_thread());
	}
	nanoseconds_to_absolutetime(NSEC_PER_SEC, &second);
	mapple_audio_set_haptic(vib_haptic);
	printf("audio: vibr: iOS's haptics run the motor\n");
	for (;; ms(5)) {
		uint64_t now = mach_absolute_time();
		bool want = now < __atomic_load_n(&vib_until, __ATOMIC_RELAXED) && now >= rest;

		if (on && now - since > second) {       /* at most 1 s at a time, then 1 s off */
			want = false;
			rest = now + second;
		}
		if (want != on) {
			on = vib(want) ? want : false;
			since = now;
		}
	}
}

void
mtk_audio_start(void)
{
	thread_t t;

	(void)PE_parse_boot_argn("mp_audio_steps", &steps, sizeof(steps));
	(void)PE_parse_boot_argn("mp_audio_below", &below_base, sizeof(below_base));
	(void)PE_parse_boot_argn("mp_audio_gain", &gain, sizeof(gain));
	(void)PE_parse_boot_argn("mp_audio_secs", &secs, sizeof(secs));
	(void)PE_parse_boot_argn("mp_vib_steps", &vib_steps, sizeof(vib_steps));
	steps = steps > 4 ? 4 : steps;
	below_base = below_base > 90 * 8 ? 90 * 8 : below_base;
	gain = gain > SPEAKER_GUARD_MAX_GAIN ? SPEAKER_GUARD_MAX_GAIN : gain;
	secs = secs < 1 ? 1 : secs > 60 ? 60 : secs;
	printf("audio: mp_audio_steps=%u (below %u.%u dB, gain %u.%u dB, %u s), mp_vib_steps=%u\n", steps, below_base / 8,
	    below_base % 8 * 10 / 8, gain / 8, gain % 8 * 10 / 8, secs, vib_steps);
	if (kernel_thread_start(audio_thread, NULL, &t) == KERN_SUCCESS) {
		thread_deallocate(t);
	}
	if (vib_steps >= 1 && kernel_thread_start(vib_thread, NULL, &t) == KERN_SUCCESS) {
		thread_deallocate(t);
	}
}
#endif /* ARM64_BOARD_CONFIG_MT6781 */
