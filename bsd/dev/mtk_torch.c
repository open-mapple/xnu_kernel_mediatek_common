/*
 * The rear flash LED as a torch: FLED1 of the MT6371 PMU on I2C5 at 0x34. The same chip charges the battery and
 * drives the display backlight, so of it only these are ever written, read-modify-write and read back:
 * FLED1TORCTRL (0x75) [4:0] to Android's Quick Settings level, and FLEDEN (0x7E) CS1 (bit 1) and TORCH_EN (bit 3).
 * STROBE_EN (0x7E bit 2) is never set; found set, the torch is not lit. Facts from realme's mt6370_pmu_fled.c and
 * flashlights-mt6370.c, and Android's sysfs with its torch on (torch_brightness 2).
 *
 * Boot-arg mp_torch_steps (default 0): 0 status, read only (the chip's ID, 0x75, 0x7A, 0x7E, FLEDSTAT1/2, BASESTAT);
 * 1 also the USB link's "torch on|off|status": on at 50 mA, then once a second the status registers (logged when they
 * change) and the battery's temperature; off after 30 s, at 45 C or when the temperature cannot be read.
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

extern kern_return_t thread_terminate(thread_t thread);
extern int mapple_i2c_xfer_bus(uint32_t bus, uint8_t addr, const uint8_t *wbuf, uint32_t wlen, uint8_t *rbuf, uint32_t rlen);
extern int mapple_battery_temp_decic(int32_t *out);                                     /* bsd/dev/mt6358_gauge.c */
void mtk_torch_start(void);                                                             /* bsd/dev/mtk_audio.c */
void mtk_torch_link(const char *args, char *out, size_t outLen);                        /* bsd/dev/usb_link.c */

#define TORCH_BUS       5
#define TORCH_ADDR      0x34
#define REG_ID          0x00            /* [7:4]: 0xF0 on this phone (Android: "chip_id_check: E4(0xF0)") */
#define REG_TORCH_CUR   0x75            /* [4:0]: 25 + 12.5 x n mA */
#define REG_VMIDTRK     0x7a
#define REG_FLEDEN      0x7e
#define REG_FLEDSTAT1   0xd9
#define REG_FLEDSTAT2   0xda
#define REG_BASESTAT    0xdb
#define EN_CS1          0x02
#define EN_STROBE       0x04
#define EN_TORCH        0x08
#define TORCH_LEVEL     2               /* 50 mA: Android's Quick Settings torch */
#define TORCH_SECS      30
#define TORCH_MAX_DECIC 450             /* 45.0 C */

static LCK_GRP_DECLARE(torch_grp, "mp_torch");
static LCK_MTX_DECLARE(torch_lock, &torch_grp);
static uint32_t steps;
static bool chip_ok, lit;
static uint64_t lit_at;

static bool
rd(uint8_t reg, uint8_t *value)
{
	return mapple_i2c_xfer_bus(TORCH_BUS, TORCH_ADDR, &reg, 1, value, 1) == 0;
}

/* The only write: 0x75 [4:0] = TORCH_LEVEL, or 0x7E's CS1/TORCH_EN; the other bits as read; read back. */
static bool
wr(uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t old = 0, now = 0, w[2];
	bool allowed = (reg == REG_TORCH_CUR && mask == 0x1f && value == TORCH_LEVEL) ||
	    (reg == REG_FLEDEN && mask != 0 && !(mask & ~(EN_CS1 | EN_TORCH)) && !(value & ~mask));

	if (!allowed || !rd(reg, &old)) {
		printf("torch: %s write %#x mask %#x <- %#x\n", allowed ? "no read before" : "refused", reg, mask, value);
		return false;
	}
	w[0] = reg;
	w[1] = (uint8_t)((old & ~mask) | value);
	if (reg == REG_FLEDEN && (w[1] & EN_STROBE)) {
		printf("torch: 0x7e reads %#x with STROBE_EN: not written\n", old);
		return false;
	}
	if (mapple_i2c_xfer_bus(TORCH_BUS, TORCH_ADDR, w, 2, NULL, 0) != 0 || !rd(reg, &now) || now != w[1]) {
		printf("torch: %#x <- %#x FAILED (reads %#x)\n", reg, w[1], now);
		return false;
	}
	return true;
}

static int
status(char *out, size_t len)
{
	uint8_t id = 0, cur = 0, vmid = 0, en = 0, s1 = 0, s2 = 0, base = 0;
	bool ok = rd(REG_ID, &id) && rd(REG_TORCH_CUR, &cur) && rd(REG_VMIDTRK, &vmid) && rd(REG_FLEDEN, &en) &&
	    rd(REG_FLEDSTAT1, &s1) && rd(REG_FLEDSTAT2, &s2) && rd(REG_BASESTAT, &base);

	if (!ok) {
		return snprintf(out, len, "torch: the chip does not answer on I2C%u (the device tree's i2c5: mkadt --torch)\n",
		           TORCH_BUS);
	}
	chip_ok = (id & 0xf0) == 0xf0;
	return snprintf(out, len, "torch: id %#x (%s), 0x75 %#x (%u.%u mA), 0x7a %#x, 0x7e %#x, fledstat %#x %#x, "
	           "basestat %#x; %s\n", id, chip_ok ? "MT6371" : "not the chip Android saw", cur, (250 + 125 * (cur & 0x1f)) / 10,
	           (250 + 125 * (cur & 0x1f)) % 10, vmid, en, s1, s2, base, lit ? "lit" : "off");
}

static bool
off(const char *why)
{
	bool ok = wr(REG_FLEDEN, EN_CS1 | EN_TORCH, 0);

	lit = !ok && lit;
	printf("torch: off (%s)%s\n", why, ok ? "" : ": FAILED");
	return ok;
}

/* While lit: once a second, the battery's temperature and the status registers; off at the limits. */
static void
watch(__unused void *arg, __unused wait_result_t w)
{
	uint8_t last[3] = { 0xff, 0xff, 0xff };

	for (;;) {
		uint8_t s[3] = { 0, 0, 0 };
		int32_t t = 0;
		uint64_t ns;

		delay_for_interval(1, NSEC_PER_SEC);
		lck_mtx_lock(&torch_lock);
		if (!lit) {
			lck_mtx_unlock(&torch_lock);
			break;
		}
		if (rd(REG_FLEDSTAT1, &s[0]) && rd(REG_FLEDSTAT2, &s[1]) && rd(REG_BASESTAT, &s[2]) &&
		    (s[0] != last[0] || s[1] != last[1] || s[2] != last[2])) {
			printf("torch: lit: fledstat %#x %#x, basestat %#x\n", s[0], s[1], s[2]);
			last[0] = s[0];
			last[1] = s[1];
			last[2] = s[2];
		}
		absolutetime_to_nanoseconds(mach_absolute_time() - lit_at, &ns);
		if (mapple_battery_temp_decic(&t) != 0) {
			(void)off("the battery's temperature cannot be read");
		} else if (t >= TORCH_MAX_DECIC) {
			printf("torch: battery %d.%d C\n", t / 10, t % 10);
			(void)off("the battery is 45 C or more");
		} else if (ns >= (uint64_t)TORCH_SECS * NSEC_PER_SEC) {
			(void)off("30 s");
		}
		lck_mtx_unlock(&torch_lock);
	}
	thread_terminate(current_thread());
}

static void
on(char *out, size_t len)
{
	uint8_t en = 0;
	int32_t t = 0;
	thread_t th;

	if (lit) {
		snprintf(out, len, "torch: already lit\n");
	} else if (!chip_ok) {
		snprintf(out, len, "torch: the chip's ID was not 0xF_ (torch status first): not lit\n");
	} else if (mapple_battery_temp_decic(&t) != 0 || t >= TORCH_MAX_DECIC) {
		snprintf(out, len, "torch: the battery's temperature is unknown or 45 C or more: not lit\n");
	} else if (!rd(REG_FLEDEN, &en) || (en & EN_STROBE)) {
		snprintf(out, len, "torch: 0x7e unread or STROBE_EN set (%#x): not lit\n", en);
	} else if (wr(REG_TORCH_CUR, 0x1f, TORCH_LEVEL) && wr(REG_FLEDEN, EN_CS1, EN_CS1) &&
	    wr(REG_FLEDEN, EN_TORCH, EN_TORCH)) {
		delay_for_interval(500, NSEC_PER_USEC);
		lit = true;
		lit_at = mach_absolute_time();
		if (kernel_thread_start(watch, NULL, &th) == KERN_SUCCESS) {
			thread_deallocate(th);
		} else {
			(void)off("no watch thread");
		}
		snprintf(out, len, "torch: %s at 50 mA, battery %d.%d C; off after %u s\n", lit ? "lit" : "NOT lit",
		    t / 10, t % 10, TORCH_SECS);
	} else {
		(void)off("a write failed");
		snprintf(out, len, "torch: a write failed: off\n");
	}
}

void
mtk_torch_link(const char *args, char *out, size_t outLen)
{
	while (*args == ' ') {
		args++;
	}
	lck_mtx_lock(&torch_lock);
	if (steps < 1 && strncmp(args, "status", 6) != 0) {
		snprintf(out, outLen, "torch: mp_torch_steps=%u: only status\n", steps);
	} else if (strncmp(args, "on", 2) == 0) {
		on(out, outLen);
	} else if (strncmp(args, "off", 3) == 0) {
		snprintf(out, outLen, "torch: %s\n", off("asked") ? "off" : "off FAILED");
	} else {
		status(out, outLen);
	}
	lck_mtx_unlock(&torch_lock);
}

/* Step 0 at boot, once I2C5's driver is there (it registers after this is called). */
static void
boot_status(__unused void *arg, __unused wait_result_t w)
{
	char line[200];
	uint8_t id;

	for (int i = 0; i < 40 && !rd(REG_ID, &id); i++) {
		delay_for_interval(250, NSEC_PER_MSEC);
	}
	lck_mtx_lock(&torch_lock);
	status(line, sizeof(line));
	lck_mtx_unlock(&torch_lock);
	printf("%s", line);
	thread_terminate(current_thread());
}

void
mtk_torch_start(void)
{
	thread_t t;

	(void)PE_parse_boot_argn("mp_torch_steps", &steps, sizeof(steps));
	steps = steps > 1 ? 1 : steps;
	printf("torch: mp_torch_steps=%u\n", steps);
	if (kernel_thread_start(boot_status, NULL, &t) == KERN_SUCCESS) {
		thread_deallocate(t);
	}
}
#endif /* ARM64_BOARD_CONFIG_MT6781 */
