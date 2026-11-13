/*
 * The rear flash LED as a torch: FLED1 of the MT6371 PMU on I2C5 at 0x34. The same chip charges the battery and
 * drives the display backlight, so of it only these are ever written, read-modify-write and read back:
 * FLED1TORCTRL (0x75) [4:0] to 0, 1 or 2 (25, 37.5, 50 mA: Android's Quick Settings torch is 2, the most), and
 * FLEDEN (0x7E) CS1 (bit 1) and TORCH_EN (bit 3). STROBE_EN (0x7E bit 2) is never set; found set, nothing is lit.
 * Facts from realme's mt6370_pmu_fled.c, mt6370_pmu_irq.c and flashlights-mt6370.c, and Android's sysfs.
 *
 * Boot-arg mp_torch_steps (default 0): 0 status, read only (the chip's ID, 0x75, 0x7A, 0x7E, FLEDSTAT1/2, BASESTAT);
 * 1 levels 1-3 from the USB link ("torch on [1-3]|off|status") and sysctl kern.mapple_torch (root; reads the level,
 * plus 0x100 while overheated). While lit, once a second: the status registers and the battery's temperature. Off
 * after 10 minutes, on a flash fault (low voltage, short), and, as overheated, at 45 C, with the chip's
 * over-temperature flag, or when the temperature cannot be read; overheated ends below 40 C with the flag clear.
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
#include <sys/errno.h>
#include <sys/sysctl.h>
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
#define REG_FLEDSTAT1   0xd9            /* bit 3 low voltage, 6 LED 2 short, 7 LED 1 short */
#define REG_FLEDSTAT2   0xda            /* bit 5 LED 1 in torch */
#define REG_BASESTAT    0xdb            /* bit 5 over-temperature */
#define FLED_FAULTS     0xc8
#define BASE_OTP        0x20
#define EN_CS1          0x02
#define EN_STROBE       0x04
#define EN_TORCH        0x08
#define TORCH_MAX_SEL   2               /* 50 mA: Android's Quick Settings torch */
#define TORCH_SECS      600
#define TORCH_MAX_DECIC 450             /* 45.0 C */
#define TORCH_COOL_DECIC 400
#define OVERHEATED      0x100

static LCK_GRP_DECLARE(torch_grp, "mp_torch");
static LCK_MTX_DECLARE(torch_lock, &torch_grp);
static uint32_t steps, level;           /* level 0 off, 1-3 = selector level - 1 */
static bool chip_ok, overheated, watching;
static uint64_t lit_at;

static bool
rd(uint8_t reg, uint8_t *value)
{
	return mapple_i2c_xfer_bus(TORCH_BUS, TORCH_ADDR, &reg, 1, value, 1) == 0;
}

/* The only write: 0x75 [4:0] = 0..2, or 0x7E's CS1/TORCH_EN; the other bits as read; read back. */
static bool
wr(uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t old = 0, now = 0, w[2];
	bool allowed = (reg == REG_TORCH_CUR && mask == 0x1f && value <= TORCH_MAX_SEL) ||
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
	           "basestat %#x; level %u%s\n", id, chip_ok ? "MT6371" : "not the chip Android saw", cur,
	           (250 + 125 * (cur & 0x1f)) / 10, (250 + 125 * (cur & 0x1f)) % 10, vmid, en, s1, s2, base, level,
	           overheated ? ", overheated" : "");
}

static void
off(const char *why)
{
	bool ok = wr(REG_FLEDEN, EN_CS1 | EN_TORCH, 0);

	if (ok) {
		level = 0;
	}
	printf("torch: off (%s)%s\n", why, ok ? "" : ": FAILED");
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
		bool regs, temp;

		delay_for_interval(1, NSEC_PER_SEC);
		lck_mtx_lock(&torch_lock);
		if (level == 0) {
			watching = false;
			lck_mtx_unlock(&torch_lock);
			break;
		}
		regs = rd(REG_FLEDSTAT1, &s[0]) && rd(REG_FLEDSTAT2, &s[1]) && rd(REG_BASESTAT, &s[2]);
		if (regs && (s[0] != last[0] || s[1] != last[1] || s[2] != last[2])) {
			printf("torch: lit (level %u): fledstat %#x %#x, basestat %#x\n", level, s[0], s[1], s[2]);
			memcpy(last, s, sizeof(last));
		}
		temp = mapple_battery_temp_decic(&t) == 0;
		absolutetime_to_nanoseconds(mach_absolute_time() - lit_at, &ns);
		if (!temp || t >= TORCH_MAX_DECIC || (regs && (s[2] & BASE_OTP))) {
			overheated = true;
			printf("torch: overheated: battery %s%d.%d C, basestat %#x\n", temp ? "" : "unknown, ", t / 10, t % 10, s[2]);
			off("overheated");
		} else if (regs && (s[0] & FLED_FAULTS)) {
			off("a flash fault");
		} else if (ns >= (uint64_t)TORCH_SECS * NSEC_PER_SEC) {
			off("10 minutes");
		}
		lck_mtx_unlock(&torch_lock);
	}
	thread_terminate(current_thread());
}

/* Overheated until the battery is under 40 C with the chip's flag clear (looked at when asked). */
static void
cool_check(void)
{
	uint8_t base = 0;
	int32_t t = 0;

	if (overheated && level == 0 && mapple_battery_temp_decic(&t) == 0 && t < TORCH_COOL_DECIC &&
	    rd(REG_BASESTAT, &base) && !(base & BASE_OTP)) {
		overheated = false;
		printf("torch: cooled down (battery %d.%d C)\n", t / 10, t % 10);
	}
}

/* Level 0 off, 1-3 = 25, 37.5, 50 mA. Under torch_lock. */
static int
torch_set(uint32_t want, const char *who)
{
	uint8_t en = 0;
	int32_t t = 0;
	thread_t th;

	if (steps < 1) {
		return EPERM;
	}
	if (want == 0) {
		if (level != 0) {
			off(who);
		}
		return level == 0 ? 0 : EIO;
	}
	cool_check();
	if (want > TORCH_MAX_SEL + 1) {
		return EINVAL;
	}
	if (!chip_ok || overheated) {
		return EBUSY;
	}
	if (mapple_battery_temp_decic(&t) != 0 || t >= TORCH_MAX_DECIC) {
		overheated = true;
		return EBUSY;
	}
	if (!rd(REG_FLEDEN, &en) || (en & EN_STROBE)) {
		printf("torch: 0x7e unread or STROBE_EN set (%#x): not lit\n", en);
		return EIO;
	}
	if (!wr(REG_TORCH_CUR, 0x1f, (uint8_t)(want - 1))) {
		off("a write failed");
		return EIO;
	}
	if (level == 0) {
		if (!wr(REG_FLEDEN, EN_CS1, EN_CS1) || !wr(REG_FLEDEN, EN_TORCH, EN_TORCH)) {
			off("a write failed");
			return EIO;
		}
		delay_for_interval(500, NSEC_PER_USEC);
		lit_at = mach_absolute_time();
	}
	level = want;
	printf("torch: level %u (%u.%u mA) from %s, battery %d.%d C\n", level, (250 + 125 * (level - 1)) / 10,
	    (250 + 125 * (level - 1)) % 10, who, t / 10, t % 10);
	if (!watching) {
		watching = kernel_thread_start(watch, NULL, &th) == KERN_SUCCESS;
		if (watching) {
			thread_deallocate(th);
		} else {
			off("no watch thread");
			return EIO;
		}
	}
	return 0;
}

static int
sysctl_torch SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int value, error, changed = 0;

	lck_mtx_lock(&torch_lock);
	cool_check();
	value = (int)level | (overheated ? OVERHEATED : 0);
	lck_mtx_unlock(&torch_lock);
	error = sysctl_io_number(req, value, sizeof(value), &value, &changed);
	if (error || !changed) {
		return error;
	}
	if (value < 0) {
		return EINVAL;
	}
	lck_mtx_lock(&torch_lock);
	error = torch_set((uint32_t)value, "sysctl");
	lck_mtx_unlock(&torch_lock);
	return error;
}

SYSCTL_PROC(_kern, OID_AUTO, mapple_torch, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, NULL, 0, sysctl_torch, "I",
    "the torch: 0 off, 1-3 = 25, 37.5, 50 mA; reads 0x100 while overheated");

void
mtk_torch_link(const char *args, char *out, size_t outLen)
{
	int error;

	while (*args == ' ') {
		args++;
	}
	lck_mtx_lock(&torch_lock);
	if (strncmp(args, "on", 2) == 0) {
		uint32_t want = args[2] == ' ' && args[3] >= '1' && args[3] <= '3' ? (uint32_t)(args[3] - '0') : TORCH_MAX_SEL + 1;

		error = torch_set(want, "the link");
		snprintf(out, outLen, "torch: level %u: %s\n", want, error == 0 ? "lit" : error == EBUSY ? "overheated or no chip" :
		    error == EPERM ? "mp_torch_steps=0" : "FAILED");
	} else if (strncmp(args, "off", 3) == 0) {
		error = torch_set(0, "asked");
		snprintf(out, outLen, "torch: %s\n", error == 0 ? "off" : error == EPERM ? "mp_torch_steps=0" : "off FAILED");
	} else {
		cool_check();
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
