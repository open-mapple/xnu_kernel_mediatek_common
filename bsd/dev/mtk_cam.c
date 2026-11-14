/*
 * The realme 8i's cameras, behind the boot-arg mp_cam_steps: steps 0 to N-1 run (default 0, nothing; 0 on a safe
 * boot, whose line has mp_bootrec=0). Facts from realme's kernel and device tree, and Android's log of this phone.
 *
 * 0  read only, at boot and on the USB link's "cam status": the camera pins (mode, direction, driven and read
 *    level); the LDO chip's registers 0x00-0x0F on I2C2 (it may not answer while its enable, GPIO122, is low); the
 *    S5K JN1's chip ID (I2C4 0x2d, register 0x0000) only when the LDO chip says its rails are on; the SoC side
 *    (osfmk/arm64/mt6781_cam.c: power domains, clock muxes, and CAMSYS/SENINF/CAMSV only when powered and clocked).
 */
#if defined(ARM64_BOARD_CONFIG_MT6781)
#include <stdbool.h>
#include <stdint.h>
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include "mtk_cam_ldo.h"

extern kern_return_t thread_terminate(thread_t thread);
extern int mapple_i2c_xfer_bus(uint32_t bus, uint8_t addr, const uint8_t *wbuf, uint32_t wlen, uint8_t *rbuf, uint32_t rlen);
extern int mapple_gpio_get(uint32_t pin, int *high);                                    /* iokit/Kernel/arm/MTKIO.cpp */
extern void mt6781_gpio_pin(uint32_t pin, uint32_t *mode, bool *output, bool *high);   /* osfmk/arm64/mt6781_afe.c */
extern size_t mt6781_cam_status(char *out, size_t len);                                 /* osfmk/arm64/mt6781_cam.c */
void mtk_cam_start(void);                                                               /* MTKPwrap::start */
void mtk_cam_link(const char *args, char *out, size_t outLen);                          /* bsd/dev/usb_link.c */

#define JN1_BUS         4
#define JN1_ADDR        0x2d
#define JN1_CHIP_ID     0x38e1

static LCK_GRP_DECLARE(cam_grp, "mp_cam");
static LCK_MTX_DECLARE(cam_lock, &cam_grp);
static uint32_t steps;
static char boot_line[2048];

/* RST of the four sensors, the LDO chip's enable, their MCLK pins (CMMCLKn: mode 1, GPIO60 mode 3), I2C2 and I2C4 */
static const struct {
	uint16_t pin;
	const char *name;
} pins[] = {
	{ 20, "rst-main" }, { 18, "rst-front" }, { 22, "rst-macro" }, { 26, "rst-depth" }, { 122, "ldo-en" },
	{ 141, "mclk-main" }, { 140, "mclk-front" }, { 142, "mclk-macro" }, { 60, "mclk-depth" },
	{ 148, "scl2" }, { 149, "sda2" }, { 152, "scl4" }, { 153, "sda4" },
};

#define say(...) do { if (o < len) o += (size_t)snprintf(out + o, len - o, __VA_ARGS__); } while (0)

/* Step 0. Under cam_lock. */
static size_t
status(char *out, size_t len)
{
	uint8_t ldo[CAM_LDO_REGS] = { 0 }, reg, id[2], idreg[2] = { 0, 0 };
	size_t o = 0;
	uint32_t n;
	int err = 0;

	say("cam: step 0: pins (mode, out/in, driven, read):");
	for (n = 0; n < sizeof(pins) / sizeof(pins[0]); n++) {
		uint32_t mode = 0;
		bool output = false, high = false;
		int level = -1;

		mt6781_gpio_pin(pins[n].pin, &mode, &output, &high);
		(void)mapple_gpio_get(pins[n].pin, &level);
		say(" %u %s %u %s %d %d;", (uint32_t)pins[n].pin, pins[n].name, mode, output ? "out" : "in", high, level);
	}
	say("\n");
	for (n = 0; n < CAM_LDO_REGS; n++) {
		reg = (uint8_t)n;
		if ((err = mapple_i2c_xfer_bus(CAM_LDO_BUS, CAM_LDO_ADDR, &reg, 1, &ldo[n], 1)) != 0) {
			break;
		}
	}
	if (n < CAM_LDO_REGS) {
		say("cam: step 0: LDO chip (I2C%u 0x%02x): %s at register %#x\n", CAM_LDO_BUS, CAM_LDO_ADDR,
		    err == -1 ? "no bus (the device tree's i2c2)" : "no answer", n);
	} else {
		say("cam: step 0: LDO chip:");
		for (n = 0; n < CAM_LDO_REGS; n++) {
			say(" %02x", ldo[n]);
		}
		say("; enables %#x: LDO7 %u mV, LDO2 %u mV, LDO3 %u mV, LDO5 %u mV (JN1 rails %s)\n", ldo[CAM_LDO_EN],
		    cam_ldo_mv(7, ldo[CAM_LDO_OUT(7)]), cam_ldo_mv(2, ldo[CAM_LDO_OUT(2)]), cam_ldo_mv(3, ldo[CAM_LDO_OUT(3)]),
		    cam_ldo_mv(5, ldo[CAM_LDO_OUT(5)]), cam_ldo_jn1_on(ldo[CAM_LDO_EN]) ? "on" : "off");
	}
	if (n < CAM_LDO_REGS || !cam_ldo_jn1_on(ldo[CAM_LDO_EN])) {
		say("cam: step 0: JN1 chip ID not read: its rails are not known to be on\n");
	} else if ((err = mapple_i2c_xfer_bus(JN1_BUS, JN1_ADDR, idreg, 2, id, 2)) != 0) {
		say("cam: step 0: JN1 (I2C%u 0x%02x): %s\n", JN1_BUS, JN1_ADDR, err == -1 ? "no bus (the device tree's i2c4)" :
		    "no answer");
	} else {
		say("cam: step 0: JN1 chip ID %#x (%s)\n", (uint32_t)id[0] << 8 | id[1],
		    ((uint32_t)id[0] << 8 | id[1]) == JN1_CHIP_ID ? "S5K JN1" : "not the JN1");
	}
	if (o < len) {
		o += mt6781_cam_status(out + o, len - o);
	}
	return o;
}

void
mtk_cam_link(const char *args, char *out, size_t outLen)
{
	(void)args;
	if (steps < 1) {
		snprintf(out, outLen, "cam: mp_cam_steps=0: nothing read\n");
		return;
	}
	lck_mtx_lock(&cam_lock);
	status(out, outLen);
	lck_mtx_unlock(&cam_lock);
}

/* Step 0 at boot, once the GPIO and I2C2 drivers are there (a transfer of no bytes is refused without bus traffic). */
static void
boot_status(__unused void *arg, __unused wait_result_t w)
{
	int level;

	for (int i = 0; i < 40 && (mapple_gpio_get(122, &level) != 0 ||
	    mapple_i2c_xfer_bus(CAM_LDO_BUS, CAM_LDO_ADDR, NULL, 0, NULL, 0) == -1); i++) {
		delay_for_interval(250, NSEC_PER_MSEC);
	}
	lck_mtx_lock(&cam_lock);
	status(boot_line, sizeof(boot_line));
	printf("%s", boot_line);
	lck_mtx_unlock(&cam_lock);
	thread_terminate(current_thread());
}

void
mtk_cam_start(void)
{
	thread_t t;
	uint32_t record = 1;

	(void)PE_parse_boot_argn("mp_bootrec", &record, sizeof(record));
	(void)PE_parse_boot_argn("mp_cam_steps", &steps, sizeof(steps));
	if (record == 0) {
		steps = 0;
	}
	if (steps > 1) {
		printf("cam: mp_cam_steps=%u: only step 0 is there, taken as 1\n", steps);
		steps = 1;
	}
	printf("cam: mp_cam_steps=%u%s\n", steps, record == 0 ? " (safe boot)" : "");
	if (steps >= 1 && kernel_thread_start(boot_status, NULL, &t) == KERN_SUCCESS) {
		thread_deallocate(t);
	}
}
#endif /* ARM64_BOARD_CONFIG_MT6781 */
