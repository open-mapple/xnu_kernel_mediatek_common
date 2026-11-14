/*
 * The MT6366's power off as realme's kernel for this phone has it (facts only):
 *   mt6358-misc.c's shutdown hook (the EOSC calibration clock on; with the DT's dcxo-switch, the DCXO to RTC mode),
 *   then pm_power_off: mt6358-core.c's PWRHOLD = 0. (mt6358-misc.c's mt_power_off is used only with the DT's
 *   apply-lpsd-solution, which the MT6366's node, mt6366.dtsi, does not have.)
 * Registers from include/linux/mfd/mt6358/registers.h.
 * MTKPwrap.cpp supplies the PMIC access; tools/mt6358_poweroff_test.c checks the order on the host.
 */
#ifndef _IOKIT_ARM_MT6358POWEROFF_H
#define _IOKIT_ARM_MT6358POWEROFF_H

#include <stddef.h>
#include <stdint.h>

#define MT6358_SCK_TOP_CKPDN_CON0_CLR 0x051e
#define MT6358_DCXO_CW02        0x0790
#define MT6358_PPCCTL0          0x0a08

/*
 * Every PMIC write of the power off, in order; nothing else is written. rmw: as Android's regmap_update_bits (read,
 * change the mask's bits, no write when nothing changes), else a plain write. check: read back.
 */
struct mt6358_poweroff_reg {
	uint16_t addr, mask, value;
	uint8_t rmw, check;
};

static const struct mt6358_poweroff_reg mt6358_poweroff_regs[] = {
	{ 0x051e, 0x0004, 0x0004, 1, 0 },       /* SCK_TOP_CKPDN_CON0_CLR: RG_RTC_EOSC32_CK_PDN */
	{ 0x0790, 0x0001, 0x0000, 1, 1 },       /* DCXO_CW02: XO_EN32K_MAN */
	{ 0x0a08, 0x0001, 0x0000, 1, 0 },       /* PPCCTL0: RG_PWRHOLD */
};

struct mt6358_poweroff_ops {
	int (*read)(uint32_t addr, uint32_t *value);    /* 0: done */
	int (*write)(uint32_t addr, uint32_t value);
	void (*delay_us)(uint32_t us);
};

static inline int
mt6358_poweroff_write(const struct mt6358_poweroff_ops *ops, uint32_t addr, uint32_t mask, uint32_t value)
{
	const struct mt6358_poweroff_reg *r = NULL;
	uint32_t old = 0, now = 0;
	unsigned int i;

	for (i = 0; i < sizeof(mt6358_poweroff_regs) / sizeof(mt6358_poweroff_regs[0]); i++) {
		if (mt6358_poweroff_regs[i].addr == addr && mt6358_poweroff_regs[i].mask == mask &&
		    mt6358_poweroff_regs[i].value == value) {
			r = &mt6358_poweroff_regs[i];
		}
	}
	if (r == NULL) {
		return -1;
	}
	if (r->rmw) {
		if (ops->read(addr, &old) != 0) {
			return -1;
		}
		if (((old & ~mask) | value) == old) {
			return 0;
		}
		value = (old & ~mask) | value;
	}
	if (ops->write(addr, value) != 0) {
		return -1;
	}
	if (r->check && (ops->read(addr, &now) != 0 || (now & mask) != r->value)) {
		return -1;
	}
	return 0;
}

/* The shutdown hook's writes, before PWRHOLD. NULL when done, else the step that failed. */
static inline const char *
mt6358_poweroff_prepare(const struct mt6358_poweroff_ops *ops)
{
	if (mt6358_poweroff_write(ops, MT6358_SCK_TOP_CKPDN_CON0_CLR, 0x0004, 0x0004) != 0) {
		return "SCK_TOP_CKPDN_CON0_CLR";
	}
	if (mt6358_poweroff_write(ops, MT6358_DCXO_CW02, 0x0001, 0) != 0) {
		return "DCXO_CW02";
	}
	return NULL;
}

/* PWRHOLD = 0: the PMIC cuts the power. */
static inline int
mt6358_poweroff_cut(const struct mt6358_poweroff_ops *ops)
{
	return mt6358_poweroff_write(ops, MT6358_PPCCTL0, 0x0001, 0);
}

#endif /* _IOKIT_ARM_MT6358POWEROFF_H */
