/*
 * iokit/Kernel/arm/MT6358PowerOff.h against realme's mt6358-misc.c / mt6358-core.c order, on a fake PMIC.
 *   cc -Wall tools/mt6358_poweroff_test.c -o poweroff_test && ./poweroff_test
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../iokit/Kernel/arm/MT6358PowerOff.h"

static uint16_t regs[0x8000];
static uint32_t log_addr[64], log_val[64], nlog;
static int fail_addr, ignore_addr;

static int
fake_read(uint32_t addr, uint32_t *v)
{
	*v = regs[addr / 2];
	return 0;
}

static int
fake_write(uint32_t addr, uint32_t v)
{
	if ((int)addr == fail_addr) {
		return -1;
	}
	log_addr[nlog] = addr;
	log_val[nlog++] = v;
	if ((int)addr == ignore_addr) {
		return 0;
	}
	regs[addr / 2] = v;
	return 0;
}

static void
fake_delay(uint32_t us)
{
	(void)us;
}

static const struct mt6358_poweroff_ops ops = { fake_read, fake_write, fake_delay };

static void
reset(void)
{
	memset(regs, 0, sizeof(regs));
	regs[MT6358_DCXO_CW02 / 2] = 0x0003;
	regs[MT6358_PPCCTL0 / 2] = 0x0001;
	nlog = 0;
	fail_addr = ignore_addr = -1;
}

static void
expect(const uint32_t (*w)[2], unsigned int n)
{
	assert(nlog == n);
	for (unsigned int i = 0; i < n; i++) {
		assert(log_addr[i] == w[i][0] && log_val[i] == w[i][1]);
	}
}

int
main(void)
{
	/* this phone's Android: the misc driver's shutdown hook (dcxo-switch), then mt6358-core's PWRHOLD */
	static const uint32_t android[][2] = { { 0x51e, 0x0004 }, { 0x790, 0x0002 }, { 0xa08, 0x0000 } };
	reset();
	assert(mt6358_poweroff_prepare(&ops) == NULL && mt6358_poweroff_cut(&ops) == 0);
	expect(android, 3);

	/* DCXO_CW02 does not read back: stop before PWRHOLD */
	reset();
	ignore_addr = 0x790;
	assert(strcmp(mt6358_poweroff_prepare(&ops), "DCXO_CW02") == 0);
	assert(nlog == 2);

	/* only the table's writes, with its exact mask and value; unchanged fields are not written */
	reset();
	assert(mt6358_poweroff_write(&ops, 0xa08, 0x0001, 1) == -1);
	assert(mt6358_poweroff_write(&ops, 0xa08, 0x0003, 0) == -1);
	assert(mt6358_poweroff_write(&ops, 0x1508, 0x0001, 0) == -1);
	regs[MT6358_PPCCTL0 / 2] = 0;
	assert(mt6358_poweroff_cut(&ops) == 0 && nlog == 0);

	printf("mt6358_poweroff: ok\n");
	return 0;
}
