/*
 * viva (MT6781): CONNSYS / WiFi bring-up, step 1 (USB link "wifi" command).
 *
 * The WiFi is the SoC's CONNSYS (CONNAC 1.x, legacy WMT, gen4m chip SOC2_1X1 over AXI at 0x18000000) with an
 * MT6631 or MT6635 RF "A-die" (picks the rails and the firmware flavor). Facts, sequences and citations:
 * ~/Developer/ios-viva/wifi/docs/MT6781-WIFI-HW.md (sources: mt6781-devs/android_kernel_xiaomi_mt6781
 * common_main/platform/mt6781.c, mt6781_pos_gen.c; clk-mt6781-pg.c).
 *
 *   wifi [status]  read only: SPM CONN power, bus protection, EMI remap, CONN reset, PMIC rails, and (only when
 *                  CONN is powered and its bus protection released) chip id / HW version / MCU idle magic.
 *   wifi detect    A-die detection as Linux's consys_adie_chipid_checking_flow (6631 rails: VCN18 only):
 *                  VCN18 on, A-die pinmux, CONN CPU held in reset, MTCMOS on, bus protection off, chip id poll,
 *                  EMI HW mode, AFE, XO (co-clock), A-die SPI read of 0xB024 (6631) and 0xB02C (6635), then
 *                  everything back off (pinmux and VCN18 restored to what they were). Refused if CONN is
 *                  already powered. Nothing here touches 0x18xxxxxx unless CONN power and bus protection were
 *                  verified first (an access with protection set can hang the bus: a soft hang, reboot by hand).
 *
 * PMIC writes go through VivaPwrap::writeRegulator (VCN18 enable/mode bits only, no voltages).
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>

extern int viva_pwrap_read(uint32_t addr, uint32_t *value);                                  /* VivaPwrap.cpp */
extern int viva_pwrap_write_regulator(uint32_t addr, uint32_t mask, uint32_t value);

/* AP side */
#define SPM_POWERON_CONFIG_EN   0x10006000
#define SPM_CONN_PWR_CON        0x10006304
#define SPM_PWR_STATUS          0x1000616c
#define SPM_PWR_STATUS_2ND      0x10006170
#define CONN_PWR_STA            (1u << 1)
#define PWR_RST_B               (1u << 0)
#define PWR_ISO                 (1u << 1)
#define PWR_ON                  (1u << 2)
#define PWR_ON_2ND              (1u << 3)
#define PWR_CLK_DIS             (1u << 4)
#define INFRA_PROT_SET          0x100012a0
#define INFRA_PROT_CLR          0x100012a4
#define INFRA_PROT_STA1         0x10001228
#define INFRA_PROT1_SET         0x100012a8
#define INFRA_PROT1_CLR         0x100012ac
#define INFRA_PROT1_STA1        0x10001258
#define INFRA_CCIF4_CG_CLR      0x100010c4
#define INFRA_CONN_CCU_CONFIG   0x10001c14
#define INFRA_EMI_MAPPING       0x10001380
#define TOPRGU_SWSYSRST         0x10007018
#define GPIO_MODE23             0x10005470
#define GPIO_MODE24             0x10005480
#define IOCFG_LT_DRV            0x10002c00
/* CONN side (only with CONN powered and protection released) */
#define CONN_VERSION            0x180b1010
#define CONN_CFG_ID             0x180b101c
#define CONN_HW_VER             0x18002000
#define CONN_FW_VER             0x18002004
#define CONN_MCU_COM_REG0       0x18002600
#define CONN_MCU_EMI_SLPPROT    0x180c1168
#define CONN_AFE_WBG_PLL_04     0x180b3030
#define CONN_AFE_WBG_GL5_02     0x180b3104
#define CONN_XO_1200            0x180c1200
#define CONN_XO_1204            0x180c1204
#define CONN_XO_1240            0x180c1240
#define CONN_ADIE_CTL           0x180c1500
#define CONN_SPI_STA            0x180c6000
#define CONN_SPI_ADDR           0x180c6050
#define CONN_SPI_WDAT           0x180c6054
#define CONN_SPI_RDAT           0x180c6058
#define CONN_VERSION_VALUE      0x100b0000
/* PMIC MT6366 */
#define PMIC_VCN18_CON0         0x1c58
#define PMIC_VCN18_OP_EN        0x1c5a
#define PMIC_VCN18_OP_CFG       0x1c60
#define PMIC_VCN18_CON1         0x1c66  /* bit 15 QI (output on) */
#define PMIC_VCN18_ANA          0x1e8c  /* [11:8] 12 = 1.8 V */
#define PMIC_VCN33_CON0_0       0x1d1c
#define PMIC_VCN33_CON0_1       0x1d2a
#define PMIC_VCN33_CON1         0x1d2c
#define PMIC_VCN33_ANA          0x1e28
#define PMIC_VCN28_CON0         0x1d88
#define PMIC_VCN28_CON1         0x1d96
#define PMIC_VS2_VOSEL          0x172e

static inline uint32_t rd(uint64_t pa) { return ml_phys_read_word_64(pa); }
static inline void wr(uint64_t pa, uint32_t v) { ml_phys_write_word_64(pa, v); }
static inline void rmw(uint64_t pa, uint32_t mask, uint32_t val) { wr(pa, (rd(pa) & ~mask) | (val & mask)); }

static bool
poll(uint64_t pa, uint32_t mask, uint32_t want, uint32_t us)
{
	for (uint32_t t = 0; t < us; t += 10) {
		if ((rd(pa) & mask) == want) {
			return true;
		}
		delay(10);
	}
	return (rd(pa) & mask) == want;
}

static uint32_t
pmic(uint32_t addr)
{
	uint32_t v = 0xffffffff;
	(void)viva_pwrap_read(addr, &v);
	return v;
}

#define say(...) do { if (o < len) o += (size_t)snprintf(out + o, len - o, __VA_ARGS__); } while (0)

static bool
conn_powered(void)
{
	return (rd(SPM_PWR_STATUS) & CONN_PWR_STA) && (rd(SPM_PWR_STATUS_2ND) & CONN_PWR_STA);
}

/* Bus protection bits CONNSYS needs released (clk-mt6781-pg.c conn steps 1-4). */
static bool
conn_unprotected(void)
{
	return (rd(INFRA_PROT_STA1) & 0x16000) == 0 && (rd(INFRA_PROT1_STA1) & 0x40000) == 0;
}

static size_t
wifi_status(char *out, size_t len, size_t o)
{
	uint32_t pwr = rd(SPM_CONN_PWR_CON);
	say("conn power: PWR_CON 0x%08x (on %u/%u iso %u rst_b %u clk_dis %u), status %u/%u, bus protect 0x%08x/0x%08x (%s)\n",
	    pwr, !!(pwr & PWR_ON), !!(pwr & PWR_ON_2ND), !!(pwr & PWR_ISO), !!(pwr & PWR_RST_B), !!(pwr & PWR_CLK_DIS),
	    !!(rd(SPM_PWR_STATUS) & CONN_PWR_STA), !!(rd(SPM_PWR_STATUS_2ND) & CONN_PWR_STA), rd(INFRA_PROT_STA1),
	    rd(INFRA_PROT1_STA1), conn_unprotected() ? "conn released" : "conn protected");
	say("emi remap 0x%08x, toprgu swsysrst 0x%08x (conn reset bit12 %u), pinmux mode23 0x%08x mode24 0x%08x drv 0x%08x\n",
	    rd(INFRA_EMI_MAPPING), rd(TOPRGU_SWSYSRST), !!(rd(TOPRGU_SWSYSRST) & (1u << 12)), rd(GPIO_MODE23),
	    rd(GPIO_MODE24), rd(IOCFG_LT_DRV));
	uint32_t ana18 = pmic(PMIC_VCN18_ANA);
	say("pmic: vcn18 con0 %04x op %04x cfg %04x qi %u vosel %u (%s); vcn33 en0 %04x en1(wifi) %04x qi %u ana %04x; "
	    "vcn28 %04x qi %u; vs2 vosel %04x\n", pmic(PMIC_VCN18_CON0), pmic(PMIC_VCN18_OP_EN), pmic(PMIC_VCN18_OP_CFG),
	    !!(pmic(PMIC_VCN18_CON1) & 0x8000), (ana18 >> 8) & 0xf, ((ana18 >> 8) & 0xf) == 12 ? "1.8 V" : "NOT 1.8 V",
	    pmic(PMIC_VCN33_CON0_0), pmic(PMIC_VCN33_CON0_1), !!(pmic(PMIC_VCN33_CON1) & 0x8000), pmic(PMIC_VCN33_ANA),
	    pmic(PMIC_VCN28_CON0), !!(pmic(PMIC_VCN28_CON1) & 0x8000), pmic(PMIC_VS2_VOSEL));
	if (conn_powered() && conn_unprotected()) {
		say("conn: version 0x%08x (want 0x%08x), cfg id %u, hw 0x%04x fw 0x%04x, mcu com_reg0 0x%08x (0x1d1e = idle loop)\n",
		    rd(CONN_VERSION), CONN_VERSION_VALUE, rd(CONN_CFG_ID) & 0xf, rd(CONN_HW_VER) & 0xffff,
		    rd(CONN_FW_VER) & 0xffff, rd(CONN_MCU_COM_REG0));
	} else {
		say("conn registers not read (powered off or protected)\n");
	}
	return o;
}

static uint32_t
adie_read(uint32_t addr)
{
	if (!poll(CONN_SPI_STA, 1u << 0, 0, 5000)) {
		return 0xfffffffe;
	}
	wr(CONN_SPI_ADDR, addr);
	wr(CONN_SPI_WDAT, 0);
	if (!poll(CONN_SPI_STA, 1u << 5, 0, 5000)) {
		return 0xfffffffd;
	}
	return rd(CONN_SPI_RDAT);
}

/* Power-off (mtk_wcn_consys_hw.c / clk-mt6781-pg.c conn off): protection on with acks, MTCMOS off. */
static size_t
conn_off(char *out, size_t len, size_t o)
{
	static const struct { uint64_t set, sta; uint32_t bit; } prot[] = {
		{ INFRA_PROT1_SET, INFRA_PROT1_STA1, 0x40000 }, { INFRA_PROT_SET, INFRA_PROT_STA1, 0x4000 },
		{ INFRA_PROT_SET, INFRA_PROT_STA1, 0x2000 }, { INFRA_PROT_SET, INFRA_PROT_STA1, 0x10000 },
	};
	for (unsigned i = 0; i < sizeof(prot) / sizeof(prot[0]); i++) {
		wr(prot[i].set, prot[i].bit);
		if (!poll(prot[i].sta, prot[i].bit, prot[i].bit, 10000)) {
			say("off: bus protection 0x%x not acknowledged\n", prot[i].bit);
		}
	}
	rmw(SPM_CONN_PWR_CON, PWR_ISO, PWR_ISO);
	rmw(SPM_CONN_PWR_CON, PWR_CLK_DIS, PWR_CLK_DIS);
	rmw(SPM_CONN_PWR_CON, PWR_RST_B, 0);
	rmw(SPM_CONN_PWR_CON, PWR_ON, 0);
	rmw(SPM_CONN_PWR_CON, PWR_ON_2ND, 0);
	bool off = poll(SPM_PWR_STATUS, CONN_PWR_STA, 0, 10000) && poll(SPM_PWR_STATUS_2ND, CONN_PWR_STA, 0, 10000);
	say("off: MTCMOS %s (PWR_CON 0x%08x)\n", off ? "off" : "STILL ON", rd(SPM_CONN_PWR_CON));
	return o;
}

static size_t
wifi_detect(char *out, size_t len, size_t o)
{
	if (rd(SPM_PWR_STATUS) & CONN_PWR_STA) {
		say("detect: CONN is already powered: refused (wifi status)\n");
		return o;
	}
	uint32_t ana18 = pmic(PMIC_VCN18_ANA);
	if (((ana18 >> 8) & 0xf) != 12) {
		say("detect: VCN18 voltage select %u is not 1.8 V: refused (no voltage writes here)\n", (ana18 >> 8) & 0xf);
		return o;
	}
	uint32_t old18 = pmic(PMIC_VCN18_CON0), oldOp = pmic(PMIC_VCN18_OP_EN), oldCfg = pmic(PMIC_VCN18_OP_CFG);
	uint32_t oldM23 = rd(GPIO_MODE23), oldM24 = rd(GPIO_MODE24), oldDrv = rd(IOCFG_LT_DRV);
	/* 1. VCN18 (consys_hw_vcn18_ctrl: EN=1 LP=0, SW_OP_EN=1 HW0_OP_EN=1, HW0_OP_CFG=1) */
	int e = viva_pwrap_write_regulator(PMIC_VCN18_CON0, 0x3, 0x1);
	e |= viva_pwrap_write_regulator(PMIC_VCN18_OP_EN, 0x3, 0x3);
	e |= viva_pwrap_write_regulator(PMIC_VCN18_OP_CFG, 0x2, 0x2);
	delay(300);
	say("detect: vcn18 %s, qi %u\n", e ? "WRITE FAILED" : "on", !!(pmic(PMIC_VCN18_CON1) & 0x8000));
	if (e) {
		goto rails;
	}
	/* 2. A-die interface pins (set_if_pinmux(1)), 150 us */
	rmw(GPIO_MODE24, 0x77777777, 0x11111111);
	rmw(GPIO_MODE23, 0x70000000, 0x10000000);
	rmw(IOCFG_LT_DRV, 0x07ffffff, 0x1209);
	delay(150);
	/* 3. CONN CPU held in reset (key 0x88, bit 12), SPM register access */
	rmw(TOPRGU_SWSYSRST, 0xff001000, 0x88001000);
	rmw(SPM_POWERON_CONFIG_EN, 0xffff0001, 0x0b160001);
	/* 4. MTCMOS on (consys_hw_power_ctrl_mt6781_gen) */
	wr(INFRA_CCIF4_CG_CLR, 1u << 24);
	rmw(SPM_CONN_PWR_CON, PWR_ON, PWR_ON);
	if (!poll(SPM_PWR_STATUS, CONN_PWR_STA, CONN_PWR_STA, 5000)) {
		say("detect: PWR_STATUS did not come up\n");
		goto off;
	}
	rmw(SPM_CONN_PWR_CON, PWR_ON_2ND, PWR_ON_2ND);
	if (!poll(SPM_PWR_STATUS_2ND, CONN_PWR_STA, CONN_PWR_STA, 5000)) {
		say("detect: PWR_STATUS_2ND did not come up\n");
		goto off;
	}
	rmw(SPM_CONN_PWR_CON, PWR_CLK_DIS, 0);
	delay(1);
	rmw(SPM_CONN_PWR_CON, PWR_ISO, 0);
	rmw(SPM_CONN_PWR_CON, PWR_RST_B, PWR_RST_B);
	delay(5000);
	/* 5. bus protection off, each acknowledged */
	{
		static const struct { uint64_t clr, sta; uint32_t bit; } prot[] = {
			{ INFRA_PROT_CLR, INFRA_PROT_STA1, 0x10000 }, { INFRA_PROT_CLR, INFRA_PROT_STA1, 0x4000 },
			{ INFRA_PROT1_CLR, INFRA_PROT1_STA1, 0x40000 }, { INFRA_PROT_CLR, INFRA_PROT_STA1, 0x2000 },
		};
		for (unsigned i = 0; i < sizeof(prot) / sizeof(prot[0]); i++) {
			wr(prot[i].clr, prot[i].bit);
			if (!poll(prot[i].sta, prot[i].bit, 0, 10000)) {
				say("detect: bus protection 0x%x did not release: CONN registers not touched\n", prot[i].bit);
				goto off;
			}
		}
	}
	wr(INFRA_CONN_CCU_CONFIG, 0);
	delay(10);
	/* 6. chip id (polling_consys_chipid: 10 x 20 ms) */
	{
		bool idOK = false;
		for (int i = 0; i < 10 && !(idOK = rd(CONN_VERSION) == CONN_VERSION_VALUE); i++) {
			delay(20000);
		}
		say("detect: conn version 0x%08x (%s), cfg id %u, hw 0x%04x, fw 0x%04x\n", rd(CONN_VERSION),
		    idOK ? "ok" : "UNEXPECTED", rd(CONN_CFG_ID) & 0xf, rd(CONN_HW_VER) & 0xffff, rd(CONN_FW_VER) & 0xffff);
		if (!idOK) {
			goto off;
		}
	}
	/* 7. EMI HW mode, AFE, XO (legacy co-clock branch: WMT_SOC.cfg co_clock_flag=1) */
	rmw(CONN_MCU_EMI_SLPPROT, 1u << 0, 1u << 0);
	wr(CONN_AFE_WBG_PLL_04, 0x00040088);
	wr(CONN_AFE_WBG_GL5_02, 0xa8808800);
	rmw(CONN_XO_1240, 1u << 7, 0);
	rmw(CONN_XO_1204, 0xff00, 0x0600);
	rmw(CONN_XO_1200, 0xffff, 0x0708);
	rmw(CONN_XO_1204, 1u << 0, 0);
	/* 8. A-die id over the RF SPI (consys_adie_chipid_checking_flow) */
	rmw(CONN_ADIE_CTL, 1u << 0, 1u << 0);
	{
		uint32_t a = adie_read(0xb024), b = adie_read(0xb02c);
		say("detect: a-die spi 0xb024 -> 0x%08x (id 0x%04x), 0xb02c -> 0x%08x (id 0x%04x): %s\n", a, a >> 16, b, b >> 16,
		    (a >> 16) == 0x6631 ? "MT6631 (firmware *_soc2_2_*_2_1*)" : (b >> 16) == 0x6635 ?
		    "MT6635 (firmware *_2a_1*; its 1.3 V rail was off, so this read is lucky)" :
		    "no answer (MT6635 needs VS2 1.4625 V + GPIO12 on to answer)");
	}
	rmw(CONN_ADIE_CTL, 1u << 0, 0);
off:
	o = conn_off(out, len, o);
	wr(GPIO_MODE24, (rd(GPIO_MODE24) & ~0x77777777u) | (oldM24 & 0x77777777u));
	wr(GPIO_MODE23, (rd(GPIO_MODE23) & ~0x70000000u) | (oldM23 & 0x70000000u));
	wr(IOCFG_LT_DRV, (rd(IOCFG_LT_DRV) & ~0x07ffffffu) | (oldDrv & 0x07ffffffu));
rails:
	viva_pwrap_write_regulator(PMIC_VCN18_OP_CFG, 0x2, oldCfg & 0x2);
	viva_pwrap_write_regulator(PMIC_VCN18_OP_EN, 0x3, oldOp & 0x3);
	viva_pwrap_write_regulator(PMIC_VCN18_CON0, 0x3, old18 & 0x3);
	say("detect: rails and pinmux restored (vcn18 con0 %04x qi %u)\n", pmic(PMIC_VCN18_CON0),
	    !!(pmic(PMIC_VCN18_CON1) & 0x8000));
	return o;
}

void
viva_wifi_link(const char *args, char *out, size_t len)
{
	size_t o = 0;
	if (len == 0) {
		return;
	}
	out[0] = 0;
	while (*args == ' ') {
		args++;
	}
	if (strncmp(args, "detect", 6) == 0) {
		printf("VIVA WIFI: a-die detection\n");
		o = wifi_detect(out, len, o);
		printf("%s", out);
	} else {
		o = wifi_status(out, len, o);
	}
	(void)o;
}
#endif /* ARM64_BOARD_CONFIG_SWEET */
