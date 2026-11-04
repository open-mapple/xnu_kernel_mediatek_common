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
 *   wifi on        step 2 (MT6631 A-die): the full Linux power-on (mtk_wcn_consys_hw_reg_ctrl + mt6781_pos_gen.c):
 *                  5 MiB EMI window (iokit/Kernel/arm/VivaWiFi.cpp) remapped at 0xF0000000 for the MCU, the RAM
 *                  patches mcu/bt/wifi from WIFI_FW_DIR (48-byte header dropped) at +0x0/+0x170000/+0x2A0000 and
 *                  their dates at +0x88300/310/320, rails VCN18 + VCN33 (3.3 V) + VCN28 (co-clock efuse PALDO),
 *                  pinmux, MTCMOS, bus protection, chip id, EMI HW mode, AFE, EMI entries, XO, A-die select, bus
 *                  timeout, MCU mem delay, CPU reset released, then the MCU must reach its idle loop
 *                  (0x18002600 == 0x1D1E). CONNSYS stays on (no WMT/STP yet, no EMI MPU SMC: tried without first).
 *   wifi off       CONN CPU back in reset, MTCMOS off, rails and pinmux as before "wifi on".
 *
 * PMIC writes go through VivaPwrap::writeRegulator (VCN18/VCN33/VCN28 enable/mode bits, VCN33 = 3.3 V only).
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <sys/vnode.h>
#include <sys/vnode_internal.h>
#include <sys/fcntl.h>
#include <sys/uio.h>

extern int viva_pwrap_read(uint32_t addr, uint32_t *value);                                  /* VivaPwrap.cpp */
extern int viva_pwrap_write_regulator(uint32_t addr, uint32_t mask, uint32_t value);
extern void *viva_wifi_emi_alloc(uint32_t size, uint64_t *pa);                                /* VivaWiFi.cpp */
extern void viva_wifi_emi_flush(uint32_t offset, uint32_t len);

#define WIFI_FW_DIR             "/usr/share/firmware/wifi/viva/"
#define EMI_SIZE                0x500000
#define EMI_CONN_BASE           0xf0000000u

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
#define INFRA_PERI_MAPPING      0x10001388
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
#define CONN_BUSHANGCR          0x18002440
#define CONN_BT_EMI_ENTRY       0x18002504
#define CONN_WF_EMI_ENTRY       0x18002508
#define CONN_MCU_DBG_216C       0x1800216c
#define CONN_HOST_CSR           0x18007000      /* readability check: write 0x30000, bits 16/17 = HCLK/OSC on */
#define CONN_MCU_PC             0x18007104
#define CONN_ADIE_SEL           0x180c1130
#define CONN_RGU_DLY_CHAIN      0x180c0284
#define CONN_IDLE_MAGIC         0x00001d1e
/* PMIC MT6366 */
#define PMIC_VCN18_CON0         0x1c58
#define PMIC_VCN18_OP_EN        0x1c5a
#define PMIC_VCN18_OP_CFG       0x1c60
#define PMIC_VCN18_CON1         0x1c66  /* bit 15 QI (output on) */
#define PMIC_VCN18_ANA          0x1e8c  /* [11:8] 12 = 1.8 V */
#define PMIC_VCN33_CON0_0       0x1d1c
#define PMIC_VCN33_OP_EN        0x1d1e
#define PMIC_VCN33_OP_CFG       0x1d24
#define PMIC_VCN28_OP_EN        0x1d8a
#define PMIC_VCN28_OP_CFG       0x1d90
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
		wr(CONN_HOST_CSR, 0x30000);
		delay(200);
		say("conn: host csr 0x%08x (hclk %u osc %u), mcu pc 0x%08x, dbg 0x%08x\n", rd(CONN_HOST_CSR),
		    !!(rd(CONN_HOST_CSR) & (1u << 16)), !!(rd(CONN_HOST_CSR) & (1u << 17)), rd(CONN_MCU_PC),
		    rd(CONN_MCU_DBG_216C));
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

/* ---- step 2: CONNSYS on with the RAM patches (MT6631 A-die) ---- */

struct wmt_rom_patch {                  /* common_main/core/include/wmt_core.h: 48 bytes, stripped */
	char     date[16];
	char     plat[4];
	uint16_t hwVer, swVer;          /* big endian */
	uint32_t patchAddr;             /* byte 0 = sequence flag (0x11), bytes 1..2 = EMI offset >> 8 */
	uint32_t patchType;
	uint32_t crc[4];
};

static struct {
	bool     on;
	void    *emi;
	uint64_t emiPA;
	uint32_t old18, old18Op, old18Cfg, old33, old33Op, old33Cfg, old28, old28Op, old28Cfg;
	uint32_t oldM23, oldM24, oldDrv;
} vw;

/* Read WIFI_FW_DIR/name into the EMI window at its header's offset (header dropped); its date at +dateOff. */
static int
load_patch(const char *name, uint32_t wantOff, uint32_t dateOff, char *out, size_t len, size_t *po)
{
	size_t o = *po;
	char path[160];
	snprintf(path, sizeof(path), "%s%s", WIFI_FW_DIR, name);
	vfs_context_t ctx = vfs_context_kernel();
	vnode_t vp = NULLVP;
	int err = vnode_open(path, FREAD, 0, 0, &vp, ctx);
	if (err != 0) {
		say("on: %s: open error %d (push it with tools/viva-link.py push)\n", path, err);
		*po = o;
		return err;
	}
	off_t size = 0;
	struct wmt_rom_patch h;
	int resid = 0;
	err = vnode_size(vp, &size, ctx);
	if (err == 0 && size <= (off_t)sizeof(h)) {
		err = EINVAL;
	}
	if (err == 0) {
		err = vn_rdwr(UIO_READ, vp, (caddr_t)&h, sizeof(h), 0, UIO_SYSSPACE, IO_NOAUTH | IO_UNIT,
		    vfs_context_ucred(ctx), &resid, vfs_context_proc(ctx));
	}
	uint32_t off = h.patchAddr & 0x00ffff00u;     /* 0xF02A0011: 0x11 = flag (zeroed), 0xF0000000 + offset */
	uint32_t body = (uint32_t)(size - (off_t)sizeof(h));
	if (err == 0 && (resid != 0 || off != wantOff || (uint64_t)off + body > EMI_SIZE || dateOff + 16 > EMI_SIZE)) {
		say("on: %s: unexpected header (patch addr 0x%08x -> offset 0x%x, want 0x%x, %u bytes)\n", name,
		    h.patchAddr, off, wantOff, body);
		err = EINVAL;
	}
	if (err == 0) {
		err = vn_rdwr(UIO_READ, vp, (caddr_t)vw.emi + off, (int)body, sizeof(h), UIO_SYSSPACE, IO_NOAUTH | IO_UNIT,
		    vfs_context_ucred(ctx), &resid, vfs_context_proc(ctx));
		if (err == 0 && resid != 0) {
			err = EIO;
		}
	}
	vnode_close(vp, FREAD, ctx);
	if (err == 0) {
		memcpy((char *)vw.emi + dateOff, h.date, 16);
		viva_wifi_emi_flush(off, body);
		viva_wifi_emi_flush(dateOff & ~63u, 64);
		say("on: %s: %u bytes at EMI +0x%06x (MCU 0x%08x), type %u, date %.16s\n", name, body, off,
		    EMI_CONN_BASE + off, __builtin_bswap32(h.patchType), h.date);
	} else {
		say("on: %s: error %d\n", name, err);
	}
	*po = o;
	return err;
}

static void
rails_restore(void)
{
	viva_pwrap_write_regulator(PMIC_VCN28_CON0, 0x1, vw.old28 & 0x1);
	viva_pwrap_write_regulator(PMIC_VCN28_OP_CFG, 0x2, vw.old28Cfg & 0x2);
	viva_pwrap_write_regulator(PMIC_VCN28_OP_EN, 0x2, vw.old28Op & 0x2);
	viva_pwrap_write_regulator(PMIC_VCN33_OP_CFG, 0x2, vw.old33Cfg & 0x2);
	viva_pwrap_write_regulator(PMIC_VCN33_OP_EN, 0x3, vw.old33Op & 0x3);
	viva_pwrap_write_regulator(PMIC_VCN33_CON0_0, 0x3, vw.old33 & 0x3);
	delay(300);
	viva_pwrap_write_regulator(PMIC_VCN18_OP_CFG, 0x2, vw.old18Cfg & 0x2);
	viva_pwrap_write_regulator(PMIC_VCN18_OP_EN, 0x3, vw.old18Op & 0x3);
	viva_pwrap_write_regulator(PMIC_VCN18_CON0, 0x3, vw.old18 & 0x3);
}

static void
pinmux_restore(void)
{
	wr(GPIO_MODE24, (rd(GPIO_MODE24) & ~0x77777777u) | (vw.oldM24 & 0x77777777u));
	wr(GPIO_MODE23, (rd(GPIO_MODE23) & ~0x70000000u) | (vw.oldM23 & 0x70000000u));
	wr(IOCFG_LT_DRV, (rd(IOCFG_LT_DRV) & ~0x07ffffffu) | (vw.oldDrv & 0x07ffffffu));
}

static size_t
wifi_off(char *out, size_t len, size_t o)
{
	if (!vw.on) {
		say("off: not on (wifi on)\n");
		return o;
	}
	rmw(TOPRGU_SWSYSRST, 0xff001000, 0x88001000);  /* CPU back in reset first */
	o = conn_off(out, len, o);
	pinmux_restore();
	rails_restore();
	vw.on = false;
	say("off: rails and pinmux restored; the EMI window stays allocated for the next \"wifi on\"\n");
	return o;
}

static size_t
wifi_on(char *out, size_t len, size_t o)
{
	if (vw.on || (rd(SPM_PWR_STATUS) & CONN_PWR_STA)) {
		say("on: CONN is already powered (wifi status / wifi off)\n");
		return o;
	}
	if (((pmic(PMIC_VCN18_ANA) >> 8) & 0xf) != 12) {
		say("on: VCN18 is not 1.8 V: refused\n");
		return o;
	}
	/* EMI window + RAM patches (the CPU stays in reset until they are in DRAM) */
	vw.emi = viva_wifi_emi_alloc(EMI_SIZE, &vw.emiPA);
	if (vw.emi == NULL) {
		say("on: no contiguous 5 MiB below 4 GiB for the EMI window\n");
		return o;
	}
	bzero(vw.emi, EMI_SIZE);
	viva_wifi_emi_flush(0, EMI_SIZE);
	say("on: EMI window PA 0x%llx (%s aligned), MCU sees it at 0x%08x\n", vw.emiPA,
	    (vw.emiPA & 0xffffff) == 0 ? "16 MiB" : "64 KiB", EMI_CONN_BASE);
	if (load_patch("soc2_2_ram_mcu_2_1_hdr.bin", 0x000000, 0x88320, out, len, &o) ||
	    load_patch("soc2_2_ram_bt_2_1_hdr.bin", 0x170000, 0x88300, out, len, &o) ||
	    load_patch("soc2_2_ram_wifi_2_1_hdr.bin", 0x2a0000, 0x88310, out, len, &o)) {
		return o;
	}
	rmw(INFRA_EMI_MAPPING, 0xfffff, (uint32_t)(vw.emiPA >> 16));
	rmw(INFRA_PERI_MAPPING, 0xfffff, 0x1000);
	/* rails: VCN18, then VCN33 at 3.3 V (6631), VCN28 on in co-clock mode (efuse PALDO, HW mode off) */
	vw.old18 = pmic(PMIC_VCN18_CON0);
	vw.old18Op = pmic(PMIC_VCN18_OP_EN);
	vw.old18Cfg = pmic(PMIC_VCN18_OP_CFG);
	vw.old33 = pmic(PMIC_VCN33_CON0_0);
	vw.old33Op = pmic(PMIC_VCN33_OP_EN);
	vw.old33Cfg = pmic(PMIC_VCN33_OP_CFG);
	vw.old28 = pmic(PMIC_VCN28_CON0);
	vw.old28Op = pmic(PMIC_VCN28_OP_EN);
	vw.old28Cfg = pmic(PMIC_VCN28_OP_CFG);
	vw.oldM23 = rd(GPIO_MODE23);
	vw.oldM24 = rd(GPIO_MODE24);
	vw.oldDrv = rd(IOCFG_LT_DRV);
	int e = viva_pwrap_write_regulator(PMIC_VCN18_CON0, 0x3, 0x1);
	e |= viva_pwrap_write_regulator(PMIC_VCN18_OP_EN, 0x3, 0x3);
	e |= viva_pwrap_write_regulator(PMIC_VCN18_OP_CFG, 0x2, 0x2);
	e |= viva_pwrap_write_regulator(PMIC_VCN33_ANA, 0x300, 0x100);
	e |= viva_pwrap_write_regulator(PMIC_VCN33_CON0_0, 0x3, 0x1);
	e |= viva_pwrap_write_regulator(PMIC_VCN33_OP_EN, 0x3, 0x3);
	e |= viva_pwrap_write_regulator(PMIC_VCN33_OP_CFG, 0x2, 0x2);
	e |= viva_pwrap_write_regulator(PMIC_VCN28_OP_EN, 0x2, 0x0);
	e |= viva_pwrap_write_regulator(PMIC_VCN28_OP_CFG, 0x2, 0x0);
	e |= viva_pwrap_write_regulator(PMIC_VCN28_CON0, 0x1, 0x1);
	delay(300);
	say("on: rails %s: vcn18 qi %u, vcn33 qi %u (ana %04x), vcn28 qi %u\n", e ? "WRITE FAILED" : "on",
	    !!(pmic(PMIC_VCN18_CON1) & 0x8000), !!(pmic(PMIC_VCN33_CON1) & 0x8000), pmic(PMIC_VCN33_ANA),
	    !!(pmic(PMIC_VCN28_CON1) & 0x8000));
	vw.on = true;           /* from here "wifi off" undoes everything */
	if (e) {
		goto fail;
	}
	rmw(GPIO_MODE24, 0x77777777, 0x11111111);
	rmw(GPIO_MODE23, 0x70000000, 0x10000000);
	rmw(IOCFG_LT_DRV, 0x07ffffff, 0x1209);
	delay(150);
	rmw(TOPRGU_SWSYSRST, 0xff001000, 0x88001000);
	rmw(SPM_POWERON_CONFIG_EN, 0xffff0001, 0x0b160001);
	wr(INFRA_CCIF4_CG_CLR, 1u << 24);
	rmw(SPM_CONN_PWR_CON, PWR_ON, PWR_ON);
	if (!poll(SPM_PWR_STATUS, CONN_PWR_STA, CONN_PWR_STA, 5000)) {
		say("on: PWR_STATUS did not come up\n");
		goto fail;
	}
	rmw(SPM_CONN_PWR_CON, PWR_ON_2ND, PWR_ON_2ND);
	if (!poll(SPM_PWR_STATUS_2ND, CONN_PWR_STA, CONN_PWR_STA, 5000)) {
		say("on: PWR_STATUS_2ND did not come up\n");
		goto fail;
	}
	rmw(SPM_CONN_PWR_CON, PWR_CLK_DIS, 0);
	delay(1);
	rmw(SPM_CONN_PWR_CON, PWR_ISO, 0);
	rmw(SPM_CONN_PWR_CON, PWR_RST_B, PWR_RST_B);
	delay(5000);
	{
		static const struct { uint64_t clr, sta; uint32_t bit; } prot[] = {
			{ INFRA_PROT_CLR, INFRA_PROT_STA1, 0x10000 }, { INFRA_PROT_CLR, INFRA_PROT_STA1, 0x4000 },
			{ INFRA_PROT1_CLR, INFRA_PROT1_STA1, 0x40000 }, { INFRA_PROT_CLR, INFRA_PROT_STA1, 0x2000 },
		};
		for (unsigned i = 0; i < sizeof(prot) / sizeof(prot[0]); i++) {
			wr(prot[i].clr, prot[i].bit);
			if (!poll(prot[i].sta, prot[i].bit, 0, 10000)) {
				say("on: bus protection 0x%x did not release\n", prot[i].bit);
				goto fail;
			}
		}
	}
	wr(INFRA_CONN_CCU_CONFIG, 0);
	delay(10);
	{
		bool idOK = false;
		for (int i = 0; i < 10 && !(idOK = rd(CONN_VERSION) == CONN_VERSION_VALUE); i++) {
			delay(20000);
		}
		if (!idOK) {
			say("on: conn version 0x%08x, not 0x%08x\n", rd(CONN_VERSION), CONN_VERSION_VALUE);
			goto fail;
		}
	}
	rmw(CONN_MCU_EMI_SLPPROT, 1u << 0, 1u << 0);           /* 11. EMI HW mode */
	wr(CONN_AFE_WBG_PLL_04, 0x00040088);                    /* 13. AFE */
	wr(CONN_AFE_WBG_GL5_02, 0xa8808800);
	wr(CONN_BT_EMI_ENTRY, EMI_CONN_BASE + 0x170000);        /* 14. EMI entries */
	wr(CONN_WF_EMI_ENTRY, EMI_CONN_BASE + 0x2a0000);
	rmw(CONN_XO_1240, 1u << 7, 0);                          /* 15. XO, legacy co-clock */
	rmw(CONN_XO_1204, 0xff00, 0x0600);
	rmw(CONN_XO_1200, 0xffff, 0x0708);
	rmw(CONN_XO_1204, 1u << 0, 0);
	rmw(CONN_ADIE_SEL, 0x300, 0x100);                       /* 16. A-die = MT6631 */
	wr(CONN_BUSHANGCR, 0x90000002);                         /* 18. bus timeout */
	rmw(CONN_RGU_DLY_CHAIN, 0xf0000, 0x40000);              /* 19. MCU mem PDN delay */
	rmw(TOPRGU_SWSYSRST, 0xff001000, 0x88000000);           /* 20. CPU out of reset: ROM, then the patches */
	{
		bool idle = false;
		uint32_t t;
		for (t = 0; t < 41 && !(idle = rd(CONN_MCU_COM_REG0) == CONN_IDLE_MAGIC); t++) {
			delay(6000);
		}
		say("on: MCU %s after ~%u ms: com_reg0 0x%08x, pc 0x%08x, dbg 0x%08x\n", idle ? "IDLE LOOP" :
		    "NOT IDLE", t * 6, rd(CONN_MCU_COM_REG0), rd(CONN_MCU_PC), rd(CONN_MCU_DBG_216C));
		if (!idle) {
			say("on: left on for inspection (wifi status, peek); wifi off to power down. Next suspects: EMI MPU "
			    "SMC, patch placement\n");
		}
	}
	delay(20000);
	return o;
fail:
	o = wifi_off(out, len, o);
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
	if (strncmp(args, "on", 2) == 0) {
		printf("VIVA WIFI: power on\n");
		o = wifi_on(out, len, o);
		printf("%s", out);
	} else if (strncmp(args, "off", 3) == 0) {
		o = wifi_off(out, len, o);
		printf("%s", out);
	} else if (strncmp(args, "detect", 6) == 0) {
		if (vw.on) {
			snprintf(out, len, "detect: CONN is on (wifi off first)\n");
			return;
		}
		printf("VIVA WIFI: a-die detection\n");
		o = wifi_detect(out, len, o);
		printf("%s", out);
	} else {
		o = wifi_status(out, len, o);
	}
	(void)o;
}
#endif /* ARM64_BOARD_CONFIG_SWEET */
