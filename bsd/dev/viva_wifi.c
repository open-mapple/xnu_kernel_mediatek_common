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
 *   wifi wmt       step 2b (after "wifi on", before "wifi fw"): what Linux's WMT does over STP/BTIF before the WiFi
 *                  driver may start (wmt_core_stp_init + mtk_wcn_soc_sw_init, MT6781 options): BTIF in PIO (clock
 *                  infracfg_ao bit 31, CONNSYS wake pulse), STP "mand" mode: register read 0x80000008 (proves the
 *                  MCU answers), query/set STP -> full mode (seq/ack, header sum, CRC-16 IBM), set chip id 0x6781,
 *                  query A-die, RF calibration, then VCN28 (efuse PALDO) off. 0127 showed the WiFi RAM code starting
 *                  (ready bits 0x2 -> 0x4) but never finishing its init without this.
 *   wifi fw        step 3 (after "wifi on"), gen4m wlanDownloadFW without WMT: driver own (LPCTL), WFDMA rings
 *                  (TX 0/1/3/15, RX 0/1, 16 entries) in contiguous memory, GLO_CFG with the FW-download bypass,
 *                  WIFI_FW_DIR/WIFI_RAM_CODE_soc2_2_2_1.bin (CONNAC tailer format): DL_TO_EMI regions copied into
 *                  the EMI window at Addr & 0xFFFFFF, the others sent on TX ring 3 (INIT_CMD DOWNLOAD_CONFIG, 2 KiB
 *                  PDA chunks, result event on RX ring 1), then WIFI_START (override, RAM entry) and the ready bits
 *                  0x180C1140 & 0x6. gen4m sources: chips/common/fw_dl.c, cmm_asic_connac.c, hif_pdma.h.
 *   wifi cap       step 4 (firmware ready): CMD_ID_GET_NIC_CAPABILITY_V2 (0x8A) on TX ring 15 (WIFI_CMD: 32-byte TXD +
 *                  32-byte command header, wlan_lib.c wlanQueryNicCapabilityV2), event 0xEC on RX ring 1 (20-byte RXD +
 *                  12-byte event header + {u16 count, rsv, TLVs (u32 tag, u32 len, body)}): HW/SW version, MAC, PHY caps.
 *   wifi init      step 5: basic config, NVRAM (WIFI_FW_DIR/WIFI from nvdata, else the vendor default + a random
 *                  locally administered MAC), domain info, DBDC off, RX filter, BSS 0 activated
 *                  (MT6781-WIFI-CMDS.md §2-§3).
 *   wifi scan [2g] [SSID]  SCAN_REQ_V2, beacons/probe responses into a table until SCAN_DONE, listed by RSSI.
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
#include <sys/random.h>

extern int viva_pwrap_read(uint32_t addr, uint32_t *value);                                  /* VivaPwrap.cpp */
extern int viva_pwrap_write_regulator(uint32_t addr, uint32_t mask, uint32_t value);
extern void *viva_wifi_emi_alloc(uint32_t size, uint64_t *pa);                                /* VivaWiFi.cpp */
extern void viva_wifi_emi_flush(uint32_t offset, uint32_t len);
extern void *viva_wifi_dma_alloc(uint32_t size, uint64_t *pa);
extern void viva_wifi_dma_flush(uint32_t offset, uint32_t len);

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

/* WiFi (WFDMA) side, step 3 */
#define WF_LPCTL                0x18007000      /* BIT0 HOST_SET_OWN (read: 1 = firmware own), BIT1 HOST_CLR_OWN */
#define WF_INT_STA              0x18004200
#define WF_INT_MSK              0x18004204
#define WF_GLO_CFG              0x18004208
#define WF_RST_PTR              0x1800420c
#define WF_PAUSE_TX_Q           0x18004224
#define WF_APSRC_SLPPROT        0x18004160
#define WF_TX_RING(n)           (0x18004300 + 0x10 * (n))      /* +0 BASE +4 CNT +8 CIDX +C DIDX */
#define WF_RX_RING(n)           (0x18004400 + 0x10 * (n))
#define WF_TX_EXT(n)            (0x18004500 + 4 * (n))
#define WF_RX_EXT(n)            (0x18004580 + 4 * (n))
#define WF_READY                0x180c1140      /* (v & 0x6) == 0x6: WiFi firmware ready */

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
		say("wifi: ready bits 0x%08x (0x6 = firmware ready), LPCTL 0x%08x, GLO_CFG 0x%08x, int_sta 0x%08x; "
		    "tx3 cidx %u didx %u, tx15 cidx %u didx %u, rx0 cidx %u didx %u, rx1 cidx %u didx %u\n", rd(WF_READY),
		    rd(WF_LPCTL), rd(WF_GLO_CFG), rd(WF_INT_STA), rd(WF_TX_RING(3) + 8), rd(WF_TX_RING(3) + 12),
		    rd(WF_TX_RING(15) + 8), rd(WF_TX_RING(15) + 12), rd(WF_RX_RING(0) + 8), rd(WF_RX_RING(0) + 12),
		    rd(WF_RX_RING(1) + 8), rd(WF_RX_RING(1) + 12));
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

/* ---- step 2b: WMT over STP/BTIF ---- */

#define BTIF_BASE               0x1100c000
#define BTIF_RBR                (BTIF_BASE + 0x00)
#define BTIF_THR                (BTIF_BASE + 0x00)
#define BTIF_IER                (BTIF_BASE + 0x04)
#define BTIF_FIFOCTRL           (BTIF_BASE + 0x08)
#define BTIF_FAKELCR            (BTIF_BASE + 0x0c)
#define BTIF_LSR                (BTIF_BASE + 0x14)
#define BTIF_DMA_EN             (BTIF_BASE + 0x4c)
#define BTIF_TRI_LVL            (BTIF_BASE + 0x60)
#define BTIF_WAK                (BTIF_BASE + 0x64)
#define BTIF_HANDSHAKE          (BTIF_BASE + 0x6c)
#define LSR_DR                  (1u << 0)
#define LSR_THRE                (1u << 5)
#define INFRA_CG0_CLR           0x10001084
#define INFRA_CG0_STA           0x10001090
#define INFRA_CG0_BTIF          (1u << 31)
#define STP_TYPE_WMT            4

static struct {
	bool    full;
	uint8_t txseq, txack, rxseq;
	uint16_t crc[256];
} st;

static void
crc_init(void)
{
	for (uint32_t i = 0; i < 256; i++) {            /* CRC-16 IBM (reflected 0xA001): osal.c crc16_table */
		uint16_t c = (uint16_t)i;
		for (int k = 0; k < 8; k++) {
			c = (c & 1) ? (uint16_t)((c >> 1) ^ 0xa001) : (uint16_t)(c >> 1);
		}
		st.crc[i] = c;
	}
}

static uint16_t
crc16(const uint8_t *b, uint32_t n)
{
	uint16_t c = 0;
	while (n--) {
		c = (uint16_t)((c >> 8) ^ st.crc[(c ^ *b++) & 0xff]);
	}
	return c;
}

static bool
btif_put(const uint8_t *b, uint32_t n)
{
	for (uint32_t i = 0; i < n; i++) {
		if (!poll(BTIF_LSR, LSR_THRE, LSR_THRE, 20000)) {
			return false;
		}
		wr(BTIF_THR, b[i]);
	}
	return true;
}

static int
btif_get(uint32_t us)
{
	for (uint32_t t = 0; t <= us; t += 10) {
		if (rd(BTIF_LSR) & LSR_DR) {
			return (int)(rd(BTIF_RBR) & 0xff);
		}
		delay(10);
	}
	return -1;
}

/* hal_btif_hw_init in PIO, plus the AP -> CONNSYS wake pulse (hal_btif_raise_wak_sig). */
static void
btif_init(void)
{
	if (rd(INFRA_CG0_STA) & INFRA_CG0_BTIF) {
		wr(INFRA_CG0_CLR, INFRA_CG0_BTIF);
	}
	wr(BTIF_FAKELCR, 0);
	rmw(BTIF_HANDSHAKE, 1, 1);
	rmw(BTIF_FIFOCTRL, 1u << 1, 1u << 1);
	rmw(BTIF_FIFOCTRL, 1u << 1, 0);
	rmw(BTIF_FIFOCTRL, 1u << 2, 1u << 2);
	rmw(BTIF_FIFOCTRL, 1u << 2, 0);
	wr(BTIF_TRI_LVL, 8 | (1u << 4));                /* TX 8, RX 1, loopback off */
	wr(BTIF_DMA_EN, 1u << 2);                       /* PIO both ways, auto reset */
	wr(BTIF_IER, 0);                                /* polled */
	rmw(BTIF_WAK, 1, 0);
	delay(160);
	rmw(BTIF_WAK, 1, 1);
	st.full = false;
	st.txseq = 0;
	st.txack = 7;
	st.rxseq = 0;
	crc_init();
}

static bool
stp_ack(void)
{
	uint8_t h[4] = { (uint8_t)(0x80 | st.txack), 0, 0, 0 };
	h[3] = (uint8_t)(h[0] + h[1] + h[2]);
	return btif_put(h, 4);
}

/* One WMT packet in the current STP mode (stp_core.c mtk_wcn_stp_send_data, mand / full). */
static bool
stp_send(const uint8_t *p, uint32_t n)
{
	uint8_t h[4], c[2] = { 0, 0 };
	h[0] = (uint8_t)(0x80 | (st.txseq << 3) | (st.full ? st.txack : 0));
	h[1] = (uint8_t)((STP_TYPE_WMT << 4) | ((n >> 8) & 0xf));
	h[2] = (uint8_t)n;
	h[3] = st.full ? (uint8_t)(h[0] + h[1] + h[2]) : 0;
	if (st.full) {
		uint16_t crc = crc16(p, n);
		c[0] = (uint8_t)crc;
		c[1] = (uint8_t)(crc >> 8);
		st.txseq = (st.txseq + 1) & 7;
	}
	return btif_put(h, 4) && btif_put(p, n) && btif_put(c, 2);
}

/* Next STP data frame (ack-only frames skipped; full mode: acknowledged). Payload into out, length returned, <0 none. */
static int
stp_recv(uint8_t *out, uint32_t max, uint32_t ms, uint8_t *type)
{
	for (uint32_t tries = 0; tries < 16; tries++) {
		int b0;
		do {                                    /* sync: bit 7 */
			b0 = btif_get(ms * 1000);
		} while (b0 >= 0 && !(b0 & 0x80));
		int b1 = b0 < 0 ? -1 : btif_get(20000), b2 = b1 < 0 ? -1 : btif_get(20000), b3 = b2 < 0 ? -1 : btif_get(20000);
		if (b3 < 0) {
			return -1;
		}
		uint32_t n = ((uint32_t)(b1 & 0xf) << 8) | (uint32_t)b2;
		if (n == 0) {
			continue;                       /* an acknowledgement */
		}
		uint32_t got = 0;
		for (uint32_t i = 0; i < n + 2; i++) {
			int v = btif_get(20000);
			if (v < 0) {
				return -2;
			}
			if (i < n && got < max) {
				out[got++] = (uint8_t)v;
			}
		}
		if (type) {
			*type = (uint8_t)(b1 >> 4);
		}
		if (st.full) {
			st.txack = (uint8_t)((b0 >> 3) & 7);
			st.rxseq = (st.txack + 1) & 7;
			stp_ack();
		}
		return (int)got;
	}
	return -3;
}

/* WMT command -> its event (02 opcode ...); the event bytes are compared with `want` (wlen bytes) if given. */
static int
wmt_cmd(const char *what, const uint8_t *cmd, uint32_t clen, const uint8_t *want, uint32_t wlen, uint32_t ms,
    uint8_t *ev, uint32_t evmax, char *out, size_t len, size_t *po)
{
	size_t o = *po;
	uint8_t type = 0;
	int n = -1;
	if (!stp_send(cmd, clen)) {
		say("wmt: %s: BTIF did not take it (LSR 0x%02x)\n", what, rd(BTIF_LSR) & 0xff);
		*po = o;
		return -1;
	}
	for (int k = 0; k < 4; k++) {
		n = stp_recv(ev, evmax, ms, &type);
		if (n < 0 || (type == STP_TYPE_WMT && n >= 2 && ev[0] == 0x02 && ev[1] == cmd[1])) {
			break;
		}
		say("wmt: %s: other frame type %u, %d bytes (%02x %02x ..)\n", what, type, n, n > 0 ? ev[0] : 0,
		    n > 1 ? ev[1] : 0);
	}
	if (n < 0) {
		say("wmt: %s: no answer (%d, LSR 0x%02x)\n", what, n, rd(BTIF_LSR) & 0xff);
	} else {
		say("wmt: %s ->", what);
		for (int i = 0; i < n && i < 24; i++) {
			say(" %02x", ev[i]);
		}
		bool ok = want == NULL || ((uint32_t)n >= wlen && memcmp(ev, want, wlen) == 0);
		say("%s\n", ok ? "" : "  (UNEXPECTED: shown, sequence goes on)");
	}
	*po = o;
	return n;
}

static size_t
wifi_wmt(char *out, size_t len, size_t o)
{
	if (!vw.on || rd(CONN_MCU_COM_REG0) != CONN_IDLE_MAGIC) {
		say("wmt: CONNSYS not on with the MCU idle (wifi on first)\n");
		return o;
	}
	btif_init();
	say("wmt: BTIF clock %s, LSR 0x%02x\n", (rd(INFRA_CG0_STA) & INFRA_CG0_BTIF) ? "STILL GATED" : "on",
	    rd(BTIF_LSR) & 0xff);
	uint8_t ev[64];
	/* wmt_core_hw_check: register read 0x80000008 (GEN_HCR) in mand mode */
	static const uint8_t regrd[] = { 0x01, 0x08, 0x10, 0x00, 0x02, 0x01, 0x00, 0x01, 0x08, 0x00, 0x00, 0x80,
		                         0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff };
	/* MT6781 answers 02 08 0c 00 | status 00, 00 00 01 | address | value (0127+0130 on the phone) */
	static const uint8_t regrdEv[] = { 0x02, 0x08, 0x0c, 0x00, 0x00 };
	int rn = wmt_cmd("read 0x80000008", regrd, sizeof(regrd), regrdEv, sizeof(regrdEv), 1000, ev, sizeof(ev), out, len, &o);
	if (rn < 0) {
		return o;
	}
	if (rn >= 16) {
		say("wmt: GEN_HCR 0x%02x%02x%02x%02x (the MCU answers over STP/BTIF)\n", ev[15], ev[14], ev[13], ev[12]);
	}
	static const uint8_t qstp[] = { 0x01, 0x04, 0x01, 0x00, 0x04 };
	static const uint8_t qstpDef[] = { 0x02, 0x04, 0x06, 0x00, 0x00, 0x04, 0x11, 0x00, 0x00, 0x00 };
	static const uint8_t qstpFull[] = { 0x02, 0x04, 0x06, 0x00, 0x00, 0x04, 0xdb, 0x0e, 0x68, 0x01 };
	static const uint8_t sstp[] = { 0x01, 0x04, 0x05, 0x00, 0x03, 0xdb, 0x0e, 0x68, 0x01 };
	static const uint8_t sstpEv[] = { 0x02, 0x04, 0x02, 0x00, 0x00, 0x03 };
	(void)wmt_cmd("query stp", qstp, sizeof(qstp), qstpDef, sizeof(qstpDef), 1000, ev, sizeof(ev), out, len, &o);
	if (wmt_cmd("set stp full", sstp, sizeof(sstp), sstpEv, sizeof(sstpEv), 1000, ev, sizeof(ev), out, len, &o) < 0) {
		return o;
	}
	st.full = true;                                 /* host side switch, 10 ms */
	st.txseq = 0;
	st.txack = 7;
	delay(10000);
	if (wmt_cmd("query stp (full)", qstp, sizeof(qstp), qstpFull, sizeof(qstpFull), 1000, ev, sizeof(ev), out, len,
	    &o) < 0) {
		return o;
	}
	static const uint8_t chipid[] = { 0x01, 0x02, 0x05, 0x00, 0x11, 0x81, 0x67, 0x00, 0x00 };
	static const uint8_t chipidEv[] = { 0x02, 0x02, 0x01, 0x00, 0x00 };
	if (wmt_cmd("set chip id 0x6781", chipid, sizeof(chipid), chipidEv, sizeof(chipidEv), 1000, ev, sizeof(ev), out,
	    len, &o) < 0) {
		return o;
	}
	static const uint8_t adie[] = { 0x01, 0x02, 0x01, 0x00, 0x12 };
	static const uint8_t adieEv[] = { 0x02, 0x02, 0x05, 0x00, 0x00 };
	(void)wmt_cmd("query a-die", adie, sizeof(adie), adieEv, sizeof(adieEv), 1000, ev, sizeof(ev), out, len, &o);
	static const uint8_t cal[] = { 0x01, 0x14, 0x01, 0x00, 0x01 };
	/* MT6781: 02 14 <len 0x22a> 00 01 + calibration data (0130 on the phone) */
	static const uint8_t calEv[] = { 0x02, 0x14 };
	int c = wmt_cmd("rf calibration", cal, sizeof(cal), calEv, sizeof(calEv), 5000, ev, sizeof(ev), out, len, &o);
	/* wmt_ic_soc.c: the efuse PALDO (VCN28) goes off after the calibration */
	viva_pwrap_write_regulator(PMIC_VCN28_CON0, 0x1, 0x0);
	say("wmt: %s; VCN28 off. Next: wifi fw\n", c >= 0 ? "done" : "calibration FAILED");
	return o;
}

/* ---- step 3: WiFi RAM code over WFDMA ---- */

#define GLO_TX_EN               (1u << 0)
#define GLO_TX_BUSY             (1u << 1)
#define GLO_RX_EN               (1u << 2)
#define GLO_RX_BUSY             (1u << 3)
#define GLO_ENABLE              (GLO_TX_EN | GLO_RX_EN | (3u << 4) | (1u << 6) | (2u << 10) | (1u << 12) | \
	                         (1u << 26) | (1u << 30))      /* asicPdmaConfig */
#define GLO_FWDL_BYPASS         (1u << 9)       /* bypass_dmashdl_txring3 while downloading */
#define DESC_DMADONE            (1u << 31)
#define DESC_LASTSEC0           (1u << 30)
#define RING_N                  16
#define BUF_SZ                  0x1000
#define RX_BUF_LEN              3840
#define DMA_SIZE                0x48000
#define OFF_TX(n)               ((n) == 3 ? 0x0000u : (n) == 15 ? 0x0100u : (n) == 0 ? 0x0200u : 0x0300u)
#define OFF_RX(n)               ((n) == 0 ? 0x0400u : 0x0500u)
#define OFF_TXBUF               0x01000u        /* ring 3 then ring 15 buffers: 16 x 4 KiB each */
#define OFF_RXBUF(n)            ((n) == 1 ? 0x21000u : 0x31000u)
#define WIFI_RAM_CODE           "WIFI_RAM_CODE_soc2_2_2_1.bin"
#define PDA_CHUNK               2048

static struct {
	uint8_t *va;
	uint64_t pa;
	uint32_t txIdx[16], rxIdx[2];
	uint8_t  seq;
} wd;

static uint32_t *
desc(uint32_t off, uint32_t i)
{
	return (uint32_t *)(void *)(wd.va + off + 16 * i);
}

static void
wfdma_setup(void)
{
	bzero(wd.va, DMA_SIZE);
	static const uint32_t tx[] = { 0, 1, 3, 15 };
	for (unsigned r = 0; r < 2; r++) {
		for (uint32_t i = 0; i < RING_N; i++) {
			uint32_t *d = desc(OFF_RX(r), i);
			d[0] = (uint32_t)(wd.pa + OFF_RXBUF(r) + BUF_SZ * i);
			d[1] = (uint32_t)RX_BUF_LEN << 16;
		}
	}
	viva_wifi_dma_flush(0, DMA_SIZE);
	wr(WF_GLO_CFG, rd(WF_GLO_CFG) & ~(GLO_TX_EN | GLO_RX_EN));
	(void)poll(WF_GLO_CFG, GLO_TX_BUSY | GLO_RX_BUSY, 0, 100000);
	wr(WF_RST_PTR, 0xffffffff);
	for (unsigned k = 0; k < 4; k++) {
		uint32_t n = tx[k];
		wr(WF_TX_RING(n) + 0, (uint32_t)(wd.pa + OFF_TX(n)));
		wr(WF_TX_EXT(n), (uint32_t)(wd.pa >> 32));
		wr(WF_TX_RING(n) + 4, RING_N);
		wr(WF_TX_RING(n) + 8, 0);
		wd.txIdx[n] = 0;
	}
	for (unsigned r = 0; r < 2; r++) {
		wr(WF_RX_RING(r) + 0, (uint32_t)(wd.pa + OFF_RX(r)));
		wr(WF_RX_EXT(r), (uint32_t)(wd.pa >> 32));
		wr(WF_RX_RING(r) + 4, RING_N);
		wr(WF_RX_RING(r) + 8, RING_N - 1);
		wd.rxIdx[r] = 0;
	}
	wr(WF_INT_MSK, 0);
	wr(WF_INT_STA, 0xffffffff);
	wr(WF_PAUSE_TX_Q, 0);
	rmw(WF_APSRC_SLPPROT, 1u << 4, 1u << 4);
	wr(WF_GLO_CFG, rd(WF_GLO_CFG) | GLO_ENABLE | GLO_FWDL_BYPASS);
	wd.seq = 0;
}

/* One packet on TX ring n (3 or 15): copied into its buffer, CIDX advanced, wait for the DMA to take it. */
static bool
wfdma_tx(uint32_t n, const void *pkt, uint32_t plen)
{
	uint32_t i = wd.txIdx[n], boff = OFF_TXBUF + (n == 15 ? RING_N * BUF_SZ : 0) + BUF_SZ * i;
	if (plen > BUF_SZ) {
		return false;
	}
	memcpy(wd.va + boff, pkt, plen);
	viva_wifi_dma_flush(boff, plen);
	uint32_t *d = desc(OFF_TX(n), i);
	d[0] = (uint32_t)(wd.pa + boff);
	d[1] = (plen << 16) | DESC_LASTSEC0;
	d[2] = 0;
	d[3] = (uint32_t)((wd.pa + boff) >> 32) & 0xf;
	viva_wifi_dma_flush(OFF_TX(n) + 16 * i, 16);
	wd.txIdx[n] = (i + 1) % RING_N;
	wr(WF_TX_RING(n) + 8, wd.txIdx[n]);
	return poll(WF_TX_RING(n) + 12, 0xfff, wd.txIdx[n], 200000);
}

/* Next received packet on RX ring r within `us`: copied to out (max), length returned, 0 = nothing. */
static uint32_t
wfdma_rx(uint32_t r, uint8_t *out, uint32_t max, uint32_t us)
{
	uint32_t i = wd.rxIdx[r], *d = desc(OFF_RX(r), i);
	for (uint32_t t = 0; t <= us; t += 50) {
		viva_wifi_dma_flush(OFF_RX(r) + 16 * i, 16);
		if (d[1] & DESC_DMADONE) {
			uint32_t n = (d[1] >> 16) & 0x3fff, boff = OFF_RXBUF(r) + BUF_SZ * i;
			viva_wifi_dma_flush(boff, BUF_SZ);
			memcpy(out, wd.va + boff, MIN(n, max));
			d[1] = (uint32_t)RX_BUF_LEN << 16;
			viva_wifi_dma_flush(OFF_RX(r) + 16 * i, 16);
			wr(WF_RX_RING(r) + 8, i);              /* give the entry back */
			wd.rxIdx[r] = (i + 1) % RING_N;
			return n ? n : 1;
		}
		delay(50);
	}
	return 0;
}

/* INIT packet (cmm_asic_connac.c asicFillInitCmdTxdInfo): 32-byte TXD + 32-byte INIT_HIF_TX_HEADER + payload. */
static uint32_t
init_pkt(uint8_t *pkt, uint8_t cid, const void *payload, uint32_t plen, uint8_t *seqOut)
{
	uint32_t total = 64 + plen;
	bzero(pkt, 64);
	pkt[0] = (uint8_t)total;
	pkt[1] = (uint8_t)(total >> 8);
	uint16_t pq = cid ? 0x8000 : 0xf800;            /* INIT_CMD_PQ_ID / INIT_CMD_PDA_PQ_ID */
	pkt[2] = (uint8_t)pq;
	pkt[3] = (uint8_t)(pq >> 8);
	pkt[5] = 0xa0;                                  /* header format */
	pkt[7] = cid ? 2 : 3;                           /* PktFt: INIT_PKT_FT_CMD / INIT_PKT_FT_PDA_FWDL */
	pkt[32] = (uint8_t)(total - 32);
	pkt[33] = (uint8_t)((total - 32) >> 8);
	pkt[36] = cid;
	pkt[37] = 0xa0;
	pkt[39] = ++wd.seq;
	if (seqOut) {
		*seqOut = wd.seq;
	}
	if (payload != pkt + 64) {
		memmove(pkt + 64, payload, plen);
	}
	return total;
}

/* Send an init command, wait for its INIT_EVENT_ID_CMD_RESULT on RX ring 1 (20-byte RXD + 8-byte event head). */
static int
init_cmd(uint8_t cid, const void *payload, uint32_t plen, uint32_t waitMs, char *out, size_t len, size_t *po)
{
	size_t o = *po;
	uint8_t pkt[128], ev[256], seq;
	uint32_t n = init_pkt(pkt, cid, payload, plen, &seq);
	int ret = -1;
	if (!wfdma_tx(3, pkt, n)) {
		say("fw: cmd 0x%02x: TX ring 3 did not take it (didx %u cidx %u)\n", cid, rd(WF_TX_RING(3) + 12),
		    rd(WF_TX_RING(3) + 8));
		goto done;
	}
	for (int tries = 0; tries < 6; tries++) {
		uint32_t got = 0, ring = 1;
		for (uint32_t ms = 0; ms < waitMs && got == 0; ms += 10) {      /* both RX rings */
			if ((got = wfdma_rx(1, ev, sizeof(ev), 5000)) == 0 && (got = wfdma_rx(0, ev, sizeof(ev), 5000)) != 0) {
				ring = 0;
			}
		}
		if (got == 0) {
			say("fw: cmd 0x%02x seq %u: no event in %u ms (rx1 didx %u, rx0 didx %u, int_sta 0x%08x, ready 0x%08x)\n",
			    cid, seq, waitMs, rd(WF_RX_RING(1) + 12), rd(WF_RX_RING(0) + 12), rd(WF_INT_STA), rd(WF_READY));
			goto done;
		}
		if (ring == 0) {
			say("fw: cmd 0x%02x: event on RX ring 0 (%u bytes)\n", cid, got);
		}
		uint8_t *e = ev + 20;
		if (got >= 32 && e[4] == 1 && e[5] == seq) {
			ret = e[8];
			if (ret != 0) {
				say("fw: cmd 0x%02x seq %u: result status %u (1 params, 2 CRC, 3 decrypt)\n", cid, seq, e[8]);
			}
			goto done;
		}
		say("fw: cmd 0x%02x: other event (%u bytes, eid 0x%02x seq %u): %02x %02x %02x %02x %02x %02x %02x %02x\n",
		    cid, got, e[4], e[5], ev[0], ev[1], ev[2], ev[3], e[0], e[1], e[2], e[3]);
	}
done:
	*po = o;
	return ret;
}

static size_t
wifi_fw(char *out, size_t len, size_t o)
{
	if (!vw.on || rd(CONN_MCU_COM_REG0) != CONN_IDLE_MAGIC) {
		say("fw: CONNSYS not on with the MCU idle (wifi on first)\n");
		return o;
	}
	if ((rd(WF_READY) & 0x6) == 0x6) {
		say("fw: WiFi firmware already running (ready 0x%08x)\n", rd(WF_READY));
		return o;
	}
	wd.va = viva_wifi_dma_alloc(DMA_SIZE, &wd.pa);
	if (wd.va == NULL) {
		say("fw: no DMA memory\n");
		return o;
	}
	/* driver own */
	wr(WF_LPCTL, 1u << 1);
	if (!poll(WF_LPCTL, 1u << 0, 0, 100000)) {
		say("fw: driver own not granted (LPCTL 0x%08x)\n", rd(WF_LPCTL));
		return o;
	}
	wfdma_setup();
	say("fw: driver own, WFDMA at PA 0x%llx, GLO_CFG 0x%08x, ready bits 0x%08x\n", wd.pa, rd(WF_GLO_CFG), rd(WF_READY));

	char path[160];
	snprintf(path, sizeof(path), "%s%s", WIFI_FW_DIR, WIFI_RAM_CODE);
	vfs_context_t ctx = vfs_context_kernel();
	vnode_t vp = NULLVP;
	int err = vnode_open(path, FREAD, 0, 0, &vp, ctx);
	if (err != 0) {
		say("fw: %s: open error %d (push it)\n", path, err);
		return o;
	}
	off_t size = 0;
	uint8_t tail[36], reg[40], pkt[64 + PDA_CHUNK];
	int resid = 0;
	(void)vnode_size(vp, &size, ctx);
#define RD(buf, n, at) (vn_rdwr(UIO_READ, vp, (caddr_t)(buf), (int)(n), (off_t)(at), UIO_SYSSPACE, IO_NOAUTH | IO_UNIT, \
	vfs_context_ucred(ctx), &resid, vfs_context_proc(ctx)) == 0 && resid == 0)
	if (size < 36 + 40 || !RD(tail, 36, size - 36) || tail[2] == 0 || tail[2] > 8) {
		say("fw: bad tailer\n");
		goto close;
	}
	uint32_t nreg = tail[2], fileOff = 0, entry = 0;
	say("fw: %u regions, version %.10s built %.15s\n", nreg, (const char *)tail + 7, (const char *)tail + 17);
	for (uint32_t r = 0; r < nreg; r++) {
		if (!RD(reg, 40, size - 36 - 40 * (off_t)(nreg - r))) {
			say("fw: region %u tailer unreadable\n", r);
			goto close;
		}
		uint32_t addr, rlen;
		memcpy(&addr, reg + 16, 4);
		memcpy(&rlen, reg + 20, 4);
		uint8_t feat = reg[24];
		if ((feat & 0x20) && entry == 0) {
			entry = addr;                           /* VALID_RAM_ENTRY */
		}
		if (feat & 0x40) {
			say("fw: region %u not downloaded (feature 0x%02x)\n", r, feat);
		} else if (feat & 0x80) {                       /* DL_TO_EMI */
			uint32_t eoff = addr & 0xffffff;
			if ((uint64_t)eoff + rlen > EMI_SIZE || !RD((uint8_t *)vw.emi + eoff, rlen, fileOff)) {
				say("fw: region %u: EMI copy failed (offset 0x%x, %u bytes)\n", r, eoff, rlen);
				goto close;
			}
			viva_wifi_emi_flush(eoff, rlen);
			say("fw: region %u: %u bytes -> EMI +0x%06x\n", r, rlen, eoff);
		} else {
			uint32_t mode = 0;
			if (feat & 0x01) {
				mode = (1u << 3) | (feat & 0x6) | 1u | ((feat & 0x10) ? (1u << 6) : 0);
			}
			mode |= 1u << 31;                       /* ACK option */
			uint32_t cfg[3] = { addr, rlen, mode };
			int res = init_cmd(1, cfg, sizeof(cfg), 500, out, len, &o);
			if (res != 0) {
				say("fw: region %u: DOWNLOAD_CONFIG (0x%08x, %u, 0x%08x) failed (%d)\n", r, addr, rlen, mode, res);
				goto close;
			}
			for (uint32_t done = 0; done < rlen; done += PDA_CHUNK) {
				uint32_t n = MIN((uint32_t)PDA_CHUNK, rlen - done);
				if (!RD(pkt + 64, n, fileOff + done)) {
					say("fw: region %u: read error\n", r);
					goto close;
				}
				uint32_t total = init_pkt(pkt, 0, pkt + 64, n, NULL);
				if (!wfdma_tx(3, pkt, total)) {
					say("fw: region %u: chunk at %u not taken by the DMA\n", r, done);
					goto close;
				}
			}
			say("fw: region %u: %u bytes -> 0x%08x (mode 0x%08x)\n", r, rlen, addr, mode);
		}
		fileOff += rlen;
	}
	{
		uint32_t start[2] = { entry ? 1u : 0u, entry };
		int res = init_cmd(2, start, sizeof(start), 3000, out, len, &o);
		bool ready = poll(WF_READY, 0x6, 0x6, 1000000);
		if (res != 0) {
			say("fw: WIFI_START 0x%08x: no good result (%d); ready bits 0x%08x, MCU pc 0x%08x\n", entry, res,
			    rd(WF_READY), rd(CONN_MCU_PC));
			goto close;
		}
		wr(WF_GLO_CFG, rd(WF_GLO_CFG) & ~GLO_FWDL_BYPASS);
		say("fw: WIFI_START 0x%08x ok; firmware %s (ready bits 0x%08x), MCU pc 0x%08x\n", entry,
		    ready ? "READY" : "NOT READY after 1 s", rd(WF_READY), rd(CONN_MCU_PC));
	}
#undef RD
close:
	vnode_close(vp, FREAD, ctx);
	return o;
}

/* ---- step 4: normal commands (firmware running) ---- */

/* Station state after "wifi init" (bsd/dev/viva_wifi.c is still driven from the USB link). */
#define WL_MAX_BSS              48
#define WL_IE_MAX               512

struct wl_bss {
	uint8_t  bssid[6], ssid[32], ssid_len, ch;
	int8_t   rssi;
	uint16_t capinfo, bintval;
	uint16_t ie_len;
	uint8_t  ie[WL_IE_MAX];
	uint32_t seen;                  /* scan generation */
};

static struct {
	bool     cap, init;
	uint8_t  mac[6];
	uint8_t  scanSeq, scanGen;
	uint32_t nbss, frames, dataFrames;
	struct wl_bss bss[WL_MAX_BSS];
} wl;

/*
 * Received packets (MT6781-WIFI-CMDS.md §5): dispatched on the RXD packet type, never on the ring. 0xE000 = event
 * (12-byte header at +20), 0xE001 = frame forwarded by the MCU (beacons, probe/auth/assoc responses, EAPOL),
 * type 2 = data from the MAC. The frame starts after the RXD (20), the optional groups (G4 16, G1 16, G2 8, G3 24,
 * in that order) and 2 bytes of padding if RXD byte 6 bit 6 is set.
 */
#define PKT_EVENT               1
#define PKT_FRAME               2
#define PKT_DATA                3

static int
rx_classify(const uint8_t *b, uint32_t n, uint32_t *foff)
{
	if (n < 20) {
		return 0;
	}
	uint16_t ptype = (uint16_t)(b[2] | b[3] << 8);
	if ((ptype & 0xe00f) == 0xe000) {
		return n >= 32 ? PKT_EVENT : 0;
	}
	uint32_t type = ptype >> 13;
	if ((ptype & 0xe00f) != 0xe001 && type != 2) {
		return 0;                               /* TX status, RX vector, MSDU report: not used */
	}
	uint32_t off = 20 + ((ptype & (1u << 12)) ? 16 : 0) + ((ptype & (1u << 9)) ? 16 : 0) +
	    ((ptype & (1u << 10)) ? 8 : 0) + ((ptype & (1u << 11)) ? 24 : 0) + ((b[6] & 0x40) ? 2 : 0);
	uint32_t total = (uint32_t)(b[0] | b[1] << 8);
	if (total > n) {
		total = n;
	}
	if (off >= total) {
		return 0;
	}
	*foff = off;
	return type == 2 ? PKT_DATA : PKT_FRAME;
}

/* RSSI (dBm) of a received frame from G3 RXV cycle 4 (RCPI0/1, the larger, ignoring > 220), 0 if no G3. */
static int
rx_rssi(const uint8_t *b)
{
	uint16_t ptype = (uint16_t)(b[2] | b[3] << 8);
	if (!(ptype & (1u << 11))) {
		return 0;
	}
	uint32_t g3 = 20 + ((ptype & (1u << 12)) ? 16 : 0) + ((ptype & (1u << 9)) ? 16 : 0) + ((ptype & (1u << 10)) ? 8 : 0);
	int r0 = b[g3 + 12], r1 = b[g3 + 13], r = 0;
	if (r0 <= 220) {
		r = r0;
	}
	if (r1 <= 220 && r1 > r) {
		r = r1;
	}
	return r / 2 - 110;
}

static void wifi_frame(const uint8_t *b, uint32_t n, uint32_t foff, int kind);     /* scan table, join, EAPOL */

/* Next packet from RX ring 1, then RX ring 0, within us microseconds; frames and data go to wifi_frame(). */
static uint32_t
rx_next(uint8_t *buf, uint32_t max, uint32_t us, int *kind)
{
	uint32_t got = wfdma_rx(1, buf, max, us / 2);
	if (got == 0) {
		got = wfdma_rx(0, buf, max, us / 2);
	}
	*kind = 0;
	if (got) {
		uint32_t foff = 0;
		*kind = rx_classify(buf, MIN(got, max), &foff);
		if (*kind == PKT_FRAME || *kind == PKT_DATA) {
			wifi_frame(buf, MIN(got, max), foff, *kind);
		}
	}
	return got;
}

static uint8_t wcmd_pkt[64 + 2048];

/*
 * WIFI_CMD (nic_cmd_event.h, asicFillCmdTxdInfo) on TX ring 15. eid != 0: wait for that event into ev (returns its
 * length). eid 0 (set commands: the stock driver expects no answer): only drain what comes back for 30 ms, logging
 * events (an unknown CID may get INIT_EVENT_ID_CMD_RESULT 0xFD); returns 0 if the DMA took it.
 */
static int
wifi_cmd(uint8_t cid, bool set, const void *payload, uint32_t plen, uint8_t eid, uint8_t *ev, uint32_t evmax,
    uint32_t ms, char *out, size_t len, size_t *po)
{
	size_t o = *po;
	uint8_t *pkt = wcmd_pkt;
	uint32_t total = 64 + plen;
	static uint8_t scratch[RX_BUF_LEN];
	if (plen > 2048) {
		return -1;
	}
	if (ev == NULL) {
		ev = scratch;
		evmax = sizeof(scratch);
	}
	bzero(pkt, 64);
	pkt[0] = (uint8_t)total;
	pkt[1] = (uint8_t)(total >> 8);
	pkt[2] = 0x00;                                  /* CMD_PQ_ID 0x8000 */
	pkt[3] = 0x80;
	pkt[5] = 0xa0;                                  /* header format */
	pkt[7] = 2;                                     /* TXD_PKT_FT_CMD */
	pkt[32] = (uint8_t)(total - 32);                /* u2Length */
	pkt[33] = (uint8_t)((total - 32) >> 8);
	pkt[36] = cid;
	pkt[37] = 0xa0;                                 /* CMD_PACKET_TYPE_ID */
	pkt[38] = set ? 1 : 0;
	pkt[39] = ++wd.seq;
	pkt[42] = 3;                                    /* S2D_INDEX_CMD_H2N_H2C */
	if (plen) {
		memcpy(pkt + 64, payload, plen);
	}
	uint8_t seq = wd.seq;
	int ret = -1;
	if (!wfdma_tx(15, pkt, total)) {
		say("cmd 0x%02x: TX ring 15 did not take it (cidx %u didx %u)\n", cid, rd(WF_TX_RING(15) + 8),
		    rd(WF_TX_RING(15) + 12));
		goto done;
	}
	if (eid == 0) {
		ret = 0;
		ms = 30;
	}
	for (uint32_t t = 0; t < ms;) {
		int kind;
		uint32_t got = rx_next(ev, evmax, 10000, &kind);
		if (got == 0) {
			t += 10;
			continue;
		}
		if (kind != PKT_EVENT) {
			continue;
		}
		uint8_t *e = ev + 20;
		if (eid != 0 && e[4] == eid) {
			ret = (int)got;
			goto done;
		}
		if (e[4] != 0x07) {                     /* EVENT_ID_SLEEPY_INFO: ignored, we stay in driver own */
			say("cmd 0x%02x: event 0x%02x seq %u (%u bytes): %02x %02x %02x %02x\n", cid, e[4], e[5], got,
			    e[12], e[13], e[14], e[15]);
		}
	}
	if (eid != 0) {
		say("cmd 0x%02x seq %u: no event 0x%02x in %u ms (rx1 didx %u)\n", cid, seq, eid, ms, rd(WF_RX_RING(1) + 12));
	}
done:
	*po = o;
	return ret;
}

static size_t
wifi_cap(char *out, size_t len, size_t o)
{
	if (!vw.on || (rd(WF_READY) & 0x6) != 0x6) {
		say("cap: WiFi firmware not running (wifi on, wifi wmt, wifi fw)\n");
		return o;
	}
	static uint8_t ev[2048];
	int n = wifi_cmd(0x8a, false, NULL, 0, 0xec, ev, sizeof(ev), 3000, out, len, &o);
	if (n > 0) {
		wl.cap = true;
	}
	if (n < 36) {
		say("cap: no NIC_CAPABILITY_V2 event\n");
		return o;
	}
	uint8_t *b = ev + 20 + 12;
	uint32_t count = b[0] | b[1] << 8, off = 4, end = (uint32_t)MIN(n, (int)sizeof(ev)) - 32;
	say("cap: %u elements (%d bytes)\n", count, n);
	for (uint32_t i = 0; i < count && off + 8 <= end; i++) {
		uint32_t tag, blen;
		memcpy(&tag, b + off, 4);
		memcpy(&blen, b + off + 4, 4);
		const uint8_t *d = b + off + 8;
		if (off + 8 + blen > end) {
			break;
		}
		switch (tag) {
		case 0x5:
			say("cap: hw: product 0x%04x eco %u, mac ip 0x%08x, bb ip 0x%08x, top ip 0x%08x, config %u\n",
			    d[0] | d[1] << 8, d[2], d[4] | d[5] << 8 | d[6] << 16 | (uint32_t)d[7] << 24,
			    d[8] | d[9] << 8 | d[10] << 16 | (uint32_t)d[11] << 24,
			    d[12] | d[13] << 8 | d[14] << 16 | (uint32_t)d[15] << 24, d[16]);
			break;
		case 0x6:
			say("cap: fw: version %u.%u build %u, branch %.4s, date %.16s\n", d[1], d[0], d[2] | d[3] << 8,
			    (const char *)d + 4, (const char *)d + 8);
			break;
		case 0x7:
			say("cap: MAC %02x:%02x:%02x:%02x:%02x:%02x (efuse)\n", d[0], d[1], d[2], d[3], d[4], d[5]);
			break;
		case 0x8:
			say("cap: phy: HT %u VHT %u 5 GHz %u max bandwidth %u (0=20 1=40 2=80), nss %u, LDPC %u\n", d[0], d[1],
			    d[2], d[3], blen > 4 ? d[4] : 0, blen > 5 ? d[5] : 0);
			break;
		case 0x14:
			say("cap: a-die version: chip 0x%04x eco %u\n", d[0] | d[1] << 8, d[2]);
			break;
		default:
			say("cap: tag 0x%02x, %u bytes\n", tag, blen);
			break;
		}
		off += 8 + blen;
	}
	return o;
}

/* ---- step 5: station bring-up and scan (MT6781-WIFI-CMDS.md §2-§4) ---- */

#define CID_DUMMY               0x00
#define CID_BASIC_CONFIG        0x02
#define CID_SCAN_REQ_V2         0x03
#define CID_SET_RX_FILTER       0x0a
#define CID_SET_DOMAIN_INFO     0x0f
#define CID_BSS_ACTIVATE_CTRL   0x11
#define CID_SET_DBDC_PARMS      0x28
#define CID_SET_NVRAM_SETTINGS  0x48
#define EID_SCAN_DONE           0x0d
#define NVRAM_SIZE              2048
#define NVRAM_FILE              "WIFI"                          /* nvdata APCFG/APRDEB/WIFI, copied by hand */
#define NVRAM_DEFAULT           "stWifiCfgDefault_6631.bin"     /* vendor default (MAC zero) */

static int
read_file(const char *name, uint8_t *buf, uint32_t max)
{
	char path[160];
	snprintf(path, sizeof(path), "%s%s", WIFI_FW_DIR, name);
	vfs_context_t ctx = vfs_context_kernel();
	vnode_t vp = NULLVP;
	int resid = 0, err = vnode_open(path, FREAD, 0, 0, &vp, ctx);
	if (err != 0) {
		return -err;
	}
	err = vn_rdwr(UIO_READ, vp, (caddr_t)buf, (int)max, 0, UIO_SYSSPACE, IO_NOAUTH, vfs_context_ucred(ctx), &resid,
	    vfs_context_proc(ctx));
	vnode_close(vp, FREAD, ctx);
	return err ? -err : (int)(max - (uint32_t)resid);
}

static bool
mac_valid(const uint8_t *m)
{
	static const uint8_t zero[6], ff[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	return memcmp(m, zero, 6) != 0 && memcmp(m, ff, 6) != 0 && !(m[0] & 1);
}

/* CMD_SET_DOMAIN_INFO V1 (56 B), country from the NVRAM, the default group's tables (rlm_domain.c:744-806). */
static void
domain_info(uint8_t *d, uint16_t cc, bool passive)
{
	static const uint8_t act[6][5] = { { 81, 1, 1, 1, 13 }, { 115, 2, 4, 36, 4 }, { 118, 2, 4, 52, 4 },
		                           { 121, 2, 4, 100, 12 }, { 125, 2, 4, 149, 5 }, { 0 } };
	static const uint8_t pas[6][5] = { { 81, 1, 1, 1, 0 }, { 82, 1, 1, 14, 0 }, { 115, 2, 4, 36, 0 },
		                           { 118, 2, 4, 52, 4 }, { 121, 2, 4, 100, 12 }, { 125, 2, 4, 149, 0 } };
	bzero(d, 56);
	d[0] = (uint8_t)cc;
	d[1] = (uint8_t)(cc >> 8);
	d[2] = passive ? 1 : 0;
	for (int i = 0; i < 6; i++) {
		memcpy(d + 4 + 8 * i, passive ? pas[i] : act[i], 5);
	}
}

static size_t
wifi_init(char *out, size_t len, size_t o)
{
	if (!vw.on || (rd(WF_READY) & 0x6) != 0x6) {
		say("init: WiFi firmware not running (wifi on, wifi wmt, wifi fw)\n");
		return o;
	}
	static uint8_t nv[NVRAM_SIZE];
	uint8_t p[64];
	const char *src = NVRAM_FILE;
	int n = read_file(NVRAM_FILE, nv, sizeof(nv));
	if (n != NVRAM_SIZE) {
		src = NVRAM_DEFAULT;
		n = read_file(NVRAM_DEFAULT, nv, sizeof(nv));
	}
	if (n != NVRAM_SIZE) {
		say("init: no NVRAM (%s%s or %s: %d); push one\n", WIFI_FW_DIR, NVRAM_FILE, NVRAM_DEFAULT, n);
		return o;
	}
	if (mac_valid(nv + 4)) {
		memcpy(wl.mac, nv + 4, 6);
	} else {
		/* no factory MAC: a locally administered one, the same until the next "wifi init" */
		read_random(wl.mac, 6);
		wl.mac[0] = (uint8_t)((wl.mac[0] & 0xfc) | 0x02);
		memcpy(nv + 4, wl.mac, 6);                     /* as Android's wlan_assistant patches it in */
	}
	uint16_t cc = (uint16_t)(nv[10] << 8 | nv[11]);
	say("init: NVRAM %s (version 0x%04x), MAC %02x:%02x:%02x:%02x:%02x:%02x%s, country %c%c\n", src, nv[0] | nv[1] << 8,
	    wl.mac[0], wl.mac[1], wl.mac[2], wl.mac[3], wl.mac[4], wl.mac[5], src == NVRAM_FILE ? " (factory)" : " (random)",
	    nv[10] ? nv[10] : '0', nv[11] ? nv[11] : '0');

	int err = 0;
	err |= wifi_cmd(CID_DUMMY, true, NULL, 0, 0, NULL, 0, 0, out, len, &o);
	if (!wl.cap) {
		o = wifi_cap(out, len, o);
	}
	bzero(p, sizeof(p));
	err |= wifi_cmd(CID_BASIC_CONFIG, true, p, 12, 0, NULL, 0, 0, out, len, &o);           /* 802.3 data path */
	err |= wifi_cmd(CID_SET_NVRAM_SETTINGS, true, nv, NVRAM_SIZE, 0, NULL, 0, 0, out, len, &o);
	domain_info(p, cc, true);
	err |= wifi_cmd(CID_SET_DOMAIN_INFO, true, p, 56, 0, NULL, 0, 0, out, len, &o);
	domain_info(p, cc, false);
	err |= wifi_cmd(CID_SET_DOMAIN_INFO, true, p, 56, 0, NULL, 0, 0, out, len, &o);
	bzero(p, 40);
	p[4] = 1;                                       /* ucCmdVer */
	p[6] = 40;                                      /* u2CmdLen; ucDbdcEn 0 */
	err |= wifi_cmd(CID_SET_DBDC_PARMS, true, p, 40, 0, NULL, 0, 0, out, len, &o);
	bzero(p, 68);
	p[0] = 0x0b;                                    /* directed | multicast | broadcast */
	err |= wifi_cmd(CID_SET_RX_FILTER, true, p, 68, 0, NULL, 0, 0, out, len, &o);
	uint8_t act[12] = { 0, 1, 0, 1 };               /* BSS 0 active, AIS, own MAC index 1 */
	memcpy(act + 4, wl.mac, 6);
	act[10] = 4;                                    /* broadcast/multicast WTBL entry */
	err |= wifi_cmd(CID_BSS_ACTIVATE_CTRL, true, act, sizeof(act), 0, NULL, 0, 0, out, len, &o);
	wl.init = (err == 0);
	say("init: %s (basic config, NVRAM, domain x2, DBDC off, RX filter, BSS 0 active). Next: wifi scan\n",
	    wl.init ? "done" : "a command was not taken");
	return o;
}

static const uint8_t *
ie_find(const uint8_t *ie, uint32_t len, uint8_t id, const uint8_t *oui4)
{
	for (uint32_t i = 0; i + 2 <= len && i + 2 + ie[i + 1] <= len; i += 2 + ie[i + 1]) {
		if (ie[i] == id && (oui4 == NULL || (ie[i + 1] >= 4 && memcmp(ie + i + 2, oui4, 4) == 0))) {
			return ie + i;
		}
	}
	return NULL;
}

/* Beacon / probe response into the scan table (newest data wins, keyed by BSSID). */
static void
scan_frame(const uint8_t *b, const uint8_t *f, uint32_t flen)
{
	if (flen < 36) {
		return;
	}
	const uint8_t *bssid = f + 16, *ie = f + 36;
	uint32_t ielen = flen - 36;
	struct wl_bss *e = NULL;
	for (uint32_t i = 0; i < wl.nbss; i++) {
		if (memcmp(wl.bss[i].bssid, bssid, 6) == 0) {
			e = &wl.bss[i];
			break;
		}
	}
	if (e == NULL) {
		if (wl.nbss < WL_MAX_BSS) {
			e = &wl.bss[wl.nbss++];
		} else {
			for (uint32_t i = 0; i < WL_MAX_BSS; i++) {     /* replace the oldest */
				if (e == NULL || wl.bss[i].seen < e->seen) {
					e = &wl.bss[i];
				}
			}
		}
		bzero(e, sizeof(*e));
		memcpy(e->bssid, bssid, 6);
	}
	e->seen = wl.scanGen;
	e->rssi = (int8_t)rx_rssi(b);
	e->ch = b[5] > 241 ? (uint8_t)(b[5] - 241) : b[5];
	e->bintval = (uint16_t)(f[32] | f[33] << 8);
	e->capinfo = (uint16_t)(f[34] | f[35] << 8);
	const uint8_t *ds = ie_find(ie, ielen, 3, NULL), *ssid = ie_find(ie, ielen, 0, NULL);
	if (ds && ds[1] >= 1) {
		e->ch = ds[2];
	}
	if (ssid && ssid[1] <= 32 && (ssid[1] > 0 || e->ssid_len == 0)) {
		e->ssid_len = ssid[1];
		memcpy(e->ssid, ssid + 2, ssid[1]);
	}
	e->ie_len = (uint16_t)MIN(ielen, (uint32_t)WL_IE_MAX);
	memcpy(e->ie, ie, e->ie_len);
}

/* Other management frames and EAPOL: only counted for now (no join yet). */
static uint32_t wl_otherMgmt, wl_eapol;

static void
join_frame(__unused const uint8_t *b, __unused const uint8_t *f, __unused uint32_t flen)
{
	wl_otherMgmt++;
}

static void
eapol_rx(__unused const uint8_t *eth, __unused uint32_t len)
{
	wl_eapol++;
}

/* RX dispatch for frames and data (called from rx_next). */
static void
wifi_frame(const uint8_t *b, uint32_t n, uint32_t foff, int kind)
{
	const uint8_t *f = b + foff;
	uint32_t total = MIN((uint32_t)(b[0] | b[1] << 8), n), flen = total - foff;
	if (b[6] & 0x80) {                              /* translated to 802.3: DA, SA, type */
		wl.dataFrames++;
		if (flen >= 14 && f[12] == 0x88 && f[13] == 0x8e) {
			eapol_rx(f, flen);
		}
		return;
	}
	if (kind != PKT_FRAME || flen < 24) {
		wl.dataFrames++;
		return;
	}
	wl.frames++;
	uint8_t fc = f[0];
	if (fc == 0x80 || fc == 0x50) {
		scan_frame(b, f, flen);
	} else if ((fc & 0x0c) == 0) {
		join_frame(b, f, flen);
	} else if ((fc & 0x0c) == 0x08 && flen >= 24 + 8 + 4) {
		/* 802.11 data (not translated): LLC/SNAP AA AA 03 00 00 00, type 0x888E = EAPOL */
		uint32_t h = 24 + ((fc & 0x80) ? 2 : 0);
		const uint8_t *l = f + h;
		if (flen >= h + 8 && l[0] == 0xaa && l[1] == 0xaa && l[6] == 0x88 && l[7] == 0x8e) {
			uint8_t eth[14 + 512];
			uint32_t plen = MIN(flen - h - 8, (uint32_t)512);
			memcpy(eth, f + 4, 6);                  /* DA = addr1 */
			memcpy(eth + 6, f + 10, 6);             /* SA = addr2 (from the AP, to-DS 0) */
			eth[12] = 0x88;
			eth[13] = 0x8e;
			memcpy(eth + 14, l + 8, plen);
			eapol_rx(eth, 14 + plen);
		}
	}
}

static const char *
bss_security(const struct wl_bss *e)
{
	static const uint8_t wpa_oui[4] = { 0x00, 0x50, 0xf2, 0x01 };
	const uint8_t *rsn = ie_find(e->ie, e->ie_len, 48, NULL);
	if (rsn && rsn[1] >= 20) {
		/* AKM list after version (2), group (4), pairwise count + suites */
		uint32_t pc = rsn[8] | rsn[9] << 8, a = 10 + 4 * pc;
		bool psk = false, sae = false, eap = false;
		if (a + 2 <= (uint32_t)rsn[1] + 2) {
			uint32_t ac = rsn[a] | rsn[a + 1] << 8;
			for (uint32_t i = 0; i < ac && a + 2 + 4 * i + 4 <= (uint32_t)rsn[1] + 2; i++) {
				uint8_t t = rsn[a + 2 + 4 * i + 3];
				psk |= (t == 2 || t == 6);
				sae |= (t == 8);
				eap |= (t == 1 || t == 5);
			}
		}
		return psk && sae ? "WPA2/WPA3" : psk ? "WPA2-PSK" : sae ? "WPA3-SAE" : eap ? "WPA2-EAP" : "RSN?";
	}
	if (ie_find(e->ie, e->ie_len, 0xdd, wpa_oui)) {
		return "WPA";
	}
	return (e->capinfo & 0x10) ? "WEP" : "open";
}

static int
wifi_do_scan(const char *ssid, bool only2g, char *out, size_t len, size_t *po)
{
	static uint8_t req[1188];
	static uint8_t ev[RX_BUF_LEN];
	size_t o = *po;
	bzero(req, sizeof(req));
	req[0] = ++wl.scanSeq;
	req[1] = 0;                                     /* BSS 0 */
	req[2] = 1;                                     /* active */
	req[3] = 1;                                     /* wildcard SSID */
	if (ssid && *ssid) {
		uint32_t sl = (uint32_t)MIN(strlen(ssid), (size_t)32);
		req[3] = 4;                                 /* specified + wildcard */
		req[4] = 1;
		memcpy(req + 8, &sl, 4);
		memcpy(req + 12, ssid, sl);
	}
	req[6] = 0x02;                                  /* DBDC scan disabled */
	req[7] = 1;                                     /* auVersion */
	if (only2g) {
		req[158] = 4;                           /* specified channels 1-13 */
		req[159] = 13;
		for (int c = 1; c <= 13; c++) {
			req[160 + 2 * (c - 1)] = 1;
			req[161 + 2 * (c - 1)] = (uint8_t)c;
		}
	}
	memset(req + 1112, 0xff, 6);                    /* aucBSSID */
	wl.scanGen++;
	uint32_t frames0 = wl.frames;
	int ret = -1;
	if (wifi_cmd(CID_SCAN_REQ_V2, true, req, sizeof(req), 0, NULL, 0, 0, out, len, &o) != 0) {
		goto done;
	}
	for (uint32_t t = 0; t < 15000;) {
		int kind;
		uint32_t got = rx_next(ev, sizeof(ev), 10000, &kind);
		if (got == 0) {
			t += 10;
			continue;
		}
		if (kind == PKT_EVENT) {
			uint8_t *e = ev + 20, *d = ev + 32;
			if (e[4] == EID_SCAN_DONE && d[0] == wl.scanSeq) {
				say("scan: done (state %u, %u channels, %u beacons/responses received)\n", d[5], d[4],
				    wl.frames - frames0);
				ret = 0;
				goto done;
			}
			if (e[4] != 0x07) {
				say("scan: event 0x%02x (%u bytes)\n", e[4], got);
			}
		}
	}
	say("scan: no SCAN_DONE in 15 s (%u frames received)\n", wl.frames - frames0);
done:
	*po = o;
	return ret;
}

static size_t
wifi_scan(const char *args, char *out, size_t len, size_t o)
{
	if (!wl.init) {
		say("scan: wifi init first\n");
		return o;
	}
	while (*args == ' ') {
		args++;
	}
	bool only2g = strncmp(args, "2g", 2) == 0;
	if (only2g) {
		args += 2;
		while (*args == ' ') {
			args++;
		}
	}
	(void)wifi_do_scan(args, only2g, out, len, &o);
	/* strongest first */
	uint8_t order[WL_MAX_BSS], m = 0;
	for (uint32_t i = 0; i < wl.nbss; i++) {
		if (wl.bss[i].seen == wl.scanGen) {
			order[m++] = (uint8_t)i;
		}
	}
	for (uint32_t i = 1; i < m; i++) {
		for (uint32_t j = i; j > 0 && wl.bss[order[j]].rssi > wl.bss[order[j - 1]].rssi; j--) {
			uint8_t t = order[j];
			order[j] = order[j - 1];
			order[j - 1] = t;
		}
	}
	say("scan: %u networks\n", m);
	for (uint32_t i = 0; i < m; i++) {
		const struct wl_bss *e = &wl.bss[order[i]];
		say("%4d dBm ch %3u %02x:%02x:%02x:%02x:%02x:%02x %-9s %.*s\n", e->rssi, e->ch, e->bssid[0], e->bssid[1],
		    e->bssid[2], e->bssid[3], e->bssid[4], e->bssid[5], bss_security(e), e->ssid_len, e->ssid);
	}
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
	} else if (strncmp(args, "wmt", 3) == 0) {
		printf("VIVA WIFI: WMT over STP/BTIF\n");
		o = wifi_wmt(out, len, o);
		printf("%s", out);
	} else if (strncmp(args, "init", 4) == 0) {
		printf("VIVA WIFI: station init\n");
		o = wifi_init(out, len, o);
		printf("%s", out);
	} else if (strncmp(args, "scan", 4) == 0) {
		o = wifi_scan(args + 4, out, len, o);
		printf("%s", out);
	} else if (strncmp(args, "cap", 3) == 0) {
		o = wifi_cap(out, len, o);
		printf("%s", out);
	} else if (strncmp(args, "fw", 2) == 0) {
		printf("VIVA WIFI: firmware download\n");
		o = wifi_fw(out, len, o);
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
