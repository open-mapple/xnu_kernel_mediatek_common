/*
 * MT6781 USB, step 1: read-only state probe.
 *
 * The phone's USB is a MediaTek-flavoured Mentor MUSB (DT "mediatek,mt6781-usb20",
 * 0x11200000, 16 endpoints) with a generic T-PHY v1 u2 port at 0x11ca0300; its
 * "usb0" clock gate is in infracfg_ao (0x10001000, always on). LK drives this
 * controller for fastboot. The probe reports the state it leaves behind and
 * never writes a register. It prints what it has before every read that could
 * hang on a gated block, so a hang still shows how far it got.
 *
 * Boot-arg mp_usblink: 0 (default) off, 1 probe, 2 probe then panic with the
 * summary (the panic screen keeps it visible), 3 probe then start the
 * "Mapple link" (step 2, below).
 */
#if defined(ARM64_BOARD_CONFIG_MT6781)
#include <sys/param.h>
#include <sys/systm.h>
#include <kern/debug.h>
#include <pexpert/pexpert.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/sched_prim.h>
#include <kern/locks.h>
#include <kern/kalloc.h>
#include <machine/machine_routines.h>
#include <sys/proc.h>
#include <sys/vnode.h>
#include <sys/vnode_internal.h>
#include <sys/fcntl.h>
#include <sys/uio.h>
#include <sys/stat.h>
#include <miscfs/devfs/devfs.h>
#include <sys/conf.h>

extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);  /* not exported to the BSD layer */
void usblink_probe(void);
static void usblink_start(vm_offset_t tphy, vm_offset_t musb);

#define INFRA_AO_PA     0x10001000u
#define INFRA_CG_STA0   0x90
#define INFRA_CG_STA1   0x94
#define INFRA_CG_STA2   0xac
#define INFRA_CG_STA3   0xc8

#define TPHY_PA         0x11ca0000u
#define U2PORT          0x300           /* u2port0 inside the T-PHY block */
#define U3P_USBPHYACR0  0x000
#define U3P_USBPHYACR6  0x018
#define U3P_U2PHYACR4   0x020
#define U3P_U2PHYDTM0   0x068
#define U3P_U2PHYDTM1   0x06c

#define MUSB_PA         0x11200000u
#define MUSB_FADDR      0x00            /* 8 */
#define MUSB_POWER      0x01            /* 8: bit6 SOFTCONN, bit5 HSENAB, bit4 HSMODE */
#define MUSB_INTRTXE    0x06            /* 16 */
#define MUSB_INTRRXE    0x08            /* 16 */
#define MUSB_INTRUSBE   0x0b            /* 8 */
#define MUSB_DEVCTL     0x60            /* 8: bit0 SESSION, bit2 HOSTMODE, bit7 BDEVICE */
#define MUSB_HWVERS     0x6c            /* 16 */
#define MUSB_EPINFO     0x78            /* 8: [7:4] rx eps, [3:0] tx eps */
#define MTK_L1INTM      0xa4            /* 32, MediaTek level-1 interrupt mask */

static inline uint32_t rd32(vm_offset_t b, uint32_t o) { return *(volatile uint32_t *)(b + o); }
static inline uint16_t rd16(vm_offset_t b, uint32_t o) { return *(volatile uint16_t *)(b + o); }
static inline uint8_t rd8(vm_offset_t b, uint32_t o) { return *(volatile uint8_t *)(b + o); }

void
usblink_probe(void)
{
	uint32_t mode = 0;
#if DEVELOPMENT || DEBUG        /* the link and its commands are development tools: off in a release kernel */
	PE_parse_boot_argn_alias("mp_usblink", "vivausb", &mode, sizeof(mode));
#endif
	if (mode == 0) {
		return;
	}

	vm_offset_t infra = ml_io_map(INFRA_AO_PA, 0x1000);
	uint32_t cg0 = rd32(infra, INFRA_CG_STA0), cg1 = rd32(infra, INFRA_CG_STA1);
	uint32_t cg2 = rd32(infra, INFRA_CG_STA2), cg3 = rd32(infra, INFRA_CG_STA3);
	printf("USB: infra cg sta 0x%08x 0x%08x 0x%08x 0x%08x (bit set = gated)\n", cg0, cg1, cg2, cg3);

	printf("USB: reading T-PHY u2 port...\n");
	vm_offset_t tphy = ml_io_map(TPHY_PA, 0x1000) + U2PORT;
	uint32_t acr0 = rd32(tphy, U3P_USBPHYACR0), acr6 = rd32(tphy, U3P_USBPHYACR6);
	uint32_t acr4 = rd32(tphy, U3P_U2PHYACR4), dtm0 = rd32(tphy, U3P_U2PHYDTM0);
	uint32_t dtm1 = rd32(tphy, U3P_U2PHYDTM1);
	printf("USB: phy acr0 0x%08x acr6 0x%08x acr4 0x%08x dtm0 0x%08x dtm1 0x%08x\n",
	    acr0, acr6, acr4, dtm0, dtm1);

	printf("USB: reading MUSB...\n");
	vm_offset_t musb = ml_io_map(MUSB_PA, 0x1000);
	uint8_t faddr = rd8(musb, MUSB_FADDR), power = rd8(musb, MUSB_POWER);
	uint8_t devctl = rd8(musb, MUSB_DEVCTL), intrusbe = rd8(musb, MUSB_INTRUSBE);
	uint16_t txe = rd16(musb, MUSB_INTRTXE), rxe = rd16(musb, MUSB_INTRRXE);
	uint16_t hwvers = rd16(musb, MUSB_HWVERS);
	uint8_t epinfo = rd8(musb, MUSB_EPINFO);
	uint32_t l1intm = rd32(musb, MTK_L1INTM);
	printf("USB: musb hwvers 0x%04x epinfo 0x%02x faddr 0x%02x power 0x%02x devctl 0x%02x\n",
	    hwvers, epinfo, faddr, power, devctl);
	printf("USB: musb intrusbe 0x%02x txe 0x%04x rxe 0x%04x l1intm 0x%08x\n",
	    intrusbe, txe, rxe, l1intm);

	if (mode == 2) {
		panic("USB probe: cg %08x %08x %08x %08x | phy %08x %08x %08x %08x %08x | "
		    "musb hw %04x ep %02x fa %02x pw %02x dc %02x ie %02x tx %04x rx %04x l1 %08x",
		    cg0, cg1, cg2, cg3, acr0, acr6, acr4, dtm0, dtm1,
		    hwvers, epinfo, faddr, power, devctl, intrusbe, txe, rxe, l1intm);
	}
	if (mode == 3) {
		usblink_start(tphy, musb);
	}
}
/*
 * Step 2: "Mapple link" (mp_usblink=3), a vendor-class USB device for logs and
 * commands. One interface, bulk IN 0x81 and bulk OUT 0x01. Every IN packet
 * is one frame: 'V', type, u16 payload length (little endian), payload.
 * Types: 'L' console log bytes (the ring in bsd/dev/mtk_ufs.c, from the
 * start of the boot), 'R' command reply text, 'E' end of a reply, 'F' file
 * data ("pull", before the reply's text), 'O' output of programs run by
 * mappled. Every OUT packet is one text command (see vl_command), except while
 * a "push" receives its file: then OUT packets are raw file data.
 * Host side: tools/mapple-link.py.
 *
 * Running programs: /dev/mapplelink (root only, created once the root is
 * mounted) connects the userspace daemon mappled (tools/mappled): "run ..." and
 * "mappled ..." commands are queued as lines "ID TEXT" for mappled's read();
 * whatever mappled writes comes out as 'O' frames.
 *
 * PHY: LK leaves the T-PHY u2 port forced into suspend; it is woken with the
 * MediaTek usb20 phy "recover" sequence (as Linux phy-mtk-tphy u2 init) and
 * VBUS/session/ID are forced to "device" in U2PHYDTM1, so no PMIC/charger
 * code is needed. MUSB: peripheral mode, PIO only (no DMA, no cache
 * maintenance), the interrupt line stays masked (MediaTek L1INTM = 0) and a
 * kernel thread polls the status registers, which on MediaTek are
 * write-1-to-clear (Linux musb/mediatek.c). Register layout and the EP0
 * state machine follow Linux drivers/usb/musb (musb_regs.h,
 * musb_gadget_ep0.c). The thread never prints in steady state: its output
 * would feed the log it streams.
 *
 * 0182 (fork/docs/VIVA-USB-PERSONA.md): two personas. "link" (above) from boot; "std" once the daemon mapple-usbd
 * (tools/mappled/mapple-usbd.c) attaches: an iPhone-like composite device 05ac:12a8 with PTP (if0, ep2 bulk + ep3
 * interrupt), usbmux (if1, ep4) and the link (if2, ep1, unchanged). The PTP and usbmux protocols live in mapple-usbd:
 * the kernel moves their transfers through /dev/linkptp and /dev/linkmux and answers ep0 (PTP class requests, the
 * Microsoft OS descriptor that binds Windows' MTP driver, Apple vendor requests). The link counts as "active" only
 * while the host tool sends its heartbeat (vendor request 0x41/0x56, wValue 1, every second; 4 s timeout): only
 * then does the charger keep the 500 mA USB input limit (usblink_host_configured). Boot-arg mp_usblink_std=0: never
 * leave the link persona.
 */
#define U3P_USBPHYACR5  0x014
#define U3P_USBPHYACR7  0x01c

#define MUSB_INTRTX     0x02            /* 16, W1C */
#define MUSB_INTRRX     0x04            /* 16, W1C */
#define MUSB_INTRUSB    0x0a            /* 8, W1C */
#define MUSB_INDEX      0x0e            /* 8 */
#define MUSB_TXMAXP     0x10            /* 16, indexed */
#define MUSB_CSR0       0x12            /* 16, indexed (ep0) */
#define MUSB_TXCSR      0x12            /* 16, indexed */
#define MUSB_RXMAXP     0x14            /* 16, indexed */
#define MUSB_RXCSR      0x16            /* 16, indexed */
#define MUSB_COUNT0     0x18            /* 16, indexed (ep0) */
#define MUSB_RXCOUNT    0x18            /* 16, indexed */
#define MUSB_FIFO(n)    (0x20 + 4 * (n))
#define MUSB_TXFIFOSZ   0x62            /* 8: log2(size / 8) */
#define MUSB_RXFIFOSZ   0x63
#define MUSB_TXFIFOADD  0x64            /* 16: address / 8 */
#define MUSB_RXFIFOADD  0x66

#define POWER_SOFTCONN  0x40
#define POWER_HSENAB    0x20
#define POWER_HSMODE    0x10
#define DEVCTL_SESSION  0x01
#define INTR_SUSPEND    0x01
#define INTR_RESUME     0x02
#define INTR_RESET      0x04
#define INTR_DISCON     0x20

#define CSR0_RXPKTRDY   0x0001
#define CSR0_TXPKTRDY   0x0002
#define CSR0_SENTSTALL  0x0004
#define CSR0_DATAEND    0x0008
#define CSR0_SETUPEND   0x0010
#define CSR0_SENDSTALL  0x0020
#define CSR0_SVDRXPKTRDY 0x0040
#define CSR0_SVDSETUPEND 0x0080

#define TXCSR_TXPKTRDY  0x0001
#define TXCSR_FIFONOTEMPTY 0x0002
#define TXCSR_UNDERRUN  0x0004
#define TXCSR_FLUSHFIFO 0x0008
#define TXCSR_SENTSTALL 0x0020
#define TXCSR_CLRDATATOG 0x0040
#define TXCSR_MODE      0x2000
#define RXCSR_RXPKTRDY  0x0001
#define RXCSR_FLUSHFIFO 0x0010
#define RXCSR_CLRDATATOG 0x0080

#define MUSB_RAMINFO    0x79            /* 8: [3:0] FIFO RAM address bits (32-bit words) */

#define EP1_TXFIFO      64              /* after the fixed 64-byte ep0 FIFO */
#define EP1_RXFIFO      (64 + 512)
#define EP2_TXFIFO      (64 + 1024)     /* std persona: PTP bulk */
#define EP2_RXFIFO      (64 + 1536)
#define EP3_TXFIFO      (64 + 2048)     /* PTP interrupt, 64 bytes */
#define EP4_TXFIFO      (64 + 2112)     /* usbmux bulk */
#define EP4_RXFIFO      (64 + 2624)     /* ends at 3200 */

#define TOPRGU_PA       0x10007000u
#define WDT_MODE        0x00
#define WDT_SWRST       0x14
#define WDT_SWRST_KEY   0x1209

static struct {
	vm_offset_t musb;
	vm_offset_t toprgu;
	int         sleep_chan;
	enum { EP0_IDLE, EP0_IN, EP0_OUT, EP0_STATUS } ep0;
	const uint8_t *in_data;         /* ep0 IN data stage */
	uint32_t    in_left;
	bool        in_zlp;
	uint8_t     setup[8];           /* request whose OUT data stage is being received */
	uint32_t    out_len, out_got;
	uint8_t     new_addr;           /* SET_ADDRESS, applied after the status stage */
	bool        configured;
	bool        hs;
	enum { P_LINK, P_STD } persona;
	bool        std_allowed;        /* boot-arg mp_usblink_std */
	bool        want_std;           /* USBLINK_ATTACH */
	uint8_t     link_if;            /* the link's interface number: 0 (link), 2 (std) */
	char        serial[64];         /* std persona serial number (the UDID, from mapple-usbd) */
	bool        tool_active;        /* host tool heartbeat seen in the last VL_TOOL_MS */
	uint64_t    tool_ms;
	uint64_t    heartbeats;
	uint8_t     ep0_buf[256];
	char        cmd[512];
	char        reply[4096];
	uint32_t    reply_len, reply_pos;
	bool        reply_end;          /* an 'E' frame is still owed */
	uint64_t    log_pos;
	uint64_t    resets, setups, cmds, tx_pkts, rx_pkts, log_bytes;
} vl;

static inline void wr32(vm_offset_t b, uint32_t o, uint32_t v) { *(volatile uint32_t *)(b + o) = v; }
static inline void wr16(vm_offset_t b, uint32_t o, uint16_t v) { *(volatile uint16_t *)(b + o) = v; }
static inline void wr8(vm_offset_t b, uint32_t o, uint8_t v) { *(volatile uint8_t *)(b + o) = v; }
static inline void set32(vm_offset_t b, uint32_t o, uint32_t m) { wr32(b, o, rd32(b, o) | m); }
static inline void clr32(vm_offset_t b, uint32_t o, uint32_t m) { wr32(b, o, rd32(b, o) & ~m); }

extern uint32_t mapple_log_copy(uint64_t *pos, char *out, uint32_t max);   /* bsd/dev/mtk_ufs.c */
extern uint64_t mapple_log_position(void);
extern kern_return_t kernel_thread_start(thread_continue_t, void *, thread_t *);
extern int mtk_pwrap_read(uint32_t addr, uint32_t *value);               /* iokit/Kernel/arm/MTKPwrap.cpp */
extern int mapple_i2c_xfer(uint8_t addr, const uint8_t *wbuf, uint32_t wlen, uint8_t *rbuf, uint32_t rlen); /* MTKIO.cpp */
extern int mapple_i2c_xfer_bus(uint32_t bus, uint8_t addr, const uint8_t *wbuf, uint32_t wlen, uint8_t *rbuf, uint32_t rlen);
extern void bq27z561_link(const char *args, char *out, size_t outLen);   /* iokit/Kernel/arm/BQ27Z561Battery.cpp */
extern void mt6631_wifi_link(const char *args, char *out, size_t outLen);      /* bsd/dev/mt6631_wifi.c */
extern void mtk_bt_link(const char *args, char *out, size_t outLen);           /* bsd/dev/mtk_bt.c */
extern void mapple_gpu_link(const char *args, char *out, size_t outLen);       /* iokit/Kernel/arm/MappleGPU.cpp */
extern void mt6781_cpufreq_link(const char *args, char *out, size_t outLen);   /* osfmk/arm64/mt6781_cpufreq.c */
extern void mapple_disp_link(const char *args, char *out, size_t outLen);      /* iokit/Kernel/arm/MappleFramebuffer.cpp */
extern int mapple_console_muted;                                             /* osfmk/console/serial_console.c */

extern void bq27z561_usb_changed(void);                                  /* BQ27Z561Battery.cpp: poll now */

#define VL_TOOL_MS      4000            /* the link is active this long after the last heartbeat */
static uint64_t vl_now_ms(void);

/*
 * BQ27Z561Battery.cpp: the host tool keeps the link active, so the cable is a computer that wants the USB 500 mA limit.
 * A computer without the tool (or a wall charger) gets the charger's full input limit (0182).
 */
int
usblink_host_configured(void)
{
	return vl.configured && vl.tool_active ? 1 : 0;
}

/* kern.mapple_linkstate (mapple_sysctl.c), USBLINK_STATE: 0 no host, 1 configured (standard), 2 Mapple link active. */
int
usblink_state(void)
{
	return !vl.configured ? 0 : vl.tool_active ? 2 : 1;
}

static void
vl_tool_set(bool active)
{
	if (vl.tool_active == active) {
		return;
	}
	vl.tool_active = active;
	printf("LINK: host tool %s\n", active ? "active (Mapple link)" : "gone");
	bq27z561_usb_changed();
}

static void
vl_sleep_ms(unsigned ms)
{
	uint64_t deadline;
	clock_interval_to_deadline(ms, NSEC_PER_MSEC, &deadline);
	(void)assert_wait_deadline(&vl.sleep_chan, THREAD_UNINT, deadline);
	thread_block(THREAD_CONTINUE_NULL);
}

static void
vl_fifo_write(unsigned ep, const uint8_t *p, uint32_t n)
{
	uint32_t i = 0;
	for (; i + 4 <= n; i += 4) {
		uint32_t w = (uint32_t)p[i] | (uint32_t)p[i + 1] << 8 | (uint32_t)p[i + 2] << 16 | (uint32_t)p[i + 3] << 24;
		wr32(vl.musb, MUSB_FIFO(ep), w);
	}
	for (; i < n; i++) {
		wr8(vl.musb, MUSB_FIFO(ep), p[i]);
	}
}

static void
vl_fifo_read(unsigned ep, uint8_t *p, uint32_t n)
{
	uint32_t i = 0;
	for (; i + 4 <= n; i += 4) {
		uint32_t w = rd32(vl.musb, MUSB_FIFO(ep));
		p[i] = (uint8_t)w; p[i + 1] = (uint8_t)(w >> 8);
		p[i + 2] = (uint8_t)(w >> 16); p[i + 3] = (uint8_t)(w >> 24);
	}
	for (; i < n; i++) {
		p[i] = rd8(vl.musb, MUSB_FIFO(ep));
	}
}

/* Descriptors. VID 0x1209 / PID 0x0002 = pid.codes test ID, not a product. */
static const uint8_t vl_dev_desc[18] = {
	18, 1, 0x00, 0x02, 0x00, 0x00, 0x00, 64,
	0x09, 0x12, 0x02, 0x00, 0x00, 0x01, 1, 2, 3, 1,
};
static const uint8_t vl_qual_desc[10] = { 10, 6, 0x00, 0x02, 0x00, 0x00, 0x00, 64, 1, 0 };
static uint8_t vl_cfg_desc[32] = {
	9, 2, 32, 0, 1, 1, 0, 0x80, 250,                /* config: 1 interface, 500 mA */
	9, 4, 0, 0, 2, 0xff, 0x56, 0x01, 0,             /* vendor interface, 2 endpoints */
	7, 5, 0x81, 2, 0x00, 0x02, 0,                   /* bulk IN 1, 512 */
	7, 5, 0x01, 2, 0x00, 0x02, 0,                   /* bulk OUT 1, 512 */
};
static const char *const vl_strings[] = { NULL, "ios-viva", "Mapple link", "0001" };

/* std persona (0182): 05ac:12a8 like an iPhone, PTP + usbmux + link. */
static const uint8_t vl_std_dev_desc[18] = {
	18, 1, 0x00, 0x02, 0x00, 0x00, 0x00, 64,
	0xac, 0x05, 0xa8, 0x12, 0x01, 0x16, 1, 2, 3, 1,
};
static uint8_t vl_std_cfg_desc[85] = {
	9, 2, 85, 0, 3, 1, 0, 0x80, 250,                /* config: 3 interfaces, 500 mA */
	9, 4, 0, 0, 3, 0x06, 0x01, 0x01, 4,             /* PTP (still image) */
	7, 5, 0x82, 2, 0x00, 0x02, 0,                   /* bulk IN 2 */
	7, 5, 0x02, 2, 0x00, 0x02, 0,                   /* bulk OUT 2 */
	7, 5, 0x83, 3, 64, 0, 8,                        /* interrupt IN 3 */
	9, 4, 1, 0, 2, 0xff, 0xfe, 0x02, 5,             /* usbmux */
	7, 5, 0x84, 2, 0x00, 0x02, 0,                   /* bulk IN 4 */
	7, 5, 0x04, 2, 0x00, 0x02, 0,                   /* bulk OUT 4 */
	9, 4, 2, 0, 2, 0xff, 0x56, 0x01, 6,             /* Mapple link */
	7, 5, 0x81, 2, 0x00, 0x02, 0,                   /* bulk IN 1 */
	7, 5, 0x01, 2, 0x00, 0x02, 0,                   /* bulk OUT 1 */
};
static const uint8_t vl_std_bulk_mps_at[] = { 22, 29, 52, 59, 75, 82 };  /* wMaxPacketSize of the bulk endpoints */
static const char *const vl_std_strings[] = { NULL, "Apple Inc.", "iPhone", NULL /* serial */, "PTP",
	                                      "Apple USB Multiplexor", "Mapple link" };
#define VL_MS_VENDOR    0x50            /* Microsoft OS descriptor vendor code */
static const uint8_t vl_ms_os_string[18] = {
	18, 3, 'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0, VL_MS_VENDOR, 0,
};
static const uint8_t vl_ms_compat_id[40] = {
	40, 0, 0, 0, 0x00, 0x01, 0x04, 0x00, 1, 0, 0, 0, 0, 0, 0, 0,    /* header: length, bcd 1.00, index 4, 1 function */
	0, 1, 'M', 'T', 'P', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   /* interface 0: "MTP" */
};

static uint32_t
vl_mps(void)
{
	return vl.hs ? 512 : 64;
}

static uint32_t
vl_string(unsigned idx, uint8_t *out)
{
	if (idx == 0) {
		out[0] = 4; out[1] = 3; out[2] = 0x09; out[3] = 0x04;   /* en-US */
		return 4;
	}
	const char *str;
	if (vl.persona == P_STD) {
		if (idx == 0xee) {
			memcpy(out, vl_ms_os_string, sizeof(vl_ms_os_string));
			return sizeof(vl_ms_os_string);
		}
		if (idx >= sizeof(vl_std_strings) / sizeof(vl_std_strings[0])) {
			return 0;
		}
		str = idx == 3 ? vl.serial : vl_std_strings[idx];
	} else {
		if (idx >= sizeof(vl_strings) / sizeof(vl_strings[0])) {
			return 0;
		}
		str = vl_strings[idx];
	}
	uint32_t n = 2;
	for (; *str && n < 126; str++) {
		out[n++] = (uint8_t)*str;
		out[n++] = 0;
	}
	out[0] = (uint8_t)n;
	out[1] = 3;
	return n;
}

/* One endpoint pair for the current speed: IN at txfifo (txmps bytes), OUT at rxfifo (0 = no OUT). */
static void
vl_ep_config(unsigned ep, uint32_t txfifo, uint16_t txmps, uint32_t rxfifo)
{
	uint16_t mps = (uint16_t)vl_mps();
	wr8(vl.musb, MUSB_INDEX, (uint8_t)ep);
	wr16(vl.musb, MUSB_TXMAXP, txmps);
	wr8(vl.musb, MUSB_TXFIFOSZ, txmps > 64 ? 6 : 3);        /* 512 or 64 bytes, single buffered */
	wr16(vl.musb, MUSB_TXFIFOADD, (uint16_t)(txfifo >> 3));
	wr16(vl.musb, MUSB_TXCSR, TXCSR_MODE | TXCSR_CLRDATATOG);
	if (rd16(vl.musb, MUSB_TXCSR) & TXCSR_FIFONOTEMPTY) {
		wr16(vl.musb, MUSB_TXCSR, TXCSR_MODE | TXCSR_FLUSHFIFO);
	}
	if (rxfifo != 0) {
		wr16(vl.musb, MUSB_RXMAXP, mps);
		wr8(vl.musb, MUSB_RXFIFOSZ, 6);
		wr16(vl.musb, MUSB_RXFIFOADD, (uint16_t)(rxfifo >> 3));
		wr16(vl.musb, MUSB_RXCSR, RXCSR_FLUSHFIFO | RXCSR_CLRDATATOG);
		wr16(vl.musb, MUSB_RXCSR, RXCSR_FLUSHFIFO | RXCSR_CLRDATATOG);
	}
	wr8(vl.musb, MUSB_INDEX, 0);
}

/* Endpoints of the configuration just set. */
static void
vl_eps_config(void)
{
	uint16_t mps = (uint16_t)vl_mps();
	vl_ep_config(1, EP1_TXFIFO, mps, EP1_RXFIFO);
	if (vl.persona == P_STD) {
		vl_ep_config(2, EP2_TXFIFO, mps, EP2_RXFIFO);
		vl_ep_config(3, EP3_TXFIFO, 64, 0);
		vl_ep_config(4, EP4_TXFIFO, mps, EP4_RXFIFO);
	}
}

/* Next ep0 IN packet; the last one carries DATAEND. Index must be 0. */
static void
vl_ep0_in_next(void)
{
	uint32_t n = MIN(vl.in_left, 64u);
	vl_fifo_write(0, vl.in_data, n);
	vl.in_data += n;
	vl.in_left -= n;
	bool last = vl.in_left == 0 && !(n == 64 && vl.in_zlp);
	if (n < 64) {
		vl.in_zlp = false;
	}
	wr16(vl.musb, MUSB_CSR0, CSR0_TXPKTRDY | (last ? CSR0_DATAEND : 0));
	vl.ep0 = last ? EP0_STATUS : EP0_IN;
}

/* ---- std persona functions (0182): PTP (index 0, ep2 + ep3) and usbmux (index 1, ep4) ---- */

#define VF_RING         (128u << 10)    /* bytes queued per direction */
#define VF_MARKS        256             /* transfer ends queued per direction */
#define VF_WRITE_MAX    (64u << 10)
#define VF_EVENTS       8

struct vfn {
	const char *name;
	uint8_t     ep;                     /* bulk IN and OUT endpoint */
	bool        open;
	int         pending_err;            /* ECONNRESET / ECANCELED, returned once by read() */
	uint32_t    gen;                    /* bumped by every reset: writers waiting for room give up */
	uint8_t    *rx, *tx;                /* rings, VF_RING each (allocated with the device nodes) */
	uint32_t    rx_head, rx_tail, tx_head, tx_tail;     /* free-running */
	uint32_t    rx_mark[VF_MARKS], tx_mark[VF_MARKS];   /* ring positions where transfers end */
	uint32_t    rx_mhead, rx_mtail, tx_mhead, tx_mtail;
	uint32_t    rx_xfer;                /* bytes of the OUT transfer in progress */
	bool        tx_zlp;                 /* the last IN transfer ended on a full packet */
	uint32_t    tx_cont_left;           /* ptp: bytes of the container being written still to come */
	uint64_t    rx_bytes, tx_bytes, rx_xfers, tx_xfers, resets;
};
static struct vfn vfn[2] = { { .name = "linkptp", .ep = 2 }, { .name = "linkmux", .ep = 4 } };
static struct {
	uint8_t     data[64];
	uint32_t    len;
} vf_ev[VF_EVENTS];
static uint32_t vf_ev_head, vf_ev_tail;
LCK_GRP_DECLARE(vf_grp, "mp_usblink");
LCK_MTX_DECLARE(vf_mtx, &vf_grp);

/* Drop everything queued in both directions; read() reports err once. vf_mtx held. */
static void
vf_reset_locked(struct vfn *f, int err)
{
	f->rx_tail = f->rx_head;
	f->rx_mtail = f->rx_mhead;
	f->rx_xfer = 0;
	f->tx_tail = f->tx_head;
	f->tx_mtail = f->tx_mhead;
	f->tx_zlp = false;
	f->tx_cont_left = 0;
	f->pending_err = err;
	f->gen++;
	f->resets++;
	wakeup(&f->rx_head);
	wakeup(&f->tx_tail);
}

static void
vf_reset_all(int err)
{
	lck_mtx_lock(&vf_mtx);
	for (unsigned i = 0; i < 2; i++) {
		vf_reset_locked(&vfn[i], err);
	}
	vf_ev_tail = vf_ev_head;
	lck_mtx_unlock(&vf_mtx);
}

/* PTP cancel / device reset (ep0): the transaction in flight is dropped on both sides. */
static void
vf_ptp_cancel(int err)
{
	lck_mtx_lock(&vf_mtx);
	vf_reset_locked(&vfn[0], err);
	lck_mtx_unlock(&vf_mtx);
	wr8(vl.musb, MUSB_INDEX, 2);
	wr16(vl.musb, MUSB_TXCSR, TXCSR_MODE | TXCSR_FLUSHFIFO);
	wr8(vl.musb, MUSB_INDEX, 0);
}

static bool
vf_ptp_busy(void)
{
	return vfn[0].pending_err == ECANCELED;
}

/* A control request with an OUT data stage, all data received (vl.setup, vl.ep0_buf). */
static void
vl_setup_out_done(void)
{
	uint8_t type = vl.setup[0], req = vl.setup[1];
	if (type == 0x21 && req == 0x64) {              /* PTP CANCEL_REQUEST: 0x4001 + transaction id */
		vf_ptp_cancel(ECANCELED);
	}
	/* vendor OUT requests with data (Apple's): accepted, ignored */
}

/* SETUP packet on ep0 (index 0, RXPKTRDY set). */
static void
vl_setup(void)
{
	uint8_t st[8];
	uint16_t count = rd16(vl.musb, MUSB_COUNT0);
	if (count != 8) {
		wr16(vl.musb, MUSB_CSR0, CSR0_SVDRXPKTRDY | CSR0_SENDSTALL);
		return;
	}
	vl_fifo_read(0, st, 8);
	vl.setups++;
	uint8_t type = st[0], req = st[1], kind = type & 0x60, rcpt = type & 0x1f;
	uint16_t value = (uint16_t)(st[2] | st[3] << 8);
	uint16_t index = (uint16_t)(st[4] | st[5] << 8);
	uint16_t length = (uint16_t)(st[6] | st[7] << 8);
	const uint8_t *data = NULL;
	uint32_t n = 0;
	bool ok = true, std = vl.persona == P_STD;

	if (kind == 0x40 && req == 0x56 && rcpt == 1 && (index & 0xff) == vl.link_if && length == 0) {
		/* host tool heartbeat (1) / bye (0) */
		vl.heartbeats++;
		vl.tool_ms = vl_now_ms();
		vl_tool_set(value != 0 && vl.configured);
	} else if (kind == 0x40 && type == 0xc0 && req == VL_MS_VENDOR && std && index == 4) {
		data = vl_ms_compat_id; n = sizeof(vl_ms_compat_id);   /* Windows: interface 0 is MTP */
	} else if (kind == 0x40) {
		ok = !(type & 0x80);    /* Apple's vendor OUT requests (charging current, ...): acked, no effect */
	} else if (kind == 0x20 && std && rcpt == 1 && (index & 0xff) == 0) {
		switch (req) {
		case 0x64:                                      /* CANCEL_REQUEST, 6 data bytes */
			ok = !(type & 0x80) && length == 6;
			break;
		case 0x66:                                      /* DEVICE_RESET */
			ok = type == 0x21 && length == 0;
			if (ok) {
				vf_ptp_cancel(ECONNRESET);
			}
			break;
		case 0x67:                                      /* GET_DEVICE_STATUS: OK or busy */
			vl.ep0_buf[0] = 4; vl.ep0_buf[1] = 0;
			vl.ep0_buf[2] = vf_ptp_busy() ? 0x19 : 0x01; vl.ep0_buf[3] = 0x20;
			data = vl.ep0_buf; n = 4;
			ok = (type & 0x80) != 0;
			break;
		default:
			ok = false;
			break;
		}
	} else if (kind != 0) {
		ok = false;
	} else if (req == 6 && (type & 0x80)) {         /* GET_DESCRIPTOR */
		vl.hs = (rd8(vl.musb, MUSB_POWER) & POWER_HSMODE) != 0;
		switch (value >> 8) {
		case 1:
			data = std ? vl_std_dev_desc : vl_dev_desc; n = sizeof(vl_dev_desc);
			break;
		case 2:
			if (std) {
				for (unsigned i = 0; i < sizeof(vl_std_bulk_mps_at); i++) {
					vl_std_cfg_desc[vl_std_bulk_mps_at[i]] = (uint8_t)vl_mps();
					vl_std_cfg_desc[vl_std_bulk_mps_at[i] + 1] = (uint8_t)(vl_mps() >> 8);
				}
				data = vl_std_cfg_desc; n = sizeof(vl_std_cfg_desc);
			} else {
				vl_cfg_desc[22] = vl_cfg_desc[29] = (uint8_t)vl_mps();
				vl_cfg_desc[23] = vl_cfg_desc[30] = (uint8_t)(vl_mps() >> 8);
				data = vl_cfg_desc; n = sizeof(vl_cfg_desc);
			}
			break;
		case 3: n = vl_string(value & 0xff, vl.ep0_buf); data = vl.ep0_buf; ok = n != 0; break;
		case 6: data = vl_qual_desc; n = sizeof(vl_qual_desc); break;
		default: ok = false; break;
		}
	} else if (req == 5 && type == 0x00) {          /* SET_ADDRESS */
		vl.new_addr = (uint8_t)(value & 0x7f);
	} else if (req == 9 && type == 0x00) {          /* SET_CONFIGURATION */
		vl.configured = value != 0;
		if (!vl.configured) {
			vl_tool_set(false);
		}
		vl.hs = (rd8(vl.musb, MUSB_POWER) & POWER_HSMODE) != 0;
		vl_eps_config();
		vf_reset_all(ECONNRESET);
		if (vl.configured) {
			printf("LINK: configured, %s speed, %s persona\n", vl.hs ? "high" : "full", std ? "std" : "link");
		}
	} else if (req == 8 && (type & 0x80)) {         /* GET_CONFIGURATION */
		vl.ep0_buf[0] = vl.configured ? 1 : 0; data = vl.ep0_buf; n = 1;
	} else if (req == 0 && (type & 0x80)) {         /* GET_STATUS */
		vl.ep0_buf[0] = 0; vl.ep0_buf[1] = 0; data = vl.ep0_buf; n = 2;
	} else if (req == 10 && (type & 0x80)) {        /* GET_INTERFACE */
		vl.ep0_buf[0] = 0; data = vl.ep0_buf; n = 1;
	} else if (req == 11 && type == 0x01) {         /* SET_INTERFACE */
	} else if (req == 1 && type == 0x02) {          /* CLEAR_FEATURE(ENDPOINT_HALT): reset the toggle */
		unsigned ep = index & 0x7f;
		if (ep >= 1 && ep <= (std ? 4u : 1u)) {
			wr8(vl.musb, MUSB_INDEX, (uint8_t)ep);
			if (index & 0x80) {
				wr16(vl.musb, MUSB_TXCSR, TXCSR_MODE | TXCSR_CLRDATATOG);
			} else {
				wr16(vl.musb, MUSB_RXCSR, RXCSR_CLRDATATOG);
			}
			wr8(vl.musb, MUSB_INDEX, 0);
		}
	} else {
		ok = false;
	}
	if (ok && !(type & 0x80) && length != 0) {     /* OUT data stage: vl_ep0 collects it */
		if (length > sizeof(vl.ep0_buf)) {
			ok = false;
		} else {
			memcpy(vl.setup, st, 8);
			vl.out_len = length;
			vl.out_got = 0;
			wr16(vl.musb, MUSB_CSR0, CSR0_SVDRXPKTRDY);
			vl.ep0 = EP0_OUT;
			return;
		}
	}
	if (!ok) {
		wr16(vl.musb, MUSB_CSR0, CSR0_SVDRXPKTRDY | CSR0_SENDSTALL);
		vl.ep0 = EP0_IDLE;
		return;
	}
	if (!(type & 0x80)) {
		wr16(vl.musb, MUSB_CSR0, CSR0_SVDRXPKTRDY | CSR0_DATAEND);
		vl.ep0 = EP0_STATUS;
		return;
	}
	if (n > length) {
		n = length;
	}
	vl.in_data = data;
	vl.in_left = n;
	vl.in_zlp = n < length;                         /* a short transfer ending on 64 needs a ZLP */
	wr16(vl.musb, MUSB_CSR0, CSR0_SVDRXPKTRDY);
	vl_ep0_in_next();
}

/* Endpoint 0 interrupt (INTRTX bit 0). */
static void
vl_ep0(void)
{
	wr8(vl.musb, MUSB_INDEX, 0);
	uint16_t csr = rd16(vl.musb, MUSB_CSR0);
	if (csr & CSR0_SENTSTALL) {
		wr16(vl.musb, MUSB_CSR0, 0);
		vl.ep0 = EP0_IDLE;
		csr = rd16(vl.musb, MUSB_CSR0);
	}
	if (csr & CSR0_SETUPEND) {
		wr16(vl.musb, MUSB_CSR0, CSR0_SVDSETUPEND);
		vl.ep0 = EP0_IDLE;
		csr = rd16(vl.musb, MUSB_CSR0);
	}
	switch (vl.ep0) {
	case EP0_IN:
		if (!(csr & CSR0_TXPKTRDY)) {
			vl_ep0_in_next();
		}
		return;
	case EP0_OUT:                   /* data stage of a control write */
		if (csr & CSR0_RXPKTRDY) {
			uint32_t n = rd16(vl.musb, MUSB_COUNT0);
			uint32_t take = MIN(n, vl.out_len - vl.out_got);
			vl_fifo_read(0, vl.ep0_buf + vl.out_got, take);
			vl.out_got += take;
			if (vl.out_got >= vl.out_len || n < 64) {
				wr16(vl.musb, MUSB_CSR0, CSR0_SVDRXPKTRDY | CSR0_DATAEND);
				vl.ep0 = EP0_STATUS;
				vl_setup_out_done();
			} else {
				wr16(vl.musb, MUSB_CSR0, CSR0_SVDRXPKTRDY);
			}
		}
		return;
	case EP0_STATUS:                /* status stage done */
		if (vl.new_addr) {
			wr8(vl.musb, MUSB_FADDR, vl.new_addr);
			vl.new_addr = 0;
		}
		vl.ep0 = EP0_IDLE;
		break;
	case EP0_IDLE:
		break;
	}
	if (csr & CSR0_RXPKTRDY) {
		vl_setup();
	}
}

/* Queue reply text (truncated to the buffer). */
static void
vl_reply(const char *fmt, ...) __printflike(1, 2);

static void
vl_reply(const char *fmt, ...)
{
	va_list ap;
	if (vl.reply_len >= sizeof(vl.reply) - 1) {
		return;
	}
	va_start(ap, fmt);
	int n = vsnprintf(vl.reply + vl.reply_len, sizeof(vl.reply) - vl.reply_len, fmt, ap);
	va_end(ap);
	if (n > 0) {
		vl.reply_len = MIN(vl.reply_len + (uint32_t)n, (uint32_t)sizeof(vl.reply) - 1);
	}
}

static bool
vl_arg(char **p, uint64_t *v)
{
	char *s = *p;
	while (*s == ' ') {
		s++;
	}
	if (*s == 0) {
		return false;
	}
	char *end;
	*v = strtouq(s, &end, 0);
	if (end == s) {
		return false;
	}
	*p = end;
	return true;
}

/* ---- files: push (host -> phone) and pull (phone -> host) ---- */

extern int mapple_kern_rename(const char *from, const char *to);        /* bsd/vfs/vfs_syscalls.c */
extern int mapple_kern_mkdir(const char *path, int mode);

#define VL_PUSH_BUF     (64u << 10)
#define VL_PUSH_MAX     (512ull << 20)
#define VL_PUSH_IDLE_MS 10000

static struct {
	bool        active;
	vnode_t     vp;
	char        path[256], tmp[272];
	uint64_t    size, got;
	uint32_t    fnv, want;
	uint8_t    *buf;
	uint32_t    buf_len;
	uint64_t    last_ms;
	int         error;
} vpush;

static struct {
	bool        active;
	vnode_t     vp;
	uint64_t    off, size;
	uint32_t    fnv;
} vpull;

static uint64_t
vl_now_ms(void)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / NSEC_PER_MSEC;
}

static uint32_t
vl_fnv(uint32_t h, const uint8_t *p, uint32_t n)
{
	for (uint32_t i = 0; i < n; i++) {
		h = (h ^ p[i]) * 16777619u;
	}
	return h;
}

/* A path we accept: absolute, no "..", not under /dev. */
static bool
vl_path_ok(const char *p)
{
	size_t n = strlen(p);
	if (p[0] != '/' || n >= sizeof(vpush.path) || strncmp(p, "/dev/", 5) == 0) {
		return false;
	}
	for (size_t i = 0; i + 2 < n + 1; i++) {
		if (p[i] == '/' && p[i + 1] == '.' && p[i + 2] == '.' && (p[i + 3] == '/' || p[i + 3] == 0)) {
			return false;
		}
	}
	return true;
}

static void
vl_push_flush(void)
{
	if (vpush.buf_len == 0 || vpush.error) {
		vpush.buf_len = 0;
		return;
	}
	vfs_context_t ctx = vfs_context_kernel();
	int resid = 0;
	vpush.error = vn_rdwr(UIO_WRITE, vpush.vp, (caddr_t)vpush.buf, (int)vpush.buf_len,
	    (off_t)(vpush.got - vpush.buf_len), UIO_SYSSPACE, IO_NOAUTH | IO_UNIT, vfs_context_ucred(ctx), &resid,
	    vfs_context_proc(ctx));
	if (vpush.error == 0 && resid != 0) {
		vpush.error = EIO;
	}
	vpush.buf_len = 0;
}

/* End of a push: the file is renamed into place only when complete and its checksum matches. */
static void
vl_push_finish(const char *why)
{
	vfs_context_t ctx = vfs_context_kernel();
	vl_push_flush();
	vnode_close(vpush.vp, FWRITE, ctx);
	vpush.vp = NULLVP;
	bool complete = why == NULL && vpush.error == 0 && vpush.got == vpush.size;
	if (complete && vpush.fnv != vpush.want) {
		why = "checksum mismatch";
	} else if (why == NULL && vpush.error) {
		why = "write error";
	}
	int err = 0;
	if (why == NULL) {
		err = mapple_kern_rename(vpush.tmp, vpush.path);
		if (err) {
			why = "rename failed";
		}
	}
	vl.reply_len = vl.reply_pos = 0;
	if (why == NULL) {
		vl_reply("push %s: %llu bytes, fnv1a 0x%08x, ok\n", vpush.path, vpush.got, vpush.fnv);
	} else {
		vl_reply("push %s: FAILED (%s, error %d): %llu of %llu bytes, fnv1a 0x%08x (expected 0x%08x); left in %s\n",
		    vpush.path, why, vpush.error ? vpush.error : err, vpush.got, vpush.size, vpush.fnv, vpush.want,
		    vpush.tmp);
	}
	vl.reply_end = true;
	kfree_data(vpush.buf, VL_PUSH_BUF);
	vpush.buf = NULL;
	vpush.active = false;
}

/* "push SIZE MODE FNV PATH": the next SIZE bytes of OUT data go to PATH.linkpush, then renamed to PATH. */
static void
vl_push_start(char *rest)
{
	uint64_t size, mode, fnv;
	if (!vl_arg(&rest, &size) || !vl_arg(&rest, &mode) || !vl_arg(&rest, &fnv)) {
		vl_reply("usage: push SIZE MODE FNV1A PATH (tools/mapple-link.py push LOCAL PATH [MODE])\n");
		return;
	}
	while (*rest == ' ') {
		rest++;
	}
	if (size == 0 || size > VL_PUSH_MAX || !vl_path_ok(rest)) {
		vl_reply("push: bad size or path (absolute, not /dev, at most %llu MiB)\n", VL_PUSH_MAX >> 20);
		return;
	}
	if (rootvnode == NULLVP) {
		vl_reply("push: no root file system yet\n");
		return;
	}
	strlcpy(vpush.path, rest, sizeof(vpush.path));
	snprintf(vpush.tmp, sizeof(vpush.tmp), "%s.linkpush", vpush.path);
	vfs_context_t ctx = vfs_context_kernel();
	vnode_t vp = NULLVP;
	int err = vnode_open(vpush.tmp, O_CREAT | O_TRUNC | FWRITE | O_NOFOLLOW, (int)(mode & 07777), VNODE_LOOKUP_NOFOLLOW,
	    &vp, ctx);
	if (err == ENOENT) {
		/* missing parent directories: create them (0755) and retry */
		char dir[256];
		strlcpy(dir, vpush.path, sizeof(dir));
		for (char *p = dir + 1; *p; p++) {
			if (*p == '/') {
				*p = 0;
				(void)mapple_kern_mkdir(dir, 0755);
				*p = '/';
			}
		}
		err = vnode_open(vpush.tmp, O_CREAT | O_TRUNC | FWRITE | O_NOFOLLOW, (int)(mode & 07777),
		    VNODE_LOOKUP_NOFOLLOW, &vp, ctx);
	}
	if (err) {
		vl_reply("push %s: cannot create %s (error %d)\n", vpush.path, vpush.tmp, err);
		return;
	}
	vpush.buf = kalloc_data(VL_PUSH_BUF, Z_WAITOK);
	if (vpush.buf == NULL) {
		vnode_close(vp, FWRITE, ctx);
		vl_reply("push: no memory\n");
		return;
	}
	vpush.vp = vp;
	vpush.size = size;
	vpush.got = 0;
	vpush.buf_len = 0;
	vpush.fnv = 2166136261u;
	vpush.want = (uint32_t)fnv;
	vpush.error = 0;
	vpush.last_ms = vl_now_ms();
	vpush.active = true;
	vl_reply("push %s: ready for %llu bytes\n", vpush.path, size);
}

/* One OUT packet of push data (index 1, RXPKTRDY set). */
static void
vl_push_data(void)
{
	uint32_t n = rd16(vl.musb, MUSB_RXCOUNT);
	if (n > vpush.size - vpush.got) {
		n = (uint32_t)(vpush.size - vpush.got);
	}
	if (n > VL_PUSH_BUF - vpush.buf_len) {
		vl_push_flush();
	}
	vl_fifo_read(1, vpush.buf + vpush.buf_len, n);
	wr16(vl.musb, MUSB_RXCSR, 0);
	vl.rx_pkts++;
	vpush.fnv = vl_fnv(vpush.fnv, vpush.buf + vpush.buf_len, n);
	vpush.buf_len += n;
	vpush.got += n;
	vpush.last_ms = vl_now_ms();
	if (vpush.buf_len == VL_PUSH_BUF) {
		vl_push_flush();
	}
	if (vpush.got == vpush.size) {
		vl_push_finish(NULL);
	}
}

extern int mtk_ufs_read_part(const char *name, uint64_t off, void *buf, uint32_t len, uint64_t *sizep);

/*
 * "pull PATH": the file as 'F' frames, then a reply line with its size and checksum. "pull part:NAME" reads a GPT
 * partition of the UFS root LUN instead (read only, at most 256 MiB; e.g. part:nvdata).
 */
static void
vl_pull_start(char *rest)
{
	while (*rest == ' ') {
		rest++;
	}
	if (strncmp(rest, "part:", 5) == 0) {
		uint64_t size = 0;
		int err = mtk_ufs_read_part(rest + 5, 0, NULL, 0, &size);
		if (err || size == 0 || size > (256ull << 20)) {
			vl_reply("pull %s: error %d (size %llu)\n", rest, err, size);
			return;
		}
		vpull.vp = NULLVP;
		vpull.off = 0;
		vpull.size = size;
		vpull.fnv = 2166136261u;
		vpull.active = true;
		strlcpy(vpush.path, rest, sizeof(vpush.path));
		return;
	}
	if (rest[0] != '/' || rootvnode == NULLVP) {
		vl_reply("pull: absolute path needed (and a mounted root)\n");
		return;
	}
	vfs_context_t ctx = vfs_context_kernel();
	vnode_t vp = NULLVP;
	int err = vnode_open(rest, FREAD, 0, 0, &vp, ctx);
	if (err) {
		vl_reply("pull %s: error %d\n", rest, err);
		return;
	}
	struct vnode_attr va;
	VATTR_INIT(&va);
	VATTR_WANTED(&va, va_data_size);
	if (vnode_vtype(vp) != VREG || vnode_getattr(vp, &va, ctx) != 0) {
		vnode_close(vp, FREAD, ctx);
		vl_reply("pull %s: not a regular file\n", rest);
		return;
	}
	vpull.vp = vp;
	vpull.off = 0;
	vpull.size = va.va_data_size;
	vpull.fnv = 2166136261u;
	vpull.active = true;
	strlcpy(vpush.path, rest, sizeof(vpush.path));    /* for the final line */
}

/* Next 'F' payload (at most max bytes); false at the end (reply text queued). */
static uint32_t
vl_pull_chunk(uint8_t *out, uint32_t max)
{
	vfs_context_t ctx = vfs_context_kernel();
	uint32_t n = (uint32_t)MIN((uint64_t)max, vpull.size - vpull.off);
	int resid = 0, err = 0;
	if (n > 0 && vpull.vp == NULLVP) {
		err = mtk_ufs_read_part(vpush.path + 5, vpull.off, out, n, NULL);
	} else if (n > 0) {
		err = vn_rdwr(UIO_READ, vpull.vp, (caddr_t)out, (int)n, (off_t)vpull.off, UIO_SYSSPACE, IO_NOAUTH,
		    vfs_context_ucred(ctx), &resid, vfs_context_proc(ctx));
		n -= (uint32_t)resid;
	}
	if (err || n == 0) {
		if (vpull.vp != NULLVP) {
			vnode_close(vpull.vp, FREAD, ctx);
		}
		vpull.active = false;
		vl_reply("pull %s: %llu bytes, fnv1a 0x%08x%s\n", vpush.path, vpull.off, vpull.fnv, err ? " (read error)" : "");
		return 0;
	}
	vpull.fnv = vl_fnv(vpull.fnv, out, n);
	vpull.off += n;
	return n;
}

/* ---- /dev/mapplelink: commands for mappled, its output back ---- */

LCK_GRP_DECLARE(vl_mappled_grp, "mapplelink");
LCK_MTX_DECLARE(vl_mappled_mtx, &vl_mappled_grp);
#define VL_OUT_RING     (64u << 10)
static char     vd_cmds[4096];          /* "ID TEXT\n" lines for mappled */
static uint32_t vd_cmd_len;
static uint8_t  vd_out[VL_OUT_RING];    /* mappled output, streamed as 'O' frames */
static uint32_t vd_out_head, vd_out_tail;       /* free-running */
static uint64_t vd_out_dropped;
static uint32_t vd_next_id = 1, vd_readers;
static int      vd_pid;
static bool     vd_node;

static int
mapplelink_open(__unused dev_t dev, __unused int flags, __unused int devtype, proc_t p)
{
	lck_mtx_lock(&vl_mappled_mtx);
	vd_pid = proc_pid(p);
	lck_mtx_unlock(&vl_mappled_mtx);
	return 0;
}

static int
mapplelink_close(__unused dev_t dev, __unused int flags, __unused int devtype, __unused proc_t p)
{
	return 0;
}

/* One command line per read (blocks until there is one). */
static int
mapplelink_read(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	int err = 0;
	lck_mtx_lock(&vl_mappled_mtx);
	vd_readers++;
	while (vd_cmd_len == 0 && err == 0) {
		err = msleep(&vd_cmd_len, &vl_mappled_mtx, PCATCH | PSOCK, "mapplelink", NULL);
	}
	vd_readers--;
	if (err == 0) {
		uint32_t n = 0;
		while (n < vd_cmd_len && vd_cmds[n] != '\n') {
			n++;
		}
		n = MIN(n + 1, vd_cmd_len);
		uint32_t give = (uint32_t)MIN((user_ssize_t)n, uio_resid(uio));
		err = uiomove(vd_cmds, (int)give, uio);
		memmove(vd_cmds, vd_cmds + n, vd_cmd_len - n);
		vd_cmd_len -= n;
	}
	lck_mtx_unlock(&vl_mappled_mtx);
	return err;
}

static int
mapplelink_write(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	char chunk[512];
	int err = 0;
	while (err == 0 && uio_resid(uio) > 0) {
		int n = (int)MIN((user_ssize_t)sizeof(chunk), uio_resid(uio));
		err = uiomove(chunk, n, uio);
		if (err) {
			break;
		}
		lck_mtx_lock(&vl_mappled_mtx);
		for (int i = 0; i < n; i++) {
			if (vd_out_head - vd_out_tail >= VL_OUT_RING) {
				vd_out_dropped += (uint64_t)(n - i);
				break;
			}
			vd_out[vd_out_head++ % VL_OUT_RING] = (uint8_t)chunk[i];
		}
		lck_mtx_unlock(&vl_mappled_mtx);
	}
	return err;
}

static const struct cdevsw mapplelink_cdevsw = {
	.d_open = mapplelink_open, .d_close = mapplelink_close, .d_read = mapplelink_read, .d_write = mapplelink_write,
	.d_ioctl = eno_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_ttys = NULL, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0
};

/* ---- /dev/linkptp (minor 0), /dev/linkmux (minor 1): the std persona's functions for mapple-usbd ---- */

struct usblink_serial { char s[64]; };
struct usblink_event { uint32_t len; uint8_t data[64]; };
#define USBLINK_SET_SERIAL      _IOW('V', 1, struct usblink_serial)
#define USBLINK_ATTACH          _IO('V', 2)
#define USBLINK_STATE           _IOR('V', 3, int)
#define USBLINK_PTP_EVENT       _IOW('V', 4, struct usblink_event)

static int
vf_open(dev_t dev, __unused int flags, __unused int devtype, __unused proc_t p)
{
	int m = minor(dev), err = 0;
	if (m > 1) {
		return ENXIO;
	}
	lck_mtx_lock(&vf_mtx);
	if (vfn[m].open) {
		err = EBUSY;
	} else if (vfn[m].rx == NULL) {
		err = ENOMEM;
	} else {
		vfn[m].open = true;
		vf_reset_locked(&vfn[m], 0);    /* a new daemon starts clean */
	}
	lck_mtx_unlock(&vf_mtx);
	return err;
}

static int
vf_close(dev_t dev, __unused int flags, __unused int devtype, __unused proc_t p)
{
	lck_mtx_lock(&vf_mtx);
	vfn[minor(dev)].open = false;
	lck_mtx_unlock(&vf_mtx);
	return 0;
}

/* Bytes of one OUT transfer at most; blocks until there are some. */
static int
vf_read(dev_t dev, struct uio *uio, __unused int ioflag)
{
	struct vfn *f = &vfn[minor(dev)];
	uint8_t chunk[1024];
	int err = 0;
	bool got = false, end = false;

	while (!end && err == 0 && uio_resid(uio) > 0) {
		uint32_t n = 0;
		lck_mtx_lock(&vf_mtx);
		while (!got) {
			if (f->pending_err) {
				err = f->pending_err;
				f->pending_err = 0;
				break;
			}
			/* a transfer end at the read position: its bytes were read before its ZLP came */
			while (f->rx_mtail != f->rx_mhead && f->rx_mark[f->rx_mtail % VF_MARKS] == f->rx_tail) {
				f->rx_mtail++;
			}
			if (f->rx_head != f->rx_tail) {
				break;
			}
			err = msleep(&f->rx_head, &vf_mtx, PCATCH | PSOCK, "mp_usblink", NULL);
			if (err) {
				break;
			}
		}
		if (err == 0 && f->rx_head != f->rx_tail) {
			uint32_t lim = f->rx_head - f->rx_tail;
			bool ends = false;
			if (f->rx_mtail != f->rx_mhead && f->rx_mark[f->rx_mtail % VF_MARKS] - f->rx_tail <= lim) {
				lim = f->rx_mark[f->rx_mtail % VF_MARKS] - f->rx_tail;
				ends = true;
			}
			n = (uint32_t)MIN((user_ssize_t)MIN(lim, (uint32_t)sizeof(chunk)), uio_resid(uio));
			for (uint32_t i = 0; i < n; i++) {
				chunk[i] = f->rx[(f->rx_tail + i) % VF_RING];
			}
			f->rx_tail += n;
			if (ends && n == lim) {
				f->rx_mtail++;
				end = true;
			}
		}
		lck_mtx_unlock(&vf_mtx);
		if (n == 0) {
			break;
		}
		err = uiomove((caddr_t)chunk, (int)n, uio);
		got = true;
	}
	return err;
}

/* One IN transfer (ptp: part of a container, see VIVA-USB-PERSONA.md); blocks while the queue is full. */
static int
vf_write(dev_t dev, struct uio *uio, __unused int ioflag)
{
	struct vfn *f = &vfn[minor(dev)];
	user_ssize_t len = uio_resid(uio);
	if (len == 0) {
		return 0;
	}
	if (len > VF_WRITE_MAX) {
		return EMSGSIZE;
	}
	uint32_t n = (uint32_t)len;
	uint8_t *buf = kalloc_data(n, Z_WAITOK);
	if (buf == NULL) {
		return ENOMEM;
	}
	int err = uiomove((caddr_t)buf, (int)n, uio);
	lck_mtx_lock(&vf_mtx);
	uint32_t gen = f->gen;
	while (err == 0) {
		if (!vl.configured || vl.persona != P_STD) {
			err = ENXIO;
		} else if (f->gen != gen) {
			err = ECONNRESET;
		} else if (VF_RING - (f->tx_head - f->tx_tail) >= n && f->tx_mhead - f->tx_mtail < VF_MARKS) {
			break;
		} else {
			err = msleep(&f->tx_tail, &vf_mtx, PCATCH | PSOCK, "usblinkw", NULL);
		}
	}
	if (err == 0) {
		for (uint32_t i = 0; i < n; i++) {
			f->tx[(f->tx_head + i) % VF_RING] = buf[i];
		}
		f->tx_head += n;
		bool end = true;
		if (f == &vfn[0]) {             /* PTP: a data phase over several writes is one transfer */
			if (f->tx_cont_left == 0 && n >= 4) {
				uint32_t clen = (uint32_t)buf[0] | (uint32_t)buf[1] << 8 | (uint32_t)buf[2] << 16 |
				    (uint32_t)buf[3] << 24;
				f->tx_cont_left = clen >= 12 ? clen : n;
			}
			f->tx_cont_left -= MIN(f->tx_cont_left, n);
			end = f->tx_cont_left == 0;
		}
		if (end) {
			f->tx_mark[f->tx_mhead++ % VF_MARKS] = f->tx_head;
			f->tx_xfers++;
		}
	}
	lck_mtx_unlock(&vf_mtx);
	kfree_data(buf, n);
	return err;
}

static int
vf_ioctl(__unused dev_t dev, u_long cmd, caddr_t data, __unused int flag, __unused proc_t p)
{
	switch (cmd) {
	case USBLINK_SET_SERIAL: {
		struct usblink_serial *s = (struct usblink_serial *)data;
		size_t i = 0;
		for (; i < sizeof(vl.serial) - 1 && s->s[i] > ' ' && s->s[i] < 0x7f; i++) {
			vl.serial[i] = s->s[i];
		}
		vl.serial[i] = 0;
		return 0;
	}
	case USBLINK_ATTACH:
		if (!vl.std_allowed) {
			return EPERM;
		}
		vl.want_std = true;
		return 0;
	case USBLINK_STATE:
		*(int *)data = usblink_state();
		return 0;
	case USBLINK_PTP_EVENT: {
		struct usblink_event *e = (struct usblink_event *)data;
		int err = 0;
		if (e->len == 0 || e->len > sizeof(e->data)) {
			return EINVAL;
		}
		lck_mtx_lock(&vf_mtx);
		if (vf_ev_head - vf_ev_tail >= VF_EVENTS) {
			err = ENOBUFS;
		} else {
			memcpy(vf_ev[vf_ev_head % VF_EVENTS].data, e->data, e->len);
			vf_ev[vf_ev_head++ % VF_EVENTS].len = e->len;
		}
		lck_mtx_unlock(&vf_mtx);
		return err;
	}
	default:
		return ENOTTY;
	}
}

static const struct cdevsw usblink_cdevsw = {
	.d_open = vf_open, .d_close = vf_close, .d_read = vf_read, .d_write = vf_write,
	.d_ioctl = vf_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_ttys = NULL, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0
};

/* OUT packets of one function into its ring; a packet that does not fit stays in the FIFO (the host gets NAKs). */
static void
vf_service_rx(struct vfn *f)
{
	static uint8_t pkt[512];
	wr8(vl.musb, MUSB_INDEX, f->ep);
	for (int k = 0; k < 32; k++) {
		if (!(rd16(vl.musb, MUSB_RXCSR) & RXCSR_RXPKTRDY)) {
			break;
		}
		uint32_t n = MIN((uint32_t)rd16(vl.musb, MUSB_RXCOUNT), (uint32_t)sizeof(pkt));
		bool last = n < vl_mps();
		lck_mtx_lock(&vf_mtx);
		bool room = VF_RING - (f->rx_head - f->rx_tail) >= n && f->rx_mhead - f->rx_mtail < VF_MARKS;
		lck_mtx_unlock(&vf_mtx);
		if (!room) {
			break;
		}
		vl_fifo_read(f->ep, pkt, n);
		wr16(vl.musb, MUSB_RXCSR, 0);
		lck_mtx_lock(&vf_mtx);
		for (uint32_t i = 0; i < n; i++) {
			f->rx[(f->rx_head + i) % VF_RING] = pkt[i];
		}
		f->rx_head += n;
		f->rx_xfer += n;
		f->rx_bytes += n;
		if (last && f->rx_xfer != 0) {
			f->rx_mark[f->rx_mhead++ % VF_MARKS] = f->rx_head;
			f->rx_xfer = 0;
			f->rx_xfers++;
		}
		wakeup(&f->rx_head);
		lck_mtx_unlock(&vf_mtx);
	}
	wr8(vl.musb, MUSB_INDEX, 0);
}

/* IN packets while the FIFO takes them: data up to the next transfer end, then a short packet or a ZLP. */
static void
vf_service_tx(struct vfn *f)
{
	static uint8_t pkt[512];
	uint32_t mps = vl_mps();
	wr8(vl.musb, MUSB_INDEX, f->ep);
	for (int k = 0; k < 64; k++) {
		uint16_t csr = rd16(vl.musb, MUSB_TXCSR);
		if (csr & (TXCSR_UNDERRUN | TXCSR_SENTSTALL)) {
			wr16(vl.musb, MUSB_TXCSR, TXCSR_MODE);
			csr = rd16(vl.musb, MUSB_TXCSR);
		}
		if (csr & (TXCSR_TXPKTRDY | TXCSR_FIFONOTEMPTY)) {
			delay(10);              /* the host usually takes it within a microframe */
			if (rd16(vl.musb, MUSB_TXCSR) & (TXCSR_TXPKTRDY | TXCSR_FIFONOTEMPTY)) {
				break;
			}
		}
		uint32_t n = 0;
		bool send = false;
		lck_mtx_lock(&vf_mtx);
		if (f->tx_zlp) {
			f->tx_zlp = false;
			send = true;
		} else if (f->tx_head != f->tx_tail) {
			uint32_t avail = f->tx_head - f->tx_tail;
			bool marked = f->tx_mtail != f->tx_mhead;
			uint32_t lim = marked ? f->tx_mark[f->tx_mtail % VF_MARKS] - f->tx_tail : avail;
			n = MIN(lim, mps);
			if (marked || n == mps) {       /* an unfinished PTP container only goes out in full packets */
				for (uint32_t i = 0; i < n; i++) {
					pkt[i] = f->tx[(f->tx_tail + i) % VF_RING];
				}
				f->tx_tail += n;
				f->tx_bytes += n;
				if (marked && n == lim) {
					f->tx_mtail++;
					f->tx_zlp = n == mps;
				}
				send = true;
				wakeup(&f->tx_tail);
			}
		}
		lck_mtx_unlock(&vf_mtx);
		if (!send) {
			break;
		}
		vl_fifo_write(f->ep, pkt, n);
		wr16(vl.musb, MUSB_TXCSR, TXCSR_MODE | TXCSR_TXPKTRDY);
	}
	wr8(vl.musb, MUSB_INDEX, 0);
}

/* PTP events on the interrupt endpoint 3. */
static void
vf_service_events(void)
{
	wr8(vl.musb, MUSB_INDEX, 3);
	uint16_t csr = rd16(vl.musb, MUSB_TXCSR);
	if (csr & (TXCSR_UNDERRUN | TXCSR_SENTSTALL)) {
		wr16(vl.musb, MUSB_TXCSR, TXCSR_MODE);
		csr = rd16(vl.musb, MUSB_TXCSR);
	}
	if (!(csr & (TXCSR_TXPKTRDY | TXCSR_FIFONOTEMPTY))) {
		lck_mtx_lock(&vf_mtx);
		if (vf_ev_head != vf_ev_tail) {
			uint32_t i = vf_ev_tail++ % VF_EVENTS;
			vl_fifo_write(3, vf_ev[i].data, vf_ev[i].len);
			wr16(vl.musb, MUSB_TXCSR, TXCSR_MODE | TXCSR_TXPKTRDY);
		}
		lck_mtx_unlock(&vf_mtx);
	}
	wr8(vl.musb, MUSB_INDEX, 0);
}

/* The switch to the std persona (USBLINK_ATTACH): soft disconnect, then enumerate again with the new descriptors. */
static void
vl_persona_std(void)
{
	printf("LINK: switching to the standard persona 05ac:12a8 (serial %s)\n", vl.serial);
	wr8(vl.musb, MUSB_POWER, rd8(vl.musb, MUSB_POWER) & ~POWER_SOFTCONN);
	vl.configured = false;
	vl_tool_set(false);
	vl.ep0 = EP0_IDLE;
	vl.new_addr = 0;
	vf_reset_all(ECONNRESET);
	vl_sleep_ms(300);
	vl.persona = P_STD;
	vl.link_if = 2;
	wr8(vl.musb, MUSB_INTRUSB, rd8(vl.musb, MUSB_INTRUSB));
	wr16(vl.musb, MUSB_INTRTX, rd16(vl.musb, MUSB_INTRTX));
	wr16(vl.musb, MUSB_INTRRX, rd16(vl.musb, MUSB_INTRRX));
	wr8(vl.musb, MUSB_FADDR, 0);
	wr8(vl.musb, MUSB_INDEX, 0);
	wr16(vl.musb, MUSB_INTRTXE, 0x001f);            /* ep0..4 (the line stays masked: polled) */
	wr16(vl.musb, MUSB_INTRRXE, 0x0016);            /* ep1, 2, 4 OUT */
	wr8(vl.musb, MUSB_POWER, POWER_HSENAB | POWER_SOFTCONN);
}

/* Once the root is mounted (devfs is up): /dev/mapplelink, root only. */
static void
vl_mappled_node(void)
{
	if (vd_node || rootvnode == NULLVP) {
		return;
	}
	vd_node = true;
	int major = cdevsw_add(-1, &mapplelink_cdevsw);
	if (major < 0 || devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0600, "mapplelink") == NULL) {
		printf("LINK: no /dev/mapplelink (major %d)\n", major);
	} else if (devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0600, "vivalink") != NULL) {
		printf("LINK: /dev/vivalink is now /dev/mapplelink (the old name goes at release 1)\n");
	}
	/* std persona functions (0182) */
	for (unsigned i = 0; i < 2; i++) {
		vfn[i].rx = kalloc_data(VF_RING, Z_WAITOK);
		vfn[i].tx = kalloc_data(VF_RING, Z_WAITOK);
		if (vfn[i].rx == NULL || vfn[i].tx == NULL) {
			printf("LINK: no memory for /dev/%s\n", vfn[i].name);
			return;
		}
	}
	major = cdevsw_add(-1, &usblink_cdevsw);
	for (unsigned i = 0; i < 2 && major >= 0; i++) {
		if (devfs_make_node(makedev(major, i), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0600, vfn[i].name) == NULL) {
			printf("LINK: no /dev/%s\n", vfn[i].name);
		}
	}
}

/* Queue "ID TEXT" for mappled; 0 if nobody reads or the queue is full. */
static uint32_t
vl_mappled_send(const char *text)
{
	uint32_t id = 0;
	lck_mtx_lock(&vl_mappled_mtx);
	size_t need = strlen(text) + 16;
	if (vd_readers > 0 && vd_cmd_len + need < sizeof(vd_cmds)) {
		id = vd_next_id++;
		vd_cmd_len += (uint32_t)snprintf(vd_cmds + vd_cmd_len, sizeof(vd_cmds) - vd_cmd_len, "%u %s\n", id, text);
		wakeup(&vd_cmd_len);
	}
	lck_mtx_unlock(&vl_mappled_mtx);
	return id;
}

static uint32_t
vl_mappled_out(uint8_t *out, uint32_t max)
{
	lck_mtx_lock(&vl_mappled_mtx);
	uint32_t n = MIN(max, vd_out_head - vd_out_tail);
	for (uint32_t i = 0; i < n; i++) {
		out[i] = vd_out[vd_out_tail++ % VL_OUT_RING];
	}
	lck_mtx_unlock(&vl_mappled_mtx);
	return n;
}

/*
 * Commands (one per OUT packet, text):
 *   ping | stat | help
 *   peek PA [WORDS]   read 32-bit words of physical memory/MMIO (max 64)
 *   poke PA VALUE     write one 32-bit word, then read it back
 *   rewind [BYTES]    resend the log from BYTES back (default: whole ring)
 *   pmic ADDR [N]     read N (max 32) 16-bit PMIC registers through pwrap (read only)
 *   i2c DEV REG N     read N (max 64) bytes from 16-bit register REG of I2C0 device DEV
 *   i2cw DEV REG B..  write bytes (max 6) to register REG of I2C0 device DEV
 *   i2c8 BUS DEV REG N     read N (max 64) bytes from 8-bit register REG of device DEV on I2C bus BUS
 *   i2c8w BUS DEV REG B..  write bytes (max 7) to 8-bit register REG of device DEV on I2C bus BUS
 *   bat [poll|lpm on|off]  battery (BQ27Z561Battery.cpp): gauge, charger registers, Low Power Mode
 *   wifi [status|detect|on|wmt|fw|cap|off]  CONNSYS (mt6631_wifi.c): state (read only), A-die detection, power on with RAM patches / off
 *   bt [status|on|up]  Bluetooth (mtk_bt.c, after wifi wmt): state (read only), function on + HCI probe, bridge to bluetoothd
 *   gpu [ARGS]        Mali driver (MappleGPU.cpp): status, mfg, on, off, regs, freq, selftest [fault], probe, publish
 *   cpu [ARGS]        CPU DVFS (mt6781_cpufreq.c): status, gov on|off, set|max ll|big|cci MHz, fast on|off, reset
 *   push SIZE MODE FNV PATH  receive a file (raw OUT data follows; written to PATH.linkpush, renamed when
 *                     its FNV-1a matches; missing directories are created)
 *   pull PATH         send a file ('F' frames)
 *   disp [ARGS]       display (MappleFramebuffer.cpp): status, regs (read only), scanout on|off
 *   run ARGS..        run a program through mappled (output as 'O' frames, "[ID] exit N" at the end)
 *   mappled TEXT        any other mappled command (ps, kill ID, tests)
 *   console [on|off]  kernel text on the screen (off from the GUI's first frame with mp_fb_noconsole=1)
 *   panic [TEXT]      panic (to test the panic path)
 *   reboot            watchdog software reset (back through preloader/LK)
 * peek/poke have no guard: an address in a gated block can hang the bus.
 */
/* "stat" (also kern.mapple_link, bsd/dev/mapple_sysctl.c): uptime, link and log counters. */
void usblink_stat(char *out, size_t len);

void
usblink_stat(char *out, size_t len)
{
	uint64_t ns;
	size_t o = 0;
	if (len == 0) {
		return;
	}
	out[0] = 0;
	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
#define ST(...) do { if (o < len) o += (size_t)snprintf(out + o, len - o, __VA_ARGS__); } while (0)
	ST("uptime %llu ms, %s speed, address %u\n", ns / NSEC_PER_MSEC, vl.hs ? "high" : "full",
	    vl.musb ? rd8(vl.musb, MUSB_FADDR) : 0);
	ST("log: total %llu bytes, sent up to %llu (%llu bytes this link)\n", mapple_log_position(), vl.log_pos,
	    vl.log_bytes);
	ST("usb: %llu resets, %llu setups, %llu commands, %llu IN packets, %llu OUT packets\n", vl.resets, vl.setups,
	    vl.cmds, vl.tx_pkts, vl.rx_pkts);
	ST("persona %s (%s), host %s, Mapple link %s (%llu heartbeats)\n", vl.persona == P_STD ? "std 05ac:12a8" :
	    "link 1209:0002", vl.std_allowed ? (vl.want_std ? "attached" : "waiting for mapple-usbd") : "mp_usblink_std=0",
	    vl.configured ? "configured" : "none", vl.tool_active ? "active" : "inactive", vl.heartbeats);
	for (unsigned i = 0; i < 2; i++) {
		ST("%s: %s, out %llu bytes / %llu transfers (%u queued), in %llu bytes / %llu transfers (%u queued), "
		    "%llu resets\n", vfn[i].name, vfn[i].open ? "open" : "closed", vfn[i].rx_bytes, vfn[i].rx_xfers,
		    vfn[i].rx_head - vfn[i].rx_tail, vfn[i].tx_bytes, vfn[i].tx_xfers, vfn[i].tx_head - vfn[i].tx_tail,
		    vfn[i].resets);
	}
	ST("mappled: %s (last opener pid %d), %llu output bytes dropped%s\n", vd_readers ? "waiting for commands" :
	    "not reading", vd_pid, vd_out_dropped, vd_node ? "" : ", /dev/mapplelink not created yet");
#undef ST
}

static void
vl_command(void)
{
	char *c = vl.cmd, *rest;
	uint64_t a, v, b8;

	vl.cmds++;
	vl.reply_len = vl.reply_pos = 0;
	vl.reply_end = true;
	while (*c == ' ') {
		c++;
	}
	for (rest = c; *rest && *rest != ' '; rest++) {
	}
	size_t len = (size_t)(rest - c);
#define IS(word) (len == sizeof(word) - 1 && strncmp(c, word, len) == 0)
	if (IS("ping")) {
		vl_reply("pong\n");
	} else if (IS("stat")) {
		static char stat_reply[2048];
		usblink_stat(stat_reply, sizeof(stat_reply));
		vl_reply("%s", stat_reply);
	} else if (IS("peek") && vl_arg(&rest, &a)) {
		uint64_t words = 1;
		(void)vl_arg(&rest, &words);
		words = MIN(MAX(words, 1ull), 64ull);
		a &= ~3ull;
		for (uint64_t i = 0; i < words; i++) {
			if (i % 4 == 0) {
				vl_reply("%s%010llx:", i ? "\n" : "", a + 4 * i);
			}
			vl_reply(" %08x", ml_phys_read_word_64(a + 4 * i));
		}
		vl_reply("\n");
	} else if (IS("poke") && vl_arg(&rest, &a) && vl_arg(&rest, &v)) {
		a &= ~3ull;
		ml_phys_write_word_64(a, (unsigned int)v);
		vl_reply("%010llx <- %08x, reads %08x\n", a, (unsigned int)v, ml_phys_read_word_64(a));
	} else if (IS("pmic") && vl_arg(&rest, &a)) {
		uint64_t n = 1;
		(void)vl_arg(&rest, &n);
		n = MIN(MAX(n, 1ull), 32ull);
		a &= ~1ull;
		for (uint64_t i = 0; i < n; i++) {
			uint32_t val = 0;
			int err = mtk_pwrap_read((uint32_t)(a + 2 * i), &val);
			if (err) {
				vl_reply("pmic 0x%04llx: error %d\n", a + 2 * i, err);
				break;
			}
			vl_reply("%spmic 0x%04llx:", i % 8 ? "" : (i ? "\n" : ""), a + 2 * i);
			vl_reply(" %04x", val);
		}
		vl_reply("\n");
	} else if ((IS("i2c") || IS("i2cw")) && vl_arg(&rest, &a) && vl_arg(&rest, &v)) {
		bool wr = len == 4;
		uint8_t w[8] = { (uint8_t)(v >> 8), (uint8_t)v }, r[8];
		uint64_t n = 0, b;
		if (wr) {
			while (n < 6 && vl_arg(&rest, &b)) {
				w[2 + n++] = (uint8_t)b;
			}
			int err = mapple_i2c_xfer((uint8_t)a, w, 2 + (uint32_t)n, NULL, 0);
			vl_reply("i2c 0x%02llx 0x%04llx <- %llu bytes: %s\n", a, v, n, err ? "error" : "ok");
		} else {
			(void)vl_arg(&rest, &n);
			n = MIN(MAX(n, 1ull), 64ull);
			for (uint64_t off = 0; off < n; off += 8) {
				uint32_t k = (uint32_t)MIN(8ull, n - off);
				w[0] = (uint8_t)((v + off) >> 8);
				w[1] = (uint8_t)(v + off);
				if (mapple_i2c_xfer((uint8_t)a, w, 2, r, k)) {
					vl_reply("i2c 0x%02llx 0x%04llx: error\n", a, v + off);
					break;
				}
				vl_reply("i2c 0x%02llx 0x%04llx:", a, v + off);
				for (uint32_t i = 0; i < k; i++) {
					vl_reply(" %02x", r[i]);
				}
				vl_reply("\n");
			}
		}
	} else if ((IS("i2c8") || IS("i2c8w")) && vl_arg(&rest, &b8) && vl_arg(&rest, &a) && vl_arg(&rest, &v)) {
		uint8_t w[8] = { (uint8_t)v }, r[8];
		uint64_t n = 0, b;
		if (len == 5) {
			while (n < 7 && vl_arg(&rest, &b)) {
				w[1 + n++] = (uint8_t)b;
			}
			int err = mapple_i2c_xfer_bus((uint32_t)b8, (uint8_t)a, w, 1 + (uint32_t)n, NULL, 0);
			vl_reply("i2c%llu 0x%02llx 0x%02llx <- %llu bytes: %s\n", b8, a, v, n, err ? "error" : "ok");
		} else {
			(void)vl_arg(&rest, &n);
			n = MIN(MAX(n, 1ull), 64ull);
			for (uint64_t off = 0; off < n; off += 8) {
				uint32_t k = (uint32_t)MIN(8ull, n - off);
				w[0] = (uint8_t)(v + off);
				if (mapple_i2c_xfer_bus((uint32_t)b8, (uint8_t)a, w, 1, r, k)) {
					vl_reply("i2c%llu 0x%02llx 0x%02llx: error\n", b8, a, v + off);
					break;
				}
				vl_reply("i2c%llu 0x%02llx 0x%02llx:", b8, a, v + off);
				for (uint32_t i = 0; i < k; i++) {
					vl_reply(" %02x", r[i]);
				}
				vl_reply("\n");
			}
		}
	} else if (IS("wifi")) {
		static char wifi_reply[4000];
		mt6631_wifi_link(rest, wifi_reply, sizeof(wifi_reply));
		vl_reply("%s", wifi_reply);
	} else if (IS("bt")) {
		static char bt_reply[256];
		mtk_bt_link(rest, bt_reply, sizeof(bt_reply));
		vl_reply("%s", bt_reply);
	} else if (IS("bat")) {
		static char bat_reply[4000];
		bq27z561_link(rest, bat_reply, sizeof(bat_reply));
		vl_reply("%s", bat_reply);
	} else if (IS("cpu")) {
		static char cpu_reply[4000];
		mt6781_cpufreq_link(rest, cpu_reply, sizeof(cpu_reply));
		vl_reply("%s", cpu_reply);
	} else if (IS("gpu")) {
		static char gpu_reply[4000];  /* vl.reply holds 4096 */
		mapple_gpu_link(rest, gpu_reply, sizeof(gpu_reply));
		vl_reply("%s", gpu_reply);
	} else if (IS("disp")) {
		static char disp_reply[4000];
		mapple_disp_link(rest, disp_reply, sizeof(disp_reply));
		vl_reply("%s", disp_reply);
	} else if (IS("push")) {
		vl_push_start(rest);
	} else if (IS("pull")) {
		vl_pull_start(rest);
	} else if (IS("run") || IS("mappled")) {
		char line[480];
		while (*rest == ' ') {
			rest++;
		}
		snprintf(line, sizeof(line), "%s%s", IS("run") ? "run " : "", rest);
		uint32_t id = *rest ? vl_mappled_send(line) : 0;
		if (id) {
			vl_reply("mappled: %u queued (pid %d)\n", id, vd_pid);
		} else {
			vl_reply("mappled: not running (no reader on /dev/mapplelink%s) or nothing to send\n",
			    vd_node ? "" : ", not created yet");
		}
	} else if (IS("rewind")) {
		uint64_t total = mapple_log_position(), back = UINT64_MAX;
		(void)vl_arg(&rest, &back);
		vl.log_pos = back >= total ? 0 : total - back;
		vl_reply("log rewound to %llu\n", vl.log_pos);
	} else if (IS("console")) {
		while (*rest == ' ') {
			rest++;
		}
		if (strncmp(rest, "on", 2) == 0) {
			mapple_console_muted = 0;
		} else if (strncmp(rest, "off", 3) == 0) {
			mapple_console_muted = 1;
		}
		vl_reply("screen console %s (the link log gets everything either way)\n", mapple_console_muted ? "off" : "on");
	} else if (IS("panic")) {
		while (*rest == ' ') {
			rest++;
		}
		panic("LINK: panic command: %s", rest);
	} else if (IS("reboot")) {
		printf("LINK: reboot (WDT_MODE 0x%08x)\n", rd32(vl.toprgu, WDT_MODE));
		for (;;) {
			wr32(vl.toprgu, WDT_SWRST, WDT_SWRST_KEY);
			delay(5000);
		}
	} else if (IS("help")) {
		vl_reply("ping | stat | peek PA [WORDS] | poke PA VALUE | pmic ADDR [N] | i2c DEV REG N | i2cw DEV REG B.. | i2c8[w] BUS DEV REG N|B.. | bat [poll|lpm on|off] | wifi [status|detect|on|wmt|fw|cap|off] | bt [status|on|up] | cpu [status|gov on|off|set|max ll|big|cci MHz|fast on|off] | gpu [status|mfg|on|off|regs|freq [MHz]|selftest [fault]|probe|publish] | disp [regs|scanout on|off] | push SIZE MODE FNV PATH | pull PATH | run ARGS.. | mappled ps|kill ID|tests | rewind [BYTES] | console [on|off] | panic [TEXT] | reboot\n");
	} else {
		vl_reply("unknown command: %s (try help)\n", c);
	}
#undef IS
}

/* One bulk IN frame if the endpoint is free: reply first, then log. Index 1. */
static bool
vl_tx(void)
{
	static uint8_t pkt[512];
	uint16_t csr = rd16(vl.musb, MUSB_TXCSR);
	if (csr & (TXCSR_UNDERRUN | TXCSR_SENTSTALL)) {
		wr16(vl.musb, MUSB_TXCSR, TXCSR_MODE);
		csr = rd16(vl.musb, MUSB_TXCSR);
	}
	if (csr & (TXCSR_TXPKTRDY | TXCSR_FIFONOTEMPTY)) {
		return false;
	}
	uint32_t room = vl_mps() - 4, n;
	uint8_t type;
	if (vl.reply_pos < vl.reply_len) {
		n = MIN(room, vl.reply_len - vl.reply_pos);
		memcpy(pkt + 4, vl.reply + vl.reply_pos, n);
		vl.reply_pos += n;
		type = 'R';
	} else if (vpull.active && (n = vl_pull_chunk(pkt + 4, room)) > 0) {
		type = 'F';
	} else if (vl.reply_pos < vl.reply_len) {          /* the pull's last line */
		return vl_tx();
	} else if (vl.reply_end) {
		vl.reply_end = false;
		n = 0;
		type = 'E';
	} else if ((n = vl_mappled_out(pkt + 4, room)) > 0) {
		type = 'O';
	} else {
		n = mapple_log_copy(&vl.log_pos, (char *)pkt + 4, room);
		if (n == 0) {
			return false;
		}
		vl.log_bytes += n;
		type = 'L';
	}
	pkt[0] = 'V';
	pkt[1] = type;
	pkt[2] = (uint8_t)n;
	pkt[3] = (uint8_t)(n >> 8);
	vl_fifo_write(1, pkt, n + 4);
	wr16(vl.musb, MUSB_TXCSR, TXCSR_MODE | TXCSR_TXPKTRDY);
	vl.tx_pkts++;
	return true;
}

/* One bulk OUT command packet, if any. Index 1. */
static void
vl_rx(void)
{
	uint16_t csr = rd16(vl.musb, MUSB_RXCSR);
	if (!(csr & RXCSR_RXPKTRDY)) {
		return;
	}
	if (vpush.active) {
		for (int i = 0; i < 64 && vpush.active && (rd16(vl.musb, MUSB_RXCSR) & RXCSR_RXPKTRDY); i++) {
			vl_push_data();
			if (i < 63 && !(rd16(vl.musb, MUSB_RXCSR) & RXCSR_RXPKTRDY)) {
				delay(5);       /* the host's next packet is usually right behind */
			}
		}
		return;
	}
	uint32_t n = MIN((uint32_t)rd16(vl.musb, MUSB_RXCOUNT), (uint32_t)sizeof(vl.cmd) - 1);
	vl_fifo_read(1, (uint8_t *)vl.cmd, n);
	wr16(vl.musb, MUSB_RXCSR, 0);
	vl.rx_pkts++;
	while (n > 0 && (vl.cmd[n - 1] == '\n' || vl.cmd[n - 1] == '\r' || vl.cmd[n - 1] == 0)) {
		n--;
	}
	vl.cmd[n] = 0;
	if (vl.reply_pos < vl.reply_len || vl.reply_end) {
		return;                 /* previous reply still going out: one command at a time */
	}
	printf("LINK: command \"%s\"\n", vl.cmd);
	vl_command();
}

static void
vl_thread(void *arg __unused, wait_result_t wr __unused)
{
	for (;;) {
		uint8_t usb = rd8(vl.musb, MUSB_INTRUSB);
		uint16_t tx = rd16(vl.musb, MUSB_INTRTX);
		uint16_t rx = rd16(vl.musb, MUSB_INTRRX);
		if (usb) {
			wr8(vl.musb, MUSB_INTRUSB, usb);
		}
		if (tx) {
			wr16(vl.musb, MUSB_INTRTX, tx);
		}
		if (rx) {
			wr16(vl.musb, MUSB_INTRRX, rx);
		}
		if (usb & INTR_RESET) {
			vl.resets++;
			vl.ep0 = EP0_IDLE;
			vl.new_addr = 0;
			vl.configured = false;
			vl_tool_set(false);
			vf_reset_all(ECONNRESET);
			vl.hs = (rd8(vl.musb, MUSB_POWER) & POWER_HSMODE) != 0;
			wr8(vl.musb, MUSB_FADDR, 0);
			printf("LINK: bus reset (%s speed)\n", vl.hs ? "high" : "full");
		}
		if (usb & INTR_DISCON) {
			vl.configured = false;
			vl_tool_set(false);
			vf_reset_all(ECONNRESET);
		}
		if (vl.tool_active && vl_now_ms() - vl.tool_ms > VL_TOOL_MS) {
			vl_tool_set(false);
		}
		if (vl.want_std && vl.persona == P_LINK) {
			vl_persona_std();
			continue;
		}
		/* ep0: on its interrupt, and on a waiting SETUP in case the bit was missed */
		wr8(vl.musb, MUSB_INDEX, 0);
		if ((tx & 1) || (vl.ep0 == EP0_IDLE && (rd16(vl.musb, MUSB_CSR0) & CSR0_RXPKTRDY))) {
			vl_ep0();
		}
		vl_mappled_node();
		if (vpush.active && vl_now_ms() - vpush.last_ms > VL_PUSH_IDLE_MS) {
			vl_push_finish("no data for 10 s");
		}
		if (vl.configured) {
			wr8(vl.musb, MUSB_INDEX, 1);
			vl_rx();
			/* up to 64 packets per wakeup, while the host keeps reading */
			for (int i = 0; i < 64; i++) {
				if (!vl_tx()) {
					delay(20);
					if (!vl_tx()) {
						break;
					}
				}
			}
			wr8(vl.musb, MUSB_INDEX, 0);
			if (vl.persona == P_STD) {
				for (unsigned i = 0; i < 2; i++) {
					vf_service_rx(&vfn[i]);
					vf_service_tx(&vfn[i]);
				}
				vf_service_events();
			}
		}
		vl_sleep_ms(1);
	}
}

static void
usblink_start(vm_offset_t tphy, vm_offset_t musb)
{
	thread_t thread;

	vl.musb = musb;
	vl.toprgu = ml_io_map(TOPRGU_PA, 0x1000);
	vl.log_pos = 0;                 /* whole ring: the boot so far */
	uint32_t std = 1;
	PE_parse_boot_argn_alias("mp_usblink_std", "vivausbstd", &std, sizeof(std));
	vl.std_allowed = std != 0;
	strlcpy(vl.serial, "0000000000000000000000000000000000000000", sizeof(vl.serial));
	printf("LINK: FIFO RAM %u bytes, standard persona %s\n", 4u << (rd8(musb, MUSB_RAMINFO) & 0xf),
	    vl.std_allowed ? "when mapple-usbd attaches" : "off (mp_usblink_std=0)");

	/* T-PHY u2: MediaTek usb_phy_recover (leave suspend, USB function, no BC1.1) */
	clr32(tphy, U3P_USBPHYACR7, 1u << 12);          /* PUPD_BIST_EN */
	clr32(tphy, U3P_U2PHYDTM0, 1u << 26);           /* FORCE_UART_EN */
	clr32(tphy, U3P_U2PHYDTM1, 1u << 16);           /* RG_UART_EN */
	clr32(tphy, U3P_U2PHYACR4, 3u << 8);            /* GPIO_CTL, USB20_GPIO_MODE */
	clr32(tphy, U3P_USBPHYACR6, 1u << 23);          /* BC11_SW_EN */
	set32(tphy, U3P_U2PHYACR4, 1u << 18);           /* DP_100K_MODE */
	clr32(tphy, U3P_U2PHYACR4, 3u << 16);           /* DP/DM_100K_EN */
	clr32(tphy, U3P_USBPHYACR5, 1u << 11);          /* HS_100U_U3_EN */
	set32(tphy, U3P_USBPHYACR6, 1u << 20);          /* OTG_VBUSCMP_EN */
	clr32(tphy, U3P_U2PHYDTM0, 1u << 18);           /* FORCE_SUSPENDM */
	clr32(tphy, U3P_U2PHYDTM0, (3u << 6) | (3u << 4) | (1u << 2) | (0xfu << 10));
	clr32(tphy, U3P_U2PHYDTM0, (1u << 17) | (1u << 19) | (1u << 20) | (1u << 21) | (1u << 23));
	delay(800);
	/* device mode: VBUSVALID, AVALID, BVALID, IDDIG, IDPULLUP = 1, SESSEND = 0, all forced */
	clr32(tphy, U3P_U2PHYDTM1, 1u << 4);
	set32(tphy, U3P_U2PHYDTM1, 0x2fu);
	set32(tphy, U3P_U2PHYDTM1, 0x3fu << 8);
	delay(1000);
	printf("LINK: phy dtm0 0x%08x dtm1 0x%08x acr6 0x%08x\n",
	    rd32(tphy, U3P_U2PHYDTM0), rd32(tphy, U3P_U2PHYDTM1), rd32(tphy, U3P_USBPHYACR6));

	/* MUSB: disconnected, interrupts masked at the MediaTek level, then connect */
	wr8(musb, MUSB_POWER, rd8(musb, MUSB_POWER) & ~POWER_SOFTCONN);
	wr32(musb, MTK_L1INTM, 0);
	wr8(musb, MUSB_INTRUSBE, 0);
	wr16(musb, MUSB_INTRTXE, 0);
	wr16(musb, MUSB_INTRRXE, 0);
	wr8(musb, MUSB_INTRUSB, rd8(musb, MUSB_INTRUSB));
	wr16(musb, MUSB_INTRTX, rd16(musb, MUSB_INTRTX));
	wr16(musb, MUSB_INTRRX, rd16(musb, MUSB_INTRRX));
	wr8(musb, MUSB_FADDR, 0);
	wr8(musb, MUSB_INDEX, 0);
	wr16(musb, MUSB_INTRTXE, 0x0003);               /* ep0, ep1 IN */
	wr16(musb, MUSB_INTRRXE, 0x0002);               /* ep1 OUT */
	wr8(musb, MUSB_INTRUSBE, INTR_RESET | INTR_RESUME | INTR_SUSPEND | INTR_DISCON);
	wr8(musb, MUSB_DEVCTL, rd8(musb, MUSB_DEVCTL) | DEVCTL_SESSION);
	wr8(musb, MUSB_POWER, POWER_HSENAB | POWER_SOFTCONN);
	printf("LINK: connected (power 0x%02x devctl 0x%02x), waiting for the host\n",
	    rd8(musb, MUSB_POWER), rd8(musb, MUSB_DEVCTL));

	if (kernel_thread_start(vl_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
#endif /* ARM64_BOARD_CONFIG_MT6781 */
