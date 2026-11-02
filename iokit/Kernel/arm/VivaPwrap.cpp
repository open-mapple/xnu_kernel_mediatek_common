/*
 * viva (MT6781): MediaTek PMIC wrapper ("pwrap", 0x1000d000), the AP's
 * channel to the MT6366 PMIC (MT6358 register layout). Matches the
 * /arm-io/pwrap nub ("mediatek,mt6781-pwrap").
 *
 * LK initializes the wrapper; this driver only uses the AP's software
 * channel WACS2 as Xiaomi's kernel does for this SoC
 * (drivers/soc/mediatek/mtk-pmic-wrap.c, pwrap_mt6781, viva-r-oss):
 *   wait WACS2_RDATA.FSM == IDLE, WACS2_CMD = (addr >> 1) << 16,
 *   wait FSM == WFVLDCLR, data = WACS2_RDATA[15:0], WACS2_VLDCLR = 1.
 * WACS2_CMD bit 31 would make it a PMIC *write*. This step is read-only:
 * there is no write path, and readPMIC() refuses anything that could set
 * bit 31. The regulators live in the same PMIC; writes come later, behind
 * an explicit register whitelist.
 *
 * C entry point viva_pwrap_read() for the USB link's "pmic" command.
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOLocks.h>

#define PWRAP_INIT_DONE0        0x090
#define PWRAP_WACS2_EN          0x09c
#define PWRAP_WACS2_CMD         0xc20
#define PWRAP_WACS2_RDATA       0xc24
#define PWRAP_WACS2_VLDCLR      0xc28

#define WACS_FSM(x)             (((x) >> 16) & 7)
#define WACS_FSM_IDLE           0
#define WACS_FSM_WFVLDCLR       6
#define WACS_INIT_DONE0_V2      (1u << 22)

/* MT6358/MT6366 registers (include/linux/mfd/mt6358/registers.h) */
#define MT6358_HWCID            0x008
#define MT6358_SWCID            0x00a
#define MT6358_TOPSTATUS        0x028   /* bit 1 PWRKEY_DEB, bit 3 HOMEKEY_DEB (0 = pressed) */
#define MT6358_TOP_INT_STATUS0  0x19e
#define MT6358_PSC_TOP_INT_CON0 0x910   /* bit 0 pwrkey, 1 homekey, 2 pwrkey_r, 3 homekey_r */
#define MT6358_PSC_TOP_INT_MASK_CON0 0x916
#define MT6358_PSC_TOP_INT_STATUS0 0x91c
#define MT6358_PSC_TOP_INT_RAW_STATUS0 0x91e

class VivaPwrap : public IOService {
	OSDeclareDefaultStructors(VivaPwrap);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	IOReturn readPMIC(uint32_t addr, uint32_t *value);

private:
	bool waitFSM(uint32_t fsm);
	inline uint32_t rd32(uint32_t off) { return *(volatile uint32_t *)(fRegs + off); }
	inline void wr32(uint32_t off, uint32_t v) { *(volatile uint32_t *)(fRegs + off) = v; }

	IOMemoryMap *fMap;
	vm_offset_t  fRegs;
	IOLock      *fLock;
	uint32_t     fTimeouts;
};

OSDefineMetaClassAndStructors(VivaPwrap, IOService);

static VivaPwrap *gVivaPwrap;

bool
VivaPwrap::start(IOService *provider)
{
	if (!IOService::start(provider)) {
		return false;
	}
	fLock = IOLockAlloc();
	fMap = provider->mapDeviceMemoryWithIndex(0);
	if (fLock == NULL || fMap == NULL) {
		IOLog("VIVA PWRAP: cannot map registers\n");
		return false;
	}
	fRegs = (vm_offset_t)fMap->getVirtualAddress();

	uint32_t rdata = rd32(PWRAP_WACS2_RDATA);
	IOLog("VIVA PWRAP: init_done0 0x%x wacs2_en 0x%x wacs2_rdata 0x%08x (fsm %u, init %u)\n",
	    rd32(PWRAP_INIT_DONE0), rd32(PWRAP_WACS2_EN), rdata, WACS_FSM(rdata),
	    (rdata & WACS_INIT_DONE0_V2) != 0);
	if (!(rdata & WACS_INIT_DONE0_V2)) {
		IOLog("VIVA PWRAP: WACS2 not initialized by LK, not using it\n");
		return false;
	}

	uint32_t hw = 0, sw = 0, top = 0, topint = 0, con = 0, mask = 0, sta = 0, raw = 0;
	IOReturn ret = readPMIC(MT6358_HWCID, &hw);
	if (ret == kIOReturnSuccess) {
		readPMIC(MT6358_SWCID, &sw);
		readPMIC(MT6358_TOPSTATUS, &top);
		readPMIC(MT6358_TOP_INT_STATUS0, &topint);
		readPMIC(MT6358_PSC_TOP_INT_CON0, &con);
		readPMIC(MT6358_PSC_TOP_INT_MASK_CON0, &mask);
		readPMIC(MT6358_PSC_TOP_INT_STATUS0, &sta);
		readPMIC(MT6358_PSC_TOP_INT_RAW_STATUS0, &raw);
	}
	IOLog("VIVA PWRAP: PMIC hwcid 0x%04x swcid 0x%04x (%s) topstatus 0x%04x top_int 0x%04x "
	    "psc con 0x%04x mask 0x%04x sta 0x%04x raw 0x%04x (%d)\n", hw, sw,
	    (hw & 0xff00) == 0x6600 ? "MT6366" : (hw & 0xff00) == 0x5800 ? "MT6358" : "unknown",
	    top, topint, con, mask, sta, raw, ret);

	gVivaPwrap = this;
	registerService();
	return true;
}

bool
VivaPwrap::waitFSM(uint32_t fsm)
{
	for (int i = 0; i < 10000; i++) {       /* ~10 ms, like Linux */
		if (WACS_FSM(rd32(PWRAP_WACS2_RDATA)) == fsm) {
			return true;
		}
		IODelay(1);
	}
	fTimeouts++;
	return false;
}

IOReturn
VivaPwrap::readPMIC(uint32_t addr, uint32_t *value)
{
	if (value == NULL || (addr & 1) || addr > 0xfffe) {
		return kIOReturnBadArgument;
	}
	uint32_t cmd = (addr >> 1) << 16;       /* bit 31 clear: a read */
	if (cmd & 0x80000000u) {
		return kIOReturnNotPermitted;
	}
	IOReturn ret = kIOReturnTimeout;
	IOLockLock(fLock);
	if (WACS_FSM(rd32(PWRAP_WACS2_RDATA)) == WACS_FSM_WFVLDCLR) {
		wr32(PWRAP_WACS2_VLDCLR, 1);    /* a previous read left it waiting */
	}
	if (waitFSM(WACS_FSM_IDLE)) {
		wr32(PWRAP_WACS2_CMD, cmd);
		if (waitFSM(WACS_FSM_WFVLDCLR)) {
			*value = rd32(PWRAP_WACS2_RDATA) & 0xffff;
			wr32(PWRAP_WACS2_VLDCLR, 1);
			ret = kIOReturnSuccess;
		}
	}
	IOLockUnlock(fLock);
	return ret;
}

extern "C" int
viva_pwrap_read(uint32_t addr, uint32_t *value)
{
	if (gVivaPwrap == NULL) {
		return -1;
	}
	return gVivaPwrap->readPMIC(addr, value) == kIOReturnSuccess ? 0 : -2;
}
#endif /* SWEET */
