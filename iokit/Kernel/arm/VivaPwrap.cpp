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
 * A write is WACS2_CMD = 1 << 31 | (addr >> 1) << 16 | data. The regulators
 * live in the same PMIC, so writePMIC() only accepts the interrupt enable
 * SET/CLR and status registers of the PMIC interrupt groups (writeAllowed(),
 * table in VivaPMIC.h), and writeRegulator() only the GPU rails' enable
 * and VOSEL fields under a voltage ceiling (VivaGPU.cpp); everything else
 * is refused and logged.
 *
 * The PMIC is a device-tree child (/arm-io/pwrap/pmic): published as a
 * VivaDTNub, matched by VivaPMIC. C entry point viva_pwrap_read() for the
 * USB link's "pmic" command (reads only).
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IODeviceTreeSupport.h>

#include "VivaPMIC.h"

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
	VivaDTNub::publishChildren(this, provider);
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

bool
VivaPwrap::writeAllowed(uint32_t addr)
{
	for (unsigned int g = 0; g < kVivaPMICGroupCount; g++) {
		const VivaPMICGroupRegs *r = &gVivaPMICGroups[g];
		for (unsigned int n = 0; r->con0 != 0 && n < r->nregs; n++) {
			uint32_t con = r->con0 + 6 * n;
			if (addr == con + 2 || addr == con + 4 || addr == (uint32_t)(r->sta0 + 2 * n)) {
				return true;
			}
		}
	}
	return false;
}

IOReturn
VivaPwrap::writePMIC(uint32_t addr, uint32_t value)
{
	if (!writeAllowed(addr) || value > 0xffff) {
		IOLog("VIVA PWRAP: refused PMIC write 0x%04x <- 0x%x\n", addr, value);
		return kIOReturnNotPermitted;
	}
	IOReturn ret = kIOReturnTimeout;
	IOLockLock(fLock);
	if (WACS_FSM(rd32(PWRAP_WACS2_RDATA)) == WACS_FSM_WFVLDCLR) {
		wr32(PWRAP_WACS2_VLDCLR, 1);
	}
	if (waitFSM(WACS_FSM_IDLE)) {
		wr32(PWRAP_WACS2_CMD, 0x80000000u | ((addr >> 1) << 16) | value);
		ret = kIOReturnSuccess;
	}
	IOLockUnlock(fLock);
	return ret;
}

/*
 * GPU rails (mt6358 registers.h / mt6358-regulator.c, viva-r-oss). VOSEL = (uV - 500000) / 6250;
 * ceilings 0.9 V (VGPU) and 1.0 V (VSRAM_GPU), the top of the mt6781 OPP table.
 */
static const struct {
	uint16_t addr, mask, max;
} gVivaRegulatorRegs[] = {
	{ 0x1508, 0x0001, 0x01 },       /* BUCK_VGPU_CON0: RG_BUCK_VGPU_EN */
	{ 0x1526, 0x007f, 0x40 },       /* BUCK_VGPU_ELR0: RG_BUCK_VGPU_VOSEL, <= 0.9 V */
	{ 0x1bc8, 0x0001, 0x01 },       /* LDO_VSRAM_GPU_CON0: RG_LDO_VSRAM_GPU_EN */
	{ 0x1bf4, 0x007f, 0x50 },       /* LDO_VSRAM_CON3: RG_LDO_VSRAM_GPU_VOSEL, <= 1.0 V */
	/* CONNSYS (bsd/dev/viva_wifi.c, mt6781.c consys_hw_vcn18_ctrl): VCN18 enable/mode bits only, no voltage */
	{ 0x1c58, 0x0003, 0x03 },       /* LDO_VCN18_CON0: RG_LDO_VCN18_EN, RG_LDO_VCN18_LP */
	{ 0x1c5a, 0x0003, 0x03 },       /* LDO_VCN18_OP_EN: SW_OP_EN, HW0_OP_EN */
	{ 0x1c60, 0x0002, 0x01 },       /* LDO_VCN18_OP_CFG: HW0_OP_CFG */
};

IOReturn
VivaPwrap::writeRegulator(uint32_t addr, uint32_t mask, uint32_t value)
{
	unsigned int i;
	for (i = 0; i < sizeof(gVivaRegulatorRegs) / sizeof(gVivaRegulatorRegs[0]); i++) {
		if (gVivaRegulatorRegs[i].addr == addr) {
			break;
		}
	}
	if (i == sizeof(gVivaRegulatorRegs) / sizeof(gVivaRegulatorRegs[0]) || mask == 0 ||
	    (mask & ~gVivaRegulatorRegs[i].mask) || (value & ~mask) ||
	    value / (mask & -mask) > gVivaRegulatorRegs[i].max) {
		IOLog("VIVA PWRAP: refused regulator write 0x%04x mask 0x%x <- 0x%x\n", addr, mask, value);
		return kIOReturnNotPermitted;
	}
	uint32_t old = 0, now = 0;
	IOReturn ret = readPMIC(addr, &old);
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	uint32_t v = (old & ~mask) | value;
	IOLockLock(fLock);
	ret = kIOReturnTimeout;
	if (WACS_FSM(rd32(PWRAP_WACS2_RDATA)) == WACS_FSM_WFVLDCLR) {
		wr32(PWRAP_WACS2_VLDCLR, 1);
	}
	if (waitFSM(WACS_FSM_IDLE)) {
		wr32(PWRAP_WACS2_CMD, 0x80000000u | ((addr >> 1) << 16) | (v & 0xffff));
		ret = waitFSM(WACS_FSM_IDLE) ? kIOReturnSuccess : kIOReturnTimeout;
	}
	IOLockUnlock(fLock);
	if (ret == kIOReturnSuccess && (readPMIC(addr, &now) != kIOReturnSuccess || (now & mask) != value)) {
		ret = kIOReturnIOError;
	}
	IOLog("VIVA PWRAP: regulator 0x%04x 0x%04x -> 0x%04x (reads 0x%04x) %s\n", addr, old, v, now,
	    ret == kIOReturnSuccess ? "ok" : "FAILED");
	return ret;
}

VivaPwrap *
VivaPwrap::wait(void)
{
	for (int i = 0; i < 100 && gVivaPwrap == NULL; i++) {
		IOSleep(50);
	}
	return gVivaPwrap;
}

/* Device-tree children of the PMIC bus. */
OSDefineMetaClassAndStructors(VivaDTNub, IOService);

bool
VivaDTNub::compareName(OSString *name, OSString **matched) const
{
	return IODTCompareNubName(this, name, matched);
}

void
VivaDTNub::publishChildren(IOService *parent, IORegistryEntry *dtEntry)
{
	OSIterator *iter = dtEntry->getChildIterator(gIODTPlane);
	if (iter == NULL) {
		return;
	}
	OSArray *children = OSArray::withCapacity(4);
	IORegistryEntry *child;
	while (children && (child = (IORegistryEntry *)iter->getNextObject())) {
		children->setObject(child);     /* init() below re-parents the entry: iterate a copy */
	}
	iter->release();
	for (unsigned int i = 0; children && i < children->getCount(); i++) {
		child = (IORegistryEntry *)children->getObject(i);
		VivaDTNub *nub = new VivaDTNub;
		if (nub == NULL) {
			continue;
		}
		if (!nub->init(child, gIODTPlane) || !nub->attach(parent)) {
			nub->release();
			continue;
		}
		nub->registerService();
		nub->release();
	}
	OSSafeReleaseNULL(children);
}

/* bsd/dev/viva_wifi.c: the whitelisted regulator fields (writeRegulator). */
extern "C" int
viva_pwrap_write_regulator(uint32_t addr, uint32_t mask, uint32_t value)
{
	if (gVivaPwrap == NULL) {
		return -1;
	}
	return gVivaPwrap->writeRegulator(addr, mask, value) == kIOReturnSuccess ? 0 : -2;
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
