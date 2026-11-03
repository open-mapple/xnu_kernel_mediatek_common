/*
 * viva (MT6781): MT6366 PMIC (MT6358 register layout) on the pwrap bus.
 * Matches /arm-io/pwrap/pmic ("mediatek,mt6366-pmic").
 *
 * The PMIC has one interrupt line (EINT 144, level high, via VivaEINT.cpp)
 * for all its interrupt groups. As Linux drivers/mfd/mt6358-core.c does:
 * at start every group's interrupts are disabled and pending ones cleared
 * (LK leaves some on, e.g. charger detection, which would hold the line
 * high), then children enable only what they handle with
 * registerGroupInterrupt(). The line is taken through an
 * IOInterruptEventSource: pwrap access sleeps on a lock, so the groups are
 * serviced on this driver's work loop, while EINT keeps the line masked.
 *
 * PMIC writes go through VivaPwrap::writePMIC, which only accepts the
 * interrupt enable SET/CLR and status registers.
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include "VivaPMIC.h"

OSDefineMetaClassAndStructors(VivaPMIC, IOService);

IOWorkLoop *
VivaPMIC::getWorkLoop(void) const
{
	return fWorkLoop;
}

bool
VivaPMIC::start(IOService *provider)
{
	fPwrap = OSDynamicCast(VivaPwrap, provider->getProvider());
	fWorkLoop = IOWorkLoop::workLoop();
	if (fPwrap == NULL || fWorkLoop == NULL || !IOService::start(provider)) {
		IOLog("VIVA PMIC: no pwrap\n");
		return false;
	}
	uint32_t id = 0;
	if (read(VIVA_PMIC_HWCID, &id) != kIOReturnSuccess || (id & 0xff00) != 0x6600) {
		IOLog("VIVA PMIC: hwcid 0x%04x is not an MT6366, not touching it\n", id);
		return false;
	}

	/* Everything off and acknowledged; log what LK had left on. */
	for (unsigned int g = 0; g < kVivaPMICGroupCount; g++) {
		const VivaPMICGroupRegs *r = &gVivaPMICGroups[g];
		for (unsigned int n = 0; r->con0 != 0 && n < r->nregs; n++) {
			uint32_t con = 0, sta = 0;
			read(r->con0 + 6 * n, &con);
			read(r->sta0 + 2 * n, &sta);
			if (con | sta) {
				IOLog("VIVA PMIC: %s[%u] was enabled 0x%04x pending 0x%04x\n", r->name, n, con, sta);
			}
			fPwrap->writePMIC(r->con0 + 6 * n + 4, 0xffff);
			fPwrap->writePMIC(r->sta0 + 2 * n, 0xffff);
		}
	}

	fInterrupt = IOInterruptEventSource::interruptEventSource(this, &VivaPMIC::interruptOccurred, provider, 0);
	if (fInterrupt == NULL || fWorkLoop->addEventSource(fInterrupt) != kIOReturnSuccess) {
		IOLog("VIVA PMIC: no interrupt\n");
		return false;
	}
	fInterrupt->enable();
	IOLog("VIVA PMIC: MT6366 hwcid 0x%04x, all PMIC interrupts off, line on\n", id);
	registerService();
	VivaDTNub::publishChildren(this, provider);
	return true;
}

void
VivaPMIC::free(void)
{
	if (fInterrupt != NULL) {
		fInterrupt->disable();
		if (fWorkLoop != NULL) {
			fWorkLoop->removeEventSource(fInterrupt);
		}
	}
	OSSafeReleaseNULL(fInterrupt);
	OSSafeReleaseNULL(fWorkLoop);
	IOService::free();
}

IOReturn
VivaPMIC::registerGroupInterrupt(uint32_t group, uint32_t index, uint32_t bits, OSObject *target, Handler handler)
{
	if (group >= kVivaPMICGroupCount || gVivaPMICGroups[group].con0 == 0 ||
	    index >= gVivaPMICGroups[group].nregs || bits > 0xffff || handler == NULL) {
		return kIOReturnBadArgument;
	}
	const VivaPMICGroupRegs *r = &gVivaPMICGroups[group];
	IOReturn ret = fWorkLoop->runActionBlock(^IOReturn {
		fTarget[group] = target;
		fHandler[group] = handler;
		fPwrap->writePMIC(r->sta0 + 2 * index, bits);           /* stale ones */
		return fPwrap->writePMIC(r->con0 + 6 * index + 2, bits); /* enable */
	});
	IOLog("VIVA PMIC: %s[%u] bits 0x%04x -> %s (%d)\n", r->name, index, bits,
	    target ? target->getMetaClass()->getClassName() : "?", ret);
	return ret;
}

void
VivaPMIC::interruptOccurred(OSObject *owner, IOInterruptEventSource *src __unused, int count __unused)
{
	VivaPMIC *self = OSDynamicCast(VivaPMIC, owner);
	if (self != NULL) {
		self->service();
	}
}

/* Work loop: every pending group, until the PMIC lets the line go. */
void
VivaPMIC::service(void)
{
	fInterrupts++;
	for (int round = 0; round < 8; round++) {
		uint32_t top = 0;
		if (read(VIVA_PMIC_TOP_INT_STATUS0, &top) != kIOReturnSuccess || (top & 0x1ff) == 0) {
			return;
		}
		for (unsigned int g = 0; g < kVivaPMICGroupCount; g++) {
			const VivaPMICGroupRegs *r = &gVivaPMICGroups[g];
			if (!(top & (1u << g)) || r->con0 == 0) {
				continue;
			}
			for (unsigned int n = 0; n < r->nregs; n++) {
				uint32_t sta = 0;
				if (read(r->sta0 + 2 * n, &sta) != kIOReturnSuccess || sta == 0) {
					continue;
				}
				fPwrap->writePMIC(r->sta0 + 2 * n, sta);
				if (fHandler[g] != NULL) {
					fHandler[g](fTarget[g], g, n, sta);
				} else {
					/* nobody asked for it: off */
					fStray++;
					fPwrap->writePMIC(r->con0 + 6 * n + 4, sta);
					IOLog("VIVA PMIC: stray %s[%u] 0x%04x disabled\n", r->name, n, sta);
				}
			}
		}
	}
	IOLog("VIVA PMIC: line still pending after 8 rounds\n");
}
#endif /* SWEET */
