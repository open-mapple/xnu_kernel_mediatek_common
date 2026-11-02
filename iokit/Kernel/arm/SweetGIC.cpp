/*
 * SWEET: GICv3 shared peripheral interrupts (SPIs) as an IOKit interrupt
 * controller, so device drivers take their interrupts the IOKit way
 * (IOInterruptEventSource / IOFilterInterruptEventSource on their work
 * loop) instead of polling.
 *
 * Matches the /arm-io/gic nub ("arm,gic-v3"). The platform code
 * (pexpert/arm/pe_fiq.c) has already set up the distributor, the
 * redistributors and the CPU interface; this driver only owns SPIs.
 * sleh_irq() hands every INTID that is not the timer or an IPI to the
 * handler installed with ml_install_interrupt_handler(), i.e. to
 * handleInterrupt() here, with the INTID as the source; it writes EOIR
 * after the handler returns.
 *
 * Device tree (tools/mkadt.py): the gic node has #interrupt-cells = 2 and
 * an AAPL,phandle; a device node has interrupt-parent = that phandle and
 * interrupts = <INTID flags> pairs. INTID is the absolute GIC number
 * (SPI n = 32 + n), so it is also the IOInterruptController vector number;
 * flags are Linux IRQ_TYPE_* bits (1 edge rising, 4 level high). The GIC
 * cannot invert polarity: falling-edge and level-low lines are refused.
 *
 * Every SPI is routed to CPU 0 (MPIDR 0), the only CPU whose interrupt
 * handler this driver installs.
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOInterruptController.h>
#include <IOKit/IOInterrupts.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <machine/machine_routines.h>

extern "C" {
processor_t cpu_to_processor(int cpu);
processor_t thread_bind(processor_t processor);
}

#define GICD_CTLR               0x0000
#define GICD_TYPER              0x0004
#define GICD_IGROUPR(n)         (0x0080 + 4 * ((n) / 32))
#define GICD_ISENABLER(n)       (0x0100 + 4 * ((n) / 32))
#define GICD_ICENABLER(n)       (0x0180 + 4 * ((n) / 32))
#define GICD_ICPENDR(n)         (0x0280 + 4 * ((n) / 32))
#define GICD_IPRIORITYR(n)      (0x0400 + (n))
#define GICD_ICFGR(n)           (0x0c00 + 4 * ((n) / 16))
#define GICD_IROUTER(n)         (0x6000 + 8 * (n))
#define GICD_CTLR_RWP           (1u << 31)

#define IRQ_TYPE_EDGE_RISING    1
#define IRQ_TYPE_EDGE_FALLING   2
#define IRQ_TYPE_LEVEL_HIGH     4
#define IRQ_TYPE_LEVEL_LOW      8

#define FIRST_SPI               32
#define MAX_INTID               1020

class SweetGIC : public IOInterruptController {
	OSDeclareDefaultStructors(SweetGIC);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;

	virtual IOReturn registerInterrupt(IOService *nub, int source, void *target,
	    IOInterruptHandler handler, void *refCon) APPLE_KEXT_OVERRIDE;
	virtual IOReturn getInterruptType(IOService *nub, int source, int *interruptType) APPLE_KEXT_OVERRIDE;
	virtual IOInterruptAction getInterruptHandlerAddress(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn handleInterrupt(void *refCon, IOService *nub, int source) APPLE_KEXT_OVERRIDE;

	virtual bool vectorCanBeShared(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
	virtual int  getVectorType(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
	virtual void disableVectorHard(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
	virtual void enableVector(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;

private:
	bool spec(IOService *nub, int source, uint32_t *intid, uint32_t *flags);
	void wait_rwp(void);
	inline uint32_t rd32(uint32_t off) { return *(volatile uint32_t *)(fGICD + off); }
	inline void wr32(uint32_t off, uint32_t v) { *(volatile uint32_t *)(fGICD + off) = v; }

	IOMemoryMap *fMap;
	vm_offset_t  fGICD;
	uint32_t     fNumIntids;
	uint8_t     *fType;         /* kIOInterruptTypeEdge / Level, per INTID */
	uint32_t     fSpurious;
};

OSDefineMetaClassAndStructors(SweetGIC, IOInterruptController);

bool
SweetGIC::start(IOService *provider)
{
	if (!IOInterruptController::start(provider)) {
		return false;
	}
	/* reg = <GICD base, size, GICR base, size>, mapped by the nub (ranges applied) */
	fMap = provider->mapDeviceMemoryWithIndex(0);
	if (fMap == NULL) {
		IOLog("SWEET GIC: cannot map the distributor\n");
		return false;
	}
	fGICD = (vm_offset_t)fMap->getVirtualAddress();

	uint32_t typer = rd32(GICD_TYPER);
	fNumIntids = 32 * ((typer & 0x1f) + 1);
	if (fNumIntids > MAX_INTID) {
		fNumIntids = MAX_INTID;
	}
	vectors = IONewZero(IOInterruptVector, fNumIntids);
	fType = IONewZero(uint8_t, fNumIntids);
	if (vectors == NULL || fType == NULL) {
		return false;
	}
	for (uint32_t i = 0; i < fNumIntids; i++) {
		vectors[i].interruptLock = IOLockAlloc();
		if (vectors[i].interruptLock == NULL) {
			return false;
		}
	}

	/* SPIs are delivered to CPU 0: install the handler there. */
	thread_bind(cpu_to_processor(0));
	thread_block(THREAD_CONTINUE_NULL);
	ml_install_interrupt_handler(this, 0, this,
	    OSMemberFunctionCast(IOInterruptHandler, this, &SweetGIC::handleInterrupt), NULL);
	thread_bind(PROCESSOR_NULL);
	thread_block(THREAD_CONTINUE_NULL);

	const OSSymbol *name = IODTInterruptControllerName(provider);
	if (name == NULL) {
		IOLog("SWEET GIC: the gic node has no AAPL,phandle\n");
		return false;
	}
	getPlatform()->registerInterruptController((OSSymbol *)name, this);
	IOLog("SWEET GIC: %s, %u INTIDs (GICD_TYPER 0x%08x, GICD_CTLR 0x%08x)\n",
	    name->getCStringNoCopy(), fNumIntids, typer, rd32(GICD_CTLR));
	name->release();
	registerService();
	return true;
}

void
SweetGIC::wait_rwp(void)
{
	for (int i = 0; i < 100000 && (rd32(GICD_CTLR) & GICD_CTLR_RWP); i++) {
	}
}

/* The specifier <INTID flags> of a nub's interrupt `source`. */
bool
SweetGIC::spec(IOService *nub, int source, uint32_t *intid, uint32_t *flags)
{
	OSArray *specs = OSDynamicCast(OSArray, nub->getProperty(gIOInterruptSpecifiersKey));
	OSData *data = specs ? OSDynamicCast(OSData, specs->getObject((unsigned int)source)) : NULL;

	if (data == NULL || data->getLength() < sizeof(uint32_t)) {
		return false;
	}
	const uint32_t *cells = (const uint32_t *)data->getBytesNoCopy();
	*intid = cells[0];
	*flags = data->getLength() >= 2 * sizeof(uint32_t) ? cells[1] : IRQ_TYPE_LEVEL_HIGH;
	if (*intid < FIRST_SPI || *intid >= fNumIntids) {
		IOLog("SWEET GIC: %s interrupt %d: INTID %u is not an SPI\n", nub->getName(), source, *intid);
		return false;
	}
	if (*flags & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_LEVEL_LOW)) {
		IOLog("SWEET GIC: %s INTID %u: type 0x%x needs a polarity inverter\n", nub->getName(), *intid, *flags);
		return false;
	}
	return true;
}

IOReturn
SweetGIC::getInterruptType(IOService *nub, int source, int *interruptType)
{
	uint32_t intid, flags;

	if (interruptType == NULL) {
		return kIOReturnBadArgument;
	}
	if (!spec(nub, source, &intid, &flags)) {
		return kIOReturnNoInterrupt;
	}
	*interruptType = (flags & IRQ_TYPE_EDGE_RISING) ? kIOInterruptTypeEdge : kIOInterruptTypeLevel;
	return kIOReturnSuccess;
}

IOReturn
SweetGIC::registerInterrupt(IOService *nub, int source, void *target,
    IOInterruptHandler handler, void *refCon)
{
	uint32_t intid, flags;

	if (!spec(nub, source, &intid, &flags)) {
		return kIOReturnNoInterrupt;
	}
	bool edge = (flags & IRQ_TYPE_EDGE_RISING) != 0;
	fType[intid] = edge ? kIOInterruptTypeEdge : kIOInterruptTypeLevel;

	/* Off while it is configured: group 1 NS, priority, trigger, CPU 0. */
	wr32(GICD_ICENABLER(intid), 1u << (intid % 32));
	wait_rwp();
	wr32(GICD_IGROUPR(intid), rd32(GICD_IGROUPR(intid)) | (1u << (intid % 32)));
	*(volatile uint8_t *)(fGICD + GICD_IPRIORITYR(intid)) = 0xa0;
	uint32_t shift = 2 * (intid % 16) + 1;
	uint32_t cfg = rd32(GICD_ICFGR(intid));
	wr32(GICD_ICFGR(intid), edge ? (cfg | (1u << shift)) : (cfg & ~(1u << shift)));
	*(volatile uint64_t *)(fGICD + GICD_IROUTER(intid)) = 0;
	wr32(GICD_ICPENDR(intid), 1u << (intid % 32));

	IOReturn ret = IOInterruptController::registerInterrupt(nub, source, target, handler, refCon);
	IOLog("SWEET GIC: %s interrupt %d -> INTID %u (%s) %s\n", nub->getName(), source, intid,
	    edge ? "edge" : "level", ret == kIOReturnSuccess ? "registered" : "refused");
	return ret;
}

IOInterruptAction
SweetGIC::getInterruptHandlerAddress(void)
{
	return OSMemberFunctionCast(IOInterruptAction, this, &SweetGIC::handleInterrupt);
}

/* Interrupt context, CPU 0; sleh_irq writes EOIR afterwards. */
IOReturn
SweetGIC::handleInterrupt(void *refCon __unused, IOService *nub __unused, int source)
{
	if (source < FIRST_SPI || (uint32_t)source >= fNumIntids) {
		fSpurious++;
		return kIOReturnInvalid;
	}
	IOInterruptVector *vector = &vectors[source];

	vector->interruptActive = 1;
	OSMemoryBarrier();
	if (!vector->interruptRegistered) {
		/* nobody asked for it: keep it off */
		fSpurious++;
		wr32(GICD_ICENABLER(source), 1u << (source % 32));
	} else if (!vector->interruptDisabledSoft) {
		timeStampInterruptHandlerStart(source, vector);
		vector->handler(vector->target, vector->refCon, vector->nub, vector->source);
		timeStampInterruptHandlerEnd(source, vector);
	} else {
		/* Soft-disabled (an IOInterruptEventSource is servicing it): mask until re-enabled. */
		vector->interruptDisabledHard = 1;
		disableVectorHard(source, vector);
	}
	vector->interruptActive = 0;
	return kIOReturnSuccess;
}

bool
SweetGIC::vectorCanBeShared(IOInterruptVectorNumber vectorNumber __unused, IOInterruptVector *vector __unused)
{
	return true;
}

int
SweetGIC::getVectorType(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector __unused)
{
	return fType[vectorNumber];
}

void
SweetGIC::disableVectorHard(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector __unused)
{
	wr32(GICD_ICENABLER(vectorNumber), 1u << (vectorNumber % 32));
}

void
SweetGIC::enableVector(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector __unused)
{
	wr32(GICD_ISENABLER(vectorNumber), 1u << (vectorNumber % 32));
}
#endif /* SWEET */
