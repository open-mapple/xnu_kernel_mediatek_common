/*
 * viva (MT6781): MediaTek external interrupt controller (EINT, 0x1000b000)
 * as an IOKit interrupt controller cascaded under the GIC (SweetGIC.cpp).
 * Matches /arm-io/eint ("mediatek,mt6781-eint"); its own interrupt is GIC
 * SPI 186 (level high), taken in primary interrupt context.
 *
 * Children use interrupt-parent = the eint phandle and interrupts =
 * <EINT flags> (Linux IRQ_TYPE_*: 1 rising, 2 falling, 4 high, 8 low; both
 * edges are not supported). EINT n is bit n % 32 of the register at
 * offset + 4 * (n / 32). Register layout and handling as Linux
 * drivers/pinctrl/mediatek/mtk-eint.c (Xiaomi viva-r-oss): MT6781 has 151
 * AP EINTs; sens 1 = level, pol 1 = high/rising, mask 1 = masked, ack
 * clears the latched status, dom_en routes the line to the AP.
 *
 * Only lines a driver registers are touched (Linux additionally sets every
 * dom_en bit at probe). EINT lines that are GPIO pins must already be in
 * their EINT mode: on viva the PMIC's EINT 144 is pin 202 (EINT only) and
 * the touch's EINT 13 is GPIO13 mode 0.
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

#define EINT_STA                0x000
#define EINT_ACK                0x040
#define EINT_MASK               0x080
#define EINT_MASK_SET           0x0c0
#define EINT_MASK_CLR           0x100
#define EINT_SENS_SET           0x180
#define EINT_SENS_CLR           0x1c0
#define EINT_POL_SET            0x340
#define EINT_POL_CLR            0x380
#define EINT_DOM_EN             0x400

#define EINT_AP_NUM             151

#define IRQ_TYPE_EDGE_RISING    1
#define IRQ_TYPE_EDGE_FALLING   2
#define IRQ_TYPE_LEVEL_HIGH     4
#define IRQ_TYPE_LEVEL_LOW      8

class VivaEINT : public IOInterruptController {
	OSDeclareDefaultStructors(VivaEINT);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;

	virtual IOReturn registerInterrupt(IOService *nub, int source, void *target,
	    IOInterruptHandler handler, void *refCon) APPLE_KEXT_OVERRIDE;
	virtual IOReturn getInterruptType(IOService *nub, int source, int *interruptType) APPLE_KEXT_OVERRIDE;
	virtual IOReturn handleInterrupt(void *refCon, IOService *nub, int source) APPLE_KEXT_OVERRIDE;

	virtual int  getVectorType(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
	virtual void disableVectorHard(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;
	virtual void enableVector(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector) APPLE_KEXT_OVERRIDE;

private:
	bool spec(IOService *nub, int source, uint32_t *eint, uint32_t *flags);
	inline uint32_t rd32(uint32_t off) { return *(volatile uint32_t *)(fRegs + off); }
	inline void wr32(uint32_t off, uint32_t v) { *(volatile uint32_t *)(fRegs + off) = v; }
	inline uint32_t reg(uint32_t base, uint32_t eint) { return base + 4 * (eint / 32); }

	IOMemoryMap *fMap;
	vm_offset_t  fRegs;
	uint8_t      fType[EINT_AP_NUM];
	uint32_t     fStray;
};

OSDefineMetaClassAndStructors(VivaEINT, IOInterruptController);

bool
VivaEINT::start(IOService *provider)
{
	if (!IOInterruptController::start(provider)) {
		return false;
	}
	fMap = provider->mapDeviceMemoryWithIndex(0);
	if (fMap == NULL) {
		IOLog("VIVA EINT: cannot map registers\n");
		return false;
	}
	fRegs = (vm_offset_t)fMap->getVirtualAddress();

	vectors = IONewZero(IOInterruptVector, EINT_AP_NUM);
	if (vectors == NULL) {
		return false;
	}
	for (unsigned int i = 0; i < EINT_AP_NUM; i++) {
		vectors[i].interruptLock = IOLockAlloc();
		if (vectors[i].interruptLock == NULL) {
			return false;
		}
	}

	const OSSymbol *name = IODTInterruptControllerName(provider);
	if (name == NULL) {
		return false;
	}
	getPlatform()->registerInterruptController((OSSymbol *)name, this);

	/* Our line into the GIC: primary interrupt context, dispatched below. */
	IOReturn ret = provider->registerInterrupt(0, this,
	    OSMemberFunctionCast(IOInterruptAction, this, &VivaEINT::handleInterrupt), NULL);
	if (ret == kIOReturnSuccess) {
		ret = provider->enableInterrupt(0);
	}
	IOLog("VIVA EINT: %s, %u lines, parent interrupt %s (0x%x); mask %08x %08x %08x %08x %08x\n",
	    name->getCStringNoCopy(), EINT_AP_NUM, ret == kIOReturnSuccess ? "on" : "FAILED", ret,
	    rd32(EINT_MASK), rd32(EINT_MASK + 4), rd32(EINT_MASK + 8), rd32(EINT_MASK + 12), rd32(EINT_MASK + 16));
	name->release();
	registerService();
	return ret == kIOReturnSuccess;
}

bool
VivaEINT::spec(IOService *nub, int source, uint32_t *eint, uint32_t *flags)
{
	OSArray *specs = OSDynamicCast(OSArray, nub->getProperty(gIOInterruptSpecifiersKey));
	OSData *data = specs ? OSDynamicCast(OSData, specs->getObject((unsigned int)source)) : NULL;

	if (data == NULL || data->getLength() < 2 * sizeof(uint32_t)) {
		return false;
	}
	const uint32_t *cells = (const uint32_t *)data->getBytesNoCopy();
	*eint = cells[0];
	*flags = cells[1];
	if (*eint >= EINT_AP_NUM) {
		return false;
	}
	uint32_t type = *flags & 0xf;
	if (type != IRQ_TYPE_EDGE_RISING && type != IRQ_TYPE_EDGE_FALLING &&
	    type != IRQ_TYPE_LEVEL_HIGH && type != IRQ_TYPE_LEVEL_LOW) {
		IOLog("VIVA EINT: %s EINT %u: unsupported type 0x%x\n", nub->getName(), *eint, *flags);
		return false;
	}
	return true;
}

IOReturn
VivaEINT::getInterruptType(IOService *nub, int source, int *interruptType)
{
	uint32_t eint, flags;

	if (interruptType == NULL) {
		return kIOReturnBadArgument;
	}
	if (!spec(nub, source, &eint, &flags)) {
		return kIOReturnNoInterrupt;
	}
	*interruptType = (flags & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING)) ? kIOInterruptTypeEdge : kIOInterruptTypeLevel;
	return kIOReturnSuccess;
}

IOReturn
VivaEINT::registerInterrupt(IOService *nub, int source, void *target,
    IOInterruptHandler handler, void *refCon)
{
	uint32_t eint, flags;

	if (!spec(nub, source, &eint, &flags)) {
		return kIOReturnNoInterrupt;
	}
	uint32_t bit = 1u << (eint % 32);
	bool edge = (flags & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING)) != 0;
	bool high = (flags & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_LEVEL_HIGH)) != 0;
	fType[eint] = edge ? kIOInterruptTypeEdge : kIOInterruptTypeLevel;

	wr32(reg(EINT_MASK_SET, eint), bit);
	wr32(reg(high ? EINT_POL_SET : EINT_POL_CLR, eint), bit);
	wr32(reg(edge ? EINT_SENS_CLR : EINT_SENS_SET, eint), bit);
	wr32(reg(EINT_DOM_EN, eint), rd32(reg(EINT_DOM_EN, eint)) | bit);
	wr32(reg(EINT_ACK, eint), bit);

	IOReturn ret = IOInterruptController::registerInterrupt(nub, source, target, handler, refCon);
	IOLog("VIVA EINT: %s interrupt %d -> EINT %u (%s %s) %s\n", nub->getName(), source, eint,
	    edge ? "edge" : "level", high ? "high" : "low", ret == kIOReturnSuccess ? "registered" : "refused");
	return ret;
}

/* Primary interrupt context (GIC SPI 186 on CPU 0). */
IOReturn
VivaEINT::handleInterrupt(void *refCon __unused, IOService *nub __unused, int source __unused)
{
	for (uint32_t base = 0; base < EINT_AP_NUM; base += 32) {
		uint32_t pending = rd32(reg(EINT_STA, base)) & ~rd32(reg(EINT_MASK, base));
		while (pending) {
			uint32_t b = (uint32_t)__builtin_ctz(pending);
			pending &= pending - 1;
			uint32_t eint = base + b;
			if (eint >= EINT_AP_NUM) {
				break;
			}
			IOInterruptVector *vector = &vectors[eint];

			wr32(reg(EINT_ACK, eint), 1u << b);
			vector->interruptActive = 1;
			OSMemoryBarrier();
			if (!vector->interruptRegistered) {
				fStray++;
				wr32(reg(EINT_MASK_SET, eint), 1u << b);
			} else if (!vector->interruptDisabledSoft) {
				vector->handler(vector->target, vector->refCon, vector->nub, vector->source);
			} else {
				/* soft-disabled (an IOInterruptEventSource is servicing it): mask until re-enabled */
				vector->interruptDisabledHard = 1;
				disableVectorHard(eint, vector);
			}
			vector->interruptActive = 0;
		}
	}
	return kIOReturnSuccess;
}

int
VivaEINT::getVectorType(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector __unused)
{
	return fType[vectorNumber];
}

void
VivaEINT::disableVectorHard(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector __unused)
{
	wr32(reg(EINT_MASK_SET, (uint32_t)vectorNumber), 1u << (vectorNumber % 32));
}

void
VivaEINT::enableVector(IOInterruptVectorNumber vectorNumber, IOInterruptVector *vector __unused)
{
	wr32(reg(EINT_MASK_CLR, (uint32_t)vectorNumber), 1u << (vectorNumber % 32));
}
#endif /* SWEET */
