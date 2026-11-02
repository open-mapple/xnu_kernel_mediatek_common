/*
 * viva (MT6781): MediaTek keypad controller ("kp", 0x10010000) as an HID
 * button service. Matches the /arm-io/keypad nub ("mediatek,mt6781-keypad")
 * and takes its edge interrupt through the GIC interrupt controller
 * (SweetGIC.cpp) on its own work loop; no polling.
 *
 * The controller scans the key matrix by itself (LK sets it up for its boot
 * menu) and raises an interrupt on every change; KP_MEM1..5 hold one bit per
 * matrix position, 0 = pressed (Linux drivers/input/keyboard/mt6779-keypad.c).
 * The ADT property "keymap" lists <bit usage> pairs: bit number in MEM1..5
 * (MEM1 bit 0 = 0) and a HID consumer-page usage (0xe9 volume up, 0xea
 * volume down, 0x30 power). Changes of unmapped bits are only logged (the
 * first change of each), which is how the map of a new board is found. On
 * viva one key shows up in 8 bits (0, 9, 34, 43, 68, 77, 102, 111 for
 * volume down = kpd-hw-init-map index 0); only the first is mapped.
 *
 * Events are consumer-control keyboard events, the same usage pair and
 * event kind as SweetHIDButtons and Apple's own button services.
 */
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOMemoryDescriptor.h>

#include "IOHIDEventService.h"
#include "IOHIDEvent.h"
#include "IOHIDUsageTables.h"

extern "C" uint64_t mach_absolute_time(void);

#define KP_STA          0x0000
#define KP_MEM(n)       (0x0004 + 4 * (n))      /* n = 0..4 */
#define KP_NUM_MEMS     5
#define KP_NUM_BITS     136                     /* MEM5 has 8 bits */
#define KP_MAX_KEYS     16

class VivaKeypad : public IOHIDEventService {
	OSDeclareDefaultStructors(VivaKeypad);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOWorkLoop *getWorkLoop(void) const APPLE_KEXT_OVERRIDE;

protected:
	virtual UInt32 getPrimaryUsagePage() APPLE_KEXT_OVERRIDE { return kHIDPage_Consumer; }
	virtual UInt32 getPrimaryUsage() APPLE_KEXT_OVERRIDE { return kHIDUsage_Csmr_ConsumerControl; }
	virtual UInt32 getProductID() APPLE_KEXT_OVERRIDE { return 0x6781; }
	virtual OSString *getProduct() APPLE_KEXT_OVERRIDE { return OSString::withCString("MediaTek keypad"); }

private:
	static void interruptOccurred(OSObject *owner, IOInterruptEventSource *src, int count);
	void scan(void);
	inline uint32_t rd32(uint32_t off) { return *(volatile uint32_t *)(fRegs + off); }

	IOWorkLoop             *fWorkLoop;
	IOInterruptEventSource *fInterrupt;
	IOMemoryMap            *fMap;
	vm_offset_t             fRegs;
	uint32_t                fState[KP_NUM_MEMS];
	uint32_t                fKeyBit[KP_MAX_KEYS];
	uint32_t                fKeyUsage[KP_MAX_KEYS];
	uint32_t                fNumKeys;
	uint32_t                fInterrupts;
	uint32_t                fLogged[KP_NUM_MEMS];   /* unmapped bits already logged */
};

OSDefineMetaClassAndStructors(VivaKeypad, IOHIDEventService);

IOWorkLoop *
VivaKeypad::getWorkLoop(void) const
{
	return fWorkLoop;
}

bool
VivaKeypad::start(IOService *provider)
{
	fWorkLoop = IOWorkLoop::workLoop();
	if (fWorkLoop == NULL || !IOHIDEventService::start(provider)) {
		return false;
	}
	fMap = provider->mapDeviceMemoryWithIndex(0);
	if (fMap == NULL) {
		IOLog("VIVA KP: cannot map registers\n");
		return false;
	}
	fRegs = (vm_offset_t)fMap->getVirtualAddress();

	OSData *map = OSDynamicCast(OSData, provider->getProperty("keymap"));
	if (map != NULL) {
		const uint32_t *cells = (const uint32_t *)map->getBytesNoCopy();
		for (unsigned int i = 0; i + 1 < map->getLength() / sizeof(uint32_t) && fNumKeys < KP_MAX_KEYS; i += 2) {
			if (cells[i] < KP_NUM_BITS) {
				fKeyBit[fNumKeys] = cells[i];
				fKeyUsage[fNumKeys] = cells[i + 1];
				fNumKeys++;
			}
		}
	}

	for (unsigned int i = 0; i < KP_NUM_MEMS; i++) {
		fState[i] = rd32(KP_MEM(i));
	}
	IOLog("VIVA KP: sta 0x%08x mem %08x %08x %08x %08x %08x, %u mapped keys\n", rd32(KP_STA),
	    fState[0], fState[1], fState[2], fState[3], fState[4], fNumKeys);

	fInterrupt = IOInterruptEventSource::interruptEventSource(this, &VivaKeypad::interruptOccurred, provider, 0);
	if (fInterrupt == NULL || fWorkLoop->addEventSource(fInterrupt) != kIOReturnSuccess) {
		IOLog("VIVA KP: no interrupt\n");
		return false;
	}
	fInterrupt->enable();
	registerService();
	return true;
}

void
VivaKeypad::free(void)
{
	if (fInterrupt != NULL) {
		fInterrupt->disable();
		if (fWorkLoop != NULL) {
			fWorkLoop->removeEventSource(fInterrupt);
		}
	}
	OSSafeReleaseNULL(fInterrupt);
	OSSafeReleaseNULL(fMap);
	IOHIDEventService::free();
	OSSafeReleaseNULL(fWorkLoop);
}

/* Work loop context. */
void
VivaKeypad::interruptOccurred(OSObject *owner, IOInterruptEventSource *src __unused, int count __unused)
{
	VivaKeypad *self = OSDynamicCast(VivaKeypad, owner);
	if (self != NULL) {
		self->fInterrupts++;
		self->scan();
	}
}

void
VivaKeypad::scan(void)
{
	for (unsigned int m = 0; m < KP_NUM_MEMS; m++) {
		uint32_t now = rd32(KP_MEM(m));
		uint32_t changed = now ^ fState[m];
		fState[m] = now;
		while (changed) {
			unsigned int b = (unsigned int)__builtin_ctz(changed);
			changed &= changed - 1;
			unsigned int bit = 32 * m + b;
			bool down = (now & (1u << b)) == 0;
			UInt32 usage = 0;
			for (uint32_t k = 0; k < fNumKeys; k++) {
				if (fKeyBit[k] == bit) {
					usage = fKeyUsage[k];
				}
			}
			if (usage == 0) {
				if (!(fLogged[m] & (1u << b))) {
					fLogged[m] |= 1u << b;
					IOLog("VIVA KP: unmapped bit %u %s (interrupt %u)\n", bit, down ? "down" : "up",
					    fInterrupts);
				}
				continue;
			}
			IOLog("VIVA KP: key 0x%x %s\n", (unsigned)usage, down ? "down" : "up");
			{
				IOHIDEvent *event = IOHIDEvent::keyboardEvent(mach_absolute_time(), kHIDPage_Consumer, usage, down);
				if (event != NULL) {
					dispatchEvent(event);
					event->release();
				}
			}
		}
	}
}
