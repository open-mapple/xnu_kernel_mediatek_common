/*
 * MT6781: the MT6366 PMIC's key inputs as an HID button service.
 * Matches /arm-io/pwrap/pmic/keys ("mediatek,mt6366-keys"), a child of
 * MT6358PMIC (../MT6358PMIC.cpp). On viva PWRKEY is the side (power) button and
 * HOMEKEY is volume up (live DT: kpd-hw-pwrkey / kpd-hw-rstkey,
 * CONFIG_KPD_PWRKEY_USE_PMIC).
 *
 * The PMIC raises PSC interrupts on press (pwrkey/homekey) and release
 * (pwrkey_r/homekey_r); the state is then read from TOPSTATUS (PWRKEY_DEB
 * bit 1, HOMEKEY_DEB bit 3, 0 = pressed; measured 2026-11-03), so a lost
 * edge cannot leave a key stuck. The ADT property "keymap" = <key usage>
 * pairs: key 0 = PWRKEY, 1 = HOMEKEY, usage on the HID consumer page.
 */
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOWorkLoop.h>

#include "IOHIDEventService.h"
#include "IOHIDEvent.h"
#include "IOHIDUsageTables.h"
#include "../MT6358PMIC.h"

extern "C" uint64_t mach_absolute_time(void);

#define PMIC_KEYS       2
static const uint32_t kKeyDebBit[PMIC_KEYS] = { 1u << 1, 1u << 3 };    /* TOPSTATUS */
static const uint32_t kKeyPress[PMIC_KEYS] = { MT6358_PMIC_PSC_PWRKEY, MT6358_PMIC_PSC_HOMEKEY };
static const uint32_t kKeyRelease[PMIC_KEYS] = { MT6358_PMIC_PSC_PWRKEY_R, MT6358_PMIC_PSC_HOMEKEY_R };

class MT6358Keys : public IOHIDEventService {
	OSDeclareDefaultStructors(MT6358Keys);

public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual IOWorkLoop *getWorkLoop(void) const APPLE_KEXT_OVERRIDE;

protected:
	virtual UInt32 getPrimaryUsagePage() APPLE_KEXT_OVERRIDE { return kHIDPage_Consumer; }
	virtual UInt32 getPrimaryUsage() APPLE_KEXT_OVERRIDE { return kHIDUsage_Csmr_ConsumerControl; }
	virtual UInt32 getProductID() APPLE_KEXT_OVERRIDE { return 0x6366; }
	virtual OSString *getProduct() APPLE_KEXT_OVERRIDE { return OSString::withCString("MT6366 keys"); }

private:
	static void pmicInterrupt(OSObject *target, uint32_t group, uint32_t index, uint32_t status);
	void update(uint32_t status);
	void send(unsigned int key, bool down);

	MT6358PMIC *fPMIC;
	UInt32    fUsage[PMIC_KEYS];
	bool      fDown[PMIC_KEYS];
	uint32_t  fEvents;
};

OSDefineMetaClassAndStructors(MT6358Keys, IOHIDEventService);

/* The PMIC's work loop: interrupts and HID clients are serialized on it. */
IOWorkLoop *
MT6358Keys::getWorkLoop(void) const
{
	return fPMIC ? fPMIC->getWorkLoop() : NULL;
}

bool
MT6358Keys::start(IOService *provider)
{
	fPMIC = OSDynamicCast(MT6358PMIC, provider->getProvider());
	if (fPMIC == NULL || !IOHIDEventService::start(provider)) {
		IOLog("PMIC KEYS: no PMIC\n");
		return false;
	}
	fUsage[0] = kHIDUsage_Csmr_Power;
	fUsage[1] = kHIDUsage_Csmr_VolumeIncrement;
	OSData *map = OSDynamicCast(OSData, provider->getProperty("keymap"));
	if (map != NULL) {
		const uint32_t *cells = (const uint32_t *)map->getBytesNoCopy();
		for (unsigned int i = 0; i + 1 < map->getLength() / sizeof(uint32_t); i += 2) {
			if (cells[i] < PMIC_KEYS) {
				fUsage[cells[i]] = cells[i + 1];
			}
		}
	}
	uint32_t top = 0;
	fPMIC->read(MT6358_PMIC_TOPSTATUS, &top);
	for (unsigned int k = 0; k < PMIC_KEYS; k++) {
		fDown[k] = (top & kKeyDebBit[k]) == 0;
	}
	IOReturn ret = fPMIC->registerGroupInterrupt(kMapplePMICGroupPSC, 0,
	    MT6358_PMIC_PSC_PWRKEY | MT6358_PMIC_PSC_HOMEKEY | MT6358_PMIC_PSC_PWRKEY_R | MT6358_PMIC_PSC_HOMEKEY_R,
	    this, &MT6358Keys::pmicInterrupt);
	IOLog("PMIC KEYS: power -> 0x%x, home -> 0x%x, topstatus 0x%04x (%d)\n",
	    (unsigned)fUsage[0], (unsigned)fUsage[1], top, ret);
	if (ret != kIOReturnSuccess) {
		return false;
	}
	registerService();
	return true;
}

void
MT6358Keys::pmicInterrupt(OSObject *target, uint32_t group __unused, uint32_t index __unused, uint32_t status)
{
	MT6358Keys *self = OSDynamicCast(MT6358Keys, target);
	if (self != NULL) {
		self->update(status);
	}
}

void
MT6358Keys::send(unsigned int k, bool down)
{
	fDown[k] = down;
	fEvents++;
	IOLog("PMIC KEYS: key 0x%x %s\n", (unsigned)fUsage[k], down ? "down" : "up");
	IOHIDEvent *event = IOHIDEvent::keyboardEvent(mach_absolute_time(), kHIDPage_Consumer, fUsage[k], down);
	if (event != NULL) {
		dispatchEvent(event);
		event->release();
	}
}

/* Work loop. State from TOPSTATUS; a press and release that both happened before we looked is a tap. */
void
MT6358Keys::update(uint32_t status)
{
	uint32_t top = 0;
	if (fPMIC->read(MT6358_PMIC_TOPSTATUS, &top) != kIOReturnSuccess) {
		return;
	}
	for (unsigned int k = 0; k < PMIC_KEYS; k++) {
		bool down = (top & kKeyDebBit[k]) == 0;
		if (down != fDown[k]) {
			send(k, down);
		} else if (!down && (status & kKeyPress[k]) && (status & kKeyRelease[k])) {
			send(k, true);
			send(k, false);
		}
	}
}
