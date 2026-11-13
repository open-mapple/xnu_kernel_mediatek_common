/*
 * Keyboards behind an IOHIDInterface (a Bluetooth keyboard that bluetoothd
 * creates through IOHIDResource) as an HID event service for backboardd.
 *
 * Elements and events follow IOHIDEventDriver's keyboard path
 * (parseKeyboardElement, handleKeboardReport): keyboard page usages A to
 * Right GUI and consumer page usages, one keyboard event per element whose
 * value changed in the report, on our IOHIDEventService instead of Apple's.
 */
#include <IOKit/IOLib.h>
#include <libkern/c++/OSArray.h>

#include "IOHIDEventService.h"
#include "IOHIDInterface.h"
#include "IOHIDElement.h"
#include "IOHIDEvent.h"
#include "IOHIDKeys.h"
#include "IOHIDUsageTables.h"

class MappleHIDKeyboard : public IOHIDEventService {
	OSDeclareDefaultStructors(MappleHIDKeyboard);
public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual bool didTerminate(IOService *provider, IOOptionBits options, bool *defer) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
protected:
	virtual UInt32 getPrimaryUsagePage() APPLE_KEXT_OVERRIDE { return kHIDPage_GenericDesktop; }
	virtual UInt32 getPrimaryUsage() APPLE_KEXT_OVERRIDE { return kHIDUsage_GD_Keyboard; }
	virtual OSString *getTransport() APPLE_KEXT_OVERRIDE;
	virtual OSString *getProduct() APPLE_KEXT_OVERRIDE;
	virtual UInt32 getVendorID() APPLE_KEXT_OVERRIDE;
	virtual UInt32 getProductID() APPLE_KEXT_OVERRIDE;
	virtual UInt32 getReportInterval() APPLE_KEXT_OVERRIDE;
private:
	void handleReport(AbsoluteTime timestamp, IOMemoryDescriptor *report, IOHIDReportType type,
	    UInt32 reportID, void *refcon);
	IOHIDInterface *fInterface;
	OSArray *fKeys;
	UInt32 fEvents;
};
OSDefineMetaClassAndStructors(MappleHIDKeyboard, IOHIDEventService);

static bool
isKey(IOHIDElement *element)
{
	UInt32 page = element->getUsagePage();
	UInt32 usage = element->getUsage();

	if (element->getType() == kIOHIDElementTypeCollection || usage == 0) {
		return false;
	}
	switch (page) {
	case kHIDPage_KeyboardOrKeypad:
		return usage >= kHIDUsage_KeyboardA && usage <= kHIDUsage_KeyboardRightGUI;
	case kHIDPage_Consumer:
		return true;
	default:
		return false;
	}
}

bool
MappleHIDKeyboard::start(IOService *provider)
{
	OSArray *elements;
	OSString *product;

	fInterface = OSDynamicCast(IOHIDInterface, provider);
	if (!fInterface) {
		return false;
	}
	elements = fInterface->createMatchingElements();
	fKeys = OSArray::withCapacity(8);
	if (!elements || !fKeys) {
		OSSafeReleaseNULL(elements);
		return false;
	}
	for (unsigned int i = 0; i < elements->getCount(); i++) {
		IOHIDElement *element = OSDynamicCast(IOHIDElement, elements->getObject(i));
		if (element && isKey(element)) {
			fKeys->setObject(element);
		}
	}
	elements->release();
	if (fKeys->getCount() == 0) {
		return false;
	}

	if (!IOHIDEventService::start(provider)) {
		return false;
	}
	setProperty(kIOHIDBuiltInKey, kOSBooleanFalse);
	setProperty(kIOHIDLocationIDKey, fInterface->getLocationID(), 32);
	setProperty(kIOHIDCountryCodeKey, fInterface->getCountryCode(), 32);
	if (fInterface->getManufacturer()) {
		setProperty(kIOHIDManufacturerKey, fInterface->getManufacturer());
	}

	if (!fInterface->open(this, 0, OSMemberFunctionCast(IOHIDInterface::InterruptReportAction, this,
	    &MappleHIDKeyboard::handleReport), NULL)) {
		IOLog("HID: keyboard: interface open refused\n");
		return false;
	}
	registerService();
	product = fInterface->getProduct();
	IOLog("HID: keyboard '%s' (%s, vendor 0x%x product 0x%x): %u keys\n",
	    product ? product->getCStringNoCopy() : "?",
	    fInterface->getTransport() ? fInterface->getTransport()->getCStringNoCopy() : "?",
	    (unsigned)fInterface->getVendorID(), (unsigned)fInterface->getProductID(), fKeys->getCount());
	return true;
}

bool
MappleHIDKeyboard::didTerminate(IOService *provider, IOOptionBits options, bool *defer)
{
	if (fInterface) {
		fInterface->close(this);
	}
	IOLog("HID: keyboard gone after %u key events\n", (unsigned)fEvents);
	fInterface = NULL;
	return IOHIDEventService::didTerminate(provider, options, defer);
}

void
MappleHIDKeyboard::free(void)
{
	OSSafeReleaseNULL(fKeys);
	IOHIDEventService::free();
}

void
MappleHIDKeyboard::handleReport(AbsoluteTime timestamp, IOMemoryDescriptor *report __unused,
    IOHIDReportType type, UInt32 reportID, void *refcon __unused)
{
	if (type != kIOHIDReportTypeInput) {
		return;
	}
	for (unsigned int i = 0; i < fKeys->getCount(); i++) {
		IOHIDElement *element = (IOHIDElement *)fKeys->getObject(i);

		if (element->getReportID() != reportID || element->getTimeStamp() != timestamp) {
			continue;
		}
		bool down = element->getValue() != 0;
		if (down == (element->getValue(kIOHIDValueOptionsFlagPrevious) != 0)) {
			continue;
		}
		IOHIDEvent *event = IOHIDEvent::keyboardEvent(timestamp, element->getUsagePage(),
		    element->getUsage(), down, 1, 0, 0);
		if (event) {
			/* Typed text stays out of the log: only the first event's usage. */
			if (fEvents++ == 0) {
				IOLog("HID: keyboard first event 0x%x:0x%x %s, %u clients\n",
				    (unsigned)element->getUsagePage(), (unsigned)element->getUsage(),
				    down ? "down" : "up", _clients ? _clients->getCount() : 0);
			}
			dispatchEvent(event);
			event->release();
		}
	}
}

OSString *
MappleHIDKeyboard::getTransport()
{
	OSString *s = fInterface->getTransport();

	if (s) {
		s->retain();
		return s;
	}
	return IOHIDEventService::getTransport();
}

OSString *
MappleHIDKeyboard::getProduct()
{
	OSString *s = fInterface->getProduct();

	if (s) {
		s->retain();
		return s;
	}
	return IOHIDEventService::getProduct();
}

UInt32
MappleHIDKeyboard::getVendorID()
{
	return fInterface->getVendorID();
}

UInt32
MappleHIDKeyboard::getProductID()
{
	return fInterface->getProductID();
}

UInt32
MappleHIDKeyboard::getReportInterval()
{
	UInt32 interval = fInterface->getReportInterval();

	return interval ? interval : IOHIDEventService::getReportInterval();
}
