/*
 * SWEET: the built-in touch screen HID service (SweetHID.cpp). Board
 * drivers that own a touch controller subclass it and call touch() with
 * panel pixel coordinates (viva: hid/VivaGT9896.cpp).
 */
#ifndef _IOKIT_ARM_HID_SWEETHIDTOUCH_H
#define _IOKIT_ARM_HID_SWEETHIDTOUCH_H

#include "IOHIDEventService.h"
#include "IOHIDUsageTables.h"

/*
 * Touch screen: digitizer collection events with finger children,
 * coordinates normalized to 0..1 like IOHIDEventDriver produces for
 * absolute digitizers.
 */
#define SWEET_TOUCH_FINGERS 10
/*
 * Contacts that start this close to a panel edge (pixels, ~3 mm) carry
 * kIOHIDDigitizerEventFromEdgeTip for their whole life, as Apple's touch
 * firmware reports them. UIKit derives UITouch's edge type from that bit
 * (_UIEventHIDTouchEdgeTypeForHIDEvent); without it there is no home or
 * app-switcher gesture.
 */
#define SWEET_TOUCH_EDGE_PX 48
class SweetHIDTouch : public IOHIDEventService {
	OSDeclareDefaultStructors(SweetHIDTouch);
public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	void touch(int finger, bool down, int x, int y);
protected:
	virtual UInt32 getPrimaryUsagePage() APPLE_KEXT_OVERRIDE { return kHIDPage_Digitizer; }
	virtual UInt32 getPrimaryUsage() APPLE_KEXT_OVERRIDE { return kHIDUsage_Dig_TouchScreen; }
	virtual UInt32 getProductID() APPLE_KEXT_OVERRIDE { return 2; }
protected:
	IOLock *fLock;
	int fLogicalW, fLogicalH, fOffX, fOffY;
	bool fDown[SWEET_TOUCH_FINGERS];
	bool fEdge[SWEET_TOUCH_FINGERS];
	IOFixed fX[SWEET_TOUCH_FINGERS];
	IOFixed fY[SWEET_TOUCH_FINGERS];
};

#endif /* _IOKIT_ARM_HID_SWEETHIDTOUCH_H */
