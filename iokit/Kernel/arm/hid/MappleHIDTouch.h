/*
 * the built-in touch screen HID service (MappleHID.cpp). Board
 * drivers that own a touch controller subclass it and call touch() with
 * panel pixel coordinates (viva: hid/GT9896.cpp).
 */
#ifndef _IOKIT_ARM_HID_MAPPLEHIDTOUCH_H
#define _IOKIT_ARM_HID_MAPPLEHIDTOUCH_H

#include "IOHIDEventService.h"
#include "IOHIDUsageTables.h"
#include "../MappleLetterbox.h"

/*
 * Touch screen: digitizer collection events with finger children,
 * coordinates normalized to 0..1 like IOHIDEventDriver produces for
 * absolute digitizers.
 */
#define MAPPLE_TOUCH_FINGERS 10
/*
 * Contacts that start this close to a panel edge (pixels, ~3 mm) carry
 * kIOHIDDigitizerEventFromEdgeTip for their whole life, as Apple's touch
 * firmware reports them. UIKit derives UITouch's edge type from that bit
 * (_UIEventHIDTouchEdgeTypeForHIDEvent); without it there is no home or
 * app-switcher gesture.
 */
#define MAPPLE_TOUCH_EDGE_PX 48
class MappleHIDTouch : public IOHIDEventService {
	OSDeclareDefaultStructors(MappleHIDTouch);
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
	struct mapple_lb fLb;		/* mp_fb_letterbox: the picture on the panel, w 0 = off */
	bool fDown[MAPPLE_TOUCH_FINGERS];
	bool fEdge[MAPPLE_TOUCH_FINGERS];
	IOFixed fX[MAPPLE_TOUCH_FINGERS];
	IOFixed fY[MAPPLE_TOUCH_FINGERS];
};

#endif /* _IOKIT_ARM_HID_MAPPLEHIDTOUCH_H */
