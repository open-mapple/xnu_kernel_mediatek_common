/*
 * MT6781: what iOS 16's configd, wifid and IO80211.framework look for in the registry (the station and en0
 * are bsd/dev/mt6631_wifi.c and bsd/dev/mt6631_wlan.c).
 *
 * No IONetworkingFamily / IO80211Family here, so (as hakios-xnu-kernel and ios-sapphire do, IOS16-WIFI-HAKIOS.md)
 * classes with Apple's names that carry properties only:
 *   IONetworkController <- IOEthernetController <- IO80211Controller <- MT6631WLAN, matched on the ADT /arm-io/wlan
 *     node (IOMACAddress, IOBuiltin; local-mac-address on the node: MobileGestalt waits 5 s per lookup without
 *     it or an IONetworkController under the node);
 *   IONetworkInterface <- MT6631WLANInterface, its child: IOInterfaceName / BSD Name "en0", IOInterfaceType 6,
 *     IOInterfaceUnit 0, IOBuiltin, IOPrimaryInterface, IO80211InterfaceRole "Infrastructure" (wifid matches that
 *     property; configd sees en0 as Wi-Fi because the parent conforms to IO80211Controller and makes its own
 *     "Wi-Fi" service, the one whose DHCP gives Settings its tick). newUserClient fails: IO80211.framework then
 *     uses the SIOCGA80211/SIOCSA80211 ioctls on en0.
 *   IONetworkStack on IOResources with an openable user client that refuses the naming command: en0 is born
 *     named, but configd's InterfaceNamer only reports quiet after opening it.
 * Names would clash with a real IONetworkingFamily port (none planned).
 */
#include <pexpert/arm64/board_config.h>

#if defined(MT6781)
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOLib.h>
#include <pexpert/pexpert.h>
#include <libkern/c++/OSData.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSString.h>

extern "C" void mt6631_wifi_mac(uint8_t mac[6]);          /* bsd/dev/mt6631_wifi.c */
extern "C" void mt6631_wlan_attach(void);                 /* bsd/dev/mt6631_wlan.c */

class IONetworkController : public IOService {
	OSDeclareDefaultStructors(IONetworkController);
};
OSDefineMetaClassAndStructors(IONetworkController, IOService);

class IOEthernetController : public IONetworkController {
	OSDeclareDefaultStructors(IOEthernetController);
};
OSDefineMetaClassAndStructors(IOEthernetController, IONetworkController);

class IO80211Controller : public IOEthernetController {
	OSDeclareDefaultStructors(IO80211Controller);
};
OSDefineMetaClassAndStructors(IO80211Controller, IOEthernetController);

class IONetworkInterface : public IOService {
	OSDeclareDefaultStructors(IONetworkInterface);
};
OSDefineMetaClassAndStructors(IONetworkInterface, IOService);

class MT6631WLANInterface : public IONetworkInterface {
	OSDeclareDefaultStructors(MT6631WLANInterface);
public:
	virtual IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *properties,
	    IOUserClient **handler) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(MT6631WLANInterface, IONetworkInterface);

IOReturn
MT6631WLANInterface::newUserClient(task_t, void *, UInt32 type, OSDictionary *, IOUserClient **)
{
	static bool logged;
	if (!logged) {
		logged = true;
		IOLog("WLAN: user client type %u refused (IO80211.framework falls back to the ioctls)\n", (unsigned)type);
	}
	return kIOReturnUnsupported;
}

class MT6631WLAN : public IO80211Controller {
	OSDeclareDefaultStructors(MT6631WLAN);
public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(MT6631WLAN, IO80211Controller);

static MT6631WLANInterface *gMappleWLANInterface;

static void
setNumber(IOService *s, const char *key, uint32_t v)
{
	OSNumber *n = OSNumber::withNumber(v, 32);
	if (n != NULL) {
		s->setProperty(key, n);
		n->release();
	}
}

bool
MT6631WLAN::start(IOService *provider)
{
	uint32_t enable = 1;
	PE_parse_boot_argn_alias("mp_wlan", "vivawlan", &enable, sizeof(enable));
	if (enable == 0) {
		IOLog("WLAN: off (mp_wlan=0)\n");
		return false;
	}
	/*
	 * 0148: ifnet_allocate needs DLIL (dlil_init in bsd_init), which runs concurrently with IOKit matching: on
	 * 0147 this start ran first and panicked in zalloc_flags(dlif_zone == NULL). IOBSD is published by
	 * bsd_autoconf, after dlil_init and the network domains.
	 */
	OSDictionary *bsd = IOService::resourceMatching("IOBSD");
	IOService *ready = bsd != NULL ? IOService::waitForMatchingService(bsd, 120000000000ull) : NULL;
	OSSafeReleaseNULL(bsd);
	if (ready == NULL) {
		IOLog("WLAN: BSD networking not up after 120 s, no en0\n");
		return false;
	}
	ready->release();
	if (!IO80211Controller::start(provider)) {
		return false;
	}
	uint8_t mac[6];
	mt6631_wifi_mac(mac);
	OSData *m = OSData::withBytes(mac, sizeof(mac));
	if (m != NULL) {
		setProperty("IOMACAddress", m);
		provider->setProperty("local-mac-address", m);
		m->release();
	}
	setProperty("IOBuiltin", kOSBooleanTrue);

	mt6631_wlan_attach();                     /* en0 first: the registry entry names it */

	MT6631WLANInterface *ifc = OSTypeAlloc(MT6631WLANInterface);
	if (ifc == NULL || !ifc->init()) {
		OSSafeReleaseNULL(ifc);
		return false;
	}
	ifc->setName("en0");
	ifc->setProperty("IOInterfaceName", "en0");
	ifc->setProperty("BSD Name", "en0");
	ifc->setProperty("IOInterfaceNamePrefix", "en");
	setNumber(ifc, "IOInterfaceType", 6);   /* IFT_ETHER */
	setNumber(ifc, "IOInterfaceUnit", 0);
	ifc->setProperty("IOBuiltin", kOSBooleanTrue);
	ifc->setProperty("IOPrimaryInterface", kOSBooleanTrue);
	ifc->setProperty("IO80211InterfaceRole", "Infrastructure");
	if (!ifc->attach(this) || !ifc->start(this)) {
		ifc->release();
		return false;
	}
	registerService();
	ifc->registerService();
	gMappleWLANInterface = ifc;
	IOLog("WLAN: IO80211Controller + en0 interface registered\n");
	return true;
}

/* Link state on the interface entry (informational: configd reads the link from the ifnet). */
extern "C" void
mt6631_wlan_registry_link(bool up)
{
	if (gMappleWLANInterface != NULL) {
		setNumber(gMappleWLANInterface, "IOLinkStatus", up ? 3 : 1);     /* kIONetworkLinkValid | Active */
	}
}

/* ---- IONetworkStack stand-in ---- */

class IONetworkStack : public IOService {
	OSDeclareDefaultStructors(IONetworkStack);
public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(IONetworkStack, IOService);

bool
IONetworkStack::start(IOService *provider)
{
	if (!IOService::start(provider)) {
		return false;
	}
	setProperty("IOUserClientClass", "IONetworkStackUserClient");
	setName("IONetworkStack");
	registerService();
	return true;
}

class IONetworkStackUserClient : public IOUserClient {
	OSDeclareDefaultStructors(IONetworkStackUserClient);
public:
	virtual IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setProperties(OSObject *properties) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(IONetworkStackUserClient, IOUserClient);

IOReturn
IONetworkStackUserClient::clientClose(void)
{
	terminate();
	return kIOReturnSuccess;
}

/* InterfaceNamer's "name this interface" ({IONetworkStackUserCommand, IOPathMatch, IOInterfaceUnit}): en0 is
 * already named by the ifnet KPI. */
IOReturn
IONetworkStackUserClient::setProperties(OSObject *)
{
	return kIOReturnUnsupported;
}
#endif /* MT6781 */
