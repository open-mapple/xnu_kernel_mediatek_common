/*
 * Minimal device-tree platform expert for Redmi Note 10 Pro (sweet).
 * Uses XNU's normal platform, power-domain and nub initialization. Hardware
 * interrupt and power-management services are supplied by separate drivers.
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>

class SweetPlatformExpert : public IODTPlatformExpert {
	OSDeclareDefaultStructors(SweetPlatformExpert);

public:
	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	void processTopLevel(IORegistryEntry *root) APPLE_KEXT_OVERRIDE;
	const char *deleteList(void) APPLE_KEXT_OVERRIDE;
	const char *excludeList(void) APPLE_KEXT_OVERRIDE;
	void getUTCTimeOfDay(clock_sec_t *secs, clock_nsec_t *nsecs) APPLE_KEXT_OVERRIDE;
	void setUTCTimeOfDay(clock_sec_t secs, clock_nsec_t nsecs) APPLE_KEXT_OVERRIDE;
};

extern "C" int viva_rtc_read(uint64_t *secs);   /* VivaPwrap.cpp: the PMIC's RTC (viva only) */

/* The calendar at boot from the PMIC's RTC (otherwise 1970: no TLS, every site refuses the clock). */
void
SweetPlatformExpert::getUTCTimeOfDay(clock_sec_t *secs, clock_nsec_t *nsecs)
{
	uint64_t s = 0;
	*secs = viva_rtc_read(&s) == 0 ? (clock_sec_t)s : 0;
	*nsecs = 0;
}

/* settimeofday / timed: the kernel calendar only (RTC writes need a PMIC write path; the RTC is already UTC). */
void
SweetPlatformExpert::setUTCTimeOfDay(__unused clock_sec_t secs, __unused clock_nsec_t nsecs)
{
}

OSDefineMetaClassAndStructors(SweetPlatformExpert, IODTPlatformExpert);

bool
SweetPlatformExpert::start(IOService *provider)
{
	IOLog("SWEET: starting device-tree platform expert\n");
	if (!IODTPlatformExpert::start(provider)) {
		return false;
	}
	registerService();
	IOLog("SWEET: platform expert registered\n");
	return true;
}

const char *
SweetPlatformExpert::deleteList(void)
{
	return "()";
}

const char *
SweetPlatformExpert::excludeList(void)
{
	return "('chosen', 'defaults', 'cpus', 'options')";
}

void
SweetPlatformExpert::processTopLevel(IORegistryEntry *root)
{
	/*
	 * The stock implementation requires an Apple NVRAM /options node.
	 * SWEET has no NVRAM controller; publish real CPU/device nodes without
	 * inventing one. createNubs adds the topology's logical CPU/cluster IDs.
	 */
	IORegistryEntry *cpus = root->childFromPath("cpus", gIODTPlane);
	if (cpus != NULL) {
		createNubs(this, IODTFindMatchingEntries(cpus, kIODTExclusive, NULL));
		cpus->release();
	}
	createNubs(this, IODTFindMatchingEntries(root, kIODTExclusive, excludeList()));

	/*
	 * No AppleARMIO driver publishes the /arm-io children: do it here so
	 * built-in drivers (the display, AppleCLCD) can match them. The GIC
	 * nub is SweetGIC's, the SPI interrupt controller of the other nubs
	 * (the platform code still owns timer and IPIs). The nubs hang off this
	 * platform expert: IOPlatformDevice::compareName asks its provider.
	 */
	IORegistryEntry *armio = root->childFromPath("arm-io", gIODTPlane);
	if (armio != NULL) {
		createNubs(this, IODTFindMatchingEntries(armio, kIODTExclusive, NULL));
		armio->release();
	}
}

/*
 * hw.target / hw.product come from IODTPlatformExpert: the first root
 * "compatible" string and "model" (D22AP / iPhone10,3 in the ADT), which
 * is what the iPhone userland (MobileGestalt) expects.
 */
#endif /* SWEET */
