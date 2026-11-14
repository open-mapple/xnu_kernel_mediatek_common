/*
 * Minimal device-tree platform expert for Redmi Note 10 Pro (sweet).
 * Uses XNU's normal platform, power-domain and nub initialization. Hardware
 * interrupt and power-management services are supplied by separate drivers.
 */
#include <pexpert/arm64/board_config.h>

#if defined(MT6781)
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>

class MapplePlatformExpert : public IODTPlatformExpert {
	OSDeclareDefaultStructors(MapplePlatformExpert);

public:
	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	void processTopLevel(IORegistryEntry *root) APPLE_KEXT_OVERRIDE;
	const char *deleteList(void) APPLE_KEXT_OVERRIDE;
	const char *excludeList(void) APPLE_KEXT_OVERRIDE;
	void getUTCTimeOfDay(clock_sec_t *secs, clock_nsec_t *nsecs) APPLE_KEXT_OVERRIDE;
	void setUTCTimeOfDay(clock_sec_t secs, clock_nsec_t nsecs) APPLE_KEXT_OVERRIDE;
	int haltRestart(unsigned int type) APPLE_KEXT_OVERRIDE;
};

extern "C" int mapple_rtc_read(uint64_t *secs);   /* MTKPwrap.cpp: the PMIC's RTC (MT6781 only) */
extern "C" void mtk_pmic_poweroff(void);      /* MTKPwrap.cpp: returns only when the phone is still on */
extern "C" void mapple_mem4g_report(void);        /* arm_vm_init.c: mp_mem4g's holes (MT6781 only) */

/* The calendar at boot from the PMIC's RTC (otherwise 1970: no TLS, every site refuses the clock). */
void
MapplePlatformExpert::getUTCTimeOfDay(clock_sec_t *secs, clock_nsec_t *nsecs)
{
	uint64_t s = 0;
	*secs = mapple_rtc_read(&s) == 0 ? (clock_sec_t)s : 0;
	*nsecs = 0;
}

/* settimeofday / timed: the kernel calendar only (RTC writes need a PMIC write path; the RTC is already UTC). */
void
MapplePlatformExpert::setUTCTimeOfDay(__unused clock_sec_t secs, __unused clock_nsec_t nsecs)
{
}

OSDefineMetaClassAndStructors(MapplePlatformExpert, IODTPlatformExpert);

#define TOPRGU_PA       0x10007000u
#define WDT_SWRST       0x14
#define WDT_SWRST_KEY   0x1209

extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

/* Boot-arg mp_poweroff: 0 power off restarts, 1 Android's power off; 0 on a safe boot. */
static uint32_t gMapplePowerOff;

/*
 * iOS's restart (reboot(2): Settings, apps) reaches here after the shutdown notifications: the watchdog's software
 * reset, as the USB link's "reboot" does. Power off restarts too unless mp_poweroff has the PMIC cut the power, and
 * when that fails. Panics are left as they are (wdt=-1 keeps the panic on screen).
 */
int
MapplePlatformExpert::haltRestart(unsigned int type)
{
	static vm_offset_t toprgu;

	if (type == kPERestartCPU || type == kPEHaltCPU) {
		if (toprgu == 0) {
			toprgu = ml_io_map(TOPRGU_PA, 0x1000);
		}
		if (type == kPEHaltCPU && gMapplePowerOff == 1) {
			mtk_pmic_poweroff();            /* interrupts off from here */
		}
		printf("platform: %s: the watchdog's software reset\n", type == kPEHaltCPU ? "power off" : "restart");
		for (;;) {
			*(volatile uint32_t *)(toprgu + WDT_SWRST) = WDT_SWRST_KEY;
			IODelay(5000);
		}
	}
	return IODTPlatformExpert::haltRestart(type);
}

bool
MapplePlatformExpert::start(IOService *provider)
{
	IOLog("starting device-tree platform expert\n");
	if (!IODTPlatformExpert::start(provider)) {
		return false;
	}
	registerService();
	IOLog("platform expert registered\n");
	uint32_t record = 1;
	(void)PE_parse_boot_argn("mp_bootrec", &record, sizeof(record));
	(void)PE_parse_boot_argn("mp_poweroff", &gMapplePowerOff, sizeof(gMapplePowerOff));
	if (record == 0) {
		gMapplePowerOff = 0;
	}
	IOLog("platform: mp_poweroff=%u%s\n", gMapplePowerOff, record == 0 ? " (safe boot)" : "");
#if defined(MT6781)
	mapple_mem4g_report();
#endif
	return true;
}

const char *
MapplePlatformExpert::deleteList(void)
{
	return "()";
}

const char *
MapplePlatformExpert::excludeList(void)
{
	return "('chosen', 'defaults', 'cpus', 'options')";
}

void
MapplePlatformExpert::processTopLevel(IORegistryEntry *root)
{
	/*
	 * The stock implementation requires an Apple NVRAM /options node.
	 * MT6781 has no NVRAM controller; publish real CPU/device nodes without
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
	 * nub is MappleGIC's, the SPI interrupt controller of the other nubs
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
#endif /* MT6781 */
