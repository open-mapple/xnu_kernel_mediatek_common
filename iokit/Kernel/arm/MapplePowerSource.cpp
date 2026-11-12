/*
 * Built-in iOS battery service: the MT6358 fuel gauge (mp_gauge=1) polled every 30 seconds, or a static 100%, AC,
 * not charging.
 */
#if defined(ARM64_BOARD_CONFIG_MT6781)
#include <device/device_types.h> /* io_buf_ptr_t for private OSKext headers */
#include <IOKit/IOLib.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/pwr_mgt/IOPMPowerSource.h>
#include <libkern/c++/OSCollectionIterator.h>
#include <pexpert/pexpert.h>
/* iokit/Kernel/arm/BQ27Z561Battery.cpp: viva's real battery driver will run */
extern "C" bool bq27z561_present(void);
/* bsd/dev/mt6358_gauge.c: the MT6358 fuel gauge (mp_gauge=) */
extern "C" bool mt6358_gauge_init(void);
extern "C" bool mt6358_gauge_read(int32_t *percent_hundredths, int32_t *mv, int32_t *ma, bool *external,
    bool *current, int *status);

class MapplePowerSource : public IOPMPowerSource {
	OSDeclareDefaultStructors(MapplePowerSource);
public:
	bool init(OSDictionary *dictionary = NULL) APPLE_KEXT_OVERRIDE;
	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	void free(void) APPLE_KEXT_OVERRIDE;
private:
	IOWorkLoop *fLoop;
	IOTimerEventSource *fTimer;
	bool fTimerAdded, fRunning, fGauge;
	int fLastMode;
	void putNumber(const char *key, UInt32 value);
	void putBool(const char *key, bool value);
	void publish(unsigned percent, unsigned mv, int ma, bool external,
	    bool capable, bool charging, bool full, bool measured, bool currentValid);
	void poll(void);
	static void timerFired(OSObject *owner, IOTimerEventSource *sender);
	static IOReturn stopPolling(OSObject *owner, void *, void *, void *, void *);
};
OSDefineMetaClassAndStructors(MapplePowerSource, IOPMPowerSource);

bool
MapplePowerSource::init(OSDictionary *dictionary)
{
	/* IOCatalogue calls init(dictionary). This XNU's IOPMPowerSource only
	 * overrides init(void): explicitly run it to allocate its PS dictionary
	 * and symbols, then copy the matching personality into the registry. */
	if (!IOPMPowerSource::init()) {
		return false;
	}
	fLoop = NULL;
	fTimer = NULL;
	fTimerAdded = fRunning = false;
	if (dictionary) {
		OSCollectionIterator *it = OSCollectionIterator::withCollection(dictionary);
		if (!it) {
			return false;
		}
		OSObject *object;
		bool ok = true;
		while ((object = it->getNextObject())) {
			const OSSymbol *key = OSDynamicCast(OSSymbol, object);
			if (!key || !setProperty(key, dictionary->getObject(key))) {
				ok = false;
				break;
			}
		}
		it->release();
		if (!ok) {
			return false;
		}
	}
	fLastMode = -1;
	return true;
}

void
MapplePowerSource::putNumber(const char *key, UInt32 value)
{
	const OSSymbol *symbol = OSSymbol::withCString(key);
	OSNumber *number = OSNumber::withNumber(value, 32);
	if (symbol && number) {
		setPSProperty(symbol, number);
	}
	OSSafeReleaseNULL(number);
	OSSafeReleaseNULL(symbol);
}

void
MapplePowerSource::putBool(const char *key, bool value)
{
	const OSSymbol *symbol = OSSymbol::withCString(key);
	if (symbol) {
		setPSProperty(symbol, value ? kOSBooleanTrue : kOSBooleanFalse);
		symbol->release();
	}
}

void
MapplePowerSource::publish(unsigned percent, unsigned mv, int ma, bool external,
    bool capable, bool charging, bool full, bool measured, bool currentValid)
{
	/* Use consistent mAh units (DT nominal=5020). powerd computes C*100/M. */
	const unsigned capacity = (5020u * percent + 50u) / 100u;
	setBatteryInstalled(true);
	setCurrentCapacity(capacity);
	setMaxCapacity(5020);
	setExternalConnected(external);
	setExternalChargeCapable(capable);
	setIsCharging(charging);
	setAtWarnLevel(false);
	/* An uncalibrated voltage curve must not assert a critical shutdown. */
	setAtCriticalLevel(false);
	setVoltage(mv);
	setAmperage(ma);
	setTimeRemaining(-1); /* Unknown; never publish zero minutes on error. */
	setCycleCount(0);     /* Placeholder: cycle learning is not implemented. */
	putBool("FullyCharged", full);
	putBool("AppleRawExternalConnected", external);
	putNumber("AppleRawCurrentCapacity", capacity);
	putNumber("AppleRawMaxCapacity", 5020);
	putNumber("DesignCapacity", 5020);
	putNumber("InstantAmperage", (UInt32)ma);
	/* iOS battery convention: centidegrees C; fixed 25 C, NOT measured. */
	putNumber("Temperature", 2500);
	putBool("MapplePowerSourceHardwareSampleValid", measured);
	putBool("MapplePowerSourceCurrentValid", currentValid);
	putBool("MapplePowerSourceSOCEstimated", measured);
	putBool("MapplePowerSourceTemperatureEstimated", true);
	putBool("MapplePowerSourceFallback", !measured);
	putNumber("MapplePowerSourcePercent", percent);

	/* IOPMCopyBatteryInfo's per-source compatibility dictionary. */
	OSDictionary *legacy = OSDictionary::withCapacity(7);
	if (legacy) {
		const char *keys[] = {kIOBatteryCurrentChargeKey, kIOBatteryCapacityKey,
		    kIOBatteryVoltageKey, kIOBatteryAmperageKey, kIOBatteryCycleCountKey,
		    kIOBatteryFlagsKey};
		UInt32 values[] = {capacity, 5020, mv, (UInt32)ma, 0,
		    (UInt32)(kIOBatteryInstalled | (external ? kIOBatteryChargerConnect : 0) |
		    (charging ? kIOBatteryCharge : 0))};
		for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
			OSNumber *number = OSNumber::withNumber(values[i], 32);
			if (number) {
				legacy->setObject(keys[i], number);
				number->release();
			}
		}
		setLegacyIOBatteryInfo(legacy);
		legacy->release();
	}
	updateStatus(); /* Flush PS properties and send 0xe0024100 on change. */
	int mode = measured ? 1 : 0;
	if (mode != fLastMode) {
		IOLog("BATTERY: %s, %u%% %umV external=%d charging=%d\n",
		    !measured ? "static fallback" : "MT6358 fuel gauge",
		    percent, mv, external, charging);
		fLastMode = mode;
	}
}

void
MapplePowerSource::poll(void)
{
	int32_t hundredths, gmv, gma;
	bool gexternal, gcurrent;
	int status;
	if (fGauge && mt6358_gauge_read(&hundredths, &gmv, &gma, &gexternal, &gcurrent, &status)) {
		/* status as Linux numbers it: 1 charging, 4 full */
		publish((unsigned)(hundredths + 50) / 100, (unsigned)gmv, gma, gexternal, gexternal, status == 1, status == 4,
		    true, gcurrent);
		return;
	}
	publish(100, 4200, 0, true, true, false, true, false, false);
}

void
MapplePowerSource::timerFired(OSObject *owner, IOTimerEventSource *sender)
{
	MapplePowerSource *self = OSDynamicCast(MapplePowerSource, owner);
	if (self && self->fRunning) {
		self->poll();
		sender->setTimeoutMS(30000);
	}
}

bool
MapplePowerSource::start(IOService *provider)
{
	if (bq27z561_present()) {
		IOLog("BATTERY: BQ27Z561Battery present, static source not published\n");
		return false;
	}
	if (!IOPMPowerSource::start(provider)) {
		return false;
	}
	fGauge = mt6358_gauge_init();
	/* Publish a complete source BEFORE registerService wakes powerd. */
	poll();
	if (fGauge) {
		fLoop = IOWorkLoop::workLoop();
		fTimer = IOTimerEventSource::timerEventSource(this, &MapplePowerSource::timerFired);
		if (fLoop && fTimer && fLoop->addEventSource(fTimer) == kIOReturnSuccess) {
			fTimerAdded = true;
			fRunning = true;
			fTimer->setTimeoutMS(30000);
		} else {
			fGauge = false;
			poll();
			IOLog("BATTERY: timer unavailable; static fallback\n");
		}
	}
	registerService();
	return true;
}

void
MapplePowerSource::stop(IOService *provider)
{
	if (fLoop && fTimerAdded) {
		/* Serialize against a running timer callback, including rearming. */
		(void)fLoop->runAction(&MapplePowerSource::stopPolling, this);
		fLoop->removeEventSource(fTimer);
		fTimerAdded = false;
	}
	IOPMPowerSource::stop(provider);
}

IOReturn
MapplePowerSource::stopPolling(OSObject *owner, void *, void *, void *, void *)
{
	MapplePowerSource *self = OSDynamicCast(MapplePowerSource, owner);
	if (!self) {
		return kIOReturnBadArgument;
	}
	self->fRunning = false;
	self->fTimer->cancelTimeout();
	self->fTimer->disable();
	return kIOReturnSuccess;
}

void
MapplePowerSource::free(void)
{
	OSSafeReleaseNULL(fTimer);
	OSSafeReleaseNULL(fLoop);
	IOPMPowerSource::free();
}
#endif /* ARM64_BOARD_CONFIG_MT6781 */
