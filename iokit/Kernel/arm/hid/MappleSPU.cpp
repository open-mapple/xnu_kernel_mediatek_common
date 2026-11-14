/*
 * The sensors as the iPhone X's always-on processor (SPU) presents them to iOS: one HID event service per sensor,
 * which locationd, CoreMotion, backboardd and CoreBrightness find by its usage pair (DeviceUsagePairs), fed by the
 * SCP's sensor hub (bsd/dev/mtk_scp.c). Our own class name keeps Apple's ALS and compass plug-ins off: they attach
 * to their own drivers' names only.
 *
 *   accelerometer  0xFF00/0x03  g, iOS's sign (gravity itself, not the reaction to it)
 *   gyro           0xFF00/0x09  rad/s
 *   compass        0xFF00/0x0A  uT
 *   proximity      0xFF00/0x08  detection mask, near = receiver | large body contact
 *   ambient light  0xFF00/0x04  lux, in Level
 *   orientation    0x20/0x8A    from gravity (mtk_scp_hub.h), usages 0x64-0x6A, its time as a 0xFF15/2 child
 * and, with mp_sensor_steps=5, what locationd reads as vendor-defined reports (MappleSPU.h has the formats):
 *   CMA            0xFF0C/0x01  steps (StepCountUpdate) and activity (MotionStateUpdate) from the hub's step counter
 *   gesture        0xFF0C/0x03  raise to wake (detected) from the hub's pickup gesture; dismissed when it is laid
 *                               flat again within 10 s
 *   device motion  0xFF0C/0x04  9 axes (attitude to magnetic north, heading), /0x05 6 axes, /0x07 3 axes
 *
 * Each service carries IOCFPlugInTypes for AppleSPULib, whose DriverClass opens it with type 'ashd' (our
 * MappleSPUUserClient: send-command, open, close). The orientation service sits two levels under MappleSPUDevice,
 * where locationd's LastOrientation query goes (DeviceClass: open, close, send-command).
 *
 * ReportInterval (us, 0 = off) and BatchInterval (us) from iOS become the hub's rate and latency; DetectionMode
 * (0 = off) turns proximity on and off. Orientation runs the accelerometer at 10 Hz while it is on (from the start).
 * mp_spu_prox=0 leaves the proximity service out. Calibration knobs: mp_imu_axes, mp_mag_axes (default 0x123: a nibble per output axis x, y, z = the hub's axis 1-3,
 * +8 to flip it); mp_als_div (default 100: the hub's light value per lux, alsps's vender_div); mp_stride_cm
 * (default 70).
 */
#include <IOKit/IOLib.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <libkern/c++/OSNumber.h>
#include <pexpert/pexpert.h>

#include "IOHIDEventService.h"
#include "IOHIDEvent.h"
#include "IOHIDEventTypes.h"
#include "IOHIDKeys.h"
#include "MappleSPU.h"

extern "C" {
#include "../../../../bsd/dev/mtk_scp_hub.h"
uint64_t mach_absolute_time(void);
void absolutetime_to_nanoseconds(uint64_t abstime, uint64_t *result);
void mtk_scp_sensor_set(uint8_t type, uint32_t period_us, uint32_t latency_us);         /* bsd/dev/mtk_scp.c */
void mtk_scp_sensor_rearm(uint8_t type);
volatile uint8_t *mapple_spu_ring(uint64_t *pa);
void mapple_spu_start(uint32_t steps);
void mapple_spu_record(const struct sh_record *r, uint64_t ticks);
void mapple_spu_tick(void);
}

enum {
	SPU_ACCEL, SPU_GYRO, SPU_COMPASS, SPU_PROX, SPU_ALS, SPU_ORIENT,        /* step 4 */
	SPU_CMA, SPU_GESTURE, SPU_DM9, SPU_DM6, SPU_DM3,                        /* step 5 */
	SPU_COUNT
};

static const struct {
	const char *name;
	UInt32 page, usage;
} kinds[SPU_COUNT] = {
	{ "accelerometer", 0xff00, 0x03 },
	{ "gyro", 0xff00, 0x09 },
	{ "compass", 0xff00, 0x0a },
	{ "proximity", 0xff00, 0x08 },
	{ "als", 0xff00, 0x04 },
	{ "orientation", 0x20, 0x8a },
	{ "cma", 0xff0c, 0x01 },
	{ "gesture", 0xff0c, 0x03 },
	{ "devmotion", 0xff0c, 0x04 },
	{ "devmotion6", 0xff0c, 0x05 },
	{ "devmotion3", 0xff0c, 0x07 },
};

#define SPU_PLUGIN_DRIVER       "3BC5CC87-845E-48AB-A9C2-9436001BA68A"
#define SPU_PLUGIN_DEVICE       "7ACF5332-1A35-4893-87CB-BA64E1887FAE"
#define SPU_PLUGIN_PATH         "AppleSPU.kext/PlugIns/AppleSPULib.plugin"
#define SPU_UC_TYPE             0x61736864u     /* 'ashd' */
#define ORIENT_PERIOD_US        100000
#define CMA_CONF_HIGH           2       /* CLMotionActivity::Confidence: 0 low, 1 medium, 2 high */
#define CMA_MAX_DELTA           40      /* more steps in one report ends locationd's walk bout */
#define PROX_NEAR               (kIOHIDProximityDetectionReceiver | kIOHIDProximityDetectionLargeBodyContact)

typedef IOReturn (*SPUCommand)(IOService *owner, uint32_t cmd, const uint8_t *in, uint32_t inLen, uint8_t *out,
    uint32_t *outLen);

/* AppleSPULib's connection: the DriverClass table on a sensor, the DeviceClass one on MappleSPUDevice. */
class MappleSPUUserClient : public IOUserClient {
	OSDeclareDefaultStructors(MappleSPUUserClient);
public:
	IOService *fOwner;
	SPUCommand fCommand;
	bool fDevice;

	virtual IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target, void *reference) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(MappleSPUUserClient, IOUserClient);

static IOReturn
spu_new_client(IOService *owner, task_t task, void *securityID, UInt32 type, SPUCommand command, bool device,
    IOUserClient **handler)
{
	MappleSPUUserClient *uc = OSTypeAlloc(MappleSPUUserClient);

	if (uc == NULL) {
		return kIOReturnNoMemory;
	}
	uc->fOwner = owner;
	uc->fCommand = command;
	uc->fDevice = device;
	if (!uc->initWithTask(task, securityID, type) || !uc->attach(owner)) {
		uc->release();
		return kIOReturnBadArgument;
	}
	if (!uc->start(owner)) {
		uc->detach(owner);
		uc->release();
		return kIOReturnUnsupported;
	}
	*handler = uc;
	return kIOReturnSuccess;
}

IOReturn
MappleSPUUserClient::clientClose(void)
{
	terminate();
	return kIOReturnSuccess;
}

IOReturn
MappleSPUUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *a,
    IOExternalMethodDispatch *dispatch __unused, OSObject *target __unused, void *reference __unused)
{
	uint32_t send = fDevice ? 6 : 0, open = fDevice ? 4 : 3, close = fDevice ? 5 : 4;

	if (selector == open || selector == close) {
		return kIOReturnSuccess;
	}
	if (selector == send) {
		if (a->scalarInputCount < 1 || a->structureInputDescriptor || a->structureOutputDescriptor) {
			return kIOReturnBadArgument;
		}
		uint32_t outLen = a->structureOutputSize;
		IOReturn r = fCommand(fOwner, (uint32_t)a->scalarInput[0], (const uint8_t *)a->structureInput,
		    a->structureInputSize, (uint8_t *)a->structureOutput, &outLen);
		a->structureOutputSize = outLen;
		return r;
	}
	if (!fDevice && selector == 5 && a->structureInput && a->structureInputSize) {
		IOLog("SPU: AppleSPULib asserts: %.*s\n", (int)a->structureInputSize, (const char *)a->structureInput);
		return kIOReturnSuccess;
	}
	return kIOReturnUnsupported;    /* the historical ring (1, 2, 6) and the device's hardware calls */
}

class MappleSPUSensor : public IOHIDEventService {
	OSDeclareDefaultStructors(MappleSPUSensor);
public:
	unsigned fKind;
	uint32_t fReportUs, fBatchUs, fSequence, fLast;
	uint64_t fLastTicks;

	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setProperties(OSObject *properties) APPLE_KEXT_OVERRIDE;
	virtual IOHIDEvent *copyEvent(IOHIDEventType type, IOHIDEvent *matching, IOOptionBits options) APPLE_KEXT_OVERRIDE;
	virtual IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *properties,
	    IOUserClient **handler) APPLE_KEXT_OVERRIDE;
	void post(IOHIDEvent *e);
	void vendor(uint64_t ticks, const uint8_t *data, uint32_t len);
protected:
	virtual UInt32 getPrimaryUsagePage() APPLE_KEXT_OVERRIDE { return kinds[fKind].page; }
	virtual UInt32 getPrimaryUsage() APPLE_KEXT_OVERRIDE { return kinds[fKind].usage; }
	virtual OSString *getProduct() APPLE_KEXT_OVERRIDE { return OSString::withCString(kinds[fKind].name); }
	virtual UInt32 getProductID() APPLE_KEXT_OVERRIDE { return 0x100 + fKind; }
	virtual UInt32 getReportInterval() APPLE_KEXT_OVERRIDE { return fReportUs; }
};

OSDefineMetaClassAndStructors(MappleSPUSensor, IOHIDEventService);

/* Where LastOrientation is asked: the orientation service's grandparent, as AppleSPUHIDDevice is on the iPhone. */
class MappleSPUDevice : public IOService {
	OSDeclareDefaultStructors(MappleSPUDevice);
public:
	virtual IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *properties,
	    IOUserClient **handler) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(MappleSPUDevice, IOService);

static MappleSPUSensor *gSPU[SPU_COUNT];
static IOLock *gSPULock;
static uint32_t gSteps, gSamplesSaid;
static int8_t gImuAxes[3] = { 1, 2, 3 }, gMagAxes[3] = { 1, 2, 3 };
static uint32_t gAlsDiv = 100, gStrideCm = 70, gStep3AccelUs = 20000;
static uint64_t gWokeTicks;             /* raise to wake detected, not yet dismissed */

/* the latest of each, iOS's units (Q16), for the fusion */
static int32_t gAccel[3], gGyro[3], gMag[3];
static bool gHaveAccel, gHaveMag;
static uint64_t gGyroTicks;
static struct spu_fusion gF6, gF9;
static uint64_t gDmSent[SPU_COUNT];

/* steps: the hub counts from its enable; ours counts from boot */
static bool gStepsOn, gActivityOn;
static uint32_t gHubSteps, gStepBase, gStepCount, gStepsSent, gCadenceQ16, gMotion, gMotionSeq;
static uint64_t gLastStepTicks, gBoutTicks, gStepSentTicks, gMotionStartTicks;

static uint64_t
ticks_us(uint64_t t)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(t, &ns);
	return ns / 1000;
}

static uint32_t
on(unsigned k)
{
	return gSPU[k] ? gSPU[k]->fReportUs : 0;
}

static uint32_t
fastest(uint32_t a, uint32_t b)
{
	return a == 0 ? b : b == 0 ? a : a < b ? a : b;
}

/* Ask the hub for what the services want: the accelerometer and gyro also feed orientation and device motion. */
static void
spu_update(void)
{
	IOLockLock(gSPULock);
	uint32_t dm = fastest(fastest(on(SPU_DM9), on(SPU_DM6)), on(SPU_DM3));
	uint32_t accel = fastest(fastest(on(SPU_ACCEL), on(SPU_ORIENT)), dm);
	uint32_t gyro = fastest(on(SPU_GYRO), dm);
	uint32_t mag = fastest(on(SPU_COMPASS), on(SPU_DM9));
	uint32_t latency[SPU_COUNT];
	for (unsigned k = 0; k < SPU_COUNT; k++) {
		latency[k] = gSPU[k] ? gSPU[k]->fBatchUs : 0;
	}
	bool steps = gStepsOn || gActivityOn;
	bool gesture = gSPU[SPU_GESTURE] != NULL;
	IOLockUnlock(gSPULock);

	if (gSteps == 3) {
		accel = gStep3AccelUs;  /* step 3 runs the accelerometer whatever iOS asks (mp_scp_accel_us, default 50 Hz) */
	}
	/* batching only when nothing else needs the sensor live */
	mtk_scp_sensor_set(SH_ACCEL, accel, accel == on(SPU_ACCEL) && !on(SPU_ORIENT) && !dm ? latency[SPU_ACCEL] : 0);
	mtk_scp_sensor_set(SH_GYRO, gyro, gyro == on(SPU_GYRO) && !dm ? latency[SPU_GYRO] : 0);
	mtk_scp_sensor_set(SH_MAG, mag, mag == on(SPU_COMPASS) && !on(SPU_DM9) ? latency[SPU_COMPASS] : 0);
	mtk_scp_sensor_set(SH_PROX, on(SPU_PROX), 0);
	mtk_scp_sensor_set(SH_LIGHT, on(SPU_ALS), latency[SPU_ALS]);
	mtk_scp_sensor_set(SH_STEP_COUNTER, steps ? 1000000 : 0, 0);
	mtk_scp_sensor_set(SH_PICKUP, gesture ? 1000000 : 0, 0);
}

bool
MappleSPUSensor::start(IOService *provider)
{
	if (!IOHIDEventService::start(provider)) {
		return false;
	}
	OSDictionary *plugin = OSDictionary::withCapacity(1);
	OSString *path = OSString::withCString(SPU_PLUGIN_PATH);
	if (plugin && path) {
		plugin->setObject(SPU_PLUGIN_DRIVER, path);
		setProperty("IOCFPlugInTypes", plugin);
	}
	OSSafeReleaseNULL(plugin);
	OSSafeReleaseNULL(path);
	if (fKind == SPU_PROX) {
		/* backboardd wraps only a proximity service at the top (Placement 1); any other leaves NSNull in its list */
		setProperty("Placement", 1ull, 32);
	}
	registerService();
	IOLog("SPU: %s published (0x%x/0x%x)\n", kinds[fKind].name, (unsigned)kinds[fKind].page, (unsigned)kinds[fKind].usage);
	return true;
}

static bool
number(OSDictionary *d, const char *key, uint32_t *out)
{
	OSNumber *n = OSDynamicCast(OSNumber, d->getObject(key));
	if (n) {
		*out = n->unsigned32BitValue();
	}
	return n != NULL;
}

IOReturn
MappleSPUSensor::setProperties(OSObject *properties)
{
	OSDictionary *d = OSDynamicCast(OSDictionary, properties);
	uint32_t v;
	bool changed = false;

	if (!d) {
		return kIOReturnBadArgument;
	}
	IOLockLock(gSPULock);
	if (number(d, kIOHIDReportIntervalKey, &v) && fKind != SPU_PROX) {
		changed |= v != fReportUs;
		fReportUs = v;
	}
	if (number(d, kIOHIDBatchIntervalKey, &v)) {
		changed |= v != fBatchUs;
		fBatchUs = v;
	}
	if (number(d, "DetectionMode", &v) && fKind == SPU_PROX) {
		uint32_t want = v ? 100000 : 0;
		changed |= want != fReportUs;
		fReportUs = want;
	}
	IOLockUnlock(gSPULock);
	if (changed) {
		setProperty(kIOHIDReportIntervalKey, fReportUs, 32);
		setProperty(kIOHIDBatchIntervalKey, fBatchUs, 32);
		spu_update();
	}
	return kIOReturnSuccess;
}

IOHIDEvent *
MappleSPUSensor::copyEvent(IOHIDEventType type, IOHIDEvent *matching __unused, IOOptionBits options __unused)
{
	if (fKind == SPU_ORIENT && type == kIOHIDEventTypeOrientation && fLast) {
		IOHIDEvent *e = IOHIDEvent::orientationEvent(mach_absolute_time(), kIOHIDOrientationTypeCMUsage);
		if (e) {
			e->setIntegerValue(kIOHIDEventFieldOrientationDeviceOrientationUsage, fLast);
		}
		return e;
	}
	return NULL;
}

/*
 * Commands from locationd (always 0x20 with the protocol in the bytes). The CMA's Configure (0x06) says whether
 * steps and activity are wanted; everything else is taken as done. No reply bytes: locationd checks the return only.
 */
static IOReturn
sensor_command(IOService *owner, uint32_t cmd __unused, const uint8_t *in, uint32_t inLen, uint8_t *out __unused,
    uint32_t *outLen)
{
	MappleSPUSensor *s = OSDynamicCast(MappleSPUSensor, owner);

	*outLen = 0;
	if (s && s->fKind == SPU_CMA && in && inLen >= 3 && in[0] == CMA_CONFIGURE) {
		bool activity = in[1] != 0, steps = in[2] != 0;
		if (activity != gActivityOn || steps != gStepsOn) {
			gActivityOn = activity;
			gStepsOn = steps;
			IOLog("SPU: cma: activity %d, step counting %d\n", activity, steps);
			spu_update();
		}
	}
	return kIOReturnSuccess;
}

IOReturn
MappleSPUSensor::newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *properties,
    IOUserClient **handler)
{
	if (type == SPU_UC_TYPE) {
		return spu_new_client(this, owningTask, securityID, type, sensor_command, false, handler);
	}
	return IOHIDEventService::newUserClient(owningTask, securityID, type, properties, handler);
}

/* LastOrientation: [0x02] -> {0x02, usage - 0x64, u64 mach time}; the form-factor config [0x04, ...] -> done */
static IOReturn
device_command(IOService *owner __unused, uint32_t cmd __unused, const uint8_t *in, uint32_t inLen, uint8_t *out,
    uint32_t *outLen)
{
	MappleSPUSensor *o = gSPU[SPU_ORIENT];

	if (in && inLen >= 1 && in[0] == 0x02) {
		if (out == NULL || *outLen < SPU_ORIENT_REPLY_LEN || o == NULL || o->fLast == 0) {
			*outLen = 0;
			return kIOReturnNotReady;
		}
		spu_orientation_reply(out, o->fLast, o->fLastTicks);
		*outLen = SPU_ORIENT_REPLY_LEN;
		return kIOReturnSuccess;
	}
	*outLen = 0;
	return kIOReturnSuccess;
}

IOReturn
MappleSPUDevice::newUserClient(task_t owningTask, void *securityID, UInt32 type, OSDictionary *properties,
    IOUserClient **handler)
{
	if (type == SPU_UC_TYPE) {
		return spu_new_client(this, owningTask, securityID, type, device_command, true, handler);
	}
	return IOService::newUserClient(owningTask, securityID, type, properties, handler);
}

void
MappleSPUSensor::post(IOHIDEvent *e)
{
	if (e) {
		dispatchEvent(e);
		e->release();
	}
}

void
MappleSPUSensor::vendor(uint64_t ticks, const uint8_t *data, uint32_t len)
{
	post(IOHIDEvent::vendorDefinedEvent(ticks, kinds[fKind].page, kinds[fKind].usage, 0, (UInt8 *)data, len));
}

static void
axes(const int8_t map[3], const int32_t *in, int32_t (*conv)(int32_t), int32_t out[3])
{
	int64_t v[3];
	sh_axes(map, in, v);
	for (int i = 0; i < 3; i++) {
		out[i] = conv((int32_t)v[i]);
	}
}

/* device motion on each gyro sample: both filters, each service at its own interval */
static void
device_motion(uint64_t ticks)
{
	uint32_t dt = gGyroTicks ? (uint32_t)(ticks_us(ticks) - ticks_us(gGyroTicks)) : 0;

	gGyroTicks = ticks;
	if (!gHaveAccel || dt == 0 || dt > 1000000) {
		return;
	}
	if (on(SPU_DM6) || on(SPU_DM3)) {
		spu_fuse(&gF6, gGyro, gAccel, NULL, dt);
	}
	if (on(SPU_DM9) && gHaveMag) {
		spu_fuse(&gF9, gGyro, gAccel, gMag, dt);
	}
	static const unsigned dms[3] = { SPU_DM9, SPU_DM6, SPU_DM3 };
	for (unsigned i = 0; i < 3; i++) {
		unsigned k = dms[i];
		MappleSPUSensor *s = gSPU[k];
		if (!s || !s->fReportUs || (k == SPU_DM9 && !gHaveMag) || ticks_us(ticks) - gDmSent[k] < s->fReportUs * 9 / 10) {
			continue;
		}
		gDmSent[k] = ticks_us(ticks);
		struct spu_motion m;
		uint8_t r[SPU_DM9_LEN];
		spu_motion_from(k == SPU_DM9 ? &gF9 : &gF6, gGyro, gAccel, k == SPU_DM9 ? gMag : NULL, &m);
		if (k == SPU_DM9) {
			spu_dm9(r, &m);
			s->vendor(ticks, r, SPU_DM9_LEN);
		} else {
			spu_dm(r, k == SPU_DM6 ? 3 : 4, &m);
			s->vendor(ticks, r, SPU_DM_LEN);
		}
	}
}

static void
send_steps(uint64_t ticks)
{
	MappleSPUSensor *s = gSPU[SPU_CMA];
	uint8_t m[CMA_STEP_COUNT_LEN];
	struct spu_steps st = { gStepCount, gStepCount - gStepsSent, gCadenceQ16,
		                (uint32_t)(ticks_us(gLastStepTicks) - ticks_us(gBoutTicks)), (uint8_t)gStrideCm };

	if (s && gStepsOn) {
		spu_cma_steps(m, &st);
		s->vendor(ticks, m, CMA_STEP_COUNT_LEN);
	}
	gStepsSent = gStepCount;
	gStepSentTicks = ticks;
}

static void
send_motion(uint64_t ticks, uint32_t motion)
{
	MappleSPUSensor *s = gSPU[SPU_CMA];
	uint8_t m[CMA_MOTION_STATE_LEN];

	if (motion != gMotion) {
		gMotion = motion;
		gMotionStartTicks = ticks;
	}
	if (s && gActivityOn) {
		spu_cma_motion(m, motion, CMA_CONF_HIGH, ++gMotionSeq, ticks_us(ticks), ticks_us(gMotionStartTicks));
		s->vendor(ticks, m, CMA_MOTION_STATE_LEN);
	}
}

/* the hub's step counter: a total since it was enabled (u32), on change */
static void
steps(uint32_t hub, uint64_t ticks)
{
	if (hub < gHubSteps) {
		gStepBase += gHubSteps;         /* the hub started again */
	}
	uint32_t total = gStepBase + hub, delta = total - gStepCount;
	gHubSteps = hub;
	if (delta == 0) {
		return;
	}
	uint64_t gap = gLastStepTicks ? ticks_us(ticks) - ticks_us(gLastStepTicks) : 0;
	if (gLastStepTicks == 0 || gap > 4000000) {
		gBoutTicks = ticks;
		gCadenceQ16 = 0;
	} else if (gap) {
		gCadenceQ16 = (uint32_t)(((uint64_t)delta << 16) * 1000000 / gap);
	}
	gStepCount = total;
	gLastStepTicks = ticks;
	if (ticks_us(ticks) - ticks_us(gStepSentTicks) >= 1000000 || gStepCount - gStepsSent >= CMA_MAX_DELTA) {
		send_steps(ticks);
	}
	uint32_t motion = spu_activity(gCadenceQ16, 0);
	if (motion != gMotion) {
		send_motion(ticks, motion);
	}
}

extern "C" void
mapple_spu_record(const struct sh_record *r, uint64_t ticks)
{
	MappleSPUSensor *s;
	int32_t v[3];

	if (r->action != 0) {
		return;         /* flush done, bias, calibration */
	}
	switch (r->type) {
	case SH_ACCEL:
		axes(gImuAxes, r->d, sh_accel_g, v);
		memcpy(gAccel, v, sizeof(v));
		gHaveAccel = true;
		if ((s = gSPU[SPU_ACCEL]) && (s->fReportUs || gSteps == 3)) {
			s->post(IOHIDEvent::accelerometerEvent(ticks, v[0], v[1], v[2], 0, 0, s->fSequence++));
			if (gSteps == 3 && gSamplesSaid < 30 && (s->fSequence % 50) == 0) {
				gSamplesSaid++;
				IOLog("SPU: accelerometer %d %d %d (hub x1000 m/s^2) -> %d %d %d mg\n", r->d[0], r->d[1], r->d[2],
				    (int)(((int64_t)v[0] * 1000) >> 16), (int)(((int64_t)v[1] * 1000) >> 16), (int)(((int64_t)v[2] * 1000) >> 16));
			}
		}
		if ((s = gSPU[SPU_ORIENT]) && s->fReportUs) {
			uint32_t o = sh_orientation(v[0], v[1], v[2], s->fLast);
			if (o != s->fLast) {
				s->fLast = o;
				s->fLastTicks = ticks;
				IOHIDEvent *e = IOHIDEvent::orientationEvent(ticks, kIOHIDOrientationTypeCMUsage);
				if (e) {
					e->setIntegerValue(kIOHIDEventFieldOrientationDeviceOrientationUsage, o);
					IOHIDEvent *t = IOHIDEvent::vendorDefinedEvent(ticks, 0xff15, 2, 0, (UInt8 *)&ticks, sizeof(ticks));
					if (t) {
						e->appendChild(t);
						t->release();
					}
				}
				IOLog("SPU: orientation 0x%x\n", o);
				s->post(e);
				if (gWokeTicks && (o == SH_ORIENT_FACE_UP || o == SH_ORIENT_FACE_DOWN) && gSPU[SPU_GESTURE]) {
					uint8_t m[SPU_WAKE_LEN];
					if (ticks_us(ticks) - ticks_us(gWokeTicks) < 10000000) {
						spu_wake(m, SPU_WAKE_DISMISSED, true);
						gSPU[SPU_GESTURE]->vendor(ticks, m, SPU_WAKE_LEN);
						IOLog("SPU: laid down -> raise to wake dismissed\n");
					}
					gWokeTicks = 0;
				}
			}
		}
		break;
	case SH_GYRO:
		axes(gImuAxes, r->d, sh_gyro_rads, v);
		memcpy(gGyro, v, sizeof(v));
		if ((s = gSPU[SPU_GYRO]) && s->fReportUs) {
			s->post(IOHIDEvent::gyroEvent(ticks, v[0], v[1], v[2], 0, 0, s->fSequence++));
		}
		device_motion(ticks);
		break;
	case SH_MAG:
		axes(gMagAxes, r->d, sh_mag_ut, v);
		memcpy(gMag, v, sizeof(v));
		gHaveMag = true;
		if ((s = gSPU[SPU_COMPASS]) && s->fReportUs) {
			s->post(IOHIDEvent::compassEvent(ticks, v[0], v[1], v[2], 0, 0, s->fSequence++));
		}
		break;
	case SH_PROX:
		/* proximity_t {u32 steps, i32 oneshot}: oneshot 0 = near, as Android's distance */
		if ((s = gSPU[SPU_PROX]) && s->fReportUs) {
			s->post(IOHIDEvent::proximityEvent(ticks, r->d[1] == 0 ? PROX_NEAR : 0, (UInt32)r->d[0]));
			if (s->fSequence++ < 20) {
				IOLog("SPU: proximity raw %u, oneshot %d\n", (unsigned)r->d[0], r->d[1]);
			}
		}
		break;
	case SH_LIGHT:
		if ((s = gSPU[SPU_ALS]) && s->fReportUs) {
			s->post(IOHIDEvent::ambientLightSensorEvent(ticks, r->d[0] < 0 ? 0 : (UInt32)r->d[0] / gAlsDiv));
			if (s->fSequence++ < 20) {
				IOLog("SPU: light raw %d\n", r->d[0]);
			}
		}
		break;
	case SH_STEP_COUNTER:
		steps((uint32_t)r->d[0], ticks);
		break;
	case SH_PICKUP:
		if ((s = gSPU[SPU_GESTURE])) {
			uint8_t m[SPU_WAKE_LEN];
			spu_wake(m, SPU_WAKE_DETECTED, false);
			s->vendor(ticks, m, SPU_WAKE_LEN);
			gWokeTicks = ticks;
			IOLog("SPU: pickup -> raise to wake\n");
		}
		mtk_scp_sensor_rearm(SH_PICKUP);        /* one shot: enabled again for the next */
		break;
	}
}

/* Once a second from the hub's thread: the last steps out, and static once they stop. */
extern "C" void
mapple_spu_tick(void)
{
	uint64_t now = mach_absolute_time();

	if (gStepCount != gStepsSent) {
		send_steps(now);
	}
	if (gMotion != CMA_STATIC && gLastStepTicks && ticks_us(now) - ticks_us(gLastStepTicks) > 4000000) {
		gCadenceQ16 = 0;
		send_motion(now, CMA_STATIC);
	}
}

static void
axes_arg(const char *name, int8_t map[3])
{
	uint32_t v = 0x123;

	PE_parse_boot_argn(name, &v, sizeof(v));
	for (int i = 0; i < 3; i++) {
		uint32_t n = (v >> (8 - 4 * i)) & 0xf;
		map[i] = (int8_t)((n & 8) ? -(int)(n & 7) : (int)(n & 7));
	}
}

static IOService *
attached(IOService *s, IOService *provider)
{
	if (!s->init() || !s->attach(provider)) {
		s->release();
		return NULL;
	}
	return s;
}

/* From the hub's thread once it is up: step 3 the accelerometer, step 4 the plain sensors, step 5 all. */
extern "C" void
mapple_spu_start(uint32_t steps)
{
	IOService *res = IOService::getResourceService(), *iface = NULL;
	unsigned count = steps >= 5 ? SPU_COUNT : steps == 4 ? SPU_CMA : 1;

	gSteps = steps;
	gSPULock = IOLockAlloc();
	axes_arg("mp_imu_axes", gImuAxes);
	axes_arg("mp_mag_axes", gMagAxes);
	PE_parse_boot_argn("mp_als_div", &gAlsDiv, sizeof(gAlsDiv));
	PE_parse_boot_argn("mp_stride_cm", &gStrideCm, sizeof(gStrideCm));
	PE_parse_boot_argn("mp_scp_accel_us", &gStep3AccelUs, sizeof(gStep3AccelUs));
	gAlsDiv = gAlsDiv ? gAlsDiv : 1;
	gMotion = CMA_STATIC;

	if (steps >= 5) {
		/* the orientation service's parent and grandparent, for AppleSPULib's DeviceClass */
		IOService *dev = attached(OSTypeAlloc(MappleSPUDevice), res);
		if (dev) {
			OSDictionary *plugin = OSDictionary::withCapacity(1);
			OSString *path = OSString::withCString(SPU_PLUGIN_PATH);
			if (plugin && path) {
				plugin->setObject(SPU_PLUGIN_DEVICE, path);
				dev->setProperty("IOCFPlugInTypes", plugin);
			}
			OSSafeReleaseNULL(plugin);
			OSSafeReleaseNULL(path);
			dev->setName("spu-device");
			dev->registerService();
			iface = attached(OSTypeAlloc(IOService), dev);
			if (iface) {
				iface->setName("spu-orientation");
				iface->registerService();
			}
		}
	}
	uint32_t prox = 1;
	PE_parse_boot_argn("mp_spu_prox", &prox, sizeof(prox));
	for (unsigned k = 0; k < count; k++) {
		if (k == SPU_PROX && prox == 0) {
			continue;
		}
		MappleSPUSensor *s = OSTypeAlloc(MappleSPUSensor);
		IOService *provider = (k == SPU_ORIENT && iface) ? iface : res;
		if (!s) {
			continue;
		}
		s->fKind = k;
		s->fReportUs = k == SPU_ORIENT ? ORIENT_PERIOD_US : 0;
		if (!attached(s, provider)) {
			continue;
		}
		if (!s->start(provider)) {
			s->detach(provider);
			s->release();
			continue;
		}
		IOLockLock(gSPULock);
		gSPU[k] = s;
		IOLockUnlock(gSPULock);
	}
	spu_update();
}

/* The hub's ring: 1 MiB, physically contiguous below 2 GiB (the SCP reaches 0x40000000-0x8fffffff), taken at boot;
 * read through a write-combined mapping so the CPU sees what the SCP writes. */
extern "C" volatile uint8_t *
mapple_spu_ring(uint64_t *pa)
{
	static IOBufferMemoryDescriptor *mem;
	static IOMemoryMap *map;

	if (map == NULL) {
		mem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
		    kIODirectionInOut | kIOMemoryPhysicallyContiguous, SH_RING_BYTES, 0x000000007ffff000ull);
		if (mem == NULL) {
			return NULL;
		}
		bzero(mem->getBytesNoCopy(), SH_RING_BYTES);
		mem->performOperation(kIOMemoryIncoherentIOFlush, 0, SH_RING_BYTES);
		map = mem->createMappingInTask(kernel_task, 0, kIOMapAnywhere | kIOMapWriteCombineCache, 0, 0);
		if (map == NULL) {
			OSSafeReleaseNULL(mem);
			return NULL;
		}
	}
	*pa = mem->getPhysicalSegment(0, NULL, kIOMemoryMapperNone);
	return (volatile uint8_t *)map->getVirtualAddress();
}
