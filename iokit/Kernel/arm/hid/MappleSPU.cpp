/*
 * The sensors as the iPhone X's always-on processor presents them to iOS: one HID event service per sensor, which
 * locationd, CoreMotion, backboardd and CoreBrightness find by its usage pair (DeviceUsagePairs), fed by the SCP's
 * sensor hub (bsd/dev/mtk_scp.c). Our own class name keeps Apple's ALS and compass plug-ins off: they attach to
 * their own drivers' names only.
 *
 *   accelerometer  0xFF00/0x03  g, iOS's sign (gravity itself, not the reaction to it)
 *   gyro           0xFF00/0x09  rad/s
 *   compass        0xFF00/0x0A  uT
 *   proximity      0xFF00/0x08  detection mask, near = receiver | large body contact
 *   ambient light  0xFF00/0x04  lux, in Level
 *   orientation    0x20/0x8A    from gravity (mtk_scp_hub.h), usages 0x64-0x6A, its time as a 0xFF15/2 child
 *
 * ReportInterval (us, 0 = off) and BatchInterval (us) from iOS become the hub's rate and latency; DetectionMode
 * (0 = off) turns proximity on and off. Orientation runs the accelerometer at 10 Hz while it is on (from the start).
 * Calibration knobs: mp_imu_axes, mp_mag_axes (default 0x123: a nibble per output axis x, y, z = the hub's axis 1-3,
 * +8 to flip it); mp_als_div (default 100: the hub's light value per lux, alsps's vender_div).
 */
#include <IOKit/IOLib.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <libkern/c++/OSNumber.h>
#include <pexpert/pexpert.h>

#include "IOHIDEventService.h"
#include "IOHIDEvent.h"
#include "IOHIDEventTypes.h"
#include "IOHIDKeys.h"

extern "C" uint64_t mach_absolute_time(void);

extern "C" {
#include "../../../../bsd/dev/mtk_scp_hub.h"
void mtk_scp_sensor_set(uint8_t type, uint32_t period_us, uint32_t latency_us);         /* bsd/dev/mtk_scp.c */
volatile uint8_t *mapple_spu_ring(uint64_t *pa);
void mapple_spu_start(uint32_t steps);
void mapple_spu_record(const struct sh_record *r, uint64_t ticks);
}

enum { SPU_ACCEL, SPU_GYRO, SPU_COMPASS, SPU_PROX, SPU_ALS, SPU_ORIENT, SPU_COUNT };

static const struct {
	const char *name;
	UInt32 page, usage;
	uint8_t hub;
} kinds[SPU_COUNT] = {
	{ "accelerometer", 0xff00, 0x03, SH_ACCEL },
	{ "gyro", 0xff00, 0x09, SH_GYRO },
	{ "compass", 0xff00, 0x0a, SH_MAG },
	{ "proximity", 0xff00, 0x08, SH_PROX },
	{ "als", 0xff00, 0x04, SH_LIGHT },
	{ "orientation", 0x20, 0x8a, SH_ACCEL },
};

#define ORIENT_PERIOD_US        100000
#define PROX_NEAR               (kIOHIDProximityDetectionReceiver | kIOHIDProximityDetectionLargeBodyContact)

class MappleSPUSensor : public IOHIDEventService {
	OSDeclareDefaultStructors(MappleSPUSensor);
public:
	unsigned fKind;
	uint32_t fReportUs, fBatchUs, fSequence, fLast;

	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual IOReturn setProperties(OSObject *properties) APPLE_KEXT_OVERRIDE;
	virtual IOHIDEvent *copyEvent(IOHIDEventType type, IOHIDEvent *matching, IOOptionBits options) APPLE_KEXT_OVERRIDE;
	void report(const struct sh_record *r, uint64_t ticks);
protected:
	virtual UInt32 getPrimaryUsagePage() APPLE_KEXT_OVERRIDE { return kinds[fKind].page; }
	virtual UInt32 getPrimaryUsage() APPLE_KEXT_OVERRIDE { return kinds[fKind].usage; }
	virtual OSString *getProduct() APPLE_KEXT_OVERRIDE { return OSString::withCString(kinds[fKind].name); }
	virtual UInt32 getProductID() APPLE_KEXT_OVERRIDE { return 0x100 + fKind; }
	virtual UInt32 getReportInterval() APPLE_KEXT_OVERRIDE { return fReportUs; }
};

OSDefineMetaClassAndStructors(MappleSPUSensor, IOHIDEventService);

static MappleSPUSensor *gSPU[SPU_COUNT];
static IOLock *gSPULock;
static uint32_t gSteps, gSamplesSaid;
static int8_t gImuAxes[3] = { 1, 2, 3 }, gMagAxes[3] = { 1, 2, 3 };
static uint32_t gAlsDiv = 100;

/* Ask the hub for what the services want; the accelerometer serves the orientation too. */
static void
spu_update(void)
{
	uint32_t p[SPU_COUNT] = {}, l[SPU_COUNT] = {};

	IOLockLock(gSPULock);
	for (unsigned k = 0; k < SPU_COUNT; k++) {
		if (gSPU[k]) {
			p[k] = gSPU[k]->fReportUs;
			l[k] = gSPU[k]->fBatchUs;
		}
	}
	IOLockUnlock(gSPULock);
	uint32_t ap = p[SPU_ACCEL], al = l[SPU_ACCEL];
	if (p[SPU_ORIENT] && (ap == 0 || p[SPU_ORIENT] < ap)) {
		ap = p[SPU_ORIENT];
	}
	if (p[SPU_ORIENT] && p[SPU_ACCEL] && l[SPU_ORIENT] < al) {
		al = l[SPU_ORIENT];
	} else if (p[SPU_ACCEL] == 0) {
		al = l[SPU_ORIENT];
	}
	if (gSteps == 3 && (ap == 0 || ap > 20000)) {
		ap = 20000;     /* step 3 runs the accelerometer at 50 Hz whatever iOS asks */
	}
	mtk_scp_sensor_set(SH_ACCEL, ap, al);
	for (unsigned k = SPU_GYRO; k < SPU_ORIENT; k++) {
		mtk_scp_sensor_set(kinds[k].hub, p[k], l[k]);
	}
}

bool
MappleSPUSensor::start(IOService *provider)
{
	if (!IOHIDEventService::start(provider)) {
		return false;
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

static void
axes(const int8_t map[3], const int32_t *in, int32_t (*conv)(int32_t), IOFixed out[3])
{
	int64_t v[3];
	sh_axes(map, in, v);
	for (int i = 0; i < 3; i++) {
		out[i] = conv((int32_t)v[i]);
	}
}

void
MappleSPUSensor::report(const struct sh_record *r, uint64_t ticks)
{
	IOHIDEvent *e = NULL;
	IOFixed f[3];

	switch (fKind) {
	case SPU_ACCEL:
		axes(gImuAxes, r->d, sh_accel_g, f);
		e = IOHIDEvent::accelerometerEvent(ticks, f[0], f[1], f[2], 0, 0, fSequence++);
		if (gSteps == 3 && gSamplesSaid < 30 && (fSequence % 50) == 0) {
			gSamplesSaid++;
			IOLog("SPU: accelerometer %d %d %d (hub x1000 m/s^2) -> %d %d %d mg\n", r->d[0], r->d[1], r->d[2],
			    (int)(((int64_t)f[0] * 1000) >> 16), (int)(((int64_t)f[1] * 1000) >> 16), (int)(((int64_t)f[2] * 1000) >> 16));
		}
		break;
	case SPU_GYRO:
		axes(gImuAxes, r->d, sh_gyro_rads, f);
		e = IOHIDEvent::gyroEvent(ticks, f[0], f[1], f[2], 0, 0, fSequence++);
		break;
	case SPU_COMPASS:
		axes(gMagAxes, r->d, sh_mag_ut, f);
		e = IOHIDEvent::compassEvent(ticks, f[0], f[1], f[2], 0, 0, fSequence++);
		break;
	case SPU_PROX:
		/* proximity_t {u32 steps, i32 oneshot}: oneshot 0 = near, as Android's distance */
		e = IOHIDEvent::proximityEvent(ticks, r->d[1] == 0 ? PROX_NEAR : 0, (UInt32)r->d[0]);
		if (fSequence++ < 20) {
			IOLog("SPU: proximity raw %u, oneshot %d\n", (unsigned)r->d[0], r->d[1]);
		}
		break;
	case SPU_ALS:
		e = IOHIDEvent::ambientLightSensorEvent(ticks, r->d[0] < 0 ? 0 : (UInt32)r->d[0] / gAlsDiv);
		if (fSequence++ < 20) {
			IOLog("SPU: light raw %d\n", r->d[0]);
		}
		break;
	case SPU_ORIENT: {
		axes(gImuAxes, r->d, sh_accel_g, f);
		uint32_t o = sh_orientation(f[0], f[1], f[2], fLast);
		if (o == fLast) {
			return;
		}
		fLast = o;
		e = IOHIDEvent::orientationEvent(ticks, kIOHIDOrientationTypeCMUsage);
		if (e) {
			e->setIntegerValue(kIOHIDEventFieldOrientationDeviceOrientationUsage, o);
			IOHIDEvent *t = IOHIDEvent::vendorDefinedEvent(ticks, 0xff15, 2, 0, (UInt8 *)&ticks, sizeof(ticks));
			if (t) {
				e->appendChild(t);
				t->release();
			}
		}
		IOLog("SPU: orientation 0x%x\n", o);
		break;
	}
	}
	if (e) {
		dispatchEvent(e);
		e->release();
	}
}

extern "C" void
mapple_spu_record(const struct sh_record *r, uint64_t ticks)
{
	if (r->action != 0) {
		return;         /* flush done, bias, calibration */
	}
	for (unsigned k = 0; k < SPU_COUNT; k++) {
		MappleSPUSensor *s = gSPU[k];
		if (s && kinds[k].hub == r->type && (s->fReportUs || (k == SPU_ACCEL && gSteps == 3))) {
			s->report(r, ticks);
		}
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

/* From the hub's thread once it is up: step 3 the accelerometer, step 4 all. */
extern "C" void
mapple_spu_start(uint32_t steps)
{
	gSteps = steps;
	gSPULock = IOLockAlloc();
	axes_arg("mp_imu_axes", gImuAxes);
	axes_arg("mp_mag_axes", gMagAxes);
	PE_parse_boot_argn("mp_als_div", &gAlsDiv, sizeof(gAlsDiv));
	gAlsDiv = gAlsDiv ? gAlsDiv : 1;
	for (unsigned k = 0; k < (steps >= 4 ? SPU_COUNT : 1); k++) {
		MappleSPUSensor *s = OSTypeAlloc(MappleSPUSensor);
		if (!s) {
			continue;
		}
		s->fKind = k;
		s->fReportUs = k == SPU_ORIENT ? ORIENT_PERIOD_US : 0;
		if (!s->init() || !s->attach(IOService::getResourceService())) {
			s->release();
			continue;
		}
		if (!s->start(IOService::getResourceService())) {
			s->detach(IOService::getResourceService());
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
