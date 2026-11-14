/*
 * Built-in AppleKeyStore: the system keybag is always loaded; with a passcode
 * set it is locked at boot and on lock, and unlocked by the passcode.
 *
 * Apple's AppleKeyStore kext (closed, SEP-backed) owns the keybags behind
 * data protection. libaks (AppleKeyStore.framework, statically in keybagd)
 * opens IOServiceMatching("AppleKeyStore") with type 0, calls selector 0,
 * then one selector per aks_* function (iOS 16.7 numbering below). keybagd
 * --init asks for the system bag (14) and only loads systembag.kb when the
 * answer is kIOReturnNotFound; answering with a handle is the path a
 * userspace reboot takes, so no bag file is needed.
 *
 * The passcode (MappleKeyStorePasscode.h) is checked here, with iOS's delays
 * after wrong attempts, and kept as a salted PBKDF2 verifier in
 * /private/var/keybags/mapple.kb. Without a Secure Enclave nothing is
 * hardware-bound and no data is encrypted under it: it locks the UI only.
 *
 * Every call is logged (selector, caller, argument shapes, result) so the
 * next selectors the system needs show up on the console; unknown ones fail
 * with kIOReturnUnsupported.
 */
#include <pexpert/arm64/board_config.h>

#if defined(MT6781)
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <libkern/c++/OSString.h>
#include <IOKit/IOLocks.h>
#include <IOKit/system.h>
#include <kern/clock.h>
#include <sys/proc.h>
#include <sys/vnode.h>
#include <sys/fcntl.h>
#include <sys/random.h>

#include "MappleKeyStorePasscode.h"

/* libaks selectors (AppleKeyStore.framework, iOS 16.7). */
enum {
	kAKSOpen             = 0,
	kAKSCreateBag        = 2,
	kAKSSaveBag          = 3,
	kAKSUnloadBag        = 4,
	kAKSSetSystem        = 5,
	kAKSLoadBag          = 6,
	kAKSGetLockState     = 7,
	kAKSLockDevice       = 8,
	kAKSUnlockDevice     = 9,
	kAKSWrapKey          = 10,
	kAKSUnwrapKey        = 11,
	kAKSUnlockBag        = 12,
	kAKSLockBag          = 13,
	kAKSGetSystem        = 14,
	kAKSChangeSecret     = 15,      /* aks_change_secret_se: handle, flags, -, old, old len, new, new len */
	kAKSGetDeviceState   = 17,
	kAKSGetExtDeviceState = 35,     /* aks_get_extended_device_state */
	kAKSAssertHold       = 26,
	kAKSAssertDrop       = 27,
	kAKSLockDeviceNow    = 32,
	kAKSVerifyPassword   = 42,      /* aks_verify_password: handle; packed {passcode, context} */
	kAKSUnloadSessionBags = 55,
	kAKSNotifyStateSet   = 102,
	kAKSNotifyStateIsSet = 103,
	kAKSAssertConsume    = 101,     /* aks_assert_consume */
	kAKSChangeSecretEpilogue = 105,
};

/* The system (device) bag handle. */
#define MAPPLE_AKS_SYSTEM_HANDLE 0

/*
 * aks_get_lock_state bits: 0 locked, 1 no passcode, 2 unlocked since boot
 * (MKBDeviceUnlockedSinceBoot). keybagd takes bit 13 for an SE-bound
 * passcode and bit 22 for an xART bag; both stay clear here.
 */
#define MAPPLE_AKS_LOCKED        0x1
#define MAPPLE_AKS_NO_PASSCODE   0x2
#define MAPPLE_AKS_BEEN_UNLOCKED 0x4

/* MKBGetDeviceLockState (the "sls" field): ManagedConfiguration takes 3 as "no passcode". */
#define MAPPLE_MKB_UNLOCKED      0
#define MAPPLE_MKB_LOCKED        1
#define MAPPLE_MKB_DISABLED      3

#define MAPPLE_AKS_FILE          "/private/var/keybags/mapple.kb"

static IOLock *gAKSLock;
static struct mapple_aks_rec gAKSRec;
static bool gAKSLoaded, gAKSLocked, gAKSBeenUnlocked;

static uint64_t
mapple_aks_now(void)
{
	clock_sec_t sec;
	clock_usec_t usec;

	clock_get_calendar_microtime(&sec, &usec);
	return sec;
}

/* Read the record once; a missing file is no passcode. Called with gAKSLock held. */
static void
mapple_aks_load(void)
{
	struct mapple_aks_rec slot[2];
	const struct mapple_aks_rec *r;
	vfs_context_t ctx = vfs_context_kernel();
	vnode_t vp = NULLVP;
	int resid = 0, err;

	if (gAKSLoaded) {
		return;
	}
	bzero(slot, sizeof(slot));
	err = vnode_open(MAPPLE_AKS_FILE, FREAD | O_NOFOLLOW, 0, VNODE_LOOKUP_NOFOLLOW, &vp, ctx);
	if (err == 0) {
		err = vn_rdwr(UIO_READ, vp, (caddr_t)slot, sizeof(slot), 0, UIO_SYSSPACE, IO_NODELOCKED,
		    vfs_context_ucred(ctx), &resid, vfs_context_proc(ctx));
		vnode_close(vp, FREAD, ctx);
	}
	if (err != 0 && err != ENOENT) {
		IOLog("AKS: %s: error %d, trying again at the next call\n", MAPPLE_AKS_FILE, err);
		return;
	}
	gAKSLoaded = true;
	r = mapple_aks_rec_pick(slot);
	if (r != NULL) {
		gAKSRec = *r;
	} else if (err == 0) {
		IOLog("AKS: %s holds no valid record: no passcode\n", MAPPLE_AKS_FILE);
	}
	bzero(slot, sizeof(slot));
	gAKSLocked = gAKSRec.has != 0;
	gAKSBeenUnlocked = !gAKSLocked;
	IOLog("AKS: passcode %s, %u failed attempts\n", gAKSRec.has ? "set, locked" : "not set", gAKSRec.failed);
}

/* Write r into the older slot. Called with gAKSLock held. */
static int
mapple_aks_save(struct mapple_aks_rec *r)
{
	vfs_context_t ctx = vfs_context_kernel();
	vnode_t vp = NULLVP;
	int err;

	r->magic = MAPPLE_AKS_REC_MAGIC;
	r->seq = gAKSRec.seq + 1;
	mapple_aks_rec_sum(r, r->sum);
	err = vnode_open(MAPPLE_AKS_FILE, O_CREAT | FWRITE | O_NOFOLLOW, S_IRUSR | S_IWUSR, VNODE_LOOKUP_NOFOLLOW, &vp, ctx);
	if (err == 0) {
		err = vnode_vtype(vp) == VREG ? vn_rdwr(UIO_WRITE, vp, (caddr_t)r, sizeof(*r), (off_t)(r->seq & 1) * sizeof(*r),
		    UIO_SYSSPACE, IO_NODELOCKED | IO_UNIT | IO_SYNC, vfs_context_ucred(ctx), NULL, vfs_context_proc(ctx)) : EFTYPE;
		vnode_close(vp, FWRITE, ctx);
	}
	if (err != 0) {
		IOLog("AKS: writing %s: error %d\n", MAPPLE_AKS_FILE, err);
	}
	return err;
}

static uint32_t
mapple_aks_lock_bits(void)
{
	return (gAKSRec.has ? 0 : MAPPLE_AKS_NO_PASSCODE) | (gAKSLocked ? MAPPLE_AKS_LOCKED : 0) |
	       (gAKSBeenUnlocked ? MAPPLE_AKS_BEEN_UNLOCKED : 0);
}

/* Up to MAPPLE_AKS_SECRET_MAX bytes from the caller's address space. */
static IOReturn
mapple_aks_copy_secret(uint64_t uaddr, uint64_t len, uint8_t *buf, size_t *outlen)
{
	if (len > MAPPLE_AKS_SECRET_MAX || (len > 0 && uaddr == 0)) {
		return kIOReturnBadArgument;
	}
	if (len > 0 && copyin((user_addr_t)uaddr, buf, (vm_size_t)len) != 0) {
		return kIOReturnBadArgument;
	}
	*outlen = (size_t)len;
	return kIOReturnSuccess;
}

/*
 * Check a passcode against the record; a wrong one is counted on disk before
 * the answer goes back, so a reboot does not reset the delays.
 */
static IOReturn
mapple_aks_check_secret(const uint8_t *pw, size_t len, const char *what)
{
	struct mapple_aks_rec r = gAKSRec;
	uint64_t wait, t0 = mach_absolute_time(), ns;
	int ret = mapple_aks_check(&r, pw, len, mapple_aks_now(), &wait);

	absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	if (r.failed != gAKSRec.failed || r.until != gAKSRec.until) {
		mapple_aks_save(&r);
	}
	gAKSRec = r;
	if (gAKSRec.has) {
		IOLog("AKS: %s: %s, %u failed, %llu s to wait (%llu ms)\n", what,
		    ret == MAPPLE_AKS_OK ? "right" : ret == MAPPLE_AKS_WRONG ? "wrong" : "waiting", r.failed, wait, ns / 1000000);
	}
	return ret == MAPPLE_AKS_OK ? kIOReturnSuccess : kIOReturnNotPermitted;
}

/* aks_unlock_device(handle, passcode, length) */
static IOReturn
mapple_aks_unlock(IOExternalMethodArguments *args)
{
	uint8_t pw[MAPPLE_AKS_SECRET_MAX];
	size_t len = 0;
	IOReturn ret;

	args->scalarOutputCount = 0;
	if (args->scalarInputCount < 3) {
		return kIOReturnBadArgument;
	}
	ret = mapple_aks_copy_secret(args->scalarInput[1], args->scalarInput[2], pw, &len);
	if (ret == kIOReturnSuccess) {
		ret = mapple_aks_check_secret(pw, len, "unlock");
	}
	cc_clear(sizeof(pw), pw);
	if (ret == kIOReturnSuccess) {
		gAKSLocked = false;
		gAKSBeenUnlocked = true;
	}
	return ret;
}

static IOReturn mapple_aks_read_in(IOExternalMethodArguments *args, uint8_t *buf, uint32_t *len);
static IOReturn mapple_aks_write_out(IOExternalMethodArguments *args, const uint8_t *buf, uint32_t len);

/* aks_verify_password(handle, {passcode, context}): checks without unlocking. */
static IOReturn
mapple_aks_verify(IOExternalMethodArguments *args)
{
	uint8_t buf[1024];
	const uint8_t *pw;
	uint32_t len = sizeof(buf), pwlen;
	IOReturn ret;

	args->scalarOutputCount = 0;
	ret = mapple_aks_read_in(args, buf, &len);
	if (ret == kIOReturnSuccess) {
		ret = mapple_aks_first_item(buf, len, &pw, &pwlen) ? mapple_aks_check_secret(pw, pwlen, "verify") :
		    kIOReturnBadArgument;
	}
	cc_clear(sizeof(buf), buf);
	return ret;
}

/*
 * aks_change_secret_se(handle, flags, -, old, old length, new, new length): an
 * empty new passcode removes it. With flags bit 2 keybagd asks for a new bag
 * handle, then saves it, makes it the system bag and unloads the old one; the
 * record is already final here, those steps only have to succeed.
 */
static IOReturn
mapple_aks_change_secret(IOExternalMethodArguments *args)
{
	uint8_t old[MAPPLE_AKS_SECRET_MAX], pw[MAPPLE_AKS_SECRET_MAX], salt[MAPPLE_AKS_SALT_LEN];
	struct mapple_aks_rec r = gAKSRec;
	size_t oldlen = 0, len = 0;
	uint64_t wait;
	IOReturn ret;
	int res;

	if (args->scalarInputCount < 7) {
		return kIOReturnBadArgument;
	}
	ret = mapple_aks_copy_secret(args->scalarInput[3], args->scalarInput[4], old, &oldlen);
	if (ret == kIOReturnSuccess) {
		ret = mapple_aks_copy_secret(args->scalarInput[5], args->scalarInput[6], pw, &len);
	}
	if (ret != kIOReturnSuccess) {
		return ret;
	}
	read_random(salt, sizeof(salt));
	res = mapple_aks_change(&r, old, oldlen, pw, len, salt, mapple_aks_now(), &wait);
	cc_clear(sizeof(old), old);
	cc_clear(sizeof(pw), pw);
	if (res == MAPPLE_AKS_OK) {
		if (mapple_aks_save(&r) != 0) {
			return kIOReturnIOError;
		}
		gAKSRec = r;
		gAKSLocked = false;
		gAKSBeenUnlocked = true;
		IOLog("AKS: passcode %s\n", r.has ? "set" : "removed");
	} else {
		if (r.failed != gAKSRec.failed) {
			mapple_aks_save(&r);
		}
		gAKSRec = r;
		IOLog("AKS: change passcode: old one %s, %u failed, %llu s to wait\n",
		    res == MAPPLE_AKS_WRONG ? "wrong" : "not tried", r.failed, wait);
		return kIOReturnNotPermitted;
	}
	if (args->scalarOutputCount >= 1) {
		args->scalarOutput[0] = (args->scalarInput[1] & 4) ? args->scalarInput[0] + 1 : args->scalarInput[0];
		args->scalarOutputCount = 1;
	}
	return kIOReturnSuccess;
}

class AppleKeyStore : public IOService {
	OSDeclareDefaultStructors(AppleKeyStore);

public:
	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(AppleKeyStore, IOService);

bool
AppleKeyStore::start(IOService *provider)
{
	if (!IOService::start(provider)) {
		return false;
	}
	gAKSLock = IOLockAlloc();
	if (gAKSLock == NULL) {
		return false;
	}
	setProperty(kIOUserClientClassKey, "AppleKeyStoreUserClient");
	registerService();
	IOLog("AKS: AppleKeyStore registered\n");
	return true;
}

class AppleKeyStoreUserClient : public IOUserClient {
	OSDeclareDefaultStructors(AppleKeyStoreUserClient);

public:
	IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target,
	    void *reference) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(AppleKeyStoreUserClient, IOUserClient);

IOReturn
AppleKeyStoreUserClient::clientClose(void)
{
	terminate();
	return kIOReturnSuccess;
}

/* A structure past the inline limit comes as a descriptor over the caller's memory: wired only while prepared. */
static IOByteCount
mapple_aks_md_copy(IOMemoryDescriptor *md, void *buf, IOByteCount len, bool write)
{
	IOByteCount n = 0;

	if (md->prepare() == kIOReturnSuccess) {
		n = write ? md->writeBytes(0, buf, len) : md->readBytes(0, buf, len);
		md->complete();
	}
	return n;
}

/* Copy len bytes to the caller's output structure (inband or descriptor). */
static IOReturn
mapple_aks_struct_out(IOExternalMethodArguments *args, const void *buf, uint32_t len)
{
	if (args->structureOutputDescriptor != NULL) {
		if (len > args->structureOutputDescriptor->getLength()) {
			return kIOReturnNoSpace;
		}
		if (mapple_aks_md_copy(args->structureOutputDescriptor, (void *)(uintptr_t)buf, len, true) != len) {
			return kIOReturnNoSpace;
		}
		args->structureOutputDescriptorSize = len;
		return kIOReturnSuccess;
	}
	if (args->structureOutput == NULL || len > args->structureOutputSize) {
		return kIOReturnNoSpace;
	}
	memcpy(args->structureOutput, buf, len);
	args->structureOutputSize = len;
	return kIOReturnSuccess;
}

/* Append SEQUENCE { UTF8String key, INTEGER value } (short length forms). */
static uint32_t
mapple_aks_der_pair(uint8_t *p, const char *key, uint32_t value)
{
	uint32_t klen = (uint32_t)strlen(key);
	uint32_t vlen = 1, n = 0;

	while (vlen < 4 && (value >> (8 * vlen - 1)) != 0) {
		vlen++;
	}
	if (vlen == 4 && (value >> 31) != 0) {
		vlen = 5;       /* leading zero: INTEGER is signed */
	}
	p[n++] = 0x30;
	p[n++] = (uint8_t)(2 + klen + 2 + vlen);
	p[n++] = 0x0c;
	p[n++] = (uint8_t)klen;
	memcpy(p + n, key, klen);
	n += klen;
	p[n++] = 0x02;
	p[n++] = (uint8_t)vlen;
	for (uint32_t i = vlen; i > 0; i--) {
		p[n++] = i > 4 ? 0 : (uint8_t)(value >> (8 * (i - 1)));
	}
	return n;
}

/*
 * Device state (selector 17): a DER SET of key/INTEGER pairs that libaks
 * decode_extended_state reads: ss state, sls lock state (MKBGetDeviceLockState:
 * 0 = unlocked), sb backoff, sfa failed attempts, sgs generation, sas
 * assertions, sgpe grace period, srcd recovery countdown.
 */
static IOReturn
mapple_aks_device_state(IOExternalMethodArguments *args)
{
	uint64_t now = mapple_aks_now();
	const struct {
		const char *key;
		uint32_t value;
	} fields[] = {
		{ "ss", mapple_aks_lock_bits() },
		{ "sls", (uint32_t)(!gAKSRec.has ? MAPPLE_MKB_DISABLED : gAKSLocked ? MAPPLE_MKB_LOCKED : MAPPLE_MKB_UNLOCKED) },
		{ "sb", gAKSRec.until > now ? (uint32_t)MIN(gAKSRec.until - now, mapple_aks_delay(gAKSRec.failed)) : 0 },
		{ "sfa", gAKSRec.failed }, { "sgs", 0 }, { "sas", 0 }, { "sgpe", 0 }, { "srcd", 0 },
	};
	uint8_t buf[160];
	uint32_t n = 2;
	for (unsigned int i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
		n += mapple_aks_der_pair(buf + n, fields[i].key, fields[i].value);
	}
	buf[0] = 0x31;
	buf[1] = (uint8_t)(n - 2);
	return mapple_aks_struct_out(args, buf, n);
}

static uint32_t
mapple_aks_struct_out_size(IOExternalMethodArguments *args)
{
	return args->structureOutputDescriptor != NULL ?
	       (uint32_t)args->structureOutputDescriptor->getLength() : args->structureOutputSize;
}

/*
 * Key wrapping (aks_wrap_key / aks_unwrap_key), from ios-sapphire 0160/0293: securityd wraps every keychain item's key with
 * its class key, and without these every keychain write failed (ks_crypt e00002c7 "failed to
 * 'oe' item"), so Wi-Fi passwords and the rest were never kept, and wifid waited minutes on
 * them before Settings' Wi-Fi page could open. There is no Secure Enclave here and the device
 * has no passcode, so this protects nothing: the wrapped form is a tag and the key mixed with a
 * fixed pad, the same at every boot so that saved items can be read back. Wrapped keys are 8
 * bytes longer than their key, as AES key wrap's (RFC 3394) are, the size libaks expects.
 */
#define MAPPLE_AKS_WRAP_TAG      "MAPPLEK1"
#define MAPPLE_AKS_WRAP_TAG_LEN  8
#define MAPPLE_AKS_KEY_MAX       64

static const uint8_t mapple_aks_pad[32] = {
	0x5a, 0x21, 0x9c, 0x47, 0xe3, 0x0b, 0x76, 0xd8, 0x3f, 0xa1, 0x64, 0x12, 0xc9, 0x8e, 0x2d, 0xf0,
	0x07, 0xb5, 0x4a, 0x93, 0x6e, 0xdc, 0x18, 0x81, 0xf4, 0x3b, 0xa7, 0x55, 0x0e, 0xc2, 0x69, 0x9d,
};

static IOReturn
mapple_aks_read_in(IOExternalMethodArguments *args, uint8_t *buf, uint32_t *len)
{
	uint32_t n;

	if (args->structureInputDescriptor != NULL) {
		n = (uint32_t)args->structureInputDescriptor->getLength();
		if (n > *len || mapple_aks_md_copy(args->structureInputDescriptor, buf, n, false) != n) {
			return kIOReturnBadArgument;
		}
	} else {
		n = args->structureInputSize;
		if (n > *len || (n > 0 && args->structureInput == NULL)) {
			return kIOReturnBadArgument;
		}
		memcpy(buf, args->structureInput, n);
	}
	*len = n;
	return kIOReturnSuccess;
}

static IOReturn
mapple_aks_write_out(IOExternalMethodArguments *args, const uint8_t *buf, uint32_t len)
{
	if (args->structureOutputDescriptor != NULL) {
		if (args->structureOutputDescriptor->getLength() < len ||
		    mapple_aks_md_copy(args->structureOutputDescriptor, (void *)(uintptr_t)buf, len, true) != len) {
			return kIOReturnNoSpace;
		}
		args->structureOutputDescriptorSize = len;
		return kIOReturnSuccess;
	}
	if (args->structureOutput == NULL || args->structureOutputSize < len) {
		return kIOReturnNoSpace;
	}
	memcpy(args->structureOutput, buf, len);
	args->structureOutputSize = len;
	return kIOReturnSuccess;
}

static IOReturn
mapple_aks_wrap(IOExternalMethodArguments *args, bool wrap)
{
	uint8_t in[MAPPLE_AKS_KEY_MAX + MAPPLE_AKS_WRAP_TAG_LEN], out[sizeof(in)];
	uint32_t len = sizeof(in), n;
	IOReturn ret = mapple_aks_read_in(args, in, &len);

	if (ret != kIOReturnSuccess) {
		return ret;
	}
	if (wrap) {
		if (len == 0 || len > MAPPLE_AKS_KEY_MAX) {
			return kIOReturnBadArgument;
		}
		memcpy(out, MAPPLE_AKS_WRAP_TAG, MAPPLE_AKS_WRAP_TAG_LEN);
		for (uint32_t i = 0; i < len; i++) {
			out[MAPPLE_AKS_WRAP_TAG_LEN + i] = in[i] ^ mapple_aks_pad[i % sizeof(mapple_aks_pad)];
		}
		n = len + MAPPLE_AKS_WRAP_TAG_LEN;
	} else {
		if (len <= MAPPLE_AKS_WRAP_TAG_LEN || memcmp(in, MAPPLE_AKS_WRAP_TAG, MAPPLE_AKS_WRAP_TAG_LEN) != 0) {
			return kIOReturnNotPermitted;   /* not ours: a key wrapped elsewhere cannot be read */
		}
		n = len - MAPPLE_AKS_WRAP_TAG_LEN;
		for (uint32_t i = 0; i < n; i++) {
			out[i] = in[MAPPLE_AKS_WRAP_TAG_LEN + i] ^ mapple_aks_pad[i % sizeof(mapple_aks_pad)];
		}
	}
	ret = mapple_aks_write_out(args, out, n);
	bzero(in, sizeof(in));
	bzero(out, sizeof(out));
	if (ret == kIOReturnSuccess) {
		/*
		 * wrap: the key's class back (the one asked for); unwrap: nothing. libaks passes (keybag handle,
		 * key class): the class is the second scalar (ios-sapphire 0293: handing back the handle, 0,
		 * made securityd file every item as class 0 and delete it as corrupt on the first read).
		 */
		if (wrap && args->scalarOutputCount >= 1) {
			args->scalarOutput[0] = args->scalarInputCount > 1 ? args->scalarInput[1] :
			    (args->scalarInputCount > 0 ? args->scalarInput[0] : 0);
			args->scalarOutputCount = 1;
		} else {
			args->scalarOutputCount = 0;
		}
	}
	return ret;
}

IOReturn
AppleKeyStoreUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    __unused IOExternalMethodDispatch *dispatch, __unused OSObject *target,
    __unused void *reference)
{
	IOReturn ret = kIOReturnSuccess;
	uint32_t in_scalars = args->scalarInputCount;
	uint32_t in_struct = args->structureInputDescriptor != NULL ?
	    (uint32_t)args->structureInputDescriptor->getLength() : args->structureInputSize;
	uint32_t out_scalars = args->scalarOutputCount;
	uint32_t out_struct = mapple_aks_struct_out_size(args);
	uint64_t arg0 = in_scalars > 0 ? args->scalarInput[0] : 0;

	IOLockLock(gAKSLock);
	mapple_aks_load();
	switch (selector) {
	case kAKSOpen:
	case kAKSUnloadBag:
	case kAKSSetSystem:
	case kAKSChangeSecretEpilogue:
	case kAKSUnlockBag:
	case kAKSLockBag:
	case kAKSAssertHold:
	case kAKSAssertDrop:
	case kAKSAssertConsume:
	case kAKSUnloadSessionBags:
	case kAKSNotifyStateSet:
		args->scalarOutputCount = 0;
		break;
	case kAKSLockDevice:
	case kAKSLockDeviceNow:
		args->scalarOutputCount = 0;
		gAKSLocked = gAKSRec.has != 0;
		break;
	case kAKSUnlockDevice:
		ret = mapple_aks_unlock(args);
		break;
	case kAKSVerifyPassword:
		ret = mapple_aks_verify(args);
		break;
	case kAKSChangeSecret:
		ret = mapple_aks_change_secret(args);
		break;
	case kAKSSaveBag:
		/* keybagd keeps this in systembag.kb; the record lives in MAPPLE_AKS_FILE. */
		args->scalarOutputCount = 0;
		ret = mapple_aks_write_out(args, (const uint8_t *)"MAPPLEKB", 8);
		break;
	case kAKSGetSystem:
		if (out_scalars < 1) {
			ret = kIOReturnBadArgument;
			break;
		}
		args->scalarOutput[0] = MAPPLE_AKS_SYSTEM_HANDLE;
		args->scalarOutputCount = 1;
		break;
	case kAKSGetLockState:
		if (out_scalars < 1) {
			ret = kIOReturnBadArgument;
			break;
		}
		args->scalarOutput[0] = mapple_aks_lock_bits();
		args->scalarOutputCount = 1;
		break;
	case kAKSNotifyStateIsSet:
		/* Nothing was ever set: answer "not set". */
		if (out_scalars < 1) {
			ret = kIOReturnBadArgument;
			break;
		}
		args->scalarOutput[0] = 0;
		args->scalarOutputCount = 1;
		break;
	case kAKSGetDeviceState:
	case kAKSGetExtDeviceState:
		args->scalarOutputCount = 0;
		ret = mapple_aks_device_state(args);
		break;
	case kAKSWrapKey:
		ret = mapple_aks_wrap(args, true);
		break;
	case kAKSUnwrapKey:
		ret = mapple_aks_wrap(args, false);
		break;
	default:
		ret = kIOReturnUnsupported;
		break;
	}
	IOLockUnlock(gAKSLock);

	char name[32];
	proc_selfname(name, sizeof(name));
	IOLog("AKS: %s sel %u arg0 %llu in %u/%u out %u/%u -> 0x%x\n", name, selector,
	    arg0, in_scalars, in_struct, out_scalars, out_struct, ret);
	return ret;
}
#endif /* MT6781 */
