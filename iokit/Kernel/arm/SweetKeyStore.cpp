/*
 * Built-in AppleKeyStore for SWEET: a device without a passcode whose
 * system keybag is always loaded and unlocked.
 *
 * Apple's AppleKeyStore kext (closed, SEP-backed) owns the keybags behind
 * data protection. libaks (AppleKeyStore.framework, statically in keybagd)
 * opens IOServiceMatching("AppleKeyStore") with type 0, calls selector 0,
 * then one selector per aks_* function (iOS 16.7 numbering below). keybagd
 * --init asks for the system bag (14) and only loads systembag.kb when the
 * answer is kIOReturnNotFound; answering with a handle is the path a
 * userspace reboot takes, so no bag file is needed.
 *
 * Every call is logged (selector, caller, argument shapes, result) so the
 * next selectors the system needs show up on the console; unknown ones fail
 * with kIOReturnUnsupported.
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <libkern/c++/OSString.h>
#include <sys/proc.h>

/* libaks selectors (AppleKeyStore.framework, iOS 16.7). */
enum {
	kAKSOpen             = 0,
	kAKSCreateBag        = 2,
	kAKSSaveBag          = 3,
	kAKSUnloadBag        = 4,
	kAKSSetSystem        = 5,
	kAKSLoadBag          = 6,
	kAKSGetLockState     = 7,
	kAKSUnlockDevice     = 9,
	kAKSWrapKey          = 10,
	kAKSUnwrapKey        = 11,
	kAKSUnlockBag        = 12,
	kAKSLockBag          = 13,
	kAKSGetSystem        = 14,
	kAKSGetDeviceState   = 17,
	kAKSGetExtDeviceState = 35,     /* aks_get_extended_device_state */
	kAKSAssertHold       = 26,
	kAKSAssertDrop       = 27,
	kAKSUnloadSessionBags = 55,
	kAKSNotifyStateSet   = 102,
	kAKSNotifyStateIsSet = 103,
	kAKSAssertConsume    = 101,     /* aks_assert_consume */
};

/* The system (device) bag handle. */
#define SWEET_AKS_SYSTEM_HANDLE 0

/*
 * aks lock state bits (aks_get_lock_state): bit 2 is "unlocked since boot"
 * (MobileKeyBag MKBDeviceUnlockedSinceBoot); clear lock bits = unlocked.
 */
#define SWEET_AKS_LOCK_STATE    0x4

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
	setProperty(kIOUserClientClassKey, "AppleKeyStoreUserClient");
	registerService();
	IOLog("SWEET AKS: AppleKeyStore registered (no passcode, unlocked)\n");
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

/* Copy len bytes to the caller's output structure (inband or descriptor). */
static IOReturn
sweet_aks_struct_out(IOExternalMethodArguments *args, const void *buf, uint32_t len)
{
	if (args->structureOutputDescriptor != NULL) {
		if (len > args->structureOutputDescriptor->getLength()) {
			return kIOReturnNoSpace;
		}
		args->structureOutputDescriptor->writeBytes(0, buf, len);
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

/* Append SEQUENCE { UTF8String key, INTEGER value } (short forms only). */
static uint32_t
sweet_aks_der_pair(uint8_t *p, const char *key, uint8_t value)
{
	uint32_t klen = (uint32_t)strlen(key);
	uint32_t n = 0;
	p[n++] = 0x30;
	p[n++] = (uint8_t)(2 + klen + 3);
	p[n++] = 0x0c;
	p[n++] = (uint8_t)klen;
	memcpy(p + n, key, klen);
	n += klen;
	p[n++] = 0x02;
	p[n++] = 0x01;
	p[n++] = value;
	return n;
}

/*
 * Device state (selector 17): a DER SET of key/INTEGER pairs that libaks
 * decode_extended_state reads: ss state, sls lock state (MKBGetDeviceLockState:
 * 0 = unlocked), sb backoff, sfa failed attempts, sgs generation, sas
 * assertions, sgpe grace period, srcd recovery countdown.
 */
static IOReturn
sweet_aks_device_state(IOExternalMethodArguments *args)
{
	static const struct {
		const char *key;
		uint8_t value;
	} fields[] = {
		{ "ss", SWEET_AKS_LOCK_STATE }, { "sls", 0 }, { "sb", 0 }, { "sfa", 0 },
		{ "sgs", 0 }, { "sas", 0 }, { "sgpe", 0 }, { "srcd", 0 },
	};
	uint8_t buf[128];
	uint32_t n = 2;
	for (unsigned int i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
		n += sweet_aks_der_pair(buf + n, fields[i].key, fields[i].value);
	}
	buf[0] = 0x31;
	buf[1] = (uint8_t)(n - 2);
	return sweet_aks_struct_out(args, buf, n);
}

static uint32_t
sweet_aks_struct_out_size(IOExternalMethodArguments *args)
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
#define SWEET_AKS_WRAP_TAG      "SWEETAK1"
#define SWEET_AKS_WRAP_TAG_LEN  8
#define SWEET_AKS_KEY_MAX       64

static const uint8_t sweet_aks_pad[32] = {
	0x5a, 0x21, 0x9c, 0x47, 0xe3, 0x0b, 0x76, 0xd8, 0x3f, 0xa1, 0x64, 0x12, 0xc9, 0x8e, 0x2d, 0xf0,
	0x07, 0xb5, 0x4a, 0x93, 0x6e, 0xdc, 0x18, 0x81, 0xf4, 0x3b, 0xa7, 0x55, 0x0e, 0xc2, 0x69, 0x9d,
};

static IOReturn
sweet_aks_read_in(IOExternalMethodArguments *args, uint8_t *buf, uint32_t *len)
{
	uint32_t n;

	if (args->structureInputDescriptor != NULL) {
		n = (uint32_t)args->structureInputDescriptor->getLength();
		if (n > *len || args->structureInputDescriptor->readBytes(0, buf, n) != n) {
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
sweet_aks_write_out(IOExternalMethodArguments *args, const uint8_t *buf, uint32_t len)
{
	if (args->structureOutputDescriptor != NULL) {
		if (args->structureOutputDescriptor->getLength() < len ||
		    args->structureOutputDescriptor->writeBytes(0, buf, len) != len) {
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
sweet_aks_wrap(IOExternalMethodArguments *args, bool wrap)
{
	uint8_t in[SWEET_AKS_KEY_MAX + SWEET_AKS_WRAP_TAG_LEN], out[sizeof(in)];
	uint32_t len = sizeof(in), n;
	IOReturn ret = sweet_aks_read_in(args, in, &len);

	if (ret != kIOReturnSuccess) {
		return ret;
	}
	if (wrap) {
		if (len == 0 || len > SWEET_AKS_KEY_MAX) {
			return kIOReturnBadArgument;
		}
		memcpy(out, SWEET_AKS_WRAP_TAG, SWEET_AKS_WRAP_TAG_LEN);
		for (uint32_t i = 0; i < len; i++) {
			out[SWEET_AKS_WRAP_TAG_LEN + i] = in[i] ^ sweet_aks_pad[i % sizeof(sweet_aks_pad)];
		}
		n = len + SWEET_AKS_WRAP_TAG_LEN;
	} else {
		if (len <= SWEET_AKS_WRAP_TAG_LEN || memcmp(in, SWEET_AKS_WRAP_TAG, SWEET_AKS_WRAP_TAG_LEN) != 0) {
			return kIOReturnNotPermitted;   /* not ours: a key wrapped elsewhere cannot be read */
		}
		n = len - SWEET_AKS_WRAP_TAG_LEN;
		for (uint32_t i = 0; i < n; i++) {
			out[i] = in[SWEET_AKS_WRAP_TAG_LEN + i] ^ sweet_aks_pad[i % sizeof(sweet_aks_pad)];
		}
	}
	ret = sweet_aks_write_out(args, out, n);
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
	uint32_t out_struct = sweet_aks_struct_out_size(args);
	uint64_t arg0 = in_scalars > 0 ? args->scalarInput[0] : 0;

	switch (selector) {
	case kAKSOpen:
	case kAKSUnloadBag:
	case kAKSSetSystem:
	case kAKSUnlockDevice:
	case kAKSUnlockBag:
	case kAKSLockBag:
	case kAKSAssertHold:
	case kAKSAssertDrop:
	case kAKSAssertConsume:
	case kAKSUnloadSessionBags:
	case kAKSNotifyStateSet:
		args->scalarOutputCount = 0;
		break;
	case kAKSGetSystem:
		if (out_scalars < 1) {
			ret = kIOReturnBadArgument;
			break;
		}
		args->scalarOutput[0] = SWEET_AKS_SYSTEM_HANDLE;
		args->scalarOutputCount = 1;
		break;
	case kAKSGetLockState:
		if (out_scalars < 1) {
			ret = kIOReturnBadArgument;
			break;
		}
		args->scalarOutput[0] = SWEET_AKS_LOCK_STATE;
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
		ret = sweet_aks_device_state(args);
		break;
	case kAKSWrapKey:
		ret = sweet_aks_wrap(args, true);
		break;
	case kAKSUnwrapKey:
		ret = sweet_aks_wrap(args, false);
		break;
	default:
		ret = kIOReturnUnsupported;
		break;
	}

	char name[32];
	proc_selfname(name, sizeof(name));
	IOLog("SWEET AKS: %s sel %u arg0 %llu in %u/%u out %u/%u -> 0x%x\n", name, selector,
	    arg0, in_scalars, in_struct, out_scalars, out_struct, ret);
	return ret;
}
#endif /* SWEET */
