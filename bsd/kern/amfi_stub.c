/*
 * Built-in AMFI interface for MT6781.
 *
 * On Apple platforms AppleMobileFileIntegrity registers this interface and
 * provides entitlements through the closed CoreEntitlements library. MT6781
 * has neither, and code signing reaches it on every exec (ubc_cs_blob_add).
 *
 * Entitlements: when a code signature is added, its XML entitlements blob
 * (already checked against the code directory) is parsed into a
 * MappleEntitlements object (iokit/Kernel/arm/MappleEntitlements.cpp), set as
 * the blob's csb_entitlements, and every query is answered from it. DER-only
 * signatures get none. Code signature and page hash validation stay with the
 * kernel. Trust caches remain unavailable (no Image4), so the TrustCache
 * interface is left at version 0.
 *
 * Platform binaries: AMFI marks binaries whose cdhash is in a trust cache
 * as platform binaries from its vnode_check_signature MAC hook. Without
 * trust caches, the MT6781 policy below marks every binary on the root
 * filesystem instead (the iOS system binaries are ad-hoc signed and are
 * platform binaries only through the static trust cache).
 *
 * Interfaces no kernel path uses on MT6781 (they serve other Apple kexts)
 * panic with a clear message instead of being NULL.
 */
#if defined(ARM64_BOARD_CONFIG_MT6781)
#include <kern/debug.h>
#include <kern/startup.h>
#include <libkern/amfi/amfi.h>
#include <libkern/OSAtomic.h>
#include <mach/kern_return.h>
#include <pexpert/pexpert.h>
#include <sys/codesign.h>
#include <sys/kauth.h>
#include <sys/mount.h>
#include <sys/priv.h>
#include <sys/proc.h>
#include <sys/systm.h>
#include <sys/ubc_internal.h>
#include <sys/vnode.h>
#include <security/mac_policy.h>

/* iokit/Kernel/arm/MappleEntitlements.cpp */
void *mapple_entitlements_create(const CS_GenericBlob *blob, size_t blob_len);
void *mapple_entitlements_copy_dict(const void *ent);
bool mapple_entitlements_copy_xml(const void *ent, CS_GenericBlob **out);
bool mapple_entitlements_bool(const void *ent, const char *name);
bool mapple_entitlements_string(const void *ent, const char *name, const char *value);
void *mapple_entitlements_copy_object(const void *ent, const char *name);
unsigned int mapple_entitlements_count(const void *ent);
extern void osobject_release(void *object);

/* boot-arg amfi_stub_debug=1: log the entitlements attached to each signature. */
static TUNABLE(uint32_t, amfi_stub_debug, "amfi_stub_debug", 0);

/* The entitlements object of a process's main binary, or NULL. */
static void *
mapple_proc_entitlements(proc_t proc)
{
	struct cs_blob *blob = csproc_get_blob(proc);
	return blob != NULL ? csblob_os_entitlements_get(blob) : NULL;
}

static void
amfi_stub_entitlements_invalidate(__unused void *os_entitlements)
{
	/* Queries already require a valid signature (csblob_get_entitlements). */
}

static void *
amfi_stub_entitlements_as_dict(void *os_entitlements)
{
	return mapple_entitlements_copy_dict(os_entitlements);
}

static CEError_t
amfi_stub_entitlements_query(__unused void *os_entitlements,
    __unused uint8_t cdhash[CS_CDHASH_LEN], __unused CEQuery_t query,
    __unused size_t query_length)
{
	panic("AMFI: CoreEntitlements queries are not implemented");
}

static bool
amfi_stub_entitlements_get_transmuted(__unused void *os_entitlements,
    __unused const CS_GenericBlob **blob)
{
	return false;
}

static bool
amfi_stub_entitlements_get_xml(void *os_entitlements, CS_GenericBlob **blob)
{
	return mapple_entitlements_copy_xml(os_entitlements, blob);
}

static bool
amfi_stub_get_legacy_profile_exemptions(__unused const uint8_t **profile,
    __unused size_t *profile_length)
{
	return false;
}

static bool
amfi_stub_get_udid(__unused const uint8_t **udid, __unused size_t *udid_length)
{
	return false;
}

static void *
amfi_stub_query_context_to_object(__unused CEQueryContext_t ctx)
{
	panic("AMFI: CoreEntitlements query contexts are not implemented");
}

static kern_return_t
amfi_stub_adjust_with_monitor(__unused void *os_entitlements,
    __unused const CEQueryContext_t ce_ctx, __unused const void *monitor_sig_obj,
    __unused const char *identity, __unused const uint32_t code_signing_flags)
{
	/* MT6781 has no code signing monitor. */
	return KERN_NOT_SUPPORTED;
}

/*
 * Attach the parsed XML entitlements to a code signature being added. This
 * must happen in the vnode_check_signature MAC hook, like AMFI does: later,
 * converting a 4K-page signature to multilevel hashes drops the XML blob.
 * Nothing is granted when the blob is missing, unreadable or does not match
 * its code directory hash.
 */
static void
amfi_stub_attach_entitlements(struct cs_blob *code_signing_blob)
{
	void *xml = NULL;
	size_t xml_len = 0;

	if (csblob_get_entitlements(code_signing_blob, &xml, &xml_len) != 0) {
		printf("AMFI: %s: entitlements do not match the code directory, granting none\n",
		    csblob_get_identity(code_signing_blob));
		return;
	}
	if (xml == NULL) {
		return;                 /* no XML entitlements */
	}
	void *ent = mapple_entitlements_create(xml, xml_len);
	if (ent == NULL) {
		printf("AMFI: %s: unreadable entitlements (%zu bytes), granting none\n",
		    csblob_get_identity(code_signing_blob), xml_len);
		return;
	}
	if (amfi_stub_debug) {
		/* The test keys only exist in tools/mkinit.py test signatures. */
		printf("AMFI: %s: %u entitlements (test: bool %d string %d array %d)\n",
		    csblob_get_identity(code_signing_blob), mapple_entitlements_count(ent),
		    mapple_entitlements_bool(ent, "com.apple.private.mapple.test-bool"),
		    mapple_entitlements_string(ent, "com.apple.private.mapple.test-string", "hello"),
		    mapple_entitlements_string(ent, "com.apple.private.mapple.test-array", "two"));
	}
	csblob_os_entitlements_set(code_signing_blob, ent);    /* takes a reference */
	osobject_release(ent);
}

static kern_return_t
amfi_stub_adjust_without_monitor(__unused void *os_entitlements,
    __unused struct cs_blob *code_signing_blob)
{
	/* Entitlements were attached in amfi_stub_vnode_check_signature. */
	return KERN_SUCCESS;
}

static kern_return_t
amfi_stub_query_boolean(const void *os_entitlements, const char *entitlement_name)
{
	return mapple_entitlements_bool(os_entitlements, entitlement_name) ? KERN_SUCCESS : KERN_DENIED;
}

static kern_return_t
amfi_stub_query_boolean_with_proc(const proc_t proc, const char *entitlement_name)
{
	return amfi_stub_query_boolean(mapple_proc_entitlements(proc), entitlement_name);
}

static kern_return_t
amfi_stub_query_string(const void *os_entitlements,
    const char *entitlement_name, const char *entitlement_value)
{
	return mapple_entitlements_string(os_entitlements, entitlement_name, entitlement_value) ?
	       KERN_SUCCESS : KERN_DENIED;
}

static kern_return_t
amfi_stub_query_string_with_proc(const proc_t proc,
    const char *entitlement_name, const char *entitlement_value)
{
	return amfi_stub_query_string(mapple_proc_entitlements(proc), entitlement_name,
	           entitlement_value);
}

static kern_return_t
amfi_stub_copy_object(const void *os_entitlements,
    const char *entitlement_name, void **entitlement_object)
{
	void *obj = mapple_entitlements_copy_object(os_entitlements, entitlement_name);
	if (entitlement_object) {
		*entitlement_object = obj;
	} else if (obj) {
		osobject_release(obj);
	}
	return obj ? KERN_SUCCESS : KERN_NOT_FOUND;
}

static kern_return_t
amfi_stub_copy_object_with_proc(const proc_t proc,
    const char *entitlement_name, void **entitlement_object)
{
	return amfi_stub_copy_object(mapple_proc_entitlements(proc), entitlement_name,
	           entitlement_object);
}

static const amfi_t amfi_stub = {
	.OSEntitlements_invalidate = amfi_stub_entitlements_invalidate,
	.OSEntitlements_asdict = amfi_stub_entitlements_as_dict,
	.OSEntitlements_query = amfi_stub_entitlements_query,
	.OSEntitlements_get_transmuted = amfi_stub_entitlements_get_transmuted,
	.OSEntitlements_get_xml = amfi_stub_entitlements_get_xml,
	/* CoreEntitlements is only used by the PMAP_CS code signing monitor. */
	.get_legacy_profile_exemptions = amfi_stub_get_legacy_profile_exemptions,
	.get_udid = amfi_stub_get_udid,
	.query_context_to_object = amfi_stub_query_context_to_object,
	/* .TrustCache stays version 0: trust caches are unavailable without Image4. */
	.OSEntitlements = {
		.version = OSENTITLEMENTS_INTERFACE_VERSION,
		.adjustContextWithMonitor = amfi_stub_adjust_with_monitor,
		.adjustContextWithoutMonitor = amfi_stub_adjust_without_monitor,
		.queryEntitlementBoolean = amfi_stub_query_boolean,
		.queryEntitlementBooleanWithProc = amfi_stub_query_boolean_with_proc,
		.queryEntitlementString = amfi_stub_query_string,
		.queryEntitlementStringWithProc = amfi_stub_query_string_with_proc,
		.copyEntitlementAsOSObject = amfi_stub_copy_object,
		.copyEntitlementAsOSObjectWithProc = amfi_stub_copy_object_with_proc,
	},
};

__startup_func
static void
amfi_stub_register(void)
{
	amfi_interface_register(&amfi_stub);
	printf("AMFI: built-in interface registered (XML entitlements)\n");
}
STARTUP(EARLY_BOOT, STARTUP_RANK_MIDDLE, amfi_stub_register);

/*
 * The signature reached this hook validated, so mark it CS_SIGNED like AMFI
 * does: exec copies the flag to the process, and csproc_get_blob() (csops
 * status, cdhash, platform and entitlement queries by process) ignores
 * processes without it. Every signed binary on the root filesystem is a
 * platform binary. New signatures (ubc_cs_blob_add passes its writable copy;
 * revalidation passes the read-only blob itself) get their entitlements here.
 */
static int
amfi_stub_vnode_check_signature(struct vnode *vp, __unused struct label *label,
    __unused cpu_type_t cpu_type, struct cs_blob *cs_blob,
    unsigned int *cs_flags, __unused unsigned int *signer_type,
    __unused int flags, __unused unsigned int platform,
    __unused char **fatal_failure_desc, __unused size_t *fatal_failure_desc_len)
{
	mount_t mp = vnode_mount(vp);
	*cs_flags |= CS_SIGNED;
	if (mp != NULL && (vfs_flags(mp) & MNT_ROOTFS)) {
		*cs_flags |= CS_PLATFORM_BINARY;
	}
	if (cs_blob != cs_blob->csb_ro_addr && cs_blob->csb_entitlements == NULL) {
		amfi_stub_attach_entitlements(cs_blob);
	}
	return 0;
}

/*
 * Policy "Sandbox": libsystem_sandbox calls __mac_syscall("Sandbox", op,
 * arg). Without a policy of that name every call fails with ENOPOLICY and
 * launchd reports mach lookups as "error = 159: Sandbox restriction"
 * (backboardd cannot reach mobilegestalt or iohideventsystem; tccd:
 * "profile not found"). MT6781 runs without a sandbox: operations succeed
 * (checks allow, profile init is accepted). An extension (op 5) is a token
 * naming its class and path, and consuming one (op 6) gives a handle: tccd
 * grants Photos access to assetsd through one (boot-arg mp_sbx_ext=0: both
 * fail with ENOENT, as before). A container path (op 4) is the
 * home of the process's user: PlugInKit aborts an extension whose container
 * path is missing ("plugin has invalid sandbox or container"); boot-arg
 * mp_sbx_container=0 makes it fail with ENOENT again. The first use of each
 * operation number is logged.
 */
static uint64_t mapple_sandbox_seen;

/* op 4's argument (libsystem_sandbox): pid (0: the caller's), audit token pointer, path buffer, its size */
struct mapple_sandbox_path_args {
	uint64_t pid, token, buf, size, unused;
};

static int
mapple_sandbox_container_path(struct proc *p, user_addr_t arg)
{
	struct mapple_sandbox_path_args a;
	uint32_t token[8];
	int err = copyin(arg, &a, sizeof(a));
	if (err) {
		return err;
	}
	int pid = (int)a.pid;
	if (pid == 0 && a.token != 0) {
		if ((err = copyin((user_addr_t)a.token, token, sizeof(token)))) {
			return err;
		}
		pid = (int)token[5];
	}
	proc_t t = pid == 0 || pid == proc_pid(p) ? p : proc_find(pid);
	if (t == NULL) {
		char name[32];

		proc_name(proc_pid(p), name, sizeof(name));
		printf("Sandbox: %s op 4: no process %d (ESRCH)\n", name, pid);
		return ESRCH;
	}
	kauth_cred_t cred = kauth_cred_proc_ref(t);
	uid_t uid = kauth_cred_getuid(cred);
	kauth_cred_unref(&cred);
	const char *home = uid == 501 ? "/private/var/mobile" : "/private/var/root";
	if (t != p) {
		proc_rele(t);
	}
	size_t len = strlen(home) + 1;
	if (a.buf == 0 || a.size < len) {
		return ERANGE;
	}
	return copyout(home, (user_addr_t)a.buf, len);
}

/* op 5's argument (libsystem_sandbox _sandbox_extension_issue): class, a word, path or name, flags, the 0x800-byte token
 * buffer, pid, a word; op 6's: token, its length + 1, where the handle goes */
struct mapple_sandbox_issue_args {
	uint64_t class_name, word, path, flags, token, pid, word2;
};
struct mapple_sandbox_consume_args {
	uint64_t token, length, handle;
};
#define MAPPLE_SANDBOX_TOKEN 0x800

static int
mapple_sandbox_issue(user_addr_t arg)
{
	struct mapple_sandbox_issue_args a;
	char cls[128] = "", path[MAXPATHLEN] = "", token[MAPPLE_SANDBOX_TOKEN];
	size_t len;
	int err = copyin(arg, &a, sizeof(a));
	if (err) {
		return err;
	}
	if (a.class_name) {
		(void)copyinstr((user_addr_t)a.class_name, cls, sizeof(cls), &len);
	}
	if (a.path) {
		(void)copyinstr((user_addr_t)a.path, path, sizeof(path), &len);
	}
	len = (size_t)snprintf(token, sizeof(token), "mapple;%s;%s", cls, path);
	if (a.token == 0) {
		return EINVAL;
	}
	return copyout(token, (user_addr_t)a.token, MIN(len + 1, sizeof(token)));
}

static int
mapple_sandbox_consume(user_addr_t arg)
{
	static int64_t next_handle;
	struct mapple_sandbox_consume_args a;
	int err = copyin(arg, &a, sizeof(a));
	if (err) {
		return err;
	}
	int64_t h = OSIncrementAtomic64(&next_handle) + 1;
	return a.handle ? copyout(&h, (user_addr_t)a.handle, sizeof(h)) : 0;
}

static int
mapple_sandbox_syscall(struct proc *p, int call, user_addr_t arg)
{
	static int container = -1, ext = -1;
	if (container < 0) {
		container = 1;
		PE_parse_boot_argn("mp_sbx_container", &container, sizeof(container));
	}
	if (ext < 0) {
		ext = 1;
		PE_parse_boot_argn("mp_sbx_ext", &ext, sizeof(ext));
	}
	if (call >= 0 && call < 64 && !(mapple_sandbox_seen & (1ULL << call))) {
		mapple_sandbox_seen |= 1ULL << call;
		char name[32];
		proc_name(proc_pid(p), name, sizeof(name));
		printf("Sandbox: %s op %d\n", name, call);
	}
	switch (call) {
	case 4:         /* container path (audit token / pid) */
		return container ? mapple_sandbox_container_path(p, arg) : ENOENT;
	case 5:         /* extension issue */
		return ext ? mapple_sandbox_issue(arg) : ENOENT;
	case 6:         /* extension consume */
		return ext ? mapple_sandbox_consume(arg) : ENOENT;
	case 0x43:      /* container expected */
		return ENOENT;
	default:
		return 0;
	}
}

/*
 * __mac_syscall("AMFI", 90, {u64 input flags, user pointer to u64 output}): dyld's amfi_check_dyld_policy_self.
 * Without an answer dyld treats every process as restricted: no DYLD_ variables, no interposing (test tools'
 * tracing libraries). DEVELOPMENT/DEBUG kernels allow all of them but a custom shared cache, except to set-ID programs,
 * which get none; release kernels and every other AMFI call answer as when no AMFI policy existed.
 */
#define AMFI_SYSCALL_CHECK_DYLD_POLICY_SELF     90
#define AMFI_DYLD_OUTPUT_ALLOW_AT_PATH                  (1ULL << 0)
#define AMFI_DYLD_OUTPUT_ALLOW_PATH_VARS                (1ULL << 1)
#define AMFI_DYLD_OUTPUT_ALLOW_FALLBACK_PATHS           (1ULL << 3)
#define AMFI_DYLD_OUTPUT_ALLOW_PRINT_VARS               (1ULL << 4)
#define AMFI_DYLD_OUTPUT_ALLOW_FAILED_LIBRARY_INSERTION (1ULL << 5)
#define AMFI_DYLD_OUTPUT_ALLOW_LIBRARY_INTERPOSING      (1ULL << 6)
#define AMFI_DYLD_OUTPUT_ALLOW_EMBEDDED_VARS            (1ULL << 7)

static int
amfi_stub_syscall(struct proc *p, int call, user_addr_t arg)
{
#if DEVELOPMENT || DEBUG
	if (call == AMFI_SYSCALL_CHECK_DYLD_POLICY_SELF) {
		uint64_t args[2];
		uint64_t out = proc_issetugid(p) ? 0 : AMFI_DYLD_OUTPUT_ALLOW_AT_PATH | AMFI_DYLD_OUTPUT_ALLOW_PATH_VARS |
		    AMFI_DYLD_OUTPUT_ALLOW_FALLBACK_PATHS | AMFI_DYLD_OUTPUT_ALLOW_PRINT_VARS |
		    AMFI_DYLD_OUTPUT_ALLOW_FAILED_LIBRARY_INSERTION | AMFI_DYLD_OUTPUT_ALLOW_LIBRARY_INTERPOSING |
		    AMFI_DYLD_OUTPUT_ALLOW_EMBEDDED_VARS;
		int error = copyin(arg, args, sizeof(args));
		if (error == 0) {
			error = copyout(&out, (user_addr_t)args[1], sizeof(out));
		}
		return error;
	}
#else
	(void)p;
	(void)call;
	(void)arg;
#endif
	return ENOPOLICY;
}

static const struct mac_policy_ops amfi_stub_mac_ops = {
	.mpo_vnode_check_signature = amfi_stub_vnode_check_signature,
	.mpo_policy_syscall = amfi_stub_syscall,
};

/*
 * Privileges for non-root processes come from MAC policies (priv_check_cred
 * -> mac_priv_grant); on iOS Sandbox/AMFI grant them. Without a grant UIKit
 * apps die in CoreAnimation setup: work_interval_create(CA_CLIENT) needs
 * PRIV_WORK_INTERVAL ("Failed to create work interval", SpringBoard).
 * Grant that one; log the first denial of each other privilege per process.
 */
#define MAPPLE_PRIV_LOG_MAX      64
static struct {
	int  priv;
	char name[20];
} mapple_priv_logged[MAPPLE_PRIV_LOG_MAX];
static int mapple_priv_logged_count;

static int
mapple_sandbox_priv_grant(kauth_cred_t cred __unused, int priv)
{
	if (priv == PRIV_WORK_INTERVAL) {
		return 0;
	}
	char name[20];
	proc_selfname(name, sizeof(name));
	for (int i = 0; i < mapple_priv_logged_count; i++) {
		if (mapple_priv_logged[i].priv == priv &&
		    strncmp(mapple_priv_logged[i].name, name, sizeof(name)) == 0) {
			return EPERM;
		}
	}
	if (mapple_priv_logged_count < MAPPLE_PRIV_LOG_MAX) {
		mapple_priv_logged[mapple_priv_logged_count].priv = priv;
		strlcpy(mapple_priv_logged[mapple_priv_logged_count].name, name, sizeof(name));
		mapple_priv_logged_count++;
		printf("Sandbox: %s privilege %d denied\n", name, priv);
	}
	return EPERM;
}

static const struct mac_policy_ops mapple_sandbox_mac_ops = {
	.mpo_policy_syscall = mapple_sandbox_syscall,
	.mpo_priv_grant = mapple_sandbox_priv_grant,
};

static struct mac_policy_conf mapple_sandbox_mac_conf = {
	.mpc_name = "Sandbox",
	.mpc_fullname = "MT6781 permissive sandbox stand-in",
	.mpc_ops = &mapple_sandbox_mac_ops,
};

static mac_policy_handle_t mapple_sandbox_mac_handle;

static struct mac_policy_conf amfi_stub_mac_conf = {
	.mpc_name = "AMFI",                     /* the name dyld's __mac_syscall asks for */
	.mpc_fullname = "MT6781 built-in Mobile File Integrity",
	.mpc_ops = &amfi_stub_mac_ops,
};

static mac_policy_handle_t amfi_stub_mac_handle;

/* Called from bsd_init once MAC policies can register, before any exec. */
void amfi_stub_policy_register(void);

void
amfi_stub_policy_register(void)
{
	int error = mac_policy_register(&amfi_stub_mac_conf, &amfi_stub_mac_handle, NULL);
	printf("AMFI: platform binary policy %s (%d)\n",
	    error ? "NOT registered" : "registered", error);
	error = mac_policy_register(&mapple_sandbox_mac_conf, &mapple_sandbox_mac_handle, NULL);
	printf("Sandbox: permissive policy %s (%d)\n",
	    error ? "NOT registered" : "registered", error);
}
#endif /* ARM64_BOARD_CONFIG_MT6781 */
