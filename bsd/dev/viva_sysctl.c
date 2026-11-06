/*
 * viva: sysctls for Viva Manager (gpu/app, requests KA1/KA2 in gpu/docs/RICHIESTE-KERNEL.md).
 *
 * kern.viva_link (KA1): the USB link's status commands from userspace. One call: newp = the command text (as after
 *   "cmd" on the link), oldp = the reply (the same text the link gets, at most 8 KiB, cut with "...\n"). Everybody
 *   may run the read-only ones: stat, gpu [status|freq|dvfs|lat|calls|clocks|kv], disp [kv], bat, cpu,
 *   wifi [status|sta]. Root (the app's setuid helper) may also run the other gpu/disp/bat/cpu/wifi subcommands
 *   ("gpu freq 800", "gpu dvfs off", "bat lpm on", ...). Never peek/poke/pmic/i2c, push/pull/run, panic, reboot.
 *   "gpu kv" / "disp kv": stable key=value lines (VivaGPU.cpp, SweetFramebuffer.cpp).
 * kern.viva_nextbootargs (KA2): the boot-args of the next boot, i.e. the cmdline in the SWEETXNU payload of the
 *   active slot's boot partition (androidboot.slot_suffix, else boot_b): read by everybody, written by root
 *   (bsd/dev/sweet_ufs.c sweet_ufs_bootargs: one 4 KiB block, checked, read back).
 * kern.viva_nextbootargs_orig: what that partition held the first time either sysctl was used (for "restore").
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/sysctl.h>
#include <sys/errno.h>
#include <sys/kauth.h>
#include <kern/locks.h>
#include <pexpert/pexpert.h>

extern void viva_link_stat(char *out, size_t len);                          /* bsd/dev/viva_usb.c */
extern void viva_battery_link(const char *args, char *out, size_t len);     /* iokit/Kernel/arm/VivaBattery.cpp */
extern void viva_wifi_link(const char *args, char *out, size_t len);        /* bsd/dev/viva_wifi.c */
extern void viva_gpu_link(const char *args, char *out, size_t len);         /* iokit/Kernel/arm/VivaGPU.cpp */
extern void viva_cpufreq_link(const char *args, char *out, size_t len);     /* osfmk/arm64/viva_cpufreq.c */
extern void viva_disp_link(const char *args, char *out, size_t len);        /* iokit/Kernel/arm/SweetFramebuffer.cpp */
extern int sweet_ufs_bootargs(const char *part, bool write, char *cmdline, size_t len); /* bsd/dev/sweet_ufs.c */
extern int sweet_ufs_bootfile(bool write, uint64_t base, char *cmdline, size_t len);     /* bsd/dev/sweet_ufs.c, 0180 */
extern uint64_t sweet_vivaboot_hash(const char *s);
extern int sweet_ufs_bootrec_raw(bool write, uint8_t *blk);                          /* bsd/dev/sweet_ufs.c, 0180 */

#define VL_CMD_MAX      256
#define VL_REPLY_MAX    8192
#define BOOTARGS_MAX    608

LCK_GRP_DECLARE(viva_sysctl_grp, "viva_sysctl");
LCK_MTX_DECLARE(viva_sysctl_mtx, &viva_sysctl_grp);

typedef void (*viva_link_fn)(const char *args, char *out, size_t len);

static const struct {
	const char   *word;
	viva_link_fn  fn;
	const char   *ro[8];        /* subcommands everybody may run ("" = the word alone) */
} viva_link_cmds[] = {
	{ "gpu", viva_gpu_link, { "", "status", "freq", "dvfs", "lat", "calls", "clocks", "kv" } },
	{ "disp", viva_disp_link, { "", "kv" } },
	{ "bat", viva_battery_link, { "" } },
	{ "cpu", viva_cpufreq_link, { "" } },
	{ "wifi", viva_wifi_link, { "", "status", "sta" } },
};

static int
viva_link_run(const char *cmd, bool root, char *out, size_t len)
{
	const char *c = cmd, *rest;
	while (*c == ' ') {
		c++;
	}
	for (rest = c; *rest && *rest != ' '; rest++) {
	}
	size_t wl = (size_t)(rest - c);
	while (*rest == ' ') {
		rest++;
	}
	/* the arguments without trailing blanks, for the read-only match */
	char args[VL_CMD_MAX];
	strlcpy(args, rest, sizeof(args));
	for (size_t n = strlen(args); n > 0 && (args[n - 1] == ' ' || args[n - 1] == '\n'); n--) {
		args[n - 1] = 0;
	}
	if (wl == 4 && strncmp(c, "stat", 4) == 0) {
		viva_link_stat(out, len);
		return 0;
	}
	for (unsigned i = 0; i < sizeof(viva_link_cmds) / sizeof(viva_link_cmds[0]); i++) {
		if (strlen(viva_link_cmds[i].word) != wl || strncmp(c, viva_link_cmds[i].word, wl) != 0) {
			continue;
		}
		bool ro = false;
		for (unsigned k = 0; k < 8 && viva_link_cmds[i].ro[k] != NULL; k++) {
			ro |= strcmp(args, viva_link_cmds[i].ro[k]) == 0;
		}
		if (!ro && !root) {
			snprintf(out, len, "%s %s: needs root\n", viva_link_cmds[i].word, args);
			return EPERM;
		}
		viva_link_cmds[i].fn(args, out, len);
		return 0;
	}
	snprintf(out, len, "usage: stat | gpu [status|freq|dvfs|lat|calls|clocks|kv] | disp [kv] | bat | cpu | "
	    "wifi [status|sta] (root: their other subcommands)\n");
	return *c ? ENOTSUP : 0;
}

static int
sysctl_viva_link SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char cmd[VL_CMD_MAX];
	int error = 0;

	bzero(cmd, sizeof(cmd));
	if (req->newptr != USER_ADDR_NULL) {
		if (req->newlen >= sizeof(cmd)) {
			return EINVAL;
		}
		error = SYSCTL_IN(req, cmd, req->newlen);
		if (error) {
			return error;
		}
		cmd[req->newlen] = 0;
	}
	char *out = kalloc_data(VL_REPLY_MAX, Z_WAITOK | Z_ZERO);
	if (out == NULL) {
		return ENOMEM;
	}
	bool root = kauth_cred_issuser(kauth_cred_get());
	lck_mtx_lock(&viva_sysctl_mtx);
	int run = viva_link_run(cmd, root, out, VL_REPLY_MAX);
	lck_mtx_unlock(&viva_sysctl_mtx);
	size_t n = strnlen(out, VL_REPLY_MAX);
	if (n >= VL_REPLY_MAX - 1) {
		strlcpy(out + VL_REPLY_MAX - 5, "...\n", 5);     /* cut */
		n = VL_REPLY_MAX - 1;
	}
	error = SYSCTL_OUT(req, out, n + 1);
	kfree_data(out, VL_REPLY_MAX);
	return error ? error : run;
}

SYSCTL_PROC(_kern, OID_AUTO, viva_link, CTLTYPE_OPAQUE | CTLFLAG_RW | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    0, 0, sysctl_viva_link, "-", "viva link commands (KA1)");

/* ---- KA2 ---- */

static char viva_bootargs_orig[BOOTARGS_MAX + 1];
static bool viva_bootargs_have_orig;

static const char *
viva_boot_part(void)
{
	char slot[8];
	bzero(slot, sizeof(slot));
	if (PE_parse_boot_argn("androidboot.slot_suffix", slot, sizeof(slot)) && strcmp(slot, "_a") == 0) {
		return "boot_a";
	}
	return "boot_b";
}

/* viva_sysctl_mtx held */
static void
viva_bootargs_remember(void)
{
	if (!viva_bootargs_have_orig &&
	    sweet_ufs_bootargs(viva_boot_part(), false, viva_bootargs_orig, sizeof(viva_bootargs_orig)) == 0) {
		viva_bootargs_have_orig = true;
	}
}

/*
 * 0180: LK write-protects the boot partitions while iOS runs, so a line that cannot go into the payload goes into the
 * VIVABOOT record in userdata (sweet_ufs_bootfile), which the shim reads before XNU while the payload still holds the
 * line it was saved against. viva_bootargs_src: where the next boot's line comes from.
 */
static int viva_bootpart_wp = -1;               /* boot partition: -1 unknown, 0 writable, 1 write-protected */
static const char *viva_bootargs_src = "payload";

/* viva_sysctl_mtx held. The next boot's command line: the VIVABOOT record if it applies, else the payload's. */
static int
viva_next_line(char *cur, size_t len, char *payload, size_t plen)
{
	int error = sweet_ufs_bootargs(viva_boot_part(), false, payload, plen);
	if (error) {
		return error;
	}
	if (sweet_ufs_bootfile(false, sweet_vivaboot_hash(payload), cur, len) == 0) {
		viva_bootargs_src = "vivaboot";
	} else {
		strlcpy(cur, payload, len);
		viva_bootargs_src = "payload";
	}
	return 0;
}

/* viva_sysctl_mtx held. Payload first (it fails while LK's write protection is on), else the VIVABOOT record. */
static int
viva_next_line_set(char *want, size_t len, const char *payload)
{
	uint64_t base = sweet_vivaboot_hash(payload);
	if (want[0] == 0 || strcmp(want, payload) == 0) {
		/* empty = back to the payload's line: never an empty command line in the boot partition */
		char old[BOOTARGS_MAX + 1];
		if (sweet_ufs_bootfile(false, base, old, sizeof(old)) == 0) {
			char none[1] = "";
			return sweet_ufs_bootfile(true, base, none, sizeof(none));
		}
		return 0;
	}
	int error = viva_bootpart_wp == 1 ? EROFS : sweet_ufs_bootargs(viva_boot_part(), true, want, len);
	if (error == 0) {
		viva_bootpart_wp = 0;
		return 0;               /* the payload changed: an old record no longer applies (other base) */
	}
	if (error != EROFS && error != EIO) {
		return error;
	}
	if (error == EROFS) {
		viva_bootpart_wp = 1;
	}
	return sweet_ufs_bootfile(true, base, want, len);
}

static int
sysctl_viva_nextbootargs SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char cur[BOOTARGS_MAX + 1], want[BOOTARGS_MAX + 1], payload[BOOTARGS_MAX + 1];
	int error;

	bzero(cur, sizeof(cur));
	lck_mtx_lock(&viva_sysctl_mtx);
	viva_bootargs_remember();
	error = viva_next_line(cur, sizeof(cur), payload, sizeof(payload));
	lck_mtx_unlock(&viva_sysctl_mtx);
	if (error) {
		return error;
	}
	error = SYSCTL_OUT(req, cur, strlen(cur) + 1);
	if (error || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (!kauth_cred_issuser(kauth_cred_get())) {
		return EPERM;
	}
	if (req->newlen >= sizeof(want)) {
		return EINVAL;
	}
	bzero(want, sizeof(want));
	error = SYSCTL_IN(req, want, req->newlen);
	if (error) {
		return error;
	}
	want[req->newlen] = 0;
	size_t n = strlen(want);
	while (n > 0 && (want[n - 1] == '\n' || want[n - 1] == ' ')) {
		want[--n] = 0;
	}
	lck_mtx_lock(&viva_sysctl_mtx);
	error = sweet_ufs_bootargs(viva_boot_part(), false, payload, sizeof(payload));
	if (!error) {
		error = viva_next_line_set(want, sizeof(want), payload);
	}
	lck_mtx_unlock(&viva_sysctl_mtx);
	return error;
}

static int
sysctl_viva_nextbootargs_src SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char cur[BOOTARGS_MAX + 1], payload[BOOTARGS_MAX + 1], out[96];
	lck_mtx_lock(&viva_sysctl_mtx);
	int error = viva_next_line(cur, sizeof(cur), payload, sizeof(payload));
	snprintf(out, sizeof(out), "%s %s", error ? "error" : viva_bootargs_src,
	    viva_bootpart_wp == 1 ? "boot-partition-write-protected" : viva_bootpart_wp == 0 ? "boot-partition-writable" :
	    "boot-partition-untested");
	lck_mtx_unlock(&viva_sysctl_mtx);
	return SYSCTL_OUT(req, out, strlen(out) + 1);
}

SYSCTL_PROC(_kern, OID_AUTO, viva_nextbootargs_src, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    0, 0, sysctl_viva_nextbootargs_src, "A", "where the next boot's command line comes from (0180)");

/* kern.viva_bootrec: the VIVABOOT record's raw 4 KiB block (tools/vivad/vbootrec). Read: everybody; write: root, exactly
 * 4096 bytes, stored unchecked apart from the HFS+ volume check (the shim validates what it reads). */
#define BOOTREC_SIZE    4096

static int
sysctl_viva_bootrec SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	uint8_t *blk = kalloc_data(BOOTREC_SIZE, Z_WAITOK | Z_ZERO);
	if (!blk) {
		return ENOMEM;
	}
	lck_mtx_lock(&viva_sysctl_mtx);
	int error = sweet_ufs_bootrec_raw(false, blk);
	lck_mtx_unlock(&viva_sysctl_mtx);
	if (!error) {
		error = SYSCTL_OUT(req, blk, BOOTREC_SIZE);
	}
	if (!error && req->newptr != USER_ADDR_NULL) {
		if (!kauth_cred_issuser(kauth_cred_get())) {
			error = EPERM;
		} else if (req->newlen != BOOTREC_SIZE) {
			error = EINVAL;
		} else if ((error = SYSCTL_IN(req, blk, BOOTREC_SIZE)) == 0) {
			lck_mtx_lock(&viva_sysctl_mtx);
			error = sweet_ufs_bootrec_raw(true, blk);
			lck_mtx_unlock(&viva_sysctl_mtx);
		}
	}
	kfree_data(blk, BOOTREC_SIZE);
	return error;
}

SYSCTL_PROC(_kern, OID_AUTO, viva_bootrec, CTLTYPE_OPAQUE | CTLFLAG_RW | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    0, 0, sysctl_viva_bootrec, "-", "VIVABOOT record, raw 4 KiB block (0180)");

SYSCTL_PROC(_kern, OID_AUTO, viva_nextbootargs, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    0, 0, sysctl_viva_nextbootargs, "A", "boot-args of the next boot (KA2)");

static int
sysctl_viva_nextbootargs_orig SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	lck_mtx_lock(&viva_sysctl_mtx);
	viva_bootargs_remember();
	bool have = viva_bootargs_have_orig;
	lck_mtx_unlock(&viva_sysctl_mtx);
	if (!have) {
		return ENOENT;
	}
	return SYSCTL_OUT(req, viva_bootargs_orig, strlen(viva_bootargs_orig) + 1);
}

SYSCTL_PROC(_kern, OID_AUTO, viva_nextbootargs_orig, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED,
    0, 0, sysctl_viva_nextbootargs_orig, "A", "boot-args of the next boot as first seen (KA2)");
#endif /* ARM64_BOARD_CONFIG_SWEET */
