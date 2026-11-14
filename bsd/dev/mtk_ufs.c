/*
 * MT6781 internal UFS root device.
 *
 * The Qualcomm UFS host (UFSHCI 2.1) is left enabled with the link up by the
 * bootloader, which loaded our boot image from it. This driver reuses that
 * state: it points the transfer request list at its own memory (verifying the
 * host took it before ringing any doorbell) and issues polled SCSI commands,
 * one at a time. It finds one GPT partition by name (boot-arg mp_ufsroot=,
 * default "userdata") on LUN mp_ufslun= or, by default, on the first of LUNs 0-7
 * whose GPT has it (MediaTek keeps the preloader on LUNs 0/1 and the GPT on
 * another; discovery only reads), and exposes it as the BSD disk ufsroot0
 * (rufsroot0), with 512-byte logical sectors over the device's 4 KiB blocks.
 * Root on it with rd=ufsroot0.
 *
 * All DMA goes through one physically contiguous allocation (descriptors and
 * a 1 MiB bounce buffer); the host is not cache coherent, so caches are
 * cleaned/invalidated around each command.
 *
 * Writes are refused unless boot-arg mp_ufsrw=1 is set. Every write is checked
 * against the partition bounds again right before it is issued, so nothing
 * outside the named partition can be written.
 */
#if defined(ARM64_BOARD_CONFIG_MT6781)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/buf.h>
#include <sys/conf.h>
#include <sys/disk.h>
#include <sys/errno.h>
#include <sys/proc.h>
#include <sys/uio.h>
#include <miscfs/devfs/devfs.h>
#include <kern/locks.h>
#include <kern/kalloc.h>
#include <kern/clock.h>
#include <kern/sched_prim.h>
#include <machine/machine_routines.h>
#include <arm/caches_internal.h>
#include <vm/pmap.h>
#include <vm/vm_kern.h>
#include <pexpert/pexpert.h>
#include <pexpert/device_tree.h>
#include <libkern/libkern.h>
#include <machine/atomic.h>

extern ppnum_t pmap_find_phys(pmap_t pmap, addr64_t va);
/* osfmk/arm/machine_routines.h, not exported to the BSD layer */
extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
/* osfmk/arm/cpu_data.h, not exported to the BSD layer */
extern int get_preemption_level(void);
extern kern_return_t kernel_thread_start(thread_continue_t, void *, thread_t *);
extern void thread_deallocate(thread_t);
extern int hz;
static void mapple_log_start(void);
extern void proc_sampler_start(void);
extern void usblink_probe(void);

#define REG_CAP         0x00
#define REG_IS          0x20
#define REG_IE          0x24
#define REG_HCS         0x30
#define REG_HCE         0x34
#define REG_UTRLBA      0x50
#define REG_UTRLBAU     0x54
#define REG_UTRLDBR     0x58
#define REG_UTRLCLR     0x5c
#define REG_UTRLRSR     0x60

#define HCS_DP          (1u << 0)
#define HCS_UTRLRDY     (1u << 1)
#define CAP_64AS        (1u << 24)              /* 64-bit DMA addressing */

#define UFS_BLOCK       4096u
#define BOUNCE_SIZE     (1u << 20)
#define PRDT_CHUNK      (256u << 10)            /* PRDT byte count field: 18 bits */
#define PRDT_MAX        (BOUNCE_SIZE / PRDT_CHUNK)
#define CMD_TIMEOUT_NS  (5ull * NSEC_PER_SEC)
#define CMD_SPIN_NS     (50ull * NSEC_PER_USEC)
#define CMD_SLEEP_NS    (200ull * NSEC_PER_USEC)

struct utrd {
	uint32_t dw0, dw1, dw2, dw3;
	uint32_t ucdba, ucdbau;
	uint32_t resp;          /* [31:16] response UPIU offset, [15:0] length, DWORDs */
	uint32_t prdt;          /* [31:16] PRDT offset (DWORDs), [15:0] entries */
};

struct ucd {
	uint8_t cmd[512];
	uint8_t resp[512];
	struct {
		uint32_t addr_lo, addr_hi, rsvd, dbc;
	} prdt[PRDT_MAX];
};

/* Layout of the contiguous DMA allocation. */
#define DMA_UTRL        0x0000                  /* 1 KiB aligned */
#define DMA_UCD         0x1000                  /* 128 byte aligned */
#define DMA_BOUNCE      0x4000
#define DMA_SIZE        (DMA_BOUNCE + BOUNCE_SIZE)

static struct {
	vm_offset_t regs;
	vm_offset_t dma;                        /* kernel VA of the DMA allocation */
	uint64_t dma_pa;
	uint8_t lun;
	uint8_t tag;
	uint64_t first_block, nblocks;          /* partition, in 4 KiB device blocks */
	uint32_t secsize;                       /* logical sector size for the BSD layer */
	bool writable;                          /* boot-arg mp_ufsrw=1 */
	bool cache;                             /* boot-arg mp_ufscache=1 */
	bool wait;                              /* boot-arg mp_ufswait=1 */
	int bmajor, cmajor;
	dev_t bdev;
	bool ready;
} su;

static LCK_GRP_DECLARE(mtk_ufs_grp, "mtk_ufs");
static LCK_MTX_DECLARE(mtk_ufs_lock, &mtk_ufs_grp);

static inline uint32_t
ufs_rd(uint32_t off)
{
	return *(volatile uint32_t *)(su.regs + off);
}

static inline void
ufs_wr(uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(su.regs + off) = v;
}

static void
dma_sync(uint32_t off, uint32_t len)
{
	flush_dcache64((addr64_t)(su.dma_pa + off), len, TRUE);
}

static void
dma_clean(uint32_t off, uint32_t len)
{
	clean_dcache64((addr64_t)(su.dma_pa + off), len, TRUE);
}

/*
 * One SCSI command in slot 0, data (if any) to/from the bounce buffer
 * (to the device when write is set). Returns 0 or an errno.
 */
static int
ufs_scsi(const uint8_t *cdb, unsigned cdb_len, uint32_t len, bool write)
{
	struct utrd *utrd = (struct utrd *)(su.dma + DMA_UTRL);
	struct ucd *ucd = (struct ucd *)(su.dma + DMA_UCD);
	uint64_t ucd_pa = su.dma_pa + DMA_UCD, bounce_pa = su.dma_pa + DMA_BOUNCE;

	assert(len <= BOUNCE_SIZE);
	bzero(utrd, sizeof(*utrd));
	bzero(ucd, sizeof(*ucd));

	ucd->cmd[0] = 0x01;                     /* COMMAND UPIU */
	ucd->cmd[1] = len ? (write ? 0x20 : 0x40) : 0x00;      /* W: data out, R: data in */
	ucd->cmd[2] = su.lun;
	ucd->cmd[3] = ++su.tag;
	ucd->cmd[12] = (uint8_t)(len >> 24);
	ucd->cmd[13] = (uint8_t)(len >> 16);
	ucd->cmd[14] = (uint8_t)(len >> 8);
	ucd->cmd[15] = (uint8_t)len;
	memcpy(&ucd->cmd[16], cdb, cdb_len);

	/* DD: 1 = memory to device, 2 = device to memory. */
	utrd->dw0 = (1u << 28) | ((len ? (write ? 1u : 2u) : 0u) << 25) | (1u << 24);
	utrd->dw2 = 0x0f;                       /* OCS invalid until updated */
	utrd->ucdba = (uint32_t)ucd_pa;
	utrd->ucdbau = (uint32_t)(ucd_pa >> 32);
	utrd->resp = (512u / 4) << 16 | (512u / 4);
	if (len) {
		uint32_t n = 0;
		for (uint32_t done = 0; done < len; done += PRDT_CHUNK, n++) {
			uint32_t chunk = MIN(PRDT_CHUNK, len - done);
			ucd->prdt[n].addr_lo = (uint32_t)(bounce_pa + done);
			ucd->prdt[n].addr_hi = (uint32_t)((bounce_pa + done) >> 32);
			ucd->prdt[n].dbc = chunk - 1;
		}
		utrd->prdt = (1024u / 4) << 16 | n;
	}

	if (su.cache) {
		dma_clean(DMA_UTRL, sizeof(*utrd));
		dma_clean(DMA_UCD, 16 + cdb_len);
		dma_sync(DMA_UCD + offsetof(struct ucd, resp), sizeof(ucd->resp));
		if (len) {
			dma_clean(DMA_UCD + offsetof(struct ucd, prdt),
			    (utrd->prdt & 0xffff) * sizeof(ucd->prdt[0]));
		}
	} else {
		dma_sync(DMA_UTRL, sizeof(*utrd));
		dma_sync(DMA_UCD, sizeof(*ucd));
	}
	if (len) {
		if (su.cache && write) {
			dma_clean(DMA_BOUNCE, len);
		} else {
			dma_sync(DMA_BOUNCE, len);
		}
	}
	__builtin_arm_dsb(0xf);

	ufs_wr(REG_UTRLDBR, 1);
	uint64_t deadline;
	nanoseconds_to_absolutetime(CMD_TIMEOUT_NS, &deadline);
	deadline += mach_absolute_time();
	bool sleepable = su.wait && su.ready && ml_get_interrupts_enabled() &&
	    !ml_at_interrupt_context() && get_preemption_level() == 0;
	uint64_t spin_until = 0, sleep_interval = 0;
	if (sleepable) {
		nanoseconds_to_absolutetime(CMD_SPIN_NS, &spin_until);
		spin_until += mach_absolute_time();
		nanoseconds_to_absolutetime(CMD_SLEEP_NS, &sleep_interval);
	}
	while (ufs_rd(REG_UTRLDBR) & 1) {
		uint64_t now = mach_absolute_time();
		if (now > deadline) {
			ufs_wr(REG_UTRLCLR, ~1u);
			printf("UFS: command 0x%02x timed out\n", cdb[0]);
			return ETIMEDOUT;
		}
		if (sleepable && now >= spin_until) {
			uint64_t wake = now + MIN(sleep_interval, deadline - now);
			/* Keep the mutex: slot 0 and the bounce buffer are still owned. */
			if (assert_wait_deadline_with_leeway((event_t)&su.wait, THREAD_UNINT,
			    TIMEOUT_URGENCY_SYS_CRITICAL, wake, 0) == THREAD_WAITING) {
				thread_block(THREAD_CONTINUE_NULL);
			}
		} else {
			delay(1);
		}
	}
	__builtin_arm_dsb(0xf);
	ufs_wr(REG_IS, ufs_rd(REG_IS));         /* acknowledge completion status */

	dma_sync(DMA_UTRL, sizeof(*utrd));
	if (su.cache) {
		dma_sync(DMA_UCD + offsetof(struct ucd, resp), sizeof(ucd->resp));
	} else {
		dma_sync(DMA_UCD, sizeof(*ucd));
	}
	if (len && !(su.cache && write)) {
		dma_sync(DMA_BOUNCE, len);
	}

	uint32_t ocs = utrd->dw2 & 0xff;
	if (ocs || ucd->resp[0] != 0x21 || ucd->resp[6] || ucd->resp[7]) {
		printf("UFS: command 0x%02x failed: ocs 0x%x upiu 0x%x response 0x%x status 0x%x\n",
		    cdb[0], ocs, ucd->resp[0], ucd->resp[6], ucd->resp[7]);
		/* CHECK CONDITION: the fixed-format sense data in the response UPIU's data segment (length at 32, data at 34) */
		if (!ocs && ucd->resp[0] == 0x21 && ucd->resp[7] == 0x02 && (ucd->resp[32] << 8 | ucd->resp[33]) >= 14) {
			const uint8_t *sense = &ucd->resp[34];
			unsigned key = sense[2] & 0xf;
			printf("UFS: sense key 0x%x asc 0x%02x ascq 0x%02x%s\n", key, sense[12], sense[13],
			    key == 0x7 ? " (DATA PROTECT: write-protected)" : "");
			if (key == 0x7) {
				return EROFS;
			}
		}
		return EIO;
	}
	return 0;
}

/* READ(10) of count device blocks at lba into the bounce buffer. */
static int
ufs_read_blocks(uint64_t lba, uint32_t count)
{
	if (lba > UINT32_MAX || count > BOUNCE_SIZE / UFS_BLOCK) {
		return EINVAL;
	}
	uint8_t cdb[10] = {
		0x28, 0,
		(uint8_t)(lba >> 24), (uint8_t)(lba >> 16), (uint8_t)(lba >> 8), (uint8_t)lba,
		0, (uint8_t)(count >> 8), (uint8_t)count, 0
	};
	return ufs_scsi(cdb, sizeof(cdb), count * UFS_BLOCK, false);
}

/*
 * WRITE(10) of count device blocks from the bounce buffer to lba. The range
 * must lie inside the partition: this is the only place data goes out.
 */
static int
ufs_write_blocks(uint64_t lba, uint32_t count)
{
	if (!su.writable || !su.nblocks || count == 0 || count > BOUNCE_SIZE / UFS_BLOCK ||
	    lba < su.first_block || lba - su.first_block >= su.nblocks ||
	    count > su.nblocks - (lba - su.first_block) || lba + count - 1 > UINT32_MAX) {
		printf("UFS: refusing write of %u blocks at 0x%llx\n", count, lba);
		return EPERM;
	}
	uint8_t cdb[10] = {
		0x2a, 0,
		(uint8_t)(lba >> 24), (uint8_t)(lba >> 16), (uint8_t)(lba >> 8), (uint8_t)lba,
		0, (uint8_t)(count >> 8), (uint8_t)count, 0
	};
	return ufs_scsi(cdb, sizeof(cdb), count * UFS_BLOCK, true);
}

static int
ufs_sync_cache(void)
{
	static const uint8_t cdb[10] = { 0x35 };       /* SYNCHRONIZE CACHE(10), whole LUN */
	return ufs_scsi(cdb, sizeof(cdb), 0, false);
}

static int
ufs_lookup_partition(const char *want, uint64_t *firstp, uint64_t *nblocksp)
{
	const uint8_t *b = (const uint8_t *)(su.dma + DMA_BOUNCE);
	int error = ufs_read_blocks(1, 1);
	if (error) {
		return error;
	}
	if (memcmp(b, "EFI PART", 8) != 0) {
		printf("UFS: LUN %u has no GPT\n", su.lun);
		return ENXIO;
	}
	uint64_t entry_lba;
	uint32_t nent, esz;
	memcpy(&entry_lba, b + 72, 8);
	memcpy(&nent, b + 80, 4);
	memcpy(&esz, b + 84, 4);
	if (esz != 128 || nent == 0 || nent * esz > BOUNCE_SIZE) {
		printf("UFS: unsupported GPT entries %u x %u\n", nent, esz);
		return ENXIO;
	}
	error = ufs_read_blocks(entry_lba, (nent * esz + UFS_BLOCK - 1) / UFS_BLOCK);
	if (error) {
		return error;
	}
	for (uint32_t i = 0; i < nent; i++) {
		const uint8_t *e = b + i * esz;
		char name[37];
		unsigned c;
		for (c = 0; c < 36; c++) {
			uint16_t ch = (uint16_t)(e[56 + 2 * c] | e[57 + 2 * c] << 8);
			if (ch == 0 || ch > 0x7f) {
				break;
			}
			name[c] = (char)ch;
		}
		name[c] = 0;
		if (strcmp(name, want) != 0) {
			continue;
		}
		uint64_t first, last;
		memcpy(&first, e + 32, 8);
		memcpy(&last, e + 40, 8);
		if (last < first) {
			return ENXIO;
		}
		*firstp = first;
		*nblocksp = last - first + 1;
		return 0;
	}
	printf("UFS: no GPT partition named \"%s\"\n", want);
	return ENOENT;
}

static int
ufs_find_partition(const char *want)
{
	return ufs_lookup_partition(want, &su.first_block, &su.nblocks);
}

/*
 * Read-only access to another GPT partition of the root's LUN (USB link "pull part:NAME", e.g. nvdata for the WiFi
 * calibration/MAC file). Reads only: there is no write path here, and ufs_write_blocks() still checks the root
 * partition's bounds. *sizep gets the partition size; len 0 only looks it up.
 */
int mtk_ufs_read_part(const char *name, uint64_t off, void *buf, uint32_t len, uint64_t *sizep);

int
mtk_ufs_read_part(const char *name, uint64_t off, void *buf, uint32_t len, uint64_t *sizep)
{
	static char cached[37];
	static uint64_t cfirst, cblocks;
	int error = 0;

	if (!su.ready) {
		return ENXIO;
	}
	lck_mtx_lock(&mtk_ufs_lock);
	if (strcmp(cached, name) != 0) {
		cached[0] = 0;
		error = ufs_lookup_partition(name, &cfirst, &cblocks);
		if (!error) {
			strlcpy(cached, name, sizeof(cached));
		}
	}
	uint64_t bytes = cblocks * UFS_BLOCK;
	if (!error && sizep) {
		*sizep = bytes;
	}
	if (!error && (off > bytes || len > bytes - off)) {
		error = EINVAL;
	}
	for (uint32_t done = 0; !error && done < len;) {
		uint64_t pos = off + done;
		uint32_t skip = (uint32_t)(pos % UFS_BLOCK);
		uint32_t want = MIN(len - done, BOUNCE_SIZE - skip);
		uint32_t blocks = (skip + want + UFS_BLOCK - 1) / UFS_BLOCK;
		error = ufs_read_blocks(cfirst + pos / UFS_BLOCK, blocks);
		if (!error) {
			memcpy((char *)buf + done, (const char *)(su.dma + DMA_BOUNCE) + skip, want);
			done += want;
		}
	}
	lck_mtx_unlock(&mtk_ufs_lock);
	return error;
}

/*
 * KA2 (gpu/docs/RICHIESTE-KERNEL.md): the payload's command line = cmdline[608] at +0x48 of the MAPPLEPL payload
 * that tools/mkboot.sh puts at the start of the ramdisk of boot_a / boot_b (header v2, ramdisk at
 * page_size * (1 + ceil(kernel_size / page_size))). Read only: the next boot's line goes into the MAPPLEBR record,
 * and the payload keeps the line safe boot falls back to. Errors: ENOENT (partition, header, magic), EIO.
 */
#define PAYLOAD_CMDLINE         0x48
#define PAYLOAD_CMDLINE_LEN     608

int mtk_ufs_bootargs(const char *part, char *cmdline, size_t len);

int
mtk_ufs_bootargs(const char *part, char *cmdline, size_t len)
{
	uint8_t *b = (uint8_t *)(su.dma + DMA_BOUNCE);
	uint64_t first = 0, nblocks = 0;
	int error;

	if (strcmp(part, "boot_a") != 0 && strcmp(part, "boot_b") != 0) {
		return ENOENT;
	}
	if (!su.ready) {
		return ENXIO;
	}
	lck_mtx_lock(&mtk_ufs_lock);
	error = ufs_lookup_partition(part, &first, &nblocks);
	if (error) {
		error = ENOENT;
		goto out;
	}
	/* Android boot header v2 */
	if ((error = ufs_read_blocks(first, 1)) != 0) {
		error = EIO;
		goto out;
	}
	uint32_t ksize, rsize, page, version;
	memcpy(&ksize, b + 8, 4);
	memcpy(&rsize, b + 16, 4);
	memcpy(&page, b + 36, 4);
	memcpy(&version, b + 40, 4);
	if (memcmp(b, "ANDROID!", 8) != 0 || version != 2 || (page != 2048 && page != 4096) ||
	    rsize < PAYLOAD_CMDLINE + PAYLOAD_CMDLINE_LEN) {
		printf("UFS: %s: not a v2 boot image with a payload (version %u, page %u, ramdisk %u)\n", part, version,
		    page, rsize);
		error = ENOENT;
		goto out;
	}
	uint64_t roff = (uint64_t)page * (1 + (ksize + page - 1) / page);
	uint64_t at = roff + PAYLOAD_CMDLINE, lba = first + at / UFS_BLOCK;
	uint32_t in = (uint32_t)(at % UFS_BLOCK);
	if (in + PAYLOAD_CMDLINE_LEN > UFS_BLOCK || roff / UFS_BLOCK != at / UFS_BLOCK ||
	    lba - first >= nblocks) {
		printf("UFS: %s: payload at 0x%llx spans blocks\n", part, roff);
		error = ENOENT;
		goto out;
	}
	if ((error = ufs_read_blocks(lba, 1)) != 0) {
		error = EIO;
		goto out;
	}
	uint32_t pver;
	uint32_t pin = (uint32_t)(roff % UFS_BLOCK);
	memcpy(&pver, b + pin + 8, 4);
	if (memcmp(b + pin, "MAPPLEPL", 8) != 0 || pver != 1) {
		printf("UFS: %s: no MAPPLEPL v1 payload in the ramdisk\n", part);
		error = ENOENT;
		goto out;
	}
	size_t n = MIN(len - 1, (size_t)PAYLOAD_CMDLINE_LEN);
	memcpy(cmdline, b + in, n);
	cmdline[n] = 0;
	cmdline[strnlen(cmdline, n)] = 0;
out:
	lck_mtx_unlock(&mtk_ufs_lock);
	return error;
}

/*
 * 0180: the MAPPLEBR record (fork/shim/bootrec.h, same layout), because LK write-protects boot_a/boot_b while iOS
 * runs (WRITE(10) -> CHECK CONDITION, DATA PROTECT). The first 4 KiB block of the root partition's last MiB, outside
 * the HFS+ volume (checked from its header before every write); the shim reads it before XNU and uses its command line
 * while the boot partition's payload still holds base_hash's. Writes need mp_ufsrw=1 and go through ufs_write_blocks(),
 * so they stay inside the root partition; the block is read back and compared.
 */
#define BOOTREC_TAIL_BLOCKS    256u
#define BOOTREC_LINE           608

struct bootrec_rec {
	char magic[8];
	uint32_t version;
	uint32_t flags;                         /* bit 0: enabled */
	uint64_t base_hash;                     /* FNV-1a 64 of the payload command line it replaces */
	uint64_t seq;
	uint64_t line_hash;                     /* FNV-1a 64 of cmdline */
	uint8_t reserved[24];
	char cmdline[BOOTREC_LINE];
};
_Static_assert(sizeof(struct bootrec_rec) == 672, "MAPPLEBR layout (shim/bootrec.h)");

uint64_t mapple_bootrec_hash(const char *s);
int mtk_ufs_bootfile(bool write, uint64_t base, char *cmdline, size_t len);
int mtk_ufs_bootrec_raw(bool write, uint8_t *blk);

uint64_t
mapple_bootrec_hash(const char *s)
{
	uint64_t h = 0xcbf29ce484222325ULL;
	for (unsigned i = 0; i < BOOTREC_LINE && s[i]; i++) {
		h ^= (uint8_t)s[i];
		h *= 0x100000001b3ULL;
	}
	return h;
}

/*
 * Read: the record's command line into cmdline when it is enabled, valid and for base, any base if 0 (else ENOENT).
 * Write: cmdline (empty = disable the record) for base. EROFS without mp_ufsrw=1, ENOSPC when the HFS+ volume
 * reaches the record's block, EINVAL for a bad string, EIO.
 */
/* mtk_ufs_lock held. ENOSPC unless an HFS+/HFSX volume header says the volume ends before lba. */
static int
bootrec_volume_clear(uint64_t lba)
{
	const uint8_t *b = (const uint8_t *)(su.dma + DMA_BOUNCE);
	int error = ufs_read_blocks(su.first_block, 1);
	if (error) {
		return error;
	}
	const uint8_t *vh = b + 1024;           /* block size BE32 @40, total blocks BE32 @44 */
	uint16_t sig = (uint16_t)(vh[0] << 8 | vh[1]);
	uint64_t bsize = (uint64_t)vh[40] << 24 | vh[41] << 16 | vh[42] << 8 | vh[43];
	uint64_t total = (uint64_t)vh[44] << 24 | vh[45] << 16 | vh[46] << 8 | vh[47];
	if ((sig != 0x482b && sig != 0x4858) || bsize == 0 || bsize * total > (lba - su.first_block) * UFS_BLOCK) {
		printf("UFS: MAPPLEBR: volume 0x%x of %llu bytes reaches the record's block\n", sig, bsize * total);
		return ENOSPC;
	}
	return 0;
}

/* mtk_ufs_lock held. blk (4 KiB) to lba, cache flushed, read back and compared. */
static int
bootrec_write_locked(uint64_t lba, const uint8_t *blk)
{
	memcpy((void *)(su.dma + DMA_BOUNCE), blk, UFS_BLOCK);
	int error = ufs_write_blocks(lba, 1);
	if (!error) {
		error = ufs_sync_cache();
	}
	if (!error) {
		error = ufs_read_blocks(lba, 1);
	}
	if (!error && memcmp((const void *)(su.dma + DMA_BOUNCE), blk, UFS_BLOCK) != 0) {
		error = EIO;
	}
	return error;
}

/* The record's whole 4 KiB block, unchecked (kern.mapple_bootrec, tools/mappled/bootrec). Writes: mp_ufsrw=1, volume check. */
int
mtk_ufs_bootrec_raw(bool write, uint8_t *blk)
{
	if (!su.ready || su.nblocks <= BOOTREC_TAIL_BLOCKS) {
		return ENXIO;
	}
	if (write && !su.writable) {
		return EROFS;
	}
	uint64_t lba = su.first_block + su.nblocks - BOOTREC_TAIL_BLOCKS;
	lck_mtx_lock(&mtk_ufs_lock);
	int error = 0;
	if (write) {
		error = bootrec_volume_clear(lba);
		if (!error) {
			error = bootrec_write_locked(lba, blk);
		}
		printf("UFS: MAPPLEBR raw block %s (block 0x%llx)\n", error ? "NOT written" : "written", lba);
	} else {
		error = ufs_read_blocks(lba, 1);
		if (!error) {
			memcpy(blk, (const void *)(su.dma + DMA_BOUNCE), UFS_BLOCK);
		}
	}
	lck_mtx_unlock(&mtk_ufs_lock);
	return error;
}

int
mtk_ufs_bootfile(bool write, uint64_t base, char *cmdline, size_t len)
{
	const uint8_t *b = (const uint8_t *)(su.dma + DMA_BOUNCE);
	static uint8_t blk[UFS_BLOCK];
	int error = 0;

	if (!su.ready || su.nblocks <= BOOTREC_TAIL_BLOCKS) {
		return ENXIO;
	}
	size_t n = strnlen(cmdline, len);
	if (write) {
		if (!su.writable) {
			return EROFS;
		}
		if (n == len || n >= BOOTREC_LINE) {
			return EINVAL;
		}
		for (size_t i = 0; i < n; i++) {
			if (cmdline[i] < 0x20 || cmdline[i] > 0x7e) {
				return EINVAL;
			}
		}
	}
	uint64_t lba = su.first_block + su.nblocks - BOOTREC_TAIL_BLOCKS;
	lck_mtx_lock(&mtk_ufs_lock);
	if (write && (error = bootrec_volume_clear(lba)) != 0) {
		goto out;
	}
	if ((error = ufs_read_blocks(lba, 1)) != 0) {
		goto out;
	}
	struct bootrec_rec rec;
	memcpy(&rec, b, sizeof(rec));
	if (!write) {
		bool ok = memcmp(rec.magic, "MAPPLEBR", 8) == 0 && rec.version == 1 && (rec.flags & 1) && (base == 0 || rec.base_hash == base) &&
		    strnlen(rec.cmdline, BOOTREC_LINE) < BOOTREC_LINE && mapple_bootrec_hash(rec.cmdline) == rec.line_hash;
		if (!ok) {
			error = ENOENT;
			goto out;
		}
		strlcpy(cmdline, rec.cmdline, len);
		goto out;
	}
	uint64_t seq = memcmp(rec.magic, "MAPPLEBR", 8) == 0 ? rec.seq + 1 : 1;
	bzero(&rec, sizeof(rec));
	memcpy(rec.magic, "MAPPLEBR", 8);
	rec.version = 1;
	rec.flags = n ? 1 : 0;
	rec.base_hash = base;
	rec.seq = seq;
	memcpy(rec.cmdline, cmdline, n);
	rec.line_hash = mapple_bootrec_hash(rec.cmdline);
	bzero(blk, sizeof(blk));
	memcpy(blk, &rec, sizeof(rec));
	error = bootrec_write_locked(lba, blk);
	printf("UFS: MAPPLEBR %s (block 0x%llx, seq %llu): \"%s\"\n", error ? "NOT written" : n ? "written" : "disabled",
	    lba, seq, n ? cmdline : "");
out:
	lck_mtx_unlock(&mtk_ufs_lock);
	return error;
}

static int
ufs_take_over_host(void)
{
	uint32_t hce = ufs_rd(REG_HCE), hcs = ufs_rd(REG_HCS);
	if (!(hce & 1) || !(hcs & HCS_DP) || !(hcs & HCS_UTRLRDY)) {
		printf("UFS: host not ready (hce 0x%x hcs 0x%x)\n", hce, hcs);
		return ENXIO;
	}
	if (ufs_rd(REG_UTRLDBR)) {
		printf("UFS: requests outstanding from the bootloader\n");
		return EBUSY;
	}
	/* With RAM above 4 GiB (mkadt.py --highmem) the DMA memory can be there too. */
	if (((su.dma_pa + DMA_SIZE - 1) >> 32) && !(ufs_rd(REG_CAP) & CAP_64AS)) {
		printf("UFS: DMA memory at 0x%llx but the host has no 64-bit addressing\n",
		    su.dma_pa);
		return ENXIO;
	}
	ufs_wr(REG_IE, 0);                      /* polled operation */
	ufs_wr(REG_IS, ufs_rd(REG_IS));
	ufs_wr(REG_UTRLRSR, 0);
	uint64_t list = su.dma_pa + DMA_UTRL;
	ufs_wr(REG_UTRLBA, (uint32_t)list);
	ufs_wr(REG_UTRLBAU, (uint32_t)(list >> 32));
	ufs_wr(REG_UTRLRSR, 1);
	/* Never ring a doorbell unless the host uses our list. */
	if (ufs_rd(REG_UTRLBA) != (uint32_t)list || ufs_rd(REG_UTRLBAU) != (uint32_t)(list >> 32) ||
	    !(ufs_rd(REG_UTRLRSR) & 1)) {
		printf("UFS: host did not take our request list\n");
		return EIO;
	}
	return 0;
}

static int
mtk_ufs_open(__unused dev_t dev, int flags, __unused int devtype, __unused struct proc *p)
{
	if (!su.ready) {
		return ENXIO;
	}
	return ((flags & FWRITE) && !su.writable) ? EROFS : 0;
}

static int
mtk_ufs_close(__unused dev_t dev, __unused int flags, __unused int devtype, __unused struct proc *p)
{
	return 0;
}

static void
mtk_ufs_strategy(buf_t bp)
{
	int error = 0;
	caddr_t vaddr = NULL;
	uint64_t off = (uint64_t)buf_blkno(bp) * su.secsize;
	uint32_t count = buf_count(bp);
	uint64_t part_bytes = su.nblocks * UFS_BLOCK;

	bool rd = buf_flags(bp) & B_READ;

	buf_setresid(bp, count);
	if (!rd && !su.writable) {
		error = EROFS;
		goto done;
	}
	if (off > part_bytes || count > part_bytes - off) {
		error = EINVAL;
		goto done;
	}
	if (count == 0) {
		goto done;
	}
	if (buf_map(bp, &vaddr)) {
		error = EIO;
		goto done;
	}

	lck_mtx_lock(&mtk_ufs_lock);
	for (uint32_t done = 0; done < count;) {
		uint64_t pos = off + done;
		uint64_t lba = pos / UFS_BLOCK;
		uint32_t skip = (uint32_t)(pos % UFS_BLOCK);
		uint32_t want = MIN(count - done, BOUNCE_SIZE - skip);
		uint32_t blocks = (skip + want + UFS_BLOCK - 1) / UFS_BLOCK;
		void *bounce = (void *)(su.dma + DMA_BOUNCE);
		if (rd) {
			error = ufs_read_blocks(su.first_block + lba, blocks);
			if (!error) {
				memcpy(vaddr + done, (const char *)bounce + skip, want);
			}
		} else {
			/* Partial device blocks at either end: read, merge, write back. */
			if (skip || (want % UFS_BLOCK)) {
				error = ufs_read_blocks(su.first_block + lba, blocks);
			}
			if (!error) {
				memcpy((char *)bounce + skip, vaddr + done, want);
				error = ufs_write_blocks(su.first_block + lba, blocks);
			}
		}
		if (error) {
			break;
		}
		done += want;
		buf_setresid(bp, count - done);
	}
	lck_mtx_unlock(&mtk_ufs_lock);
	buf_unmap(bp);

done:
	if (error) {
		buf_seterror(bp, error);
	}
	buf_biodone(bp);
}

/*
 * The raw device (rufsroot0). Not physio(): with a user buffer buf_map() hands the strategy the *user* address and the
 * kernel's memcpy to it faults under PAN (panic "Kernel data abort", bootrec, 2026-11-06). Here the data goes
 * bounce -> kernel buffer under the lock, kernel buffer <-> user with uiomove() without it (a fault on the user
 * buffer may need this same disk). Offset and length: multiples of the logical sector size.
 */
#define RAW_CHUNK       (64u << 10)

static int
mtk_ufs_raw_rw(struct uio *uio, bool rd)
{
	if (!su.ready) {
		return ENXIO;
	}
	if (!rd && !su.writable) {
		return EROFS;
	}
	uint64_t part = su.nblocks * UFS_BLOCK;
	if (uio_offset(uio) < 0 || (uint64_t)uio_offset(uio) > part ||
	    uio_offset(uio) % su.secsize || uio_resid(uio) % su.secsize) {
		return EINVAL;
	}
	char *k = kalloc_data(RAW_CHUNK, Z_WAITOK);
	if (!k) {
		return ENOMEM;
	}
	int error = 0;
	while (!error && uio_resid(uio) > 0 && (uint64_t)uio_offset(uio) < part) {
		uint64_t pos = (uint64_t)uio_offset(uio);
		uint32_t skip = (uint32_t)(pos % UFS_BLOCK);
		uint32_t want = (uint32_t)MIN((uint64_t)uio_resid(uio), (uint64_t)(RAW_CHUNK - skip));
		want = (uint32_t)MIN((uint64_t)want, part - pos);
		uint32_t blocks = (skip + want + UFS_BLOCK - 1) / UFS_BLOCK;
		uint64_t lba = su.first_block + pos / UFS_BLOCK;
		char *bounce = (char *)(su.dma + DMA_BOUNCE);
		if (rd) {
			lck_mtx_lock(&mtk_ufs_lock);
			error = ufs_read_blocks(lba, blocks);
			if (!error) {
				memcpy(k, bounce + skip, want);
			}
			lck_mtx_unlock(&mtk_ufs_lock);
			if (!error) {
				error = uiomove(k, (int)want, uio);
			}
		} else {
			error = uiomove(k, (int)want, uio);
			if (!error) {
				lck_mtx_lock(&mtk_ufs_lock);
				if (skip || (want % UFS_BLOCK)) {
					error = ufs_read_blocks(lba, blocks);   /* partial device blocks: read, merge, write */
				}
				if (!error) {
					memcpy(bounce + skip, k, want);
					error = ufs_write_blocks(lba, blocks);
				}
				lck_mtx_unlock(&mtk_ufs_lock);
			}
		}
	}
	kfree_data(k, RAW_CHUNK);
	return error;
}

static int
mtk_ufs_ioctl(__unused dev_t dev, u_long cmd, caddr_t data, __unused int flag,
    __unused struct proc *p, int is_char)
{
	uint32_t *f = (uint32_t *)data;
	uint64_t *o = (uint64_t *)data;

	switch (cmd) {
	case DKIOCGETBLOCKSIZE:
		*f = su.secsize;
		return 0;
	case DKIOCSETBLOCKSIZE:
		if (is_char) {
			return ENODEV;
		}
		if (*f < DEV_BSIZE || *f > UFS_BLOCK || (*f & (*f - 1))) {
			return EINVAL;
		}
		su.secsize = *f;
		return 0;
	case DKIOCGETPHYSICALBLOCKSIZE:
		*f = UFS_BLOCK;
		return 0;
	case DKIOCGETBLOCKCOUNT:
		*o = su.nblocks * UFS_BLOCK / su.secsize;
		return 0;
	case DKIOCISWRITABLE:
		*f = su.writable;
		return 0;
	case DKIOCSYNCHRONIZE:
	case DKIOCSYNCHRONIZECACHE: {
		if (!su.writable) {
			return 0;
		}
		lck_mtx_lock(&mtk_ufs_lock);
		int error = ufs_sync_cache();
		lck_mtx_unlock(&mtk_ufs_lock);
		return error;
	}
	case DKIOCGETMAXBLOCKCOUNTREAD:
		*o = BOUNCE_SIZE / su.secsize;
		return 0;
	case DKIOCGETMAXBLOCKCOUNTWRITE:
		*o = su.writable ? BOUNCE_SIZE / su.secsize : 0;
		return 0;
	case DKIOCGETMAXSEGMENTCOUNTREAD:
	case DKIOCGETMAXSEGMENTCOUNTWRITE:
		*o = 1;
		return 0;
	default:
		return ENOTTY;
	}
}

static int
mtk_ufs_bioctl(dev_t dev, u_long cmd, caddr_t data, int flag, struct proc *p)
{
	return mtk_ufs_ioctl(dev, cmd, data, flag, p, 0);
}

static int
mtk_ufs_cioctl(dev_t dev, u_long cmd, caddr_t data, int flag, struct proc *p)
{
	return mtk_ufs_ioctl(dev, cmd, data, flag, p, 1);
}

static int
mtk_ufs_psize(__unused dev_t dev)
{
	return su.ready ? (int)MIN(su.nblocks * UFS_BLOCK / su.secsize, INT_MAX) : -1;
}

static int
mtk_ufs_read(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	return mtk_ufs_raw_rw(uio, true);
}

static int
mtk_ufs_write(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	return mtk_ufs_raw_rw(uio, false);
}

static const struct bdevsw mtk_ufs_bdevsw = {
	.d_open     = mtk_ufs_open,
	.d_close    = mtk_ufs_close,
	.d_strategy = mtk_ufs_strategy,
	.d_ioctl    = mtk_ufs_bioctl,
	.d_dump     = eno_dump,
	.d_psize    = mtk_ufs_psize,
	.d_type     = D_DISK,
};

static const struct cdevsw mtk_ufs_cdevsw = {
	.d_open       = mtk_ufs_open,
	.d_close      = mtk_ufs_close,
	.d_read       = mtk_ufs_read,
	.d_write      = mtk_ufs_write,
	.d_ioctl      = mtk_ufs_cioctl,
	.d_stop       = eno_stop,
	.d_reset      = eno_reset,
	.d_ttys       = NULL,
	.d_select     = eno_select,
	.d_mmap       = eno_mmap,
	.d_strategy   = eno_strat,
	.d_reserved_1 = eno_getc,
	.d_reserved_2 = eno_putc,
	.d_type       = D_DISK,
};

static int
mtk_ufs_init(void)
{
	DTEntry node;
	uintptr_t const *reg = NULL;
	unsigned int size = 0;
	char want[64] = "userdata";
	int error;

	if (SecureDTLookupEntry(NULL, "/mapple-ufs", &node) != kSuccess ||
	    SecureDTGetProperty(node, "reg", (void const **)&reg, &size) != kSuccess ||
	    size < 2 * sizeof(uintptr_t)) {
		printf("UFS: no /mapple-ufs node in the device tree\n");
		return ENXIO;
	}
	PE_parse_boot_argn_alias("mp_ufsroot", "ufsroot", want, sizeof(want));
	uint32_t rw = 0;
	PE_parse_boot_argn_alias("mp_ufsrw", "ufsrw", &rw, sizeof(rw));
	uint32_t cache = 0, wait = 0;
	PE_parse_boot_argn_alias("mp_ufscache", "ufscache", &cache, sizeof(cache));
	PE_parse_boot_argn_alias("mp_ufswait", "ufswait", &wait, sizeof(wait));
	su.cache = (cache == 1);
	su.wait = (wait == 1);

	su.regs = ml_io_map(reg[0], reg[1]);
	/* A host without 64-bit addressing gets DMA memory below 4 GiB. */
	ppnum_t max_pnum = (ufs_rd(REG_CAP) & CAP_64AS) ? 0 : (ppnum_t)(atop_64(1ULL << 32) - 1);
	if (kmem_alloc_contig(kernel_map, &su.dma, DMA_SIZE, PAGE_MASK, max_pnum, 0,
	    KMA_DATA | KMA_ZERO | KMA_PERMANENT, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("UFS: no contiguous DMA memory\n");
		return ENOMEM;
	}
	ppnum_t pn = pmap_find_phys(kernel_pmap, (addr64_t)su.dma);
	if (!pn) {
		return ENOMEM;
	}
	su.dma_pa = ptoa_64(pn);
	su.secsize = DEV_BSIZE;
	uint32_t lun_arg = UINT32_MAX;
	PE_parse_boot_argn_alias("mp_ufslun", "ufslun", &lun_arg, sizeof(lun_arg));
	uint32_t lun_first = lun_arg < 8 ? lun_arg : 0, lun_last = lun_arg < 8 ? lun_arg : 7;

	lck_mtx_lock(&mtk_ufs_lock);
	printf("UFS: host cap 0x%x, DMA at 0x%llx\n", ufs_rd(REG_CAP), su.dma_pa);
	error = ufs_take_over_host();
	for (uint32_t lun = lun_first; !error && lun <= lun_last; lun++) {
		static const uint8_t tur[6] = { 0x00 };
		su.lun = (uint8_t)lun;
		/* The first command after the bootloader may report a unit attention. */
		int lun_error = 0;
		for (int i = 0; i < 3 && (lun_error = ufs_scsi(tur, sizeof(tur), 0, false)); i++) {
		}
		if (!lun_error) {
			lun_error = ufs_find_partition(want);
		}
		printf("UFS: LUN %u: %s (%d)\n", lun, lun_error ? "no" : "found", lun_error);
		if (!lun_error) {
			break;
		}
		if (lun == lun_last) {
			error = ENXIO;
		}
	}
	lck_mtx_unlock(&mtk_ufs_lock);
	if (error) {
		return error;
	}

	su.bmajor = bdevsw_add(-1, &mtk_ufs_bdevsw);
	if (su.bmajor < 0) {
		return ENXIO;
	}
	su.cmajor = cdevsw_add_with_bdev(-1, &mtk_ufs_cdevsw, su.bmajor);
	if (su.cmajor < 0) {
		return ENXIO;
	}
	su.bdev = makedev(su.bmajor, 0);
	devfs_make_node(su.bdev, DEVFS_BLOCK, UID_ROOT, GID_OPERATOR, 0640, "ufsroot0");
	devfs_make_node(makedev(su.cmajor, 0), DEVFS_CHAR, UID_ROOT, GID_OPERATOR, 0640, "rufsroot0");
	/* Only now, with the partition bounds known, can writes be allowed. */
	su.writable = (rw != 0);
	su.ready = true;
	printf("UFS: \"%s\" on LUN %u: first block 0x%llx, %llu MiB, %s, DMA at 0x%llx\n",
	    want, su.lun, su.first_block, su.nblocks * UFS_BLOCK >> 20,
	    su.writable ? "read-write" : "read-only", su.dma_pa);
	mapple_log_start();
	proc_sampler_start();
	return 0;
}

/*
 * Persistent console log ("mp_conlog"). Filming the phone screen is the only
 * other way to see the console, so every character that reaches the
 * console (kernel printf and /dev/console writes of daemons, both through
 * _cnputs) is also kept in a 4 MiB ring, and once the partition is known
 * and writable a thread copies new ring blocks to the LAST 4 MiB + 4 KiB of
 * the partition every mp_ufslogsecs seconds (default 10), far behind the HFS
 * image at its start. Device block 0 of that area is a header:
 *   "MAPPLECL" | u32 version 1 | u32 ring size | u64 total bytes written
 *   | u64 flush sequence
 * ring offset of byte n = n % ring size. Read it from a recovery with
 * tools/sweet-log-pull.sh. Disabled with boot-arg mp_conlog=0.
 */
#define MAPPLE_LOG_SIZE          (4u << 20)
#define MAPPLE_LOG_BLOCKS        (1 + MAPPLE_LOG_SIZE / UFS_BLOCK)
#define MAPPLE_LOG_SECS          10u

static char mapple_log_ring[MAPPLE_LOG_SIZE];
static uint64_t _Atomic mapple_log_total;
static int mapple_log_chan;
static uint32_t mapple_log_secs = MAPPLE_LOG_SECS;

/* Called for all console output, any context: never blocks. */
void mapple_log_append(const char *c, int size);

void
mapple_log_append(const char *c, int size)
{
	if (size <= 0) {
		return;
	}
	uint64_t at = os_atomic_add_orig(&mapple_log_total, (uint64_t)size, relaxed);
	for (int i = 0; i < size; i++) {
		mapple_log_ring[(at + (uint64_t)i) % MAPPLE_LOG_SIZE] = c[i];
	}
}

/*
 * Copy console bytes [*pos, total) (at most max) for live log streaming
 * (mapple_usb.c). A reader that fell more than the ring behind skips ahead.
 * Returns the byte count; *pos advances.
 */
uint32_t mapple_log_copy(uint64_t *pos, char *out, uint32_t max);

uint32_t
mapple_log_copy(uint64_t *pos, char *out, uint32_t max)
{
	uint64_t total = os_atomic_load(&mapple_log_total, relaxed);
	if (total - *pos > MAPPLE_LOG_SIZE) {
		*pos = total - MAPPLE_LOG_SIZE;
	}
	uint32_t n = (uint32_t)MIN((uint64_t)max, total - *pos);
	for (uint32_t i = 0; i < n; i++) {
		out[i] = mapple_log_ring[(*pos + i) % MAPPLE_LOG_SIZE];
	}
	*pos += n;
	return n;
}

uint64_t mapple_log_position(void);

uint64_t
mapple_log_position(void)
{
	return os_atomic_load(&mapple_log_total, relaxed);
}

struct mapple_log_header {
	char magic[8];
	uint32_t version, size;
	uint64_t total, seq;
};

static void
mapple_log_thread(__unused void *arg, __unused wait_result_t wr)
{
	uint64_t base = su.first_block + su.nblocks - MAPPLE_LOG_BLOCKS;
	uint64_t flushed = 0, seq = 0;
	void *bounce = (void *)(su.dma + DMA_BOUNCE);

	printf("LOG: console log to the last %u KiB of the partition (block 0x%llx)\n",
	    MAPPLE_LOG_BLOCKS * UFS_BLOCK >> 10, base);
	for (;;) {
		(void)tsleep(&mapple_log_chan, PZERO, "mp_conlog", hz * (int)mapple_log_secs);
		uint64_t total = os_atomic_load(&mapple_log_total, relaxed);
		if (total == flushed) {
			continue;
		}
		uint64_t pos = total - flushed >= MAPPLE_LOG_SIZE ? total - MAPPLE_LOG_SIZE : flushed;
		int error = 0;

		lck_mtx_lock(&mtk_ufs_lock);
		while (pos < total && !error) {
			uint32_t off = (uint32_t)(pos % MAPPLE_LOG_SIZE);
			uint32_t blk = off / UFS_BLOCK;
			uint64_t need = ((uint64_t)off % UFS_BLOCK + (total - pos) + UFS_BLOCK - 1) / UFS_BLOCK;
			uint32_t n = (uint32_t)MIN(MIN(need, (uint64_t)(MAPPLE_LOG_SIZE / UFS_BLOCK - blk)),
			    (uint64_t)(BOUNCE_SIZE / UFS_BLOCK));
			memcpy(bounce, mapple_log_ring + (uint64_t)blk * UFS_BLOCK, n * UFS_BLOCK);
			error = ufs_write_blocks(base + 1 + blk, n);
			pos += (uint64_t)(blk + n) * UFS_BLOCK - off;
		}
		if (!error) {
			struct mapple_log_header h = {
				.magic = { 'M', 'A', 'P', 'P', 'L', 'E', 'C', 'L' },
				.version = 1, .size = MAPPLE_LOG_SIZE, .total = total, .seq = ++seq,
			};
			bzero(bounce, UFS_BLOCK);
			memcpy(bounce, &h, sizeof(h));
			error = ufs_write_blocks(base, 1);
		}
		if (!error) {
			error = ufs_sync_cache();
		}
		lck_mtx_unlock(&mtk_ufs_lock);
		if (error) {
			printf("LOG: write failed (%d), stopping\n", error);
			return;
		}
		flushed = total;
	}
}

static void
mapple_log_start(void)
{
	uint32_t on = 1;
	thread_t thread;

	PE_parse_boot_argn_alias("mp_conlog", "sweetlog", &on, sizeof(on));
	if (!on || !su.writable || su.nblocks < 2 * MAPPLE_LOG_BLOCKS) {
		return;
	}
	PE_parse_boot_argn_alias("mp_ufslogsecs", "ufslogsecs", &mapple_log_secs, sizeof(mapple_log_secs));
	if (mapple_log_secs == 0 || mapple_log_secs > 60) {
		mapple_log_secs = MAPPLE_LOG_SECS;
	}
	if (kernel_thread_start(mapple_log_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

/* Root device for rd=ufsroot0 (IOFindBSDRoot). */
int mtk_ufs_rootdev(dev_t *dev);

int
mtk_ufs_rootdev(dev_t *dev)
{
	static bool usblink_done;
	if (!usblink_done) {
		/* before storage, so a UFS failure can still be read over USB */
		usblink_done = true;
		usblink_probe();
	}
	if (!su.ready) {
		int error = mtk_ufs_init();
		if (error) {
			return error;
		}
	}
	*dev = su.bdev;
	return 0;
}
#endif /* ARM64_BOARD_CONFIG_MT6781 */
