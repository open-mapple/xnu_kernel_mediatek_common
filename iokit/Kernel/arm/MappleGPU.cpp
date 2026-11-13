/*
 * MT6781: Mali-G57 MC2 driver (see MappleGPU.h).
 *
 * MFG power, from Xiaomi viva-r-oss (gpufreq_v1/mt6781/mtk_gpufreq_plat.c,
 * clk-mt6781-pg.c, clk-mt6781.c, mfd/mt6358/registers.h); sequences and
 * values only, the code is ours:
 *   rails     MT6366 VSRAM_GPU (LDO, EN 0x1bc8.0, VOSEL 0x1bf4[6:0]) and
 *             VGPU (buck, EN 0x1508.0, VOSEL 0x1526[6:0]), 0.5 V + 6.25 mV
 *             per step, VSRAM on first and >= VGPU + 0.1 V (0.85 V floor);
 *   MFGPLL    apmixed 0x250/0x254/0x25c: power, isolation off, PCW, enable;
 *   mfg_sel   topckgen CLK_CFG_0[17:16] = 1 (mfgpll), parked on 3 (mainpll_d5)
 *             while the PLL changes, latched by CLK_CFG_UPDATE bit 2;
 *   MTCMOS    SPM MFG0..MFG3 (PWR_CON 0x308 + 4n, status bit 2 + n), MFG1
 *             with the infracfg_ao bus protection;
 *   mfgcfg    BG3D clock gate, async/global/Q-channel setup, timestamp.
 * One operating point, chosen by boot-arg mp_gpu_freq (MHz, default 300;
 * vendor table up to 950) or "gpu freq MHz" on the USB link, VGPU raised by
 * mp_gpu_mv (default 50 mV, capped at 0.9 V); no DVFS yet.
 *
 * Mali (job-manager Valhall), written from the register map of the
 * architecture; behaviour as Panfrost expects it from Linux:
 *   one AArch64 4K address space per connection (48-bit VA, leaves of
 *   4 KiB, four per XNU page), bound to a hardware AS on demand (LRU);
 *   slot 0 fragment, slot 1 vertex/tiler/compute, one chain per slot on
 *   the hardware, FIFO per slot, a chain starts when its input fences have
 *   signalled (a queue head waiting only for chains on the hardware is
 *   started by the job interrupt's filter, K0e); GPU caches cleaned and
 *   invalidated around every chain;
 *   tiler heaps grow by 2 MiB on translation faults; any other fault fails
 *   the chain and disables its address space (never a panic); a chain that
 *   runs longer than 3 s is hard-stopped, a slot that ignores the stop
 *   resets the GPU.
 */
#include <pexpert/arm64/board_config.h>

#if defined(MT6781)
#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>
#include <libkern/OSAtomic.h>
#include <pexpert/pexpert.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>
#include <sys/proc.h>

#include "MappleGPU.h"
#include "MT6358PMIC.h"
#include "MappleSurface.h"

extern "C" void mt6781_cpufreq_boost_stats(int *on, uint64_t *starts, uint64_t *ms, uint64_t *signals);  /* 0183 */

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

/* ---- Mali registers ---- */

#define GPU_ID                  0x000
#define GPU_L2_FEATURES         0x004
#define GPU_CORE_FEATURES       0x008
#define GPU_TILER_FEATURES      0x00c
#define GPU_MEM_FEATURES        0x010
#define GPU_MMU_FEATURES        0x014
#define GPU_AS_PRESENT          0x018
#define GPU_JS_PRESENT          0x01c
#define GPU_IRQ_RAWSTAT         0x020
#define GPU_IRQ_CLEAR           0x024
#define GPU_IRQ_MASK            0x028
#define GPU_IRQ_STATUS          0x02c
#define GPU_COMMAND             0x030
#define GPU_STATUS              0x034
#define GPU_LATEST_FLUSH        0x038   /* flush ID: counts the GPU's cache flushes */
#define GPU_FAULT_STATUS        0x03c
#define GPU_FAULT_ADDRESS_LO    0x040
#define GPU_FAULT_ADDRESS_HI    0x044
#define GPU_AFBC_FEATURES       0x04c
#define GPU_TIMESTAMP_LO        0x098
#define GPU_TIMESTAMP_HI        0x09c
#define GPU_THREAD_MAX_THREADS  0x0a0
#define GPU_THREAD_MAX_WG       0x0a4
#define GPU_THREAD_MAX_BARRIER  0x0a8
#define GPU_THREAD_FEATURES     0x0ac
#define GPU_TEXTURE_FEATURES(n) (0x0b0 + 4 * (n))
#define GPU_JS_FEATURES(n)      (0x0c0 + 4 * (n))
#define GPU_SHADER_PRESENT_LO   0x100
#define GPU_SHADER_PRESENT_HI   0x104
#define GPU_TILER_PRESENT_LO    0x110
#define GPU_L2_PRESENT_LO       0x120
#define GPU_SHADER_READY_LO     0x140
#define GPU_TILER_READY_LO      0x150
#define GPU_L2_READY_LO         0x160
#define GPU_SHADER_PWRON_LO     0x180
#define GPU_SHADER_PWRON_HI     0x184
#define GPU_TILER_PWRON_LO      0x190
#define GPU_L2_PWRON_LO         0x1a0
#define GPU_SHADER_PWROFF_LO    0x1c0
#define GPU_SHADER_PWROFF_HI    0x1c4
#define GPU_TILER_PWROFF_LO     0x1d0
#define GPU_L2_PWROFF_LO        0x1e0
#define GPU_SHADER_PWRTRANS_LO  0x200
#define GPU_L2_PWRTRANS_LO      0x220
#define GPU_COHERENCY_FEATURES  0x300
#define GPU_COHERENCY_ENABLE    0x304
#define GPU_THREAD_TLS_ALLOC    0x310
#define GPU_STACK_PRESENT_LO    0xe00
#define GPU_STACK_READY_LO      0xe10
#define GPU_STACK_PWRON_LO      0xe20
#define GPU_STACK_PWROFF_LO     0xe30
#define GPU_JM_CONFIG           0xf00
#define GPU_SHADER_CONFIG       0xf04

#define GPU_IRQ_FAULT           (1u << 0)
#define GPU_IRQ_MULTIPLE_FAULT  (1u << 7)
#define GPU_IRQ_RESET_COMPLETED (1u << 8)
#define GPU_CMD_SOFT_RESET      0x01
#define GPU_CMD_HARD_RESET      0x02
#define COHERENCY_NONE          31
#define SC_VAR_ALGORITHM        (1u << 29)
#define JM_IDVS_GROUP_SIZE(n)   ((uint32_t)(n) << 16)

#define JOB_INT_RAWSTAT         0x1000
#define JOB_INT_CLEAR           0x1004
#define JOB_INT_MASK            0x1008
#define JOB_INT_STAT            0x100c
#define JS_BASE(n)              (0x1800 + 0x80 * (n))
#define JS_HEAD_LO              0x00
#define JS_HEAD_HI              0x04
#define JS_TAIL_LO              0x08
#define JS_CONFIG               0x18
#define JS_COMMAND              0x20
#define JS_STATUS               0x24
#define JS_HEAD_NEXT_LO         0x40
#define JS_HEAD_NEXT_HI         0x44
#define JS_AFFINITY_NEXT_LO     0x50
#define JS_AFFINITY_NEXT_HI     0x54
#define JS_CONFIG_NEXT          0x58
#define JS_COMMAND_NEXT         0x60
#define JS_FLUSH_ID_NEXT        0x70    /* start flush skipped if a flush happened after this ID */
#define JS_CMD_NOP              0
#define JS_CMD_START            1
#define JS_CMD_HARD_STOP        3
#define JS_CFG_START_FLUSH_CLEAN_INV    (3u << 8)
#define JS_CFG_END_FLUSH_CLEAN_INV      (3u << 12)
#define JS_CFG_THREAD_PRI(n)            ((uint32_t)(n) << 16)

#define MMU_INT_RAWSTAT         0x2000
#define MMU_INT_CLEAR           0x2004
#define MMU_INT_MASK            0x2008
#define MMU_INT_STAT            0x200c
#define AS_BASE(n)              (0x2400 + 0x40 * (n))
#define AS_TRANSTAB_LO          0x00
#define AS_TRANSTAB_HI          0x04
#define AS_MEMATTR_LO           0x08
#define AS_MEMATTR_HI           0x0c
#define AS_LOCKADDR_LO          0x10
#define AS_LOCKADDR_HI          0x14
#define AS_COMMAND              0x18
#define AS_FAULTSTATUS          0x1c
#define AS_FAULTADDRESS_LO      0x20
#define AS_FAULTADDRESS_HI      0x24
#define AS_STATUS               0x28
#define AS_TRANSCFG_LO          0x30
#define AS_TRANSCFG_HI          0x34
#define AS_CMD_UPDATE           1
#define AS_CMD_LOCK             2
#define AS_CMD_FLUSH_PT         4
#define AS_CMD_FLUSH_MEM        5
#define AS_STATUS_ACTIVE        (1u << 0)
#define TRANSCFG_ADRMODE_UNMAPPED       1u
#define TRANSCFG_ADRMODE_AARCH64_4K     6u
#define TRANSCFG_INA_BITS(n)            ((uint32_t)(n) << 6)
#define TRANSCFG_PTW_MEMATTR_WB         (2u << 24)
#define TRANSCFG_PTW_RA                 (1u << 30)
/* MEMATTR byte 0: inner/outer write-back, read+write allocate, GPU-internal shareable (not CPU coherent);
 * the others non-cacheable. */
#define MEMATTR_LO              0x4c4c4c8fu
#define MEMATTR_HI              0x4c4c4c4cu

/* AArch64 stage-1 descriptors, 4 KiB granule */
#define PTE_TABLE               3ull
#define PTE_PAGE                3ull
#define PTE_AP_UNPRIV           (1ull << 6)
#define PTE_SH_IS               (3ull << 8)
#define PTE_AF                  (1ull << 10)
#define PTE_NG                  (1ull << 11)
#define PTE_XN                  (3ull << 53)
#define PTE_ADDR                0x0000fffffffff000ull
#define GPU_PAGE                4096ull

#define JOB_TIMEOUT_MS          3000
#define JOB_STOP_GRACE_MS       500
#define JOB_TIMEOUT_AFTER_FAULT_MS 500  /* fragment chains after a fault without reset (TTRX_3076 watch) */
#define TIMER_MS                250
#define DVFS_WINDOW_MS          50      /* governor sampling period */
#define DVFS_UP_PCT             85      /* busier than this: fastest OPP */
#define DVFS_TARGET_PCT         70      /* lower: the OPP that would be this busy */
#define DVFS_HOLD_MS            200     /* no step down this soon after a step up */
#define HEAP_CHUNK              (2ull << 20)
#define GPU_MEMORY_LIMIT        (768ll << 20)  /* default of boot-arg mp_gpu_memtotal (MiB) */
#define GPU_PROCESS_LIMIT       (384ll << 20)  /* default of boot-arg mp_gpu_mem (MiB, per process) */

/* ---- MFG (MediaTek) ---- */

/* PMIC (MT6366, MT6358 layout) */
#define PMIC_VGPU_EN            0x1508  /* bit 0 */
#define PMIC_VGPU_VOSEL         0x1526  /* [6:0] */
#define PMIC_VGPU_DA_VOSEL      0x151e  /* [6:0] */
#define PMIC_VGPU_DA_EN         0x1520  /* bit 0 */
#define PMIC_VSRAM_EN           0x1bc8  /* bit 0 */
#define PMIC_VSRAM_VOSEL        0x1bf4  /* [6:0] */
#define PMIC_VSRAM_DA_VOSEL     0x1be2  /* [14:8] */
#define PMIC_VSRAM_DA_EN        0x1be4  /* bit 0 */
#define UV(sel)                 (500000u + 6250u * (sel))

/* SPM 0x10006000 */
#define SPM_POWERON_CONFIG_EN   0x000
#define SPM_PROJECT_KEY         0x0b160001u
#define SPM_PWR_STATUS          0x16c
#define SPM_PWR_STATUS_2ND      0x170
#define SPM_MFG_PWR_CON(n)      (0x308 + 4 * (n))
#define PWR_RST_B               (1u << 0)
#define PWR_ISO                 (1u << 1)
#define PWR_ON                  (1u << 2)
#define PWR_ON_2ND              (1u << 3)
#define PWR_CLK_DIS             (1u << 4)
#define PWR_SRAM_PDN            (1u << 8)
#define PWR_SRAM_PDN_ACK        (1u << 12)
#define MFG_STATUS_BIT(n)       (1u << (2 + (n)))

/* infracfg_ao 0x10001000: bus protection for MFG1 */
#define INFRA_PROT_SET          0x2a0
#define INFRA_PROT_CLR          0x2a4
#define INFRA_PROT_STA1         0x228
#define INFRA_PROT1_SET         0x2a8
#define INFRA_PROT1_CLR         0x2ac
#define INFRA_PROT1_STA1        0x258
#define INFRA_ACP_CON           0x290   /* bit 9: ACP off (gpufreq probe) */

/* topckgen 0x10000000, apmixed 0x1000c000 */
#define CK_CFG_UPDATE           0x004
#define CK_CFG_UPDATE_MFG       (1u << 2)
#define CK_CFG_0                0x040
#define CK_CFG_0_SET            0x044
#define CK_CFG_0_CLR            0x048
#define CK_MFG_SEL_SHIFT        16
#define CK_MFG_SEL_MASK         (3u << 16)
#define CK_MFG_PDN              (1u << 23)
#define CK_MFG_MFGPLL           1u
#define CK_MFG_MAINPLL_D5       3u
#define PLL_MFG_CON0            0x250   /* bit 0 EN */
#define PLL_MFG_CON1            0x254
#define PLL_MFG_PWR_CON0        0x25c   /* bit 0 PWR_ON, bit 1 ISO */

/* mfgcfg 0x13fbf000 (inside the MFG power domain) */
#define MFG_CG_CON              0x000
#define MFG_CG_SET              0x004
#define MFG_CG_CLR              0x008
#define MFG_CG_BG3D             (1u << 0)
#define MFG_ASYNC_CON           0x020
#define MFG_ASYNC_CON_1         0x024
#define MFG_GLOBAL_CON          0x0b0
#define MFG_QCHANNEL_CON        0x0b4
#define MFG_TIMESTAMP           0x130
#define MFG_BUS_IDLE_CON        0x170
#define MFG_BUS_IDLE_STA        0x178
#define MFG_I2M_PROTECTOR       0xf60

enum { kMapMali, kMapMfgcfg, kMapSpm, kMapTopck, kMapApmixed, kMapInfra };

static MappleGPUAccelerator *gMappleGPU;
static uint32_t oppForMHz(uint32_t mhz);
static volatile uint32_t gMappleGPUCapMHz;        /* mapple_gpu_set_dvfs_cap (Low Power Mode, BQ27Z561Battery.cpp); 0 = none */

static inline uint32_t rd32(vm_offset_t b, uint32_t o) { return *(volatile uint32_t *)(b + o); }
static inline void wr32(vm_offset_t b, uint32_t o, uint32_t v) { *(volatile uint32_t *)(b + o) = v; }
static inline void set32(vm_offset_t b, uint32_t o, uint32_t v) { wr32(b, o, rd32(b, o) | v); }
static inline void clr32(vm_offset_t b, uint32_t o, uint32_t v) { wr32(b, o, rd32(b, o) & ~v); }

/* Poll `cond` every microsecond for up to `us`. */
template <typename F>
static bool
pollUs(uint32_t us, F cond)
{
	for (uint32_t i = 0; i < us; i++) {
		if (cond()) {
			return true;
		}
		IODelay(1);
	}
	return cond();
}

/* Text for the USB link reply (and nothing if out is NULL). */
static void say(char **p, char *end, const char *fmt, ...) __printflike(3, 4);

static void
say(char **p, char *end, const char *fmt, ...)
{
	if (p == NULL || *p == NULL || *p >= end - 1) {
		return;
	}
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(*p, (size_t)(end - *p), fmt, ap);
	va_end(ap);
	if (n > 0) {
		*p += MIN((size_t)n, (size_t)(end - *p - 1));
	}
}

static uint64_t
nowMs(void)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / NSEC_PER_MSEC;
}

uint64_t
mapple_gpu_monotonic_ns(void)
{
	clock_sec_t sec, bsec;
	clock_usec_t usec, busec;
	clock_get_calendar_microtime(&sec, &usec);
	clock_get_boottime_microtime(&bsec, &busec);
	int64_t us = ((int64_t)sec - (int64_t)bsec) * USEC_PER_SEC + ((int64_t)usec - (int64_t)busec);
	return us > 0 ? (uint64_t)us * NSEC_PER_USEC : 0;
}

/*
 * CPU cache maintenance (to PoC) by kernel virtual address, always through a kernel mapping of the buffer.
 * (The "illegal PA 0x6860c000" panics that led here were 0x16860c000 truncated to 32 bits by XNU's
 * pmap_sync_page_attributes_phys, fixed in 0112; kernel mappings stay the simpler path anyway.)
 */
static void
cacheVA(const void *p, uint64_t len, int op)
{
	uintptr_t end = (uintptr_t)p + len;
	for (uintptr_t line = (uintptr_t)p & ~(uintptr_t)63; line < end; line += 64) {
		if (op == kMappleCacheFlush) {
			__asm__ volatile ("dc civac, %0" : : "r"(line) : "memory");
		} else {
			__asm__ volatile ("dc cvac, %0" : : "r"(line) : "memory");
		}
	}
	__asm__ volatile ("dsb sy" : : : "memory");
}

/* Page-table entries written through the kernel's cacheable mapping (the GPU's table walk is not coherent). */
static void
cleanVA(const void *p, size_t len)
{
	cacheVA(p, len, kMappleCacheClean);
}

void
cacheBuffer(IOBufferMemoryDescriptor *md, uint64_t offset, uint64_t len, int op)
{
	cacheVA((const uint8_t *)md->getBytesNoCopy() + offset, len, op);
}

static const char *
jsStatusName(uint32_t s)
{
	switch (s) {
	case 0x01: return "done";
	case 0x03: return "stopped";
	case 0x04: return "terminated";
	case 0x40: return "job config fault";
	case 0x41: return "power fault";
	case 0x42: return "job read fault";
	case 0x43: return "job write fault";
	case 0x44: return "affinity fault";
	case 0x48: return "job bus fault";
	case 0x50: return "instr invalid pc";
	case 0x51: return "instr invalid enc";
	case 0x58: return "data invalid fault";
	case 0x59: return "tile range fault";
	case 0x5a: return "address range fault";
	case 0x60: return "out of memory";
	case 0x80: return "delayed bus fault";
	case 0x88: return "shareability fault";
	default:
		if ((s & 0xf8) == 0xc0) {
			return "translation fault";
		}
		if ((s & 0xf8) == 0xc8) {
			return "permission fault";
		}
		if ((s & 0xf8) == 0xd8) {
			return "access flag fault";
		}
		return "?";
	}
}

/* ---- IOAcceleratorES, fences ---- */

OSDefineMetaClassAndStructors(IOAcceleratorES, IOService);
OSDefineMetaClassAndStructors(MappleGPUFence, OSObject);

MappleGPUFence *
MappleGPUFence::create(bool isSignalled)
{
	MappleGPUFence *f = new MappleGPUFence;
	if (f != NULL && !f->init()) {
		f->release();
		return NULL;
	}
	if (f != NULL) {
		f->signalled = isSignalled;
		f->error = kIOReturnSuccess;
		f->signalAt = 0;
		f->waiters = 0;
		f->promise = false;
		f->owner = NULL;
		f->deadline = 0;
		f->followers = NULL;
		f->deps = NULL;
		f->k0jEpoch = 0;
		f->k0jVM = NULL;
		f->k0jDep = false;
		f->k0jWalk = 0;
		f->k0jCollect = 0;
	}
	return f;
}

void
MappleGPUFence::free(void)
{
	OSSafeReleaseNULL(followers);
	OSSafeReleaseNULL(deps);
	OSObject::free();
}

/* ---- jobs ---- */

struct MappleGPUJob {
	MappleGPUJob   *next;
	MappleGPUVM    *vm;
	uint64_t      jc;
	uint32_t      requirements;
	OSArray      *deps;     /* MappleGPUFence */
	OSArray      *bos;      /* MappleGPUBO */
	MappleGPUFence *fence;
	uint64_t      startMs;
	uint32_t      flushId;  /* GPU_LATEST_FLUSH at submit (flush reduction) */
	uint64_t      submitAt, startAt;        /* mach_absolute_time (latency accounting) */
	uint64_t      seq;      /* K0h: number in its connection */
	MappleGPUJob   *vmPrev, *vmNext;          /* the connection's unfinished jobs, in seq order */
};

/* ---- address spaces ---- */

OSDefineMetaClassAndStructors(MappleGPUVM, OSObject);

extern "C" void mapple_task_graphics_footprint(task_t task, int64_t bytes);   /* osfmk/kern/task.c */

MappleGPUVM *
MappleGPUVM::create(MappleGPUAccelerator *g)
{
	MappleGPUVM *vm = new MappleGPUVM;
	if (vm == NULL || !vm->init()) {
		OSSafeReleaseNULL(vm);
		return NULL;
	}
	vm->gpu = g;
	vm->asSlot = -1;
	vm->pid = proc_selfpid();
	proc_selfname(vm->proc, sizeof(vm->proc));
	g->vmLink(vm);
	vm->fPool = OSArray::withCapacity(4);
	vm->fRangeCap = 64;
	vm->fRanges = IONewZeroData(Range, vm->fRangeCap);
	/* K0h completion page: one CPU page, mapped read-only by the process (VGPU_MAP_COMPLETION) */
	vm->completionMD = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,
	    kIODirectionInOut | kIOMemoryKernelUserShared, page_size, page_size);
	if (vm->completionMD != NULL) {
		bzero(vm->completionMD->getBytesNoCopy(), page_size);
		vm->completion = (struct vgpu_completion *)vm->completionMD->getBytesNoCopy();
	}
	if (vm->fPool == NULL || vm->fRanges == NULL || vm->completionMD == NULL || (vm->rootPA = vm->newTable()) == 0) {
		vm->release();
		return NULL;
	}
	return vm;
}

/* Gated: the completion page from seqNext/the oldest unfinished job (64-bit fields first, then the 32-bit ones). */
void
MappleGPUVM::seqUpdate(void)
{
	seqDone = jobHead != NULL ? jobHead->seq - 1 : seqNext;
	if (completion == NULL) {
		return;
	}
	completion->completed64 = seqDone;
	completion->submitted64 = seqNext;
	__atomic_store_n(&completion->submitted, (uint32_t)seqNext, __ATOMIC_RELEASE);
	__atomic_store_n(&completion->completed, (uint32_t)seqDone, __ATOMIC_RELEASE);
}

void
MappleGPUVM::free(void)
{
	if (gpu != NULL) {
		gpu->vmDestroyed(this);         /* frees its AS, unlinks it */
	}
	if (fPool != NULL) {
		gpu->chargeMemory(this, -(int64_t)(fPool->getCount() * page_size));
	}
	OSSafeReleaseNULL(fPool);
	if (task != TASK_NULL) {
		mapple_task_graphics_footprint(task, -charged);
		task_deallocate(task);
	}
	if (fRanges != NULL) {
		IODeleteData(fRanges, Range, fRangeCap);
	}
	if (inv != NULL) {
		IODelete(inv, MappleGPUBO *, invCap);
	}
	OSSafeReleaseNULL(completionMD);
	OSObject::free();
}

/* Gated. Without memory for the list the BO stays unlisted (invPending false): the caller invalidates at once. */
void
MappleGPUVM::invAdd(MappleGPUBO *bo)
{
	if (bo->invPending) {
		return;
	}
	if (invCount == invCap) {
		uint32_t cap = invCap ? invCap * 2 : 64;
		MappleGPUBO **bigger = IONew(MappleGPUBO *, cap);
		if (bigger == NULL) {
			return;
		}
		if (inv != NULL) {
			bcopy(inv, bigger, invCount * sizeof(*inv));
			IODelete(inv, MappleGPUBO *, invCap);
		}
		inv = bigger;
		invCap = cap;
	}
	inv[invCount++] = bo;
	bo->invPending = true;
}

/* Gated. */
void
MappleGPUVM::invRemove(MappleGPUBO *bo)
{
	for (uint32_t i = 0; i < invCount; i++) {
		if (inv[i] == bo) {
			inv[i] = inv[--invCount];
			break;
		}
	}
	bo->invPending = false;
}

/* A zeroed 4 KiB table (physical address), carved from 16 KiB wired pages. */
uint64_t
MappleGPUVM::newTable(void)
{
	if (fFreeLeft == 0) {
		IOBufferMemoryDescriptor *page = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
		    kIODirectionInOut, page_size, 0x000000fffffff000ull & ~(uint64_t)(page_size - 1));
		if (page == NULL) {
			return 0;
		}
		IOByteCount len = 0;
		uint64_t pa = page->getPhysicalSegment(0, &len, kIOMemoryMapperNone);
		if (pa == 0 || len < page_size) {
			page->release();
			return 0;
		}
		if (fPoolCount == kPoolMax || !gpu->chargeMemory(this, page_size)) {
			page->release();
			return 0;
		}
		bzero(page->getBytesNoCopy(), page_size);
		cleanVA(page->getBytesNoCopy(), page_size);
		fPoolPA[fPoolCount] = pa;
		fPoolKVA[fPoolCount] = (uint8_t *)page->getBytesNoCopy();
		fPoolCount++;
		fPool->setObject(page);
		page->release();
		fFreePA = pa;
		fFreeLeft = (uint32_t)(page_size / GPU_PAGE);
	}
	uint64_t pa = fFreePA;
	fFreePA += GPU_PAGE;
	fFreeLeft--;
	return pa;
}

/* Kernel address of a table (physical address) from the pool. */
uint64_t *
MappleGPUVM::tableKVA(uint64_t pa)
{
	uint64_t base = pa & ~(uint64_t)(page_size - 1);
	for (uint32_t i = 0; i < fPoolCount; i++) {
		if (fPoolPA[i] == base) {
			return (uint64_t *)(fPoolKVA[i] + (pa - base));
		}
	}
	panic("GPU: page table 0x%llx not in the pool", pa);
}

/* The level 3 table for `va`, created on the way if `create`. */
uint64_t *
MappleGPUVM::tableFor(uint64_t va, bool create)
{
	uint64_t pa = rootPA;
	for (int level = 0; level < 3; level++) {
		uint64_t *t = tableKVA(pa);
		uint32_t idx = (uint32_t)(va >> (39 - 9 * level)) & 511;
		uint64_t e = t[idx];
		if ((e & 3) != PTE_TABLE) {
			if (!create) {
				return NULL;
			}
			uint64_t next = newTable();
			if (next == 0) {
				return NULL;
			}
			t[idx] = next | PTE_TABLE;
			cleanVA(&t[idx], 8);
			e = t[idx];
		}
		pa = e & PTE_ADDR;
	}
	return tableKVA(pa);
}

uint64_t
MappleGPUVM::paOf(uint64_t va)
{
	uint64_t *l3 = tableFor(va, false);
	uint64_t e = l3 != NULL ? l3[(uint32_t)(va >> 12) & 511] : 0;
	return (e & 3) == PTE_PAGE ? (e & PTE_ADDR) | (va & 0xfff) : 0;
}

bool
MappleGPUVM::map(uint64_t va, IOMemoryDescriptor *md, uint64_t offset, uint64_t size, uint64_t pte)
{
	uint64_t done = 0;
	while (done < size) {
		IOByteCount seg = 0;
		addr64_t pa = md->getPhysicalSegment(offset + done, &seg, kIOMemoryMapperNone);
		if (pa == 0 || (pa & (GPU_PAGE - 1))) {
			unmap(va, done);
			return false;
		}
		/* a descriptor may end inside its last page (physical framebuffer): map the whole page */
		seg = (MIN((uint64_t)seg, size - done) + GPU_PAGE - 1) & ~(GPU_PAGE - 1);
		for (uint64_t off = 0; off < seg;) {
			uint64_t cur = va + done + off;
			uint64_t *l3 = tableFor(cur, true);
			if (l3 == NULL) {
				unmap(va, done + off);
				return false;
			}
			uint32_t first = (uint32_t)(cur >> 12) & 511, n = 0;
			while (off < seg && first + n < 512) {
				l3[first + n] = ((pa + off) & PTE_ADDR) | pte | PTE_PAGE;
				n++;
				off += GPU_PAGE;
			}
			cleanVA(&l3[first], n * sizeof(uint64_t));
		}
		done += seg;
	}
	__asm__ volatile ("dsb sy" : : : "memory");
	gpu->vmFlush(this, va, size);
	return true;
}

void
MappleGPUVM::unmap(uint64_t va, uint64_t size)
{
	for (uint64_t off = 0; off < size; off += GPU_PAGE) {
		uint64_t *l3 = tableFor(va + off, false);
		if (l3 == NULL) {
			continue;
		}
		uint64_t *e = &l3[(uint32_t)((va + off) >> 12) & 511];
		*e = 0;
		cleanVA(e, sizeof(*e));
	}
	__asm__ volatile ("dsb sy" : : : "memory");
	if (size > 0) {
		gpu->vmFlush(this, va, size);
	}
}

uint64_t
MappleGPUVM::leafAttrs(bool exec)
{
	return PTE_AP_UNPRIV | PTE_SH_IS | PTE_AF | PTE_NG | (exec ? 0 : PTE_XN);
}

uint64_t
MappleGPUVM::allocVA(uint64_t size, uint64_t align)
{
	if (size == 0 || size > VGPU_VA_END - VGPU_VA_START) {
		return 0;
	}
	if (fRangeCount == fRangeCap) {
		Range *bigger = IONewZeroData(Range, fRangeCap * 2);
		if (bigger == NULL) {
			return 0;
		}
		bcopy(fRanges, bigger, fRangeCount * sizeof(Range));
		IODeleteData(fRanges, Range, fRangeCap);
		fRanges = bigger;
		fRangeCap *= 2;
	}
	uint64_t cursor = VGPU_VA_START;
	uint32_t i;
	for (i = 0; i <= fRangeCount; i++) {
		uint64_t start = (cursor + align - 1) & ~(align - 1);
		uint64_t limit = i < fRangeCount ? fRanges[i].start : VGPU_VA_END;
		if (start + size <= limit && start + size > start) {
			bcopy(&fRanges[i], &fRanges[i + 1], (fRangeCount - i) * sizeof(Range));
			fRanges[i].start = start;
			fRanges[i].end = start + size;
			fRangeCount++;
			return start;
		}
		if (i < fRangeCount) {
			cursor = fRanges[i].end;
		}
	}
	return 0;
}

void
MappleGPUVM::freeVA(uint64_t va)
{
	for (uint32_t i = 0; i < fRangeCount; i++) {
		if (fRanges[i].start == va) {
			Freed *f = &fFreed[fFreedNext++ % kFreedMax];
			f->start = fRanges[i].start;
			f->end = fRanges[i].end;
			f->ms = nowMs();
			bcopy(&fRanges[i + 1], &fRanges[i], (fRangeCount - i - 1) * sizeof(Range));
			fRangeCount--;
			return;
		}
	}
}

void
MappleGPUVM::describe(uint64_t addr, char *buf, size_t len)
{
	for (uint32_t i = 0; i < fRangeCount; i++) {
		if (addr >= fRanges[i].start && addr < fRanges[i].end) {
			uint64_t *l3 = tableFor(addr, false);
			snprintf(buf, len, "inside live VA [0x%llx, 0x%llx), PTE 0x%llx", fRanges[i].start, fRanges[i].end,
			    l3 != NULL ? l3[(uint32_t)(addr >> 12) & 511] : 0);
			return;
		}
	}
	uint64_t now = nowMs();
	for (uint32_t n = 0; n < kFreedMax && n < fFreedNext; n++) {
		const Freed *f = &fFreed[(fFreedNext - 1 - n) % kFreedMax];
		if (addr >= f->start && addr < f->end) {
			snprintf(buf, len, "in VA [0x%llx, 0x%llx) freed %llu ms ago", f->start, f->end, now - f->ms);
			return;
		}
	}
	if (addr < VGPU_VA_START || addr >= VGPU_VA_END) {
		snprintf(buf, len, "outside the VA range (bad pointer)");
	} else {
		snprintf(buf, len, "not allocated (none of the last %u frees)", MIN(fFreedNext, (uint32_t)kFreedMax));
	}
}

/* ---- buffer objects ---- */

OSDefineMetaClassAndStructors(MappleGPUBO, OSObject);

void
MappleGPUBO::free(void)
{
	if (vm != NULL && va != 0) {
		vm->unmap(va, vaSize);
		vm->freeVA(va);
	}
	if (kind == kMappleBOHeap && vm != NULL) {
		vm->gpu->heapRemove(this);
	}
	if (invPending && vm != NULL) {
		vm->invRemove(this);
	}
	if (chunks != NULL) {
		for (uint32_t i = 0; i < chunkCount; i++) {
			if (chunks[i] != NULL) {
				chunks[i]->release();
				vm->gpu->chargeMemory(vm, -(int64_t)HEAP_CHUNK);
			}
		}
		IODelete(chunks, IOBufferMemoryDescriptor *, chunkCount);
	}
	if (memory != NULL) {
		if (prepared) {
			memory->complete();
			if (kind == kMappleBOUser && vm != NULL) {
				vm->userWired -= (int64_t)vaSize;
				vm->userCompletes++;
				vm->gpu->fUserWired -= (int64_t)vaSize;
				vm->gpu->fUserCompletes++;
			} else if (kind == kMappleBOSurface && vm != NULL) {
				vm->surfPrepared -= (int64_t)vaSize;
				vm->surfCompletes++;
				vm->gpu->fSurfPrepared -= (int64_t)vaSize;
				vm->gpu->fSurfCompletes++;
			}
		}
		if (kind == kMappleBONormal && vm != NULL) {
			vm->gpu->chargeMemory(vm, -(int64_t)vaSize);
		}
		if ((kind == kMappleBOSurface || kind == kMappleBOUser) && vm != NULL) {
			vm->imported -= (int64_t)vaSize;
		}
		memory->release();
	}
	OSSafeReleaseNULL(kmap);
	OSSafeReleaseNULL(mapMD);
	OSSafeReleaseNULL(surface);
	OSSafeReleaseNULL(busy);
	if (vm != NULL) {
		vm->bos--;
	}
	OSSafeReleaseNULL(vm);
	OSObject::free();
}

bool
MappleGPUBO::growHeap(uint64_t addr)
{
	if (kind != kMappleBOHeap || addr < va || addr >= va + vaSize) {
		return false;
	}
	uint32_t idx = (uint32_t)((addr - va) / HEAP_CHUNK);
	if (idx >= chunkCount) {
		return false;
	}
	if (chunks[idx] != NULL) {
		return true;            /* another unit faulted on it before the flush: already mapped */
	}
	if (!vm->gpu->chargeMemory(vm, HEAP_CHUNK)) {
		return false;
	}
	uint64_t len = MIN(HEAP_CHUNK, vaSize - (uint64_t)idx * HEAP_CHUNK);
	IOBufferMemoryDescriptor *c = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,
	    kIODirectionInOut, len, page_size);         /* cacheable: see createBO; GPU-only afterwards */
	if (c == NULL) {
		vm->gpu->chargeMemory(vm, -(int64_t)HEAP_CHUNK);
		return false;
	}
	cacheBuffer(c, 0, len, kMappleCacheFlush);
	if (!vm->map(va + (uint64_t)idx * HEAP_CHUNK, c, 0, len, PTE_AP_UNPRIV | PTE_SH_IS | PTE_AF | PTE_NG | PTE_XN)) {
		c->release();
		vm->gpu->chargeMemory(vm, -(int64_t)HEAP_CHUNK);
		return false;
	}
	chunks[idx] = c;
	return true;
}

uint64_t
MappleGPUBO::cacheOp(uint64_t offset, uint64_t len, int op)
{
	if (memory == NULL) {
		return 0;
	}
	if (len == 0 || offset + len > size) {
		len = offset < size ? size - offset : 0;
	}
	IOBufferMemoryDescriptor *buf = OSDynamicCast(IOBufferMemoryDescriptor, memory);
	if (buf != NULL) {
		cacheBuffer(buf, memOffset + offset, len, op);
		return len;
	}
	if (kmap == NULL && (kmap = memory->map()) == NULL) {        /* user memory, physical surfaces */
		return 0;
	}
	cacheVA((const uint8_t *)kmap->getVirtualAddress() + memOffset + offset, len, op);
	return len;
}

/* ---- accelerator ---- */

OSDefineMetaClassAndStructors(MappleGPUAccelerator, IOAcceleratorES);

IOWorkLoop *
MappleGPUAccelerator::getWorkLoop(void) const
{
	return fWorkLoop;
}

#define LK_VGPU_SEL     0x40    /* 900 mV, as LK leaves VGPU on the realme 8i */
#define LK_VSRAM_SEL    0x50    /* 1000 mV */

bool
MappleGPUAccelerator::start(IOService *provider)
{
	int mode = 0;
	if (!PE_parse_boot_argn_alias("mp_gpu", "vivagpu", &mode, sizeof(mode)) || mode <= 0) {
		IOLog("GPU: off (boot-arg mp_gpu=1 for bring-up, 2 to power at boot and publish)\n");
		return false;
	}
	fMode = mode;
	uint32_t mhz = 300;
	PE_parse_boot_argn_alias("mp_gpu_freq", "vivagpufreq", &mhz, sizeof(mhz));
	fOpp = fClockOpp = oppForMHz(mhz);
	fMarginMv = 50;
	PE_parse_boot_argn_alias("mp_gpu_mv", "vivagpumv", &fMarginMv, sizeof(fMarginMv));
	fMarginMv = MIN(MAX(fMarginMv, 0), 100);
	fCacheMode = 1;
	PE_parse_boot_argn_alias("mp_gpu_cache", "vivagpucache", &fCacheMode, sizeof(fCacheMode));
	int flag = 1;
	PE_parse_boot_argn_alias("mp_gpu_lazy", "vivagpulazy", &flag, sizeof(flag));
	fLazy = flag != 0 && fCacheMode != 0;
	flag = 1;
	PE_parse_boot_argn_alias("mp_gpu_wc", "vivagpuwc", &flag, sizeof(flag));
	fWC = flag != 0 && fCacheMode != 0;
	flag = 1;
	PE_parse_boot_argn_alias("mp_gpu_flushid", "vivagpuflushid", &flag, sizeof(flag));
	fFlushReduce = flag != 0;
	flag = 1;
	PE_parse_boot_argn_alias("mp_gpu_dvfs", "vivagpudvfs", &flag, sizeof(flag));
	fDvfs = flag != 0;
	flag = 1;
	PE_parse_boot_argn_alias("mp_gpu_boost", "vivagpuboost", &flag, sizeof(flag));
	fDvfsBoost = flag != 0;
	flag = 1;
	PE_parse_boot_argn_alias("mp_gpu_fastkick", "vivagpufastkick", &flag, sizeof(flag));
	fFastKick = flag != 0;
	uint32_t maxMhz = 950;
	PE_parse_boot_argn_alias("mp_gpu_freqmax", "vivagpufreqmax", &maxMhz, sizeof(maxMhz));
	fDvfsFast = oppForMHz(maxMhz);
	if (fDvfsFast > fOpp) {
		fDvfsFast = fOpp;       /* never below the floor (mp_gpu_freq) */
	}
	flag = 0;
	PE_parse_boot_argn_alias("mp_gpu_lkrails", "vivagpulkrails", &flag, sizeof(flag));
	fLkRails = flag != 0;
	if (fLkRails) {                 /* LK's 300 MHz at LK's 0.9 V / 1.0 V: no PMIC write anywhere below */
		fOpp = fClockOpp = fDvfsFast = oppForMHz(300);
		fDvfs = false;
	}
	flag = 1;
	PE_parse_boot_argn("mp_gpu_footprint", &flag, sizeof(flag));
	fFootprint = flag != 0;
	flag = JOB_TIMEOUT_MS;
	PE_parse_boot_argn("mp_gpu_jobms", &flag, sizeof(flag));
	fJobTimeoutMs = flag < 1000 ? 1000 : flag > 30000 ? 30000 : flag;
	int64_t mib = GPU_MEMORY_LIMIT >> 20;
	PE_parse_boot_argn_alias("mp_gpu_memtotal", "vivagpumemtotal", &mib, sizeof(mib));
	fMemLimit = mib << 20;
	mib = GPU_PROCESS_LIMIT >> 20;
	PE_parse_boot_argn_alias("mp_gpu_mem", "vivagpumem", &mib, sizeof(mib));
	fProcLimit = mib << 20;
	fProbeMode = 1;
	PE_parse_boot_argn_alias("mp_gpu_probeclient", "vivagpuprobe", &fProbeMode, sizeof(fProbeMode));
	fProbeLock = IOLockAlloc();
	fFastLock = IOSimpleLockAlloc();
	fCallLock = IOSimpleLockAlloc();
	fWorkLoop = IOWorkLoop::workLoop();
	if (fWorkLoop == NULL || fProbeLock == NULL || fFastLock == NULL || fCallLock == NULL ||
	    !IOAcceleratorES::start(provider)) {
		return false;
	}
	for (int i = 0; i < 6; i++) {
		fMaps[i] = provider->mapDeviceMemoryWithIndex(i);
		if (fMaps[i] == NULL) {
			IOLog("GPU: cannot map register range %d\n", i);
			return false;
		}
	}
	fMali = (vm_offset_t)fMaps[kMapMali]->getVirtualAddress();
	fMfgcfg = (vm_offset_t)fMaps[kMapMfgcfg]->getVirtualAddress();
	fSpm = (vm_offset_t)fMaps[kMapSpm]->getVirtualAddress();
	fTopck = (vm_offset_t)fMaps[kMapTopck]->getVirtualAddress();
	fApmixed = (vm_offset_t)fMaps[kMapApmixed]->getVirtualAddress();
	fInfra = (vm_offset_t)fMaps[kMapInfra]->getVirtualAddress();

	fPwrap = MTKPwrap::wait();
	if (fPwrap == NULL) {
		IOLog("GPU: no PMIC wrapper\n");
		return false;
	}
	fGate = IOCommandGate::commandGate(this);
	fTimer = IOTimerEventSource::timerEventSource(this, &MappleGPUAccelerator::jobTimer);
	fDvfsTimer = IOTimerEventSource::timerEventSource(this, &MappleGPUAccelerator::dvfsTimer);
	IOInterruptEventSource::Action actions[3] = {
		&MappleGPUAccelerator::gpuInterrupt, &MappleGPUAccelerator::mmuInterrupt, &MappleGPUAccelerator::jobInterrupt,
	};
	if (fGate == NULL || fTimer == NULL || fDvfsTimer == NULL || fWorkLoop->addEventSource(fGate) != kIOReturnSuccess ||
	    fWorkLoop->addEventSource(fTimer) != kIOReturnSuccess || fWorkLoop->addEventSource(fDvfsTimer) != kIOReturnSuccess) {
		return false;
	}
	for (int i = 0; i < 3; i++) {
		/* the job interrupt has a filter: K0e fast kick and the interrupt timestamps */
		if (i == 2) {
			fIrq[i] = IOFilterInterruptEventSource::filterInterruptEventSource(this, actions[i],
			    &MappleGPUAccelerator::jobFilter, provider, i);
		} else {
			fIrq[i] = IOInterruptEventSource::interruptEventSource(this, actions[i], provider, i);
		}
		if (fIrq[i] == NULL || fWorkLoop->addEventSource(fIrq[i]) != kIOReturnSuccess) {
			IOLog("GPU: no interrupt %d\n", i);
			return false;
		}
	}
	setProperty("MetalPluginName", "MappleMTLDriver");
	setProperty("MetalPluginClassName", "VMDevice");
	gMappleGPU = this;
	IOLog("GPU: driver started (mp_gpu=%d)\n", fMode);

	if (fMode >= 2) {
		char buf[1024];
		__block char *p = buf;
		char *end = buf + sizeof(buf);
		buf[0] = 0;
		IOReturn ret = fGate->runActionBlock(^IOReturn {
			return powerOn(&p, end);
		});
		IOLog("%s", buf);
		if (ret == kIOReturnSuccess) {
			fPublished = true;
			registerService();
			IOLog("GPU: accelerator published (MetalPluginName MappleMTLDriver)\n");
		}
	}
	return true;
}

void
MappleGPUAccelerator::free(void)
{
	for (int i = 0; i < 3; i++) {
		if (fIrq[i] != NULL) {
			fIrq[i]->disable();
			fWorkLoop->removeEventSource(fIrq[i]);
			fIrq[i]->release();
		}
	}
	if (fTimer != NULL) {
		fTimer->cancelTimeout();
		fWorkLoop->removeEventSource(fTimer);
		fTimer->release();
	}
	if (fDvfsTimer != NULL) {
		fDvfsTimer->cancelTimeout();
		fWorkLoop->removeEventSource(fDvfsTimer);
		fDvfsTimer->release();
	}
	if (fGate != NULL) {
		fWorkLoop->removeEventSource(fGate);
		fGate->release();
	}
	for (int i = 0; i < 6; i++) {
		OSSafeReleaseNULL(fMaps[i]);
	}
	if (fHeapList != NULL) {
		IODelete(fHeapList, MappleGPUBO *, fHeapCap);
	}
	if (fFastLock != NULL) {
		IOSimpleLockFree(fFastLock);
		fFastLock = NULL;
	}
	if (fCallLock != NULL) {
		IOSimpleLockFree(fCallLock);
		fCallLock = NULL;
	}
	OSSafeReleaseNULL(fPromises);
	if (fK0jStack != NULL) {
		IODelete(fK0jStack, K0jFrame, kMappleK0jDepth);
	}
	if (fProbeLock != NULL) {
		IOLockFree(fProbeLock);
	}
	OSSafeReleaseNULL(fWorkLoop);
	IOAcceleratorES::free();
}

IOReturn
MappleGPUAccelerator::newUserClient(task_t owningTask, void *securityID, UInt32 type, IOUserClient **handler)
{
	if (type != VGPU_CLIENT_TYPE) {
		char proc[20];
		proc_selfname(proc, sizeof(proc));
		probeNote(proc, type, ~0u, 0, 0);
		IOLog("GPU: %s[%d] opens user client type 0x%x (not vgpu): %s\n", proc, proc_selfpid(), type,
		    fProbeMode ? "probe client" : "refused");
		if (!fProbeMode) {
			return kIOReturnUnsupported;
		}
		MappleGPUProbeClient *probe = new MappleGPUProbeClient;
		if (probe == NULL) {
			return kIOReturnNoMemory;
		}
		if (!probe->initWithTask(owningTask, securityID, type) || !probe->attach(this)) {
			probe->release();
			return kIOReturnError;
		}
		if (!probe->start(this)) {
			probe->detach(this);
			probe->release();
			return kIOReturnError;
		}
		*handler = probe;
		return kIOReturnSuccess;
	}
	if (!fPowered) {
		return kIOReturnNotReady;
	}
	MappleGPUUserClient *client = new MappleGPUUserClient;
	if (client == NULL) {
		return kIOReturnNoMemory;
	}
	if (!client->initWithTask(owningTask, securityID, type) || !client->attach(this)) {
		client->release();
		return kIOReturnError;
	}
	if (!client->start(this)) {
		client->detach(this);
		client->release();
		return kIOReturnError;
	}
	*handler = client;
	return kIOReturnSuccess;
}

/* Gated. */
void
MappleGPUAccelerator::vmLink(MappleGPUVM *vm)
{
	vm->nextVM = fVMs;
	fVMs = vm;
}

/*
 * Gated. Limits: all GPU memory (mp_gpu_memtotal, MiB, default 2048) and per process, summed over its connections
 * (mp_gpu_mem, MiB, default 1024). Refusals end as kIOReturnNoMemory for CREATE_BO, a failed map, or a heap that
 * cannot grow (the chain fails), never a panic.
 */
bool
MappleGPUAccelerator::chargeMemory(MappleGPUVM *vm, int64_t bytes)
{
	if (bytes > 0) {
		int64_t proc = 0;
		for (MappleGPUVM *v = fVMs; v != NULL; v = v->nextVM) {
			if (v->pid == vm->pid) {
				proc += v->bytes;
			}
		}
		if (fMemory + bytes > fMemLimit || proc + bytes > fProcLimit) {
			if (fMemRefused++ < 16) {
				IOLog("GPU: %s[%d] out of GPU memory: +%lld KiB refused (process %lld/%lld MiB, all %lld/%lld "
				    "MiB)\n", vm->proc, vm->pid, bytes >> 10, proc >> 20, fProcLimit >> 20, fMemory >> 20,
				    fMemLimit >> 20);
			}
			return false;
		}
	}
	fMemory += bytes;
	vm->bytes += bytes;
	if (vm->task != TASK_NULL && (fFootprint || bytes < 0)) {
		int64_t d = bytes > 0 ? bytes : -MIN(-bytes, vm->charged);
		mapple_task_graphics_footprint(vm->task, d);
		vm->charged += d;
	}
	if (fMemory > fMemPeak) {
		fMemPeak = fMemory;
	}
	return true;
}

/* GPU memory per process (connections of one pid summed). */
void
MappleGPUAccelerator::memoryDump(char **out, char *end)
{
	say(out, end, "memory: %lld KiB (peak %lld KiB, limit %lld MiB, %lld MiB per process), %llu refusals, in the processes' "
	    "footprints %s\n", fMemory >> 10, fMemPeak >> 10, fMemLimit >> 20, fProcLimit >> 20, fMemRefused,
	    fFootprint ? "(mp_gpu_footprint=1)" : "no (mp_gpu_footprint=0)");
	say(out, end, "wired for the GPU: user memory %lld KiB (%llu prepares, %llu completes), IOSurfaces %lld KiB (%llu "
	    "prepares, %llu completes)\n", fUserWired >> 10, fUserPrepares, fUserCompletes, fSurfPrepared >> 10, fSurfPrepares,
	    fSurfCompletes);
	objectCounts(out, end, NULL);
	for (MappleGPUVM *v = fVMs; v != NULL; v = v->nextVM) {
		bool seen = false;
		for (MappleGPUVM *w = fVMs; w != v; w = w->nextVM) {
			seen = seen || w->pid == v->pid;
		}
		if (seen) {
			continue;
		}
		int64_t bytes = 0, imported = 0, charged = 0, user = 0, surf = 0;
		uint64_t up = 0, uc = 0, sp = 0, sc = 0;
		uint32_t conns = 0, bos = 0, faults = 0;
		for (MappleGPUVM *w = v; w != NULL; w = w->nextVM) {
			if (w->pid == v->pid) {
				bytes += w->bytes;
				imported += w->imported;
				charged += w->charged;
				user += w->userWired;
				surf += w->surfPrepared;
				up += w->userPrepares;
				uc += w->userCompletes;
				sp += w->surfPrepares;
				sc += w->surfCompletes;
				bos += w->bos;
				faults += w->faults;
				conns++;
			}
		}
		say(out, end, "  %s[%d]: %lld KiB in %u BOs (%lld KiB in its footprint), %lld KiB imported, %u connection%s, "
		    "%u MMU faults; wired: user %lld KiB (%llu/%llu), IOSurfaces %lld KiB (%llu/%llu)\n", v->proc, v->pid,
		    bytes >> 10, bos, charged >> 10, imported >> 10, conns, conns == 1 ? "" : "s", faults, user >> 10, up, uc,
		    surf >> 10, sp, sc);
	}
}

/* ---- unknown user clients and property reads (K1: what IOGPU/QuartzCore/Metal ask for) ---- */

#define PROBE_LOG_MAX   48      /* log lines per client; "gpu probe" keeps counting */
/* selector tags in the probe table (high 16 bits) for calls that are not external methods */
#define PROBE_TRAP      0x74720000u
#define PROBE_MEMORY    0x6d650000u
#define PROBE_NOTIFY    0x6e6f0000u

/* One table line per (process, type, selector); selector ~0 = the open itself. */
void
MappleGPUAccelerator::probeNote(const char *proc, uint32_t type, uint32_t selector, uint32_t inSize, uint32_t outSize)
{
	IOLockLock(fProbeLock);
	uint32_t i;
	for (i = 0; i < fProbeCount; i++) {
		ProbeEntry *e = &fProbe[i];
		if (e->type == type && e->selector == selector && strncmp(e->proc, proc, sizeof(e->proc)) == 0) {
			break;
		}
	}
	if (i == fProbeCount) {
		if (fProbeCount == kProbeMax) {
			fProbeDropped++;
			IOLockUnlock(fProbeLock);
			return;
		}
		ProbeEntry *e = &fProbe[fProbeCount++];
		bzero(e, sizeof(*e));
		strlcpy(e->proc, proc, sizeof(e->proc));
		e->type = type;
		e->selector = selector;
	}
	fProbe[i].count++;
	fProbe[i].inSize = inSize;
	fProbe[i].outSize = outSize;
	IOLockUnlock(fProbeLock);
}

void
MappleGPUAccelerator::propertyRead(const char *key, bool found)
{
	char proc[20];
	proc_selfname(proc, sizeof(proc));
	IOLockLock(fProbeLock);
	uint32_t i;
	for (i = 0; i < fPropCount; i++) {
		if (strncmp(fProps[i].key, key, sizeof(fProps[i].key) - 1) == 0 &&
		    strncmp(fProps[i].proc, proc, sizeof(fProps[i].proc)) == 0) {
			break;
		}
	}
	bool first = i == fPropCount;
	if (first) {
		if (fPropCount == kPropMax) {
			fPropDropped++;
			IOLockUnlock(fProbeLock);
			return;
		}
		bzero(&fProps[i], sizeof(fProps[i]));
		strlcpy(fProps[i].proc, proc, sizeof(fProps[i].proc));
		strlcpy(fProps[i].key, key, sizeof(fProps[i].key));
		fPropCount++;
	}
	fProps[i].count++;
	fProps[i].found = found;
	IOLockUnlock(fProbeLock);
	if (first) {
		IOLog("GPU: %s[%d] reads property \"%s\": %s\n", proc, proc_selfpid(), key, found ? "found" : "MISSING");
	}
}

void
MappleGPUAccelerator::probeDump(char **out, char *end)
{
	IOLockLock(fProbeLock);
	say(out, end, "unknown user clients (%u calls dropped, mp_gpu_probeclient=%d):\n", fProbeDropped, fProbeMode);
	for (uint32_t i = 0; i < fProbeCount; i++) {
		ProbeEntry *e = &fProbe[i];
		uint32_t tag = e->selector >> 16, n = e->selector & 0xffff;
		if (e->selector == ~0u) {
			say(out, end, "  %s type 0x%x: opened %u times\n", e->proc, e->type, e->count);
		} else if (tag == (PROBE_NOTIFY >> 16) && n >= 0xfffe) {
			say(out, end, "  %s type 0x%x %s: %u calls\n", e->proc, e->type, n == 0xffff ? "setProperties" :
			    "connectClient", e->count);
		} else {
			say(out, end, "  %s type 0x%x %s %u: %u calls, last in %u out %u bytes\n", e->proc, e->type,
			    tag == (PROBE_TRAP >> 16) ? "trap" : tag == (PROBE_MEMORY >> 16) ? "memory type" :
			    tag == (PROBE_NOTIFY >> 16) ? "notification port type" : "sel", n, e->count, e->inSize,
			    e->outSize);
		}
	}
	say(out, end, "property reads (%u dropped):\n", fPropDropped);
	for (uint32_t i = 0; i < fPropCount; i++) {
		say(out, end, "  %s \"%s\" %s x%u\n", fProps[i].proc, fProps[i].key, fProps[i].found ? "found" : "MISSING",
		    fProps[i].count);
	}
	IOLockUnlock(fProbeLock);
}

/* IOUserClient.cpp: registry property reads by userspace (key "*" = all properties). */
extern "C" void
mapple_gpu_property_read(IORegistryEntry *entry, const char *key, bool found)
{
	MappleGPUAccelerator *gpu = gMappleGPU;
	if (gpu != NULL && entry == gpu) {
		gpu->propertyRead(key, found);
	}
}

/* ---- probe user client ---- */

OSDefineMetaClassAndStructors(MappleGPUProbeClient, IOUserClient);

bool
MappleGPUProbeClient::initWithTask(task_t owningTask, void *securityID, UInt32 type)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type)) {
		return false;
	}
	fType = type;
	fPid = proc_selfpid();
	proc_selfname(fProc, sizeof(fProc));
	return true;
}

bool
MappleGPUProbeClient::start(IOService *provider)
{
	fGPU = OSDynamicCast(MappleGPUAccelerator, provider);
	return fGPU != NULL && IOUserClient::start(provider);
}

void
MappleGPUProbeClient::note(const char *what, uint32_t selector, uint32_t inSize, uint32_t outSize, const char *detail)
{
	fGPU->probeNote(fProc, fType, selector, inSize, outSize);
	if (fLogged < PROBE_LOG_MAX) {
		fLogged++;
		IOLog("GPU: probe %s[%d] type 0x%x %s %u%s -> unsupported\n", fProc, fPid, fType, what,
		    selector & 0xffff, detail);
	}
}

IOReturn
MappleGPUProbeClient::clientClose(void)
{
	if (fLogged < PROBE_LOG_MAX) {
		IOLog("GPU: probe %s[%d] type 0x%x closed\n", fProc, fPid, fType);
	}
	terminate();
	return kIOReturnSuccess;
}

IOReturn
MappleGPUProbeClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    __unused IOExternalMethodDispatch *dispatch, __unused OSObject *target, __unused void *reference)
{
	char detail[160];
	char *p = detail, *end = detail + sizeof(detail);
	uint32_t inSize = args->structureInputDescriptor ? (uint32_t)args->structureInputDescriptor->getLength() :
	    args->structureInputSize;
	uint32_t outSize = args->structureOutputDescriptor ? (uint32_t)args->structureOutputDescriptor->getLength() :
	    args->structureOutputSize;
	detail[0] = 0;
	say(&p, end, ": scalars in %u", args->scalarInputCount);
	for (uint32_t i = 0; i < args->scalarInputCount && i < 4; i++) {
		say(&p, end, "%s0x%llx", i ? " " : " (", args->scalarInput[i]);
	}
	if (args->scalarInputCount) {
		say(&p, end, "%s)", args->scalarInputCount > 4 ? " .." : "");
	}
	say(&p, end, ", struct in %u%s", inSize, args->structureInputDescriptor ? " (ool)" : "");
	if (args->structureInput != NULL && args->structureInputSize > 0) {
		say(&p, end, " [");
		for (uint32_t i = 0; i < args->structureInputSize && i < 16; i++) {
			say(&p, end, "%02x", ((const uint8_t *)args->structureInput)[i]);
		}
		say(&p, end, "%s]", args->structureInputSize > 16 ? ".." : "");
	}
	say(&p, end, ", out scalars %u struct %u%s", args->scalarOutputCount, outSize,
	    args->asyncWakePort != MACH_PORT_NULL ? ", async" : "");
	note("sel", selector, inSize, outSize, detail);
	return kIOReturnUnsupported;
}

IOReturn
MappleGPUProbeClient::clientMemoryForType(UInt32 type, __unused IOOptionBits *options, __unused IOMemoryDescriptor **memory)
{
	note("memory type", PROBE_MEMORY | (type & 0xffff), 0, 0, "");
	return kIOReturnUnsupported;
}

IOReturn
MappleGPUProbeClient::registerNotificationPort(mach_port_t port, UInt32 type, io_user_reference_t refCon)
{
	note("notification port type", PROBE_NOTIFY | (type & 0xffff), 0, 0, "");
	return IOUserClient::registerNotificationPort(port, type, refCon);
}

IOExternalTrap *
MappleGPUProbeClient::getTargetAndTrapForIndex(__unused IOService **targetP, UInt32 index)
{
	note("trap", PROBE_TRAP | (index & 0xffff), 0, 0, "");
	return NULL;
}

IOReturn
MappleGPUProbeClient::setProperties(OSObject *properties)
{
	char detail[200];
	OSSerialize *s = OSSerialize::withCapacity(256);
	snprintf(detail, sizeof(detail), ": %.180s", (s != NULL && properties != NULL && properties->serialize(s)) ?
	    s->text() : "?");
	OSSafeReleaseNULL(s);
	note("setProperties", PROBE_NOTIFY | 0xffff, 0, 0, detail);
	return kIOReturnUnsupported;
}

IOReturn
MappleGPUProbeClient::connectClient(IOUserClient *client)
{
	char detail[64];
	snprintf(detail, sizeof(detail), ": %s", client ? client->getMetaClass()->getClassName() : "?");
	note("connectClient", PROBE_NOTIFY | 0xfffe, 0, 0, detail);
	return kIOReturnUnsupported;
}

/* ---- MFG power ---- */

/* MFG1 bus protection steps (infracfg_ao), set in this order on power-off, released in reverse. */
static const struct { uint32_t set, clr, sta, mask; } gMfgProt[4] = {
	{ INFRA_PROT1_SET, INFRA_PROT1_CLR, INFRA_PROT1_STA1, 0x18000000 },
	{ INFRA_PROT_SET, INFRA_PROT_CLR, INFRA_PROT_STA1, 0x00600000 },
	{ INFRA_PROT_SET, INFRA_PROT_CLR, INFRA_PROT_STA1, 0x02000000 },
	{ INFRA_PROT1_SET, INFRA_PROT1_CLR, INFRA_PROT1_STA1, 0x20000000 },
};

void
MappleGPUAccelerator::protRelease(int from)
{
	for (int i = from; i >= 0; i--) {
		wr32(fInfra, gMfgProt[i].clr, gMfgProt[i].mask);
	}
}

bool
MappleGPUAccelerator::mtcmosOn(uint32_t n, char **out, char *end)
{
	uint32_t con = SPM_MFG_PWR_CON(n), bit = MFG_STATUS_BIT(n);
	if ((rd32(fSpm, SPM_PWR_STATUS) & bit) && (rd32(fSpm, SPM_PWR_STATUS_2ND) & bit)) {
		if (n == 1) {
			protRelease();  /* a power-off that gave up may have left it set */
		}
		say(out, end, "  MFG%u already on (pwr_con 0x%08x)\n", n, rd32(fSpm, con));
		return true;
	}
	set32(fSpm, con, PWR_ON);
	set32(fSpm, con, PWR_ON_2ND);
	if (!pollUs(1000, [&]{ return (rd32(fSpm, SPM_PWR_STATUS) & bit) && (rd32(fSpm, SPM_PWR_STATUS_2ND) & bit); })) {
		say(out, end, "  MFG%u: no power ack (pwr_con 0x%08x status 0x%08x/0x%08x)\n", n, rd32(fSpm, con),
		    rd32(fSpm, SPM_PWR_STATUS), rd32(fSpm, SPM_PWR_STATUS_2ND));
		return false;
	}
	clr32(fSpm, con, PWR_CLK_DIS);
	clr32(fSpm, con, PWR_ISO);
	set32(fSpm, con, PWR_RST_B);
	clr32(fSpm, con, PWR_SRAM_PDN);
	if (!pollUs(1000, [&]{ return !(rd32(fSpm, con) & PWR_SRAM_PDN_ACK); })) {
		say(out, end, "  MFG%u: no SRAM ack (pwr_con 0x%08x)\n", n, rd32(fSpm, con));
		return false;
	}
	if (n == 1) {
		protRelease();
	}
	say(out, end, "  MFG%u on (pwr_con 0x%08x)\n", n, rd32(fSpm, con));
	return true;
}

/* false: the domain was left on (MFG1 whose bus did not drain). */
bool
MappleGPUAccelerator::mtcmosOff(uint32_t n, char **out, char *end)
{
	uint32_t con = SPM_MFG_PWR_CON(n), bit = MFG_STATUS_BIT(n);
	if (!(rd32(fSpm, SPM_PWR_STATUS) & bit) && !(rd32(fSpm, SPM_PWR_STATUS_2ND) & bit)) {
		return true;
	}
	if (n == 1) {
		for (int i = 0; i < 4; i++) {
			wr32(fInfra, gMfgProt[i].set, gMfgProt[i].mask);
			if (!pollUs(1000, [&]{ return (rd32(fInfra, gMfgProt[i].sta) & gMfgProt[i].mask) == gMfgProt[i].mask; })) {
				/* the bus did not drain: powering off now could hang it; leave MFG1 on and usable */
				say(out, end, "  MFG1: bus protection step %d not acknowledged (sta 0x%08x), left on\n", i + 1,
				    rd32(fInfra, gMfgProt[i].sta));
				protRelease(i);
				return false;
			}
		}
	}
	set32(fSpm, con, PWR_SRAM_PDN);
	if (!pollUs(1000, [&]{ return (rd32(fSpm, con) & PWR_SRAM_PDN_ACK) != 0; })) {
		say(out, end, "  MFG%u: no SRAM power-down ack\n", n);
	}
	set32(fSpm, con, PWR_ISO);
	set32(fSpm, con, PWR_CLK_DIS);
	clr32(fSpm, con, PWR_RST_B);
	clr32(fSpm, con, PWR_ON);
	clr32(fSpm, con, PWR_ON_2ND);
	if (!pollUs(1000, [&]{ return !(rd32(fSpm, SPM_PWR_STATUS) & bit) && !(rd32(fSpm, SPM_PWR_STATUS_2ND) & bit); })) {
		say(out, end, "  MFG%u: still on after power off\n", n);
	}
	return true;
}

/* Set a rail's VOSEL and enable it (MTKPwrap only lets the GPU rails through, within limits). */
bool
MappleGPUAccelerator::regulatorSet(uint32_t enAddr, uint32_t voselAddr, uint32_t sel)
{
	if (fPwrap->writeRegulator(voselAddr, 0x7f, sel) != kIOReturnSuccess) {
		return false;
	}
	return fPwrap->writeRegulator(enAddr, 0x1, 1) == kIOReturnSuccess;
}

/*
 * Operating points: gpufreq_v1/mt6781 g_opp_table_segment_0 (frequency, VGPU and VSRAM_GPU in 10 uV). The
 * segment is efuse 0x11cb05cc[29:28] (devinfo index 69): 0 on this phone (LK's atag,devinfo); segment 1 only
 * adds 50 mV, which the default margin (mp_gpu_mv) covers below ~560 MHz anyway.
 */
static const struct { uint32_t khz, vgpu, vsram; } gMappleOpp[] = {
	{ 950000, 90000, 100000 }, { 940000, 89375, 99375 }, { 930000, 88750, 98750 }, { 920000, 88125, 98125 },
	{ 910000, 87500, 97500 }, { 900000, 86875, 96875 }, { 890000, 86250, 96250 }, { 880000, 85625, 95625 },
	{ 870000, 85000, 95000 }, { 860000, 84375, 94375 }, { 850000, 83750, 93750 }, { 840000, 83125, 93125 },
	{ 830000, 82500, 92500 }, { 820000, 81875, 91875 }, { 810000, 81250, 91250 }, { 800000, 80625, 90625 },
	{ 790000, 80000, 90000 }, { 775000, 79375, 89375 }, { 761000, 78750, 88750 }, { 746000, 78125, 88125 },
	{ 732000, 77500, 87500 }, { 718000, 76875, 86875 }, { 703000, 76250, 86250 }, { 689000, 75625, 85625 },
	{ 675000, 75000, 85000 }, { 660000, 74375, 85000 }, { 646000, 73750, 85000 }, { 631000, 73125, 85000 },
	{ 617000, 72500, 85000 }, { 603000, 71875, 85000 }, { 588000, 71250, 85000 }, { 574000, 70625, 85000 },
	{ 560000, 70000, 85000 }, { 543000, 69375, 85000 }, { 527000, 68750, 85000 }, { 511000, 68125, 85000 },
	{ 495000, 67500, 85000 }, { 478000, 66875, 85000 }, { 462000, 66250, 85000 }, { 446000, 65625, 85000 },
	{ 430000, 65000, 85000 }, { 413000, 64375, 85000 }, { 397000, 63750, 85000 }, { 381000, 63125, 85000 },
	{ 365000, 62500, 85000 }, { 348000, 61875, 85000 }, { 332000, 61250, 85000 }, { 316000, 60625, 85000 },
	{ 300000, 60000, 85000 },
};
#define OPP_COUNT       (sizeof(gMappleOpp) / sizeof(gMappleOpp[0]))
#define SEL_OF(uv)      (((uv) - 500000u + 6249u) / 6250u)      /* round up */
#define VGPU_SEL_MAX    0x40    /* 0.9 V: MTKPwrap's ceiling, = the top OPP */
#define VSRAM_SEL_MAX   0x50    /* 1.0 V */
#define VSRAM_SEL_MIN   0x38    /* 0.85 V */
#define BUCK_DIFF_MIN   16      /* VSRAM - VGPU in steps: 100 mV .. 250 mV (gpufreq BUCK_DIFF_*) */
#define BUCK_DIFF_MAX   40

/* Highest OPP at or below `mhz` (the lowest one if none). */
static uint32_t
oppForMHz(uint32_t mhz)
{
	for (uint32_t i = 0; i < OPP_COUNT; i++) {
		if (gMappleOpp[i].khz <= mhz * 1000) {
			return i;
		}
	}
	return OPP_COUNT - 1;
}

/* Rail settings of an OPP: VGPU + margin (mp_gpu_mv), VSRAM >= VGPU + 0.1 V, within the PMIC ceilings. */
void
MappleGPUAccelerator::oppSels(uint32_t opp, uint32_t *vg, uint32_t *vs)
{
	uint32_t g = SEL_OF(gMappleOpp[opp].vgpu * 10 + (uint32_t)fMarginMv * 1000), v = SEL_OF(gMappleOpp[opp].vsram * 10);
	g = MIN(g, (uint32_t)VGPU_SEL_MAX);
	v = MIN(MAX(MAX(v, g + BUCK_DIFF_MIN), (uint32_t)VSRAM_SEL_MIN), (uint32_t)VSRAM_SEL_MAX);
	*vg = g;
	*vs = v;
}

/* MFGPLL CON1 for a frequency (gpufreq: posdiv /8 below 375 MHz else /4, DDS from 26 MHz, PCW_CHG). */
static uint32_t
mfgpllCon1(uint32_t khz)
{
	uint32_t posdiv = khz < 375000 ? 3 : 2;
	uint32_t dds = (uint32_t)(((((uint64_t)(khz / 100) << posdiv) << 14) / 26 + 5) / 10);
	return 0x80000000u | posdiv << 24 | dds;
}

/* MFGPLL to the OPP's frequency, mfg_sel parked on mainpll_d5 while it changes (gpufreq's frequency switch). */
void
MappleGPUAccelerator::mfgClock(uint32_t opp, char **out, char *end)
{
	uint32_t cfg = rd32(fTopck, CK_CFG_0);
	if (rd32(fApmixed, PLL_MFG_CON0) & 1) {
		wr32(fTopck, CK_CFG_0_CLR, CK_MFG_SEL_MASK);
		wr32(fTopck, CK_CFG_0_SET, CK_MFG_MAINPLL_D5 << CK_MFG_SEL_SHIFT);
		wr32(fTopck, CK_CFG_UPDATE, CK_CFG_UPDATE_MFG);
		IODelay(20);
	}
	if (!(rd32(fApmixed, PLL_MFG_PWR_CON0) & 1)) {
		set32(fApmixed, PLL_MFG_PWR_CON0, 1u << 0);
		IODelay(1);
	}
	clr32(fApmixed, PLL_MFG_PWR_CON0, 1u << 1);
	IODelay(1);
	wr32(fApmixed, PLL_MFG_CON1, mfgpllCon1(gMappleOpp[opp].khz));
	set32(fApmixed, PLL_MFG_CON0, 1u << 0);
	IODelay(20);
	if (cfg & CK_MFG_PDN) {
		wr32(fTopck, CK_CFG_0_CLR, CK_MFG_PDN);
	}
	wr32(fTopck, CK_CFG_0_CLR, CK_MFG_SEL_MASK);
	wr32(fTopck, CK_CFG_0_SET, CK_MFG_MFGPLL << CK_MFG_SEL_SHIFT);
	wr32(fTopck, CK_CFG_UPDATE, CK_CFG_UPDATE_MFG);
	IODelay(20);
	fClockOpp = opp;
	say(out, end, "clock: %u MHz, mfgpll con0 0x%08x con1 0x%08x pwr 0x%08x, clk_cfg_0 0x%08x (was 0x%08x)\n",
	    gMappleOpp[opp].khz / 1000, rd32(fApmixed, PLL_MFG_CON0), rd32(fApmixed, PLL_MFG_CON1),
	    rd32(fApmixed, PLL_MFG_PWR_CON0), rd32(fTopck, CK_CFG_0), cfg);
}

/* Current rail state from the PMIC's DA (actual) registers. */
void
MappleGPUAccelerator::railsRead(uint32_t *vgEn, uint32_t *vg, uint32_t *vsEn, uint32_t *vs)
{
	*vgEn = *vg = *vsEn = *vs = 0;
	fPwrap->readPMIC(PMIC_VGPU_DA_EN, vgEn);
	fPwrap->readPMIC(PMIC_VGPU_DA_VOSEL, vg);
	fPwrap->readPMIC(PMIC_VSRAM_DA_EN, vsEn);
	fPwrap->readPMIC(PMIC_VSRAM_DA_VOSEL, vs);
	*vgEn &= 1;
	*vsEn &= 1;
	*vg &= 0x7f;
	*vs = (*vs >> 8) & 0x7f;
}

/*
 * Rails to (tvg, tvs) in steps of at most 50 mV, keeping VSRAM - VGPU within 100..250 mV where the start allows
 * it (gpufreq steps through intermediate OPPs for the same reason): raising moves VSRAM first, lowering VGPU
 * first. A rail that is off is first enabled straight at its target, VSRAM before VGPU.
 */
bool
MappleGPUAccelerator::railsTo(uint32_t tvg, uint32_t tvs)
{
	uint32_t vgEn, vg, vsEn, vs;
	railsRead(&vgEn, &vg, &vsEn, &vs);
	if (fLkRails) {                 /* never written: fine only while both are on at LK's levels */
		return vgEn && vsEn && vg == LK_VGPU_SEL && vs == LK_VSRAM_SEL;
	}
	bool ok = true;
	if (!vsEn) {
		ok = regulatorSet(PMIC_VSRAM_EN, PMIC_VSRAM_VOSEL, tvs);
		IODelay(300);           /* 240 us enable ramp + settle */
		vs = tvs;
	}
	if (ok && !vgEn) {
		ok = regulatorSet(PMIC_VGPU_EN, PMIC_VGPU_VOSEL, tvg);
		IODelay(300);           /* 200 us enable ramp + settle */
		vg = tvg;
	}
	for (int guard = 0; ok && guard < 64 && (vg != tvg || vs != tvs); guard++) {
		if (vg < tvg) {
			uint32_t nvg = MIN(tvg, vg + 8);
			if (vs < nvg + BUCK_DIFF_MIN) {
				uint32_t nvs = MIN(MIN(MAX(tvs, nvg + BUCK_DIFF_MIN), vg + BUCK_DIFF_MAX), (uint32_t)VSRAM_SEL_MAX);
				ok = fPwrap->writeRegulator(PMIC_VSRAM_VOSEL, 0x7f, nvs) == kIOReturnSuccess;
				vs = nvs;
				IODelay(100);
				if (vs < nvg + BUCK_DIFF_MIN) {
					nvg = vs > BUCK_DIFF_MIN ? MAX(vs - BUCK_DIFF_MIN, vg) : vg;
				}
			}
			ok = ok && fPwrap->writeRegulator(PMIC_VGPU_VOSEL, 0x7f, nvg) == kIOReturnSuccess;
			vg = nvg;
		} else if (vg > tvg) {
			uint32_t nvg = vg - MIN(vg - tvg, 8u);
			ok = fPwrap->writeRegulator(PMIC_VGPU_VOSEL, 0x7f, nvg) == kIOReturnSuccess;
			vg = nvg;
			uint32_t nvs = MAX(MAX(tvs, nvg + BUCK_DIFF_MIN), (uint32_t)VSRAM_SEL_MIN);
			if (ok && nvs < vs) {
				IODelay(100);
				ok = fPwrap->writeRegulator(PMIC_VSRAM_VOSEL, 0x7f, nvs) == kIOReturnSuccess;
				vs = nvs;
			}
		} else {
			ok = fPwrap->writeRegulator(PMIC_VSRAM_VOSEL, 0x7f, tvs) == kIOReturnSuccess;
			vs = tvs;
		}
		IODelay(100);           /* 8 steps x 2 us + 52 us (gpufreq settle time), rounded up */
	}
	return ok && vg == tvg && vs == tvs;
}

/*
 * Gated: switch the running GPU to another OPP (raising: voltage then clock; lowering: clock then voltage).
 * Jobs keep running; while the PLL relocks the GPU runs from mainpll_d5 (218 MHz).
 */
IOReturn
MappleGPUAccelerator::setOpp(uint32_t opp, char **out, char *end)
{
	if (opp >= OPP_COUNT) {
		return kIOReturnBadArgument;
	}
	fOpp = opp;
	if (!fPowered) {
		say(out, end, "OPP %u MHz from the next power on\n", gMappleOpp[opp].khz / 1000);
		return kIOReturnSuccess;
	}
	uint32_t tvg, tvs, vgEn, vg, vsEn, vs;
	oppSels(opp, &tvg, &tvs);
	bool up = gMappleOpp[opp].khz > gMappleOpp[fClockOpp].khz;
	if (!up) {
		mfgClock(opp, out, end);
	}
	bool ok = railsTo(tvg, tvs);
	if (up) {
		if (ok) {
			mfgClock(opp, out, end);
		} else {
			say(out, end, "rails did not reach the OPP: clock left at %u MHz\n", gMappleOpp[fClockOpp].khz / 1000);
		}
	}
	railsRead(&vgEn, &vg, &vsEn, &vs);
	say(out, end, "OPP %u MHz: vgpu %u uV, vsram_gpu %u uV%s\n", gMappleOpp[fClockOpp].khz / 1000, UV(vg), UV(vs),
	    ok ? "" : " (PMIC write refused or failed)");
	return ok ? kIOReturnSuccess : kIOReturnError;
}

bool
MappleGPUAccelerator::mfgOn(char **out, char *end)
{
	uint32_t vgEn, vgSel, vsEn, vsSel, tvg, tvs;
	railsRead(&vgEn, &vgSel, &vsEn, &vsSel);
	say(out, end, "rails before: vgpu %s %u uV, vsram_gpu %s %u uV\n", vgEn ? "on" : "off", UV(vgSel),
	    vsEn ? "on" : "off", UV(vsSel));

	/*
	 * OPP fOpp (mp_gpu_freq). Lowering (the preloader leaves 0.9 V / 1.0 V at its own frequency): clock down
	 * first, then the rails. Raising: rails, then the clock.
	 */
	oppSels(fOpp, &tvg, &tvs);
	bool lowering = vgEn && vgSel > tvg;
	if (lowering) {
		mfgClock(fOpp, out, end);
	}
	bool ok = railsTo(tvg, tvs);
	railsRead(&vgEn, &vgSel, &vsEn, &vsSel);
	say(out, end, "rails: vgpu %s %u uV, vsram_gpu %s %u uV (OPP %u MHz, margin %d mV)%s\n", vgEn ? "on" : "off",
	    UV(vgSel), vsEn ? "on" : "off", UV(vsSel), gMappleOpp[fOpp].khz / 1000, fMarginMv,
	    ok ? "" : " (PMIC write refused or failed)");
	if (!ok || !vgEn || !vsEn) {
		return false;
	}

	if (!lowering) {
		mfgClock(fOpp, out, end);       /* raising: voltage first, then the clock */
	}

	/* power domains MFG0 -> MFG3 */
	wr32(fSpm, SPM_POWERON_CONFIG_EN, SPM_PROJECT_KEY);
	for (uint32_t n = 0; n < 4; n++) {
		if (!mtcmosOn(n, out, end)) {
			return false;
		}
	}

	/* clock gate, then the mfgcfg setup gpufreq does after power on */
	wr32(fMfgcfg, MFG_CG_CLR, MFG_CG_BG3D);
	set32(fMfgcfg, MFG_ASYNC_CON, (1u << 22) | (1u << 23) | (1u << 24) | (1u << 25));
	clr32(fMfgcfg, MFG_GLOBAL_CON, (1u << 10) | (1u << 9));
	set32(fMfgcfg, MFG_QCHANNEL_CON, 1u << 4);
	clr32(fMfgcfg, MFG_GLOBAL_CON, (1u << 19) | (1u << 8));
	set32(fMfgcfg, MFG_ASYNC_CON_1, 1u << 0);
	set32(fMfgcfg, MFG_I2M_PROTECTOR, 1u << 0);
	wr32(fMfgcfg, MFG_TIMESTAMP, 1);
	if (!fAcpDone) {
		set32(fInfra, INFRA_ACP_CON, 1u << 9);
		fAcpDone = true;
	}
	say(out, end, "mfgcfg: cg 0x%08x async 0x%08x/0x%08x global 0x%08x qchannel 0x%08x\n",
	    rd32(fMfgcfg, MFG_CG_CON), rd32(fMfgcfg, MFG_ASYNC_CON), rd32(fMfgcfg, MFG_ASYNC_CON_1),
	    rd32(fMfgcfg, MFG_GLOBAL_CON), rd32(fMfgcfg, MFG_QCHANNEL_CON));
	return (rd32(fMfgcfg, MFG_CG_CON) & MFG_CG_BG3D) == 0;
}

void
MappleGPUAccelerator::mfgOff(char **out, char *end)
{
	wr32(fSpm, SPM_POWERON_CONFIG_EN, SPM_PROJECT_KEY);
	uint32_t st = rd32(fSpm, SPM_PWR_STATUS) & rd32(fSpm, SPM_PWR_STATUS_2ND);
	if ((st & MFG_STATUS_BIT(0)) && (st & MFG_STATUS_BIT(1))) {
		/* wait for the GPU bus to go idle */
		wr32(fMfgcfg, MFG_QCHANNEL_CON, 1);
		wr32(fMfgcfg, MFG_BUS_IDLE_CON, 3);
		if (!pollUs(10000, [&]{ return (rd32(fMfgcfg, MFG_BUS_IDLE_STA) & (1u << 2)) != 0; })) {
			say(out, end, "  bus not idle (0x%08x)\n", rd32(fMfgcfg, MFG_BUS_IDLE_STA));
		}
		wr32(fMfgcfg, MFG_CG_SET, MFG_CG_BG3D);
	}
	for (int n = 3; n >= 0; n--) {
		if (!mtcmosOff((uint32_t)n, out, end)) {
			wr32(fMfgcfg, MFG_CG_CLR, MFG_CG_BG3D);
			say(out, end, "MFG off aborted: MFG0/MFG1, clock and rails left on\n");
			return;
		}
	}
	st = rd32(fSpm, SPM_PWR_STATUS) & rd32(fSpm, SPM_PWR_STATUS_2ND);
	if (!(st & MFG_STATUS_BIT(0))) {
		clr32(fApmixed, PLL_MFG_CON0, 1u << 0);
		set32(fApmixed, PLL_MFG_PWR_CON0, 1u << 1);
		clr32(fApmixed, PLL_MFG_PWR_CON0, 1u << 0);
		if (!fLkRails) {
			fPwrap->writeRegulator(PMIC_VGPU_EN, 0x1, 0);
			IODelay(50);
			fPwrap->writeRegulator(PMIC_VSRAM_EN, 0x1, 0);
		}
	}
	say(out, end, "MFG off: status 0x%08x/0x%08x, mfgpll con0 0x%08x\n", rd32(fSpm, SPM_PWR_STATUS),
	    rd32(fSpm, SPM_PWR_STATUS_2ND), rd32(fApmixed, PLL_MFG_CON0));
}

/* Read-only: what LK/preloader left (registers outside the MFG domain; mfgcfg only if MFG0/1 are on). */
void
MappleGPUAccelerator::mfgDump(char **out, char *end)
{
	uint32_t v[8] = { 0 };
	fPwrap->readPMIC(PMIC_VGPU_EN, &v[0]);
	fPwrap->readPMIC(PMIC_VGPU_VOSEL, &v[1]);
	fPwrap->readPMIC(PMIC_VGPU_DA_EN, &v[2]);
	fPwrap->readPMIC(PMIC_VGPU_DA_VOSEL, &v[3]);
	fPwrap->readPMIC(PMIC_VSRAM_EN, &v[4]);
	fPwrap->readPMIC(PMIC_VSRAM_VOSEL, &v[5]);
	fPwrap->readPMIC(PMIC_VSRAM_DA_EN, &v[6]);
	fPwrap->readPMIC(PMIC_VSRAM_DA_VOSEL, &v[7]);
	say(out, end, "vgpu: en 0x%04x vosel 0x%04x | da_en 0x%04x da_vosel 0x%04x (%s, %u uV)\n", v[0], v[1], v[2],
	    v[3], (v[2] & 1) ? "on" : "off", UV(v[3] & 0x7f));
	say(out, end, "vsram_gpu: en 0x%04x vosel 0x%04x | da_en 0x%04x da_vosel 0x%04x (%s, %u uV)\n", v[4], v[5],
	    v[6], v[7], (v[6] & 1) ? "on" : "off", UV((v[7] >> 8) & 0x7f));
	uint32_t s1 = rd32(fSpm, SPM_PWR_STATUS), s2 = rd32(fSpm, SPM_PWR_STATUS_2ND);
	say(out, end, "spm: status 0x%08x 2nd 0x%08x | mfg0 0x%08x mfg1 0x%08x mfg2 0x%08x mfg3 0x%08x\n", s1, s2,
	    rd32(fSpm, SPM_MFG_PWR_CON(0)), rd32(fSpm, SPM_MFG_PWR_CON(1)), rd32(fSpm, SPM_MFG_PWR_CON(2)),
	    rd32(fSpm, SPM_MFG_PWR_CON(3)));
	uint32_t cfg = rd32(fTopck, CK_CFG_0);
	say(out, end, "clk_cfg_0 0x%08x (mfg_sel %u%s) | mfgpll con0 0x%08x con1 0x%08x pwr 0x%08x\n", cfg,
	    (cfg & CK_MFG_SEL_MASK) >> CK_MFG_SEL_SHIFT, (cfg & CK_MFG_PDN) ? ", pdn" : "", rd32(fApmixed, PLL_MFG_CON0),
	    rd32(fApmixed, PLL_MFG_CON1), rd32(fApmixed, PLL_MFG_PWR_CON0));
	say(out, end, "infra: prot sta1 0x%08x sta1_1 0x%08x acp 0x%08x\n", rd32(fInfra, INFRA_PROT_STA1),
	    rd32(fInfra, INFRA_PROT1_STA1), rd32(fInfra, INFRA_ACP_CON));
	if ((s1 & s2 & MFG_STATUS_BIT(0)) && (s1 & s2 & MFG_STATUS_BIT(1))) {
		say(out, end, "mfgcfg: cg 0x%08x async 0x%08x global 0x%08x qchannel 0x%08x\n", rd32(fMfgcfg, MFG_CG_CON),
		    rd32(fMfgcfg, MFG_ASYNC_CON), rd32(fMfgcfg, MFG_GLOBAL_CON), rd32(fMfgcfg, MFG_QCHANNEL_CON));
	} else {
		say(out, end, "mfgcfg: MFG0/MFG1 off, not read\n");
	}
}

/* ---- Mali bring-up ---- */

bool
MappleGPUAccelerator::softReset(void)
{
	wr(GPU_IRQ_MASK, 0);
	wr(GPU_IRQ_CLEAR, 0xffffffff);
	wr(GPU_COMMAND, GPU_CMD_SOFT_RESET);
	bool ok = pollUs(100000, [&]{ return (rd(GPU_IRQ_RAWSTAT) & GPU_IRQ_RESET_COMPLETED) != 0; });
	if (!ok) {
		IOLog("GPU: soft reset timed out, hard reset\n");
		wr(GPU_COMMAND, GPU_CMD_HARD_RESET);
		ok = pollUs(100000, [&]{ return (rd(GPU_IRQ_RAWSTAT) & GPU_IRQ_RESET_COMPLETED) != 0; });
	}
	wr(GPU_IRQ_CLEAR, 0xffffffff);
	return ok;
}

void
MappleGPUAccelerator::readFeatures(void)
{
	fGpuId = rd(GPU_ID);
	fL2Features = rd(GPU_L2_FEATURES);
	fCoreFeatures = rd(GPU_CORE_FEATURES);
	fTilerFeatures = rd(GPU_TILER_FEATURES);
	fMemFeatures = rd(GPU_MEM_FEATURES);
	fMmuFeatures = rd(GPU_MMU_FEATURES);
	fAsPresent = rd(GPU_AS_PRESENT);
	fJsPresent = rd(GPU_JS_PRESENT);
	fThreadMaxThreads = rd(GPU_THREAD_MAX_THREADS);
	fThreadMaxWg = rd(GPU_THREAD_MAX_WG);
	fThreadMaxBarrier = rd(GPU_THREAD_MAX_BARRIER);
	fThreadFeatures = rd(GPU_THREAD_FEATURES);
	fThreadTlsAlloc = rd(GPU_THREAD_TLS_ALLOC);
	fCoherencyFeatures = rd(GPU_COHERENCY_FEATURES);
	fAfbcFeatures = rd(GPU_AFBC_FEATURES);
	for (int i = 0; i < 4; i++) {
		fTextureFeatures[i] = rd(GPU_TEXTURE_FEATURES(i));
	}
	for (int i = 0; i < 16; i++) {
		fJsFeatures[i] = (fJsPresent & (1u << i)) ? rd(GPU_JS_FEATURES(i)) : 0;
	}
	fShaderPresent = rd(GPU_SHADER_PRESENT_LO) | (uint64_t)rd(GPU_SHADER_PRESENT_HI) << 32;
	fTilerPresent = rd(GPU_TILER_PRESENT_LO);
	fL2Present = rd(GPU_L2_PRESENT_LO);
	fStackPresent = rd(GPU_STACK_PRESENT_LO);
	fASCount = 0;
	while (fASCount < 16 && (fAsPresent & (1u << fASCount))) {
		fASCount++;
	}
}

bool
MappleGPUAccelerator::powerCores(void)
{
	uint32_t l2 = (uint32_t)fL2Present, sh = (uint32_t)fShaderPresent, st = (uint32_t)fStackPresent;
	wr(GPU_L2_PWRON_LO, l2);
	if (!pollUs(20000, [&]{ return (rd(GPU_L2_READY_LO) & l2) == l2; })) {
		IOLog("GPU: L2 not ready (0x%x)\n", rd(GPU_L2_READY_LO));
		return false;
	}
	if (st != 0) {
		wr(GPU_STACK_PWRON_LO, st);
		pollUs(20000, [&]{ return (rd(GPU_STACK_READY_LO) & st) == st; });
	}
	wr(GPU_TILER_PWRON_LO, (uint32_t)fTilerPresent);
	wr(GPU_SHADER_PWRON_LO, sh);
	if (fShaderPresent >> 32) {
		wr(GPU_SHADER_PWRON_HI, (uint32_t)(fShaderPresent >> 32));
	}
	if (!pollUs(20000, [&]{ return (rd(GPU_SHADER_READY_LO) & sh) == sh; })) {
		IOLog("GPU: shader cores not ready (0x%x of 0x%x)\n", rd(GPU_SHADER_READY_LO), sh);
		return false;
	}
	pollUs(20000, [&]{ return (rd(GPU_TILER_READY_LO) & (uint32_t)fTilerPresent) == (uint32_t)fTilerPresent; });
	return true;
}

bool
MappleGPUAccelerator::maliInit(char **out, char *end)
{
	uint32_t id = rd(GPU_ID);
	if (id == 0 || id == 0xffffffff) {
		say(out, end, "GPU_ID 0x%08x: the GPU does not answer\n", id);
		return false;
	}
	if (!softReset()) {
		say(out, end, "GPU_ID 0x%08x: reset did not complete\n", id);
		return false;
	}
	readFeatures();
	/* Panfrost's quirks for G57: TTRX_2968/TTRX_3162 varying algorithm (all revisions), IDVS group size */
	wr(GPU_SHADER_CONFIG, rd(GPU_SHADER_CONFIG) | SC_VAR_ALGORITHM);
	wr(GPU_JM_CONFIG, rd(GPU_JM_CONFIG) | JM_IDVS_GROUP_SIZE(0xf));
	wr(GPU_COHERENCY_ENABLE, COHERENCY_NONE);
	if (!powerCores()) {
		say(out, end, "GPU_ID 0x%08x: cores did not power up (l2 ready 0x%x shader ready 0x%x)\n", fGpuId,
		    rd(GPU_L2_READY_LO), rd(GPU_SHADER_READY_LO));
		return false;
	}
	for (uint32_t as = 0; as < fASCount; as++) {
		asDisable((int)as);
		fAS[as] = NULL;
	}
	uint32_t jobMask = 0, mmuMask = 0;
	for (uint32_t js = 0; js < 3; js++) {
		if (fJsPresent & (1u << js)) {
			jobMask |= (1u << js) | (1u << (16 + js));
		}
	}
	for (uint32_t as = 0; as < fASCount; as++) {
		mmuMask |= (1u << as) | (1u << (16 + as));
	}
	wr(GPU_IRQ_CLEAR, 0xffffffff);
	wr(JOB_INT_CLEAR, 0xffffffff);
	wr(MMU_INT_CLEAR, 0xffffffff);
	wr(GPU_IRQ_MASK, GPU_IRQ_FAULT | GPU_IRQ_MULTIPLE_FAULT);
	wr(JOB_INT_MASK, jobMask);
	wr(MMU_INT_MASK, mmuMask);
	if ((fMmuFeatures & 0xff) != 48) {
		say(out, end, "warning: MMU reports %u VA bits, page tables assume 48\n", fMmuFeatures & 0xff);
	}
	uint32_t cores = (uint32_t)__builtin_popcountll(fShaderPresent);
	say(out, end, "GPU: id 0x%08x (product 0x%04x r%up%u) cores %u (0x%llx) l2 on (0x%llx), "
	    "as 0x%x js 0x%x mmu 0x%08x coherency 0x%x\n", fGpuId, fGpuId >> 16, (fGpuId >> 12) & 0xf,
	    (fGpuId >> 4) & 0xff, cores, fShaderPresent, fL2Present, fAsPresent, fJsPresent, fMmuFeatures,
	    fCoherencyFeatures);
	return true;
}

IOReturn
MappleGPUAccelerator::powerOn(char **out, char *end)
{
	if (fPowered) {
		say(out, end, "already on\n");
		return kIOReturnSuccess;
	}
	if (!mfgOn(out, end)) {
		say(out, end, "MFG power-up failed: powering off again\n");
		mfgOff(out, end);
		return kIOReturnNotResponding;
	}
	if (!maliInit(out, end)) {
		mfgOff(out, end);
		return kIOReturnNotResponding;
	}
	IOLog("GPU: id 0x%08x cores 0x%llx l2 on\n", fGpuId, fShaderPresent);
	fPowered = true;
	fastLive(true);
	for (int i = 0; i < 3; i++) {
		fIrq[i]->enable();
	}
	return kIOReturnSuccess;
}

void
MappleGPUAccelerator::powerOff(char **out, char *end)
{
	if (!fPowered) {
		mfgOff(out, end);
		return;
	}
	fastSync();
	fastLive(false);        /* the filter no longer touches the registers */
	for (int i = 0; i < 3; i++) {
		fIrq[i]->disable();
	}
	wr(GPU_IRQ_MASK, 0);
	wr(JOB_INT_MASK, 0);
	wr(MMU_INT_MASK, 0);
	softReset();            /* nothing runs any more: job memory can be released */
	failAll(kIOReturnOffline);
	for (uint32_t as = 0; as < fASCount; as++) {
		if (fAS[as] != NULL) {
			fAS[as]->asSlot = -1;
			fAS[as] = NULL;
		}
	}
	wr(GPU_SHADER_PWROFF_LO, (uint32_t)fShaderPresent);
	if (fStackPresent) {
		wr(GPU_STACK_PWROFF_LO, (uint32_t)fStackPresent);
	}
	wr(GPU_TILER_PWROFF_LO, (uint32_t)fTilerPresent);
	pollUs(20000, [&]{ return rd(GPU_SHADER_READY_LO) == 0; });
	wr(GPU_L2_PWROFF_LO, (uint32_t)fL2Present);
	pollUs(20000, [&]{ return rd(GPU_L2_READY_LO) == 0; });
	fPowered = false;
	mfgOff(out, end);
}

/* ---- MMU ---- */

bool
MappleGPUAccelerator::asWait(int as)
{
	bool ok = pollUs(100000, [&]{ return !(rd(AS_BASE(as) + AS_STATUS) & AS_STATUS_ACTIVE); });
	if (!ok) {
		IOLog("GPU: AS%d stuck active\n", as);
	}
	return ok;
}

void
MappleGPUAccelerator::asCommand(int as, uint32_t cmd)
{
	asWait(as);
	wr(AS_BASE(as) + AS_COMMAND, cmd);
}

/* Lock the naturally aligned power-of-two region covering [va, va + size) (32 KiB minimum). */
void
MappleGPUAccelerator::asLock(int as, uint64_t va, uint64_t size)
{
	uint64_t region;
	if (size == 0 || size >= (1ull << 48)) {
		region = 47;    /* the whole 48-bit space */
	} else {
		uint64_t diff = va ^ (va + size - 1);
		uint32_t width = diff ? 64 - (uint32_t)__builtin_clzll(diff) : 0;
		if (width < 15) {
			width = 15;
		}
		width -= 1;
		region = (va & ~((1ull << width) - 1)) | width;
	}
	wr(AS_BASE(as) + AS_LOCKADDR_LO, (uint32_t)region);
	wr(AS_BASE(as) + AS_LOCKADDR_HI, (uint32_t)(region >> 32));
	asCommand(as, AS_CMD_LOCK);
}

/* 48-bit input (four levels from level 0; Panfrost's VA range is below 4 GiB anyway). */
void
MappleGPUAccelerator::asProgram(int as, MappleGPUVM *vm)
{
	/* flush whatever the previous owner left in the GPU caches */
	asLock(as, 0, 0);
	asCommand(as, AS_CMD_FLUSH_MEM);
	asWait(as);
	wr(AS_BASE(as) + AS_TRANSTAB_LO, (uint32_t)vm->rootPA);
	wr(AS_BASE(as) + AS_TRANSTAB_HI, (uint32_t)(vm->rootPA >> 32));
	wr(AS_BASE(as) + AS_MEMATTR_LO, MEMATTR_LO);
	wr(AS_BASE(as) + AS_MEMATTR_HI, MEMATTR_HI);
	wr(AS_BASE(as) + AS_TRANSCFG_LO, TRANSCFG_ADRMODE_AARCH64_4K | TRANSCFG_INA_BITS(55 - 48) |
	    TRANSCFG_PTW_MEMATTR_WB | TRANSCFG_PTW_RA);
	wr(AS_BASE(as) + AS_TRANSCFG_HI, 0);
	asCommand(as, AS_CMD_UPDATE);
	asWait(as);
}

void
MappleGPUAccelerator::asDisable(int as)
{
	asLock(as, 0, 0);
	asCommand(as, AS_CMD_FLUSH_MEM);
	asWait(as);
	wr(AS_BASE(as) + AS_TRANSTAB_LO, 0);
	wr(AS_BASE(as) + AS_TRANSTAB_HI, 0);
	wr(AS_BASE(as) + AS_MEMATTR_LO, 0);
	wr(AS_BASE(as) + AS_MEMATTR_HI, 0);
	wr(AS_BASE(as) + AS_TRANSCFG_LO, TRANSCFG_ADRMODE_UNMAPPED);
	wr(AS_BASE(as) + AS_TRANSCFG_HI, 0);
	asCommand(as, AS_CMD_UPDATE);
	asWait(as);
}

/* Give the VM a hardware address space: a free one, else the least recently used idle one. */
bool
MappleGPUAccelerator::asBind(MappleGPUVM *vm)
{
	vm->lastUse = ++fLRU;
	if (vm->asSlot >= 0) {
		if (vm->asFaulted && vm->running == 0) {
			asProgram(vm->asSlot, vm);
			wr(MMU_INT_CLEAR, (1u << vm->asSlot) | (1u << (16 + vm->asSlot)));
			wr(MMU_INT_MASK, rd(MMU_INT_MASK) | (1u << vm->asSlot) | (1u << (16 + vm->asSlot)));
			vm->asFaulted = false;
		}
		return !vm->asFaulted;
	}
	int pick = -1;
	for (uint32_t i = 0; i < fASCount; i++) {
		if (fAS[i] == NULL) {
			pick = (int)i;
			break;
		}
		if (fAS[i]->running == 0 && (pick < 0 || fAS[i]->lastUse < fAS[pick]->lastUse)) {
			pick = (int)i;
		}
	}
	if (pick < 0) {
		return false;
	}
	if (fAS[pick] != NULL) {
		fAS[pick]->asSlot = -1;
		fAS[pick]->asFaulted = false;
	}
	asProgram(pick, vm);
	wr(MMU_INT_CLEAR, (1u << pick) | (1u << (16 + pick)));
	wr(MMU_INT_MASK, rd(MMU_INT_MASK) | (1u << pick) | (1u << (16 + pick)));
	fAS[pick] = vm;
	vm->asSlot = pick;
	return true;
}

void
MappleGPUAccelerator::vmFlush(MappleGPUVM *vm, uint64_t va, uint64_t size)
{
	if (!fPowered || vm->asSlot < 0) {
		return;         /* binding the AS later flushes everything */
	}
	asLock(vm->asSlot, va, size);
	asCommand(vm->asSlot, AS_CMD_FLUSH_PT);
	asWait(vm->asSlot);
}

void
MappleGPUAccelerator::vmDestroyed(MappleGPUVM *vm)
{
	fGate->runActionBlock(^IOReturn {
		if (vm->asSlot >= 0) {
			if (fPowered) {
				asDisable(vm->asSlot);
			}
			fAS[vm->asSlot] = NULL;
			vm->asSlot = -1;
		}
		if (fSelfVM == vm) {
			fSelfVM = NULL;
		}
		for (MappleGPUVM **pp = &fVMs; *pp != NULL; pp = &(*pp)->nextVM) {
			if (*pp == vm) {
				*pp = vm->nextVM;
				break;
			}
		}
		return kIOReturnSuccess;
	});
}

void
MappleGPUAccelerator::heapAdd(MappleGPUBO *bo)
{
	if (fHeapCount == fHeapCap) {
		uint32_t cap = fHeapCap ? fHeapCap * 2 : 16;
		MappleGPUBO **list = IONew(MappleGPUBO *, cap);
		if (list == NULL) {
			return;
		}
		if (fHeapList != NULL) {
			bcopy(fHeapList, list, fHeapCount * sizeof(*list));
			IODelete(fHeapList, MappleGPUBO *, fHeapCap);
		}
		fHeapList = list;
		fHeapCap = cap;
	}
	fHeapList[fHeapCount++] = bo;
}

void
MappleGPUAccelerator::heapRemove(MappleGPUBO *bo)
{
	for (uint32_t i = 0; i < fHeapCount; i++) {
		if (fHeapList[i] == bo) {
			fHeapList[i] = fHeapList[--fHeapCount];
			return;
		}
	}
}

MappleGPUBO *
MappleGPUAccelerator::findHeapBO(MappleGPUVM *vm, uint64_t va)
{
	for (uint32_t i = 0; i < fHeapCount; i++) {
		MappleGPUBO *bo = fHeapList[i];
		if (bo->vm == vm && va >= bo->va && va < bo->va + bo->vaSize) {
			return bo;
		}
	}
	return NULL;
}

/* ---- jobs ---- */

IOReturn
MappleGPUAccelerator::submit(MappleGPUVM *vm, uint64_t jc, uint32_t requirements, OSArray *deps, OSArray *bos,
    MappleGPUFence *fence, uint64_t *seqOut)
{
	if (!fPowered) {
		return kIOReturnNotReady;
	}
	uint64_t t0 = mach_absolute_time(), cacheNs = 0, cacheBytes = 0;
	OSArray *ownDeps = NULL;
	if (deps == NULL && bos != NULL && bos->getCount() != 0) {
		deps = ownDeps = OSArray::withCapacity(4);
		if (deps == NULL) {
			return kIOReturnNoMemory;
		}
	}
	MappleGPUJob *job = IOMallocType(MappleGPUJob);
	fJobsLive++;
	job->vm = vm;
	vm->retain();
	job->jc = jc;
	job->requirements = requirements;
	job->flushId = rd(GPU_LATEST_FLUSH);    /* after every CPU write for this job */
	job->deps = deps;
	if (deps != NULL) {
		deps->retain();
	}
	job->bos = bos;
	if (bos != NULL) {
		bos->retain();
		for (unsigned int i = 0; i < bos->getCount(); i++) {
			MappleGPUBO *bo = (MappleGPUBO *)bos->getObject(i);
			/*
			 * K0, implicit sync as Linux's panfrost_job.c: every BO counts as written, so the job waits for the
			 * last job on each of its BOs (Mesa submits a batch's vertex/tiler and fragment chains without an
			 * in-sync between them, and every batch shares the tiler heap). An IOSurface also waits for the last
			 * job on it in any other connection. Each busy fence then signals after every earlier job on the BO.
			 */
			addImplicitDepFor(deps, bo->busy, fence, vm);
			fence->retain();
			OSSafeReleaseNULL(bo->busy);
			bo->busy = fence;
			if (bo->kind == kMappleBOSurface && bo->surface != NULL) {
				MappleSurface *s = bo->surface;
				MappleGPUFence *last = OSDynamicCast(MappleGPUFence, s->gpuFence);
				uint32_t budget = 128;
				if (last != NULL && last->promise && !last->signalled) {
					/*
					 * K0i: a pending promise stays the surface's fence until it is resolved. Its owner's jobs
					 * (the ones that will resolve it) wait for what came before it; anyone else's wait for it
					 * (K0j: unless it depends on a pending promise of ours: then it comes after this job).
					 */
					addImplicitDepFor(deps, last->owner == vm ? OSDynamicCast(MappleGPUFence, s->gpuPrev) : last,
					    fence, vm);
					s->gpuJobs++;
				} else if (last != NULL && !last->signalled && vmHasPromise(vm) &&
				    dependsOnPromiseOf(last, vm, &budget)) {
					/*
					 * K0j: the surface's last job waits (maybe through other jobs) for a pending promise of this
					 * connection, so it runs after the promised work, this job included. It stays the surface's
					 * fence (the later one); this job waits for what that one waited for on the surface.
					 */
					fPromiseBypass++;
					addImplicitDepFor(deps, last, fence, vm);
					s->gpuJobs++;
				} else {
					addImplicitDep(deps, last, fence);
					if (s->gpuFence != fence) {
						fence->retain();
						OSSafeReleaseNULL(s->gpuFence);
						OSSafeReleaseNULL(s->gpuPrev);
						s->gpuFence = fence;
						s->gpuJobs++;
					}
				}
			}
			uint64_t c0 = mach_absolute_time(), bytes = 0;
			if (bo->cacheMaint) {
				bytes = bo->cacheOp(0, 0, kMappleCacheFlush);     /* CPU writes out before the GPU reads */
				if (bo->invPending) {
					bo->vm->invRemove(bo);                  /* the flush also dropped the stale lines */
				}
			} else if (bo->lazySurface) {
				bytes = mapple_surface_before_device_read(bo->surface);
			}
			if (bytes != 0) {
				cacheBytes += bytes;
				cacheNs += mach_absolute_time() - c0;
			}
		}
	}
	job->fence = fence;
	fence->retain();
	if (deps != NULL && fence->deps != deps) {
		deps->retain();                 /* K0j: what this fence waits for, until it signals */
		OSSafeReleaseNULL(fence->deps);
		fence->deps = deps;
	}
	job->submitAt = mach_absolute_time();
	/* K0h: numbered in its connection, listed until it completes (completion page watermark) */
	job->seq = ++vm->seqNext;
	job->vmPrev = vm->jobTail;
	if (vm->jobTail != NULL) {
		vm->jobTail->vmNext = job;
	} else {
		vm->jobHead = job;
	}
	vm->jobTail = job;
	vm->seqUpdate();
	if (seqOut != NULL) {
		*seqOut = job->seq;
	}
	uint32_t slot = (requirements & VGPU_JD_REQ_FS) ? 0 : 1;
	if (fQueueTail[slot] != NULL) {
		fQueueTail[slot]->next = job;
	} else {
		fQueue[slot] = job;
	}
	fQueueTail[slot] = job;
	stateUpdate(job->submitAt);
	kick();
	uint64_t ns;
	absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	fSubmits++;
	fSubmitNs += ns;
	absolutetime_to_nanoseconds(cacheNs, &ns);
	fSubmitCacheNs += ns;
	fSubmitCacheBytes += cacheBytes;
	OSSafeReleaseNULL(ownDeps);
	return kIOReturnSuccess;
}

/* Gated: K0, the job also waits for `f` (another job's fence on one of its BOs), once, unless done already. */
void
MappleGPUAccelerator::addImplicitDep(OSArray *deps, MappleGPUFence *f, MappleGPUFence *self)
{
	if (f == NULL || f == self || f->signalled || deps == NULL ||
	    deps->getNextIndexOfObject(f, 0) != (unsigned int)-1) {
		return;
	}
	if (deps->setObject(f)) {
		fImplicitDeps++;
	}
}

MappleGPUFence *
MappleGPUAccelerator::boBusyFence(MappleGPUBO *bo)
{
	if (bo->busy != NULL && !bo->busy->signalled) {
		return bo->busy;
	}
	if (bo->kind != kMappleBOSurface || bo->surface == NULL) {
		return NULL;
	}
	MappleGPUFence *f = OSDynamicCast(MappleGPUFence, bo->surface->gpuFence);
	if (f != NULL && f->promise && !f->signalled && f->owner == bo->vm) {
		f = OSDynamicCast(MappleGPUFence, bo->surface->gpuPrev);     /* our own promise: we resolve it */
	}
	return (f != NULL && !f->signalled) ? f : NULL;
}

bool
MappleGPUAccelerator::boIdle(MappleGPUBO *bo)
{
	return boBusyFence(bo) == NULL;
}

bool
MappleGPUAccelerator::surfaceIdle(MappleSurface *s, uint64_t *jobs)
{
	MappleGPUFence *f = OSDynamicCast(MappleGPUFence, s->gpuFence);
	if (jobs != NULL) {
		*jobs = s->gpuJobs;
	}
	return f == NULL || f->signalled;
}

/* Gated. */
void
MappleGPUAccelerator::invalidatePending(MappleGPUVM *vm, MappleGPUBO *only)
{
	if (vm == NULL || vm->invCount == 0 || (only != NULL && !only->invPending)) {
		return;
	}
	uint64_t c0 = mach_absolute_time(), bytes = 0;
	for (uint32_t i = 0; i < vm->invCount;) {
		MappleGPUBO *bo = vm->inv[i];
		if ((only != NULL && bo != only) || !boIdle(bo)) {
			i++;            /* a job still uses it: stays listed */
			continue;
		}
		/* civac: drops the lines older than the GPU's writes, keeps what the CPU wrote meanwhile */
		bytes += bo->cacheOp(0, 0, kMappleCacheFlush);
		bo->invPending = false;
		vm->inv[i] = vm->inv[--vm->invCount];
	}
	if (bytes != 0) {
		uint64_t ns;
		absolutetime_to_nanoseconds(mach_absolute_time() - c0, &ns);
		fLazyWaits++;
		fLazyNs += ns;
		fLazyBytes += bytes;
	}
}

/* Gated. */
void
MappleGPUAccelerator::boMapped(MappleGPUBO *bo)
{
	if (bo->cacheMaint || !bo->cacheable) {
		return;
	}
	/* the GPU may have written it while nobody looked: drop stale lines once, then maintain it like the rest */
	bo->cacheOp(0, 0, kMappleCacheFlush);
	bo->cacheMaint = true;
	bo->lazySurface = false;
}

void
MappleGPUAccelerator::cacheStats(char **out, char *end)
{
	uint64_t n = fSubmits ? fSubmits : 1, jobs = fJobs ? fJobs : 1;
	say(out, end, "cache (mp_gpu_cache=%d, mp_gpu_wc=%d, mp_gpu_lazy=%d): submit avg %llu us (maintenance %llu us, "
	    "%llu KiB), completion maintenance avg %llu us (%llu KiB) over %llu submits\n", fCacheMode, fWC, fLazy,
	    fSubmitNs / n / 1000, fSubmitCacheNs / n / 1000, fSubmitCacheBytes / n >> 10, fDoneCacheNs / jobs / 1000,
	    fDoneCacheBytes / jobs >> 10, fSubmits);
	uint64_t w = fLazyWaits ? fLazyWaits : 1;
	say(out, end, "lazy invalidation: %llu BOs listed after jobs, %llu waits invalidated %llu KiB (avg %llu us, "
	    "%llu KiB), %llu list overflows; implicit dependencies %llu\n", fLazyListed, fLazyWaits, fLazyBytes >> 10,
	    fLazyNs / w / 1000, fLazyBytes / w >> 10, fLazyOverflows, fImplicitDeps);
}

void
MappleGPUAccelerator::kick(void)
{
	if (!fPowered) {
		return;
	}
	fastSync();
	for (uint32_t slot = 0; slot < 2; slot++) {
		if (fRunning[slot] != NULL || fQueue[slot] == NULL) {
			continue;
		}
		MappleGPUJob *job = nextStartable(slot, false);
		if (job != NULL) {
			startJob(slot, job);
		}
	}
	fastArm();
}

static bool
vmListed(MappleGPUVM *const *list, uint32_t n, MappleGPUVM *vm)
{
	for (uint32_t i = 0; i < n; i++) {
		if (list[i] == vm) {
			return true;
		}
	}
	return false;
}

/* Gated: take `job` (after `prev`, NULL = the head) out of the slot's queue. */
void
MappleGPUAccelerator::unlinkQueued(uint32_t slot, MappleGPUJob *job, MappleGPUJob *prev)
{
	if (prev != NULL) {
		prev->next = job->next;
	} else {
		fQueue[slot] = job->next;
	}
	if (fQueueTail[slot] == job) {
		fQueueTail[slot] = prev;
	}
	job->next = NULL;
}

/*
 * Gated, K0j. The slot queues are FIFO per connection, not across connections (like Linux's drm_sched entities):
 * the first job whose connection has no earlier job still waiting in this slot. Otherwise a job of SpringBoard that
 * waits for a backboardd promise, at the head of the fragment queue, would hold back the backboardd jobs that
 * resolve the promise. kick (forArm false): every dependency signalled, AS bound -> unlinked and returned.
 * fastArm (forArm true): every unsignalled dependency a running job, the wait mask in fArmWait, AS bound;
 * left in the queue (fastSync unlinks it once the filter started it).
 */
MappleGPUJob *
MappleGPUAccelerator::nextStartable(uint32_t slot, bool forArm)
{
	MappleGPUVM *blocked[16];
	uint32_t n = 0;
	MappleGPUJob *prev = NULL;
	for (MappleGPUJob *job = fQueue[slot]; job != NULL; prev = job, job = job->next) {
		if (vmListed(blocked, n, job->vm)) {
			continue;
		}
		bool ok = true;
		uint32_t wait = (forArm && fRunning[slot] != NULL) ? 1u << slot : 0;
		for (unsigned int i = 0; ok && job->deps != NULL && i < job->deps->getCount(); i++) {
			MappleGPUFence *f = (MappleGPUFence *)job->deps->getObject(i);
			if (f->signalled) {
				continue;
			}
			if (forArm && fRunning[0] != NULL && f == fRunning[0]->fence) {
				wait |= 1;
			} else if (forArm && fRunning[1] != NULL && f == fRunning[1]->fence) {
				wait |= 2;
			} else {
				ok = false;
			}
		}
		if (ok && forArm && wait == 0) {
			ok = false;             /* ready now: kick's business (its AS could not be bound) */
		}
		if (ok && asBind(job->vm)) {
			if (prev != NULL) {
				fQueueBypass++;
			}
			if (forArm) {
				fArmWait = wait;
			} else {
				unlinkQueued(slot, job, prev);
			}
			return job;
		}
		if (n == sizeof(blocked) / sizeof(blocked[0])) {
			break;
		}
		blocked[n++] = job->vm;
	}
	return NULL;
}

uint32_t
MappleGPUAccelerator::jsConfig(MappleGPUJob *job)
{
	return (uint32_t)job->vm->asSlot | JS_CFG_THREAD_PRI(8) | JS_CFG_START_FLUSH_CLEAN_INV | JS_CFG_END_FLUSH_CLEAN_INV;
}

/* The job slot registers that start `job` on the idle `slot`: gated, or from the job interrupt filter. */
void
MappleGPUAccelerator::jsWrite(uint32_t slot, MappleGPUJob *job, uint32_t config)
{
	uint32_t js = JS_BASE(slot);
	wr(js + JS_HEAD_NEXT_LO, (uint32_t)job->jc);
	wr(js + JS_HEAD_NEXT_HI, (uint32_t)(job->jc >> 32));
	wr(js + JS_AFFINITY_NEXT_LO, (uint32_t)fShaderPresent);
	wr(js + JS_AFFINITY_NEXT_HI, (uint32_t)(fShaderPresent >> 32));
	wr(js + JS_CONFIG_NEXT, config);
	/*
	 * Flush reduction (as Linux's panfrost on FLUSH_REDUCTION GPUs, every Valhall): the start-of-chain L2 clean +
	 * invalidate is skipped when a flush happened after the job was submitted. Every chain ends with one, so
	 * with jobs back to back the start flush is almost never needed.
	 */
	if (fFlushReduce) {
		wr(js + JS_FLUSH_ID_NEXT, job->flushId);
	}
	wr(js + JS_COMMAND_NEXT, JS_CMD_START);
}

void
MappleGPUAccelerator::startJob(uint32_t slot, MappleGPUJob *job)
{
	job->vm->running++;
	jsWrite(slot, job, jsConfig(job));
	jobStarted(slot, job, mach_absolute_time(), false);
}

/* Gated: `job` runs on `slot` since `at` (started here or by the filter; vm->running already counts it). */
void
MappleGPUAccelerator::jobStarted(uint32_t slot, MappleGPUJob *job, uint64_t at, bool fromFilter)
{
	if (fRunning[0] == NULL && fRunning[1] == NULL) {
		fBusyStart = at;        /* GPU busy from now (DVFS load) */
	}
	fRunning[slot] = job;
	fStopDeadline[slot] = 0;
	job->startMs = nowMs();
	job->startAt = at;
	fJobs++;
	/* start latency: from the moment it could run (submitted, every dependency signalled) */
	uint64_t ready = job->submitAt;
	for (unsigned int i = 0; job->deps != NULL && i < job->deps->getCount(); i++) {
		ready = MAX(ready, ((MappleGPUFence *)job->deps->getObject(i))->signalAt);
	}
	uint32_t kind = ready == job->submitAt ? 0 : fromFilter ? 2 : 1;
	uint64_t lat = at > ready ? at - ready : 0;
	fLatN[kind]++;
	fLatSum[kind] += lat;
	fLatMax[kind] = MAX(fLatMax[kind], lat);
	stateUpdate(at);
	if (!fTimerArmed) {
		fTimerArmed = true;
		fTimer->setTimeoutMS(TIMER_MS);
	}
	if (fDvfs && !fDvfsArmed) {
		fDvfsArmed = true;
		fWindowStart = mach_absolute_time();
		fBusyNs = 0;
		fDvfsTimer->setTimeoutMS(DVFS_WINDOW_MS);
	}
}

void
MappleGPUAccelerator::completeJob(uint32_t slot, IOReturn error, uint32_t status, uint64_t at)
{
	MappleGPUJob *job = fRunning[slot];
	if (job == NULL) {
		return;
	}
	if (at == 0) {
		at = mach_absolute_time();
	}
	fRunning[slot] = NULL;
	fStopDeadline[slot] = 0;
	job->vm->running--;
	if (fRunning[0] == NULL && fRunning[1] == NULL && fBusyStart != 0) {
		fBusyNs += at > fBusyStart ? at - fBusyStart : 0;       /* absolute-time units */
		fBusyTotal += at > fBusyStart ? at - fBusyStart : 0;
		fBusyStart = 0;
	}
	if (error == kIOReturnSuccess && at > job->startAt) {
		fExecN++;
		fExecSum += at - job->startAt;
	}
	stateUpdate(at);
	if (error != kIOReturnSuccess) {
		fJobFaults++;
		if (fJobFaults <= 32) {
			IOLog("GPU: job on slot %u failed: status 0x%02x (%s), chain 0x%llx, AS%d, after %llu ms\n",
			    slot, status, jsStatusName(status), job->jc, job->vm->asSlot, nowMs() - job->startMs);
		}
	}
	uint64_t c0 = mach_absolute_time(), bytes = 0;
	bool surfaces = false;
	for (unsigned int i = 0; job->bos != NULL && i < job->bos->getCount(); i++) {
		MappleGPUBO *bo = (MappleGPUBO *)job->bos->getObject(i);
		surfaces |= bo->kind == kMappleBOSurface;
		if (bo->cacheMaint) {
			if (bo->kind == kMappleBONormal && !(bo->flags & VGPU_BO_NOEXEC) && fLazy) {
				continue;       /* shader code: the GPU never writes it */
			}
			/*
			 * K0b: the fence signals now; the CPU lines are dropped when userspace waits for the GPU (WAIT_BO,
			 * syncobj wait), once per BO however many jobs used it. Without room in the list: at once.
			 */
			if (fLazy && bo->vm != NULL) {
				if (!bo->invPending) {
					bo->vm->invAdd(bo);
					fLazyListed++;
				}
				if (bo->invPending) {
					continue;
				}
				fLazyOverflows++;
			}
			/* drop stale CPU lines so the CPU sees what the GPU wrote; civac keeps lines the CPU dirtied meanwhile */
			bytes += bo->cacheOp(0, 0, kMappleCacheFlush);
		} else if (bo->lazySurface && mapple_gpu_surf_maint != 2) {
			mapple_surface_device_wrote(bo->surface);                /* dropped at the next IOSurfaceLock */
			if (mapple_gpu_surf_maint == 1) {
				bytes += mapple_surface_before_cpu_access(bo->surface);    /* or at once */
			}
		}
	}
	if (bytes != 0) {
		uint64_t ns;
		absolutetime_to_nanoseconds(mach_absolute_time() - c0, &ns);
		fDoneCacheNs += ns;
		fDoneCacheBytes += bytes;
	}
	MappleGPUVM *vm = job->vm;
	if (job->vmPrev != NULL) {
		job->vmPrev->vmNext = job->vmNext;
	} else if (vm->jobHead == job) {
		vm->jobHead = job->vmNext;
	}
	if (job->vmNext != NULL) {
		job->vmNext->vmPrev = job->vmPrev;
	} else if (vm->jobTail == job) {
		vm->jobTail = job->vmPrev;
	}
	vm->seqUpdate();                /* K0h: before the fence, so the page never lags what a wait already said */
	signalFence(job->fence, error, at);     /* K0g: wakes only the threads waiting for this job (and the generic ones) */
	OSSafeReleaseNULL(job->deps);
	OSSafeReleaseNULL(job->bos);
	OSSafeReleaseNULL(job->fence);
	OSSafeReleaseNULL(job->vm);
	IOFreeType(job, MappleGPUJob);
	fJobsLive--;
	if (surfaces) {
		mapple_fb_gpu_done();    /* a frame held back for the GPU may be ready: the display need not wait for a tick */
	}
}

/* Fail the jobs on the hardware (reset, power off); queued jobs stay queued (power off fails them too). */
void
MappleGPUAccelerator::failAll(IOReturn error)
{
	for (uint32_t slot = 0; slot < 2; slot++) {
		completeJob(slot, error, 0x04);
	}
	if (error == kIOReturnOffline) {
		for (uint32_t slot = 0; slot < 2; slot++) {
			while (fQueue[slot] != NULL) {
				MappleGPUJob *job = fQueue[slot];
				fQueue[slot] = job->next;
				job->next = NULL;
				fRunning[slot] = job;
				job->vm->running++;
				completeJob(slot, error, 0x04);
			}
			fQueueTail[slot] = NULL;
		}
	}
}

void
MappleGPUAccelerator::resetGPU(const char *why)
{
	fResets++;
	fWatchFragment = false;
	IOLog("GPU: reset (%s)\n", why);
	fastSync();             /* a job the filter started is one of the running ones */
	fastLive(false);
	wr(JOB_INT_MASK, 0);
	wr(MMU_INT_MASK, 0);
	softReset();            /* stop the GPU before the jobs' memory is released */
	failAll(kIOReturnAborted);
	for (uint32_t as = 0; as < fASCount; as++) {
		if (fAS[as] != NULL) {
			fAS[as]->asSlot = -1;
			fAS[as] = NULL;
		}
	}
	char buf[256], *p = buf;
	if (!maliInit(&p, buf + sizeof(buf))) {
		IOLog("GPU: reset failed, GPU unusable until gpu off/on: %s", buf);
		for (int i = 0; i < 3; i++) {
			fIrq[i]->disable();
		}
		failAll(kIOReturnOffline);
		fPowered = false;
		return;
	}
	fastLive(true);
	kick();
}

void
MappleGPUAccelerator::gpuInterrupt(OSObject *owner, IOInterruptEventSource *src __unused, int count __unused)
{
	MappleGPUAccelerator *self = OSDynamicCast(MappleGPUAccelerator, owner);
	if (self == NULL || !self->fPowered) {
		return;
	}
	uint32_t stat = self->rd(GPU_IRQ_STATUS);
	if (stat & (GPU_IRQ_FAULT | GPU_IRQ_MULTIPLE_FAULT)) {
		self->fGpuFaults++;
		if (self->fGpuFaults <= 16) {
			IOLog("GPU: GPU fault: status 0x%08x address 0x%08x%08x\n", self->rd(GPU_FAULT_STATUS),
			    self->rd(GPU_FAULT_ADDRESS_HI), self->rd(GPU_FAULT_ADDRESS_LO));
		}
	}
	self->wr(GPU_IRQ_CLEAR, stat);
}

void
MappleGPUAccelerator::mmuInterrupt(OSObject *owner, IOInterruptEventSource *src __unused, int count __unused)
{
	MappleGPUAccelerator *self = OSDynamicCast(MappleGPUAccelerator, owner);
	if (self == NULL || !self->fPowered) {
		return;
	}
	self->fastSync();       /* nothing armed while address spaces change (re-armed at the next kick) */
	uint32_t stat = self->rd(MMU_INT_STAT);
	for (uint32_t as = 0; as < self->fASCount; as++) {
		uint32_t bits = stat & ((1u << as) | (1u << (16 + as)));
		if (bits == 0) {
			continue;
		}
		uint32_t fs = self->rd(AS_BASE(as) + AS_FAULTSTATUS);
		uint64_t addr = self->rd(AS_BASE(as) + AS_FAULTADDRESS_LO) |
		    (uint64_t)self->rd(AS_BASE(as) + AS_FAULTADDRESS_HI) << 32;
		self->wr(MMU_INT_CLEAR, bits);  /* before the flush resumes the GPU: a new fault must not be lost */
		MappleGPUVM *vm = self->fAS[as];
		bool handled = false;
		if (vm != NULL && (bits & (1u << as)) && (fs & 0xf8) == 0xc0) {
			MappleGPUBO *heap = self->findHeapBO(vm, addr);
			if (heap != NULL && heap->growHeap(addr)) {
				self->fHeapGrows++;
				handled = true;
			}
		}
		if (!handled) {
			self->fMmuFaults++;
			if (vm != NULL) {
				vm->faults++;
			}
			if (self->fMmuFaults <= 32) {
				char what[96] = "no VM";
				if (vm != NULL) {
					vm->describe(addr, what, sizeof(what));
				}
				IOLog("GPU: MMU %s on AS%u at 0x%llx: status 0x%08x (%s, %s access, source 0x%04x), %s%s%s\n",
				    (bits & (1u << as)) ? "fault" : "bus error", as, addr, fs, jsStatusName(fs & 0xff),
				    ((fs >> 8) & 3) == 1 ? "execute" : ((fs >> 8) & 3) == 2 ? "read" :
				    ((fs >> 8) & 3) == 3 ? "write" : "atomic", fs >> 16, what, vm != NULL ? ", " : "",
				    vm != NULL ? vm->proc : "");
			}
			/*
			 * Kill the address space: the job on it fails. The slot stays owned by the VM (not handed to
			 * another one while that job is still on the hardware) with its interrupts masked; the VM's
			 * next job reprograms it.
			 */
			self->asDisable((int)as);
			self->wr(MMU_INT_MASK, self->rd(MMU_INT_MASK) & ~((1u << as) | (1u << (16 + as))));
			if (vm != NULL) {
				vm->asFaulted = true;
			}
		}
	}
}

void
MappleGPUAccelerator::jobInterrupt(OSObject *owner, IOInterruptEventSource *src __unused, int count __unused)
{
	MappleGPUAccelerator *self = OSDynamicCast(MappleGPUAccelerator, owner);
	if (self == NULL || !self->fPowered) {
		return;
	}
	self->fastSync();       /* the jobs the filter saw done, the ones it started */
	uint32_t stat = self->rd(JOB_INT_STAT);
	for (uint32_t slot = 0; slot < 3; slot++) {
		uint32_t done = 1u << slot, fail = 1u << (16 + slot);
		if (!(stat & (done | fail))) {
			continue;
		}
		if (stat & fail) {
			uint32_t js = JS_BASE(slot);
			uint32_t status = self->rd(js + JS_STATUS);
			self->wr(js + JS_COMMAND_NEXT, JS_CMD_NOP);
			/*
			 * A bus fault right after we disabled the job's address space (MMU fault) is the expected way
			 * that chain dies: no reset, the AS is reprogrammed for the VM's next job. TTRX_3076 (fragment
			 * jobs hanging after a bus fault) is then left to the job timeout, shortened until a fragment
			 * chain completes again. Any other bus fault still resets.
			 */
			bool asKilled = slot < 2 && self->fRunning[slot] != NULL && self->fRunning[slot]->vm->asFaulted;
			if (slot < 2) {
				self->completeJob(slot, kIOReturnIOError, status);
			}
			if ((status & 0xff) == 0x48) {
				if (asKilled) {
					self->fFaultsNoReset++;
					self->fWatchFragment = true;
				} else {
					self->wr(JOB_INT_CLEAR, stat & (done | fail));
					self->resetGPU("job bus fault (TTRX_3076: later fragment jobs may hang)");
					return;
				}
			}
		} else if (slot < 2) {
			if (slot == 0) {
				self->fWatchFragment = false;
			}
			self->completeJob(slot, kIOReturnSuccess, 0x01);
		}
		self->wr(JOB_INT_CLEAR, stat & (done | fail));
	}
	self->kick();
}

/*
 * K0e. The job interrupt's filter, primary interrupt context (CPU 0, interrupts off). Clears the done bits it sees
 * (the failures stay for the gated handler), notes when they came, and starts each armed job whose awaited slots
 * are now all done. Always true when there was something: the source stays masked until the handler has run, so
 * the filter runs at most once per handler run.
 */
bool
MappleGPUAccelerator::jobFilter(OSObject *owner, IOFilterInterruptEventSource *src __unused)
{
	return ((MappleGPUAccelerator *)owner)->jobFilterRun();
}

bool
MappleGPUAccelerator::jobFilterRun(void)
{
	IOSimpleLockLock(fFastLock);
	if (!fFastOn) {
		IOSimpleLockUnlock(fFastLock);
		return true;            /* GPU off or resetting: the handler sorts it out */
	}
	uint32_t stat = rd(JOB_INT_STAT);
	if (stat == 0) {
		IOSimpleLockUnlock(fFastLock);
		return false;
	}
	uint64_t now = mach_absolute_time();
	for (uint32_t slot = 0; slot < 2; slot++) {
		if (stat & (0x10001u << slot)) {
			fIrqAt[slot] = now;
		}
	}
	uint32_t done = stat & 3u & ~(stat >> 16);
	if (done != 0) {
		wr(JOB_INT_CLEAR, done);
		fIrqDone |= done;
	}
	for (uint32_t slot = 0; slot < 2 && fFastKick; slot++) {
		FastArm *a = &fArm[slot];
		if (a->job != NULL && !a->started && (a->wait & ~fIrqDone) == 0) {
			jsWrite(slot, a->job, a->config);
			a->started = true;
			a->startAt = mach_absolute_time();
		}
	}
	IOSimpleLockUnlock(fFastLock);
	return true;
}

/* Gated: filter on/off with the GPU's power (off: it must not read the registers; on: nothing left from before). */
void
MappleGPUAccelerator::fastLive(bool live)
{
	IOInterruptState is = IOSimpleLockLockDisableInterrupt(fFastLock);
	fFastOn = live;
	fIrqDone = 0;
	bzero(fArm, sizeof(fArm));
	IOSimpleLockUnlockEnableInterrupt(fFastLock, is);
}

/*
 * Gated, before anything looks at fRunning/fQueue: complete the jobs the filter saw done (at the time it saw them),
 * take over the jobs it started, release the reservation of those it did not. Nothing is armed afterwards (until
 * fastArm), so the filter cannot start a job behind the gated code's back.
 */
void
MappleGPUAccelerator::fastSync(void)
{
	FastArm arm[2];
	uint64_t at[2];
	IOInterruptState is = IOSimpleLockLockDisableInterrupt(fFastLock);
	uint32_t done = fIrqDone;
	fIrqDone = 0;
	at[0] = fIrqAt[0];
	at[1] = fIrqAt[1];
	memcpy(arm, fArm, sizeof(arm));
	bzero(fArm, sizeof(fArm));
	IOSimpleLockUnlockEnableInterrupt(fFastLock, is);

	if (done != 0) {
		uint64_t now = mach_absolute_time(), first = (done & 1) ? at[0] : at[1];
		if ((done & 3) == 3) {
			first = MIN(at[0], at[1]);
		}
		uint64_t d = now > first ? now - first : 0;
		fIrqN++;
		fIrqSum += d;
		fIrqMax = MAX(fIrqMax, d);
	}
	for (uint32_t slot = 0; slot < 2; slot++) {
		if (done & (1u << slot)) {
			if (slot == 0) {
				fWatchFragment = false;
			}
			completeJob(slot, kIOReturnSuccess, 0x01, at[slot]);
		}
	}
	for (uint32_t slot = 0; slot < 2; slot++) {
		MappleGPUJob *job = arm[slot].job;
		if (job == NULL) {
			continue;
		}
		if (!arm[slot].started) {
			job->vm->running--;             /* reservation of fastArm */
			continue;
		}
		/* armed = still queued (K0j: not necessarily the head), and nothing dequeues without fastSync first */
		MappleGPUJob *prev = NULL;
		for (MappleGPUJob *q = fQueue[slot]; q != NULL && q != job; q = q->next) {
			prev = q;
		}
		unlinkQueued(slot, job, prev);
		fFastStarts++;
		jobStarted(slot, job, arm[slot].startAt, true);
	}
}

/*
 * Gated, after kick: arm the head of each slot's queue when every dependency it still waits for is a job on the
 * hardware (and its own slot frees up when that slot's job is done). Its address space is bound now and reserved
 * (vm->running) so it is not given away before the filter starts it.
 */
void
MappleGPUAccelerator::fastArm(void)
{
	if (!fFastKick || !fFastOn) {
		return;
	}
	for (uint32_t slot = 0; slot < 2; slot++) {
		if (fQueue[slot] == NULL) {
			continue;
		}
		/* waits only for jobs on the hardware (a queued job or a syncobj: the normal path) */
		MappleGPUJob *job = nextStartable(slot, true);
		if (job == NULL) {
			continue;
		}
		uint32_t wait = fArmWait;
		job->vm->running++;
		IOInterruptState is = IOSimpleLockLockDisableInterrupt(fFastLock);
		fArm[slot].job = job;
		fArm[slot].wait = wait;
		fArm[slot].config = jsConfig(job);
		fArm[slot].started = false;
		fArm[slot].startAt = 0;
		IOSimpleLockUnlockEnableInterrupt(fFastLock, is);
		fFastArmed++;
	}
}

/* Gated: something changed at `at`; the time since the last change goes to the state the GPU was in. */
void
MappleGPUAccelerator::stateUpdate(uint64_t at)
{
	if (fStatSince == 0) {
		fStatSince = fStateSince = at;
	}
	if (at > fStateSince) {
		fStateNs[fState] += at - fStateSince;
		fStateSince = at;
	}
	fState = (fRunning[0] != NULL || fRunning[1] != NULL) ? kStateBusy :
	    (fQueue[0] != NULL || fQueue[1] != NULL) ? kStateWaiting : kStateIdle;
}

void
MappleGPUAccelerator::latencyReset(void)
{
	fStatSince = fStateSince = mach_absolute_time();
	bzero(fStateNs, sizeof(fStateNs));
	bzero(fLatN, sizeof(fLatN));
	bzero(fLatSum, sizeof(fLatSum));
	bzero(fLatMax, sizeof(fLatMax));
	fExecN = fExecSum = fIrqN = fIrqSum = fIrqMax = fFastStarts = fFastArmed = 0;
}

static uint64_t
absUs(uint64_t abs)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(abs, &ns);
	return ns / 1000;
}

void
MappleGPUAccelerator::latencyStats(char **out, char *end)
{
	stateUpdate(mach_absolute_time());
	uint64_t total = fStateNs[kStateBusy] + fStateNs[kStateWaiting] + fStateNs[kStateIdle];
	uint64_t t = total ? total : 1;
	say(out, end, "timing over %llu ms: GPU busy %llu%%, waiting with jobs queued %llu%%, idle (nothing queued) "
	    "%llu%%; job on the hardware avg %llu us (%llu jobs)\n", absUs(total) / 1000, fStateNs[kStateBusy] * 100 / t,
	    fStateNs[kStateWaiting] * 100 / t, fStateNs[kStateIdle] * 100 / t,
	    fExecN ? absUs(fExecSum / fExecN) : 0, fExecN);
	static const char *const kind[3] = { "ready at submit", "after a dependency, work loop", "after a dependency, IRQ filter" };
	say(out, end, "start latency (ready -> running):");
	for (int i = 0; i < 3; i++) {
		say(out, end, " %s avg %llu us max %llu (%llu)%s", kind[i], fLatN[i] ? absUs(fLatSum[i] / fLatN[i]) : 0,
		    absUs(fLatMax[i]), fLatN[i], i < 2 ? ";" : "\n");
	}
	say(out, end, "IRQ -> handler avg %llu us max %llu (%llu); fast kick %s: %llu armed, %llu started by the filter "
	    "(mp_gpu_fastkick, gpu fastkick on|off; gpu lat reset)\n", fIrqN ? absUs(fIrqSum / fIrqN) : 0, absUs(fIrqMax),
	    fIrqN, fFastKick ? "on" : "off", fFastArmed, fFastStarts);
}

/* K0g: per-selector cost of the user client calls. Any thread. */
void
MappleGPUAccelerator::callDone(uint32_t selector, uint64_t abs, bool trap)
{
	if (selector >= VGPU_SEL_COUNT) {
		return;
	}
	IOInterruptState is = IOSimpleLockLockDisableInterrupt(fCallLock);
	CallStat *c = &fCalls[selector];
	c->n++;
	c->trapN += trap;
	c->sum += abs;
	c->max = MAX(c->max, abs);
	IOSimpleLockUnlockEnableInterrupt(fCallLock, is);
}

/* Gated (the caller just got the gate). */
void
MappleGPUAccelerator::gateWaited(uint32_t selector, uint64_t abs)
{
	if (selector >= VGPU_SEL_COUNT) {
		return;
	}
	IOInterruptState is = IOSimpleLockLockDisableInterrupt(fCallLock);
	CallStat *c = &fCalls[selector];
	c->gateN++;
	c->gateSum += abs;
	c->gateMax = MAX(c->gateMax, abs);
	IOSimpleLockUnlockEnableInterrupt(fCallLock, is);
}

void
MappleGPUAccelerator::callReset(void)
{
	IOInterruptState is = IOSimpleLockLockDisableInterrupt(fCallLock);
	bzero(fCalls, sizeof(fCalls));
	IOSimpleLockUnlockEnableInterrupt(fCallLock, is);
	fWakeFence = fWakeGeneric = fSleeps = fSleepsSpurious = 0;
}

void
MappleGPUAccelerator::callStats(char **out, char *end)
{
	static const char *const names[VGPU_SEL_COUNT] = {
		"version", "param", "create_bo", "mmap_bo", "bo_offset", "gem_close", "submit", "wait_bo", "madvise",
		"sync_create", "sync_destroy", "sync_wait", "sync_reset", "sync_signal", "import_surf", "sync_bo",
		"bo_info", "ctx_create", "ctx_destroy", "sync_xfer", "import_user", "submit_batch", "surf_promise",
	};
	CallStat c[VGPU_SEL_COUNT];
	IOInterruptState is = IOSimpleLockLockDisableInterrupt(fCallLock);
	memcpy(c, fCalls, sizeof(c));
	IOSimpleLockUnlockEnableInterrupt(fCallLock, is);
	say(out, end, "calls (entry -> return, us; gate = waiting for the gate; gpu calls reset):");
	bool any = false;
	for (uint32_t s = 0; s < VGPU_SEL_COUNT; s++) {
		if (c[s].n == 0) {
			continue;
		}
		any = true;
		say(out, end, "\n  %-12s %8llu (%llu trap) avg %llu max %llu, gate avg %llu max %llu", names[s], c[s].n, c[s].trapN,
		    absUs(c[s].sum / c[s].n), absUs(c[s].max), c[s].gateN ? absUs(c[s].gateSum / c[s].gateN) : 0,
		    absUs(c[s].gateMax));
	}
	say(out, end, "%s\nwaits: %llu sleeps (%llu woke with nothing done), %llu fence wakeups, %llu generic wakeups, %u generic "
	    "waiters now\n", any ? "" : " none yet", fSleeps, fSleepsSpurious, fWakeFence, fWakeGeneric, fGenericWaiters);
	say(out, end, "surface promises (K0i): %llu made, %u pending, %llu timed out; K0j: %llu waits past an own "
	    "promise, %llu jobs started ahead of another connection's, %llu queued jobs dropped at close, dependency walk "
	    "%llu nodes + %llu cached in %llu walks (%llu cut short)\n", fPromisesMade,
	    fPromises != NULL ? fPromises->getCount() : 0, fPromisesTimedOut, fPromiseBypass, fQueueBypass, fJobsDiscarded,
	    fK0jVisits, fK0jCached, fK0jWalk, fK0jCut);
}

/*
 * "gpu clocks": the PLLs in apmixedsys (MT6781 clk-mt6781.c offsets): CON0 bit 0 = enabled, CON1 = PCW (22 bits, 14
 * fractional, from 26 MHz) and post divider [26:24]. The CPU PLLs are driven by the MCUPM firmware's DVFS on Android;
 * here they stay where the preloader/LK left them. Read only.
 */
#define PLL_AP_CON3             0x00c
#define PLL_AP_CON4             0x010
#define PLL_ARM_LL_CON0         0x200   /* little cluster, Cortex-A55 */
#define PLL_ARM_BL_CON0         0x210   /* big cluster, Cortex-A76 */
#define PLL_ARM_BB_CON0         0x220
#define PLL_CCI_CON0            0x2a0

static uint32_t
pllMHz(uint32_t con1)
{
	uint64_t pcw = con1 & 0x3fffff;
	uint32_t posdiv = (con1 >> 24) & 7;
	return (uint32_t)(((26ull * pcw) >> 14) >> posdiv);
}

void
MappleGPUAccelerator::clocksDump(char **out, char *end)
{
	static const struct { const char *name; uint32_t con0; } plls[] = {
		{ "armpll_ll (A55)", PLL_ARM_LL_CON0 }, { "armpll_bl (A76)", PLL_ARM_BL_CON0 },
		{ "armpll_bb", PLL_ARM_BB_CON0 }, { "ccipll", PLL_CCI_CON0 }, { "mfgpll", PLL_MFG_CON0 },
	};
	for (uint32_t i = 0; i < sizeof(plls) / sizeof(plls[0]); i++) {
		uint32_t con0 = rd32(fApmixed, plls[i].con0), con1 = rd32(fApmixed, plls[i].con0 + 4);
		say(out, end, "%-16s %s %4u MHz (con0 0x%08x con1 0x%08x)\n", plls[i].name, (con0 & 1) ? "on " : "off",
		    pllMHz(con1), con0, con1);
	}
	say(out, end, "ap_pll_con3 0x%08x con4 0x%08x (CPU PLL hardware-control select bits)\n", rd32(fApmixed, PLL_AP_CON3),
	    rd32(fApmixed, PLL_AP_CON4));
	/* timeouts: userspace CLOCK_MONOTONIC (what deadlines use since 0120) vs Mach continuous time (before) */
	uint64_t mono = mapple_gpu_monotonic_ns(), cont;
	absolutetime_to_nanoseconds(mach_continuous_time(), &cont);
	say(out, end, "clocks: CLOCK_MONOTONIC (calendar - boottime) %llu ms, mach_continuous_time %llu ms, difference %lld ms\n",
	    mono / NSEC_PER_MSEC, cont / NSEC_PER_MSEC, ((int64_t)cont - (int64_t)mono) / (int64_t)NSEC_PER_MSEC);
}

/*
 * A job past its time: what the hardware says, to tell a fault (JS_STATUS 0x40 and up, a GPU or AS fault status)
 * from a job still running (JS_STATUS 0x08 ACTIVE, no fault) that is only slow.
 */
void
MappleGPUAccelerator::logStuckJob(uint32_t slot, const char *when)
{
	MappleGPUJob *job = fRunning[slot];
	uint32_t js = JS_BASE(slot);
	int as = job != NULL && job->vm != NULL ? job->vm->asSlot : -1;

	IOLog("GPU: slot %u %s (%s[%d]): JS_STATUS 0x%02x head 0x%08x%08x tail 0x%08x%08x; GPU_STATUS 0x%x IRQ_RAWSTAT 0x%x "
	    "FAULT_STATUS 0x%x at 0x%08x%08x; JOB_INT_RAWSTAT 0x%x\n", slot, when, job != NULL && job->vm != NULL ? job->vm->proc : "?",
	    job != NULL && job->vm != NULL ? job->vm->pid : -1, rd(js + JS_STATUS), rd(js + JS_HEAD_HI),
	    rd(js + JS_HEAD_LO), rd(js + JS_TAIL_LO + 4), rd(js + JS_TAIL_LO), rd(GPU_STATUS), rd(GPU_IRQ_RAWSTAT),
	    rd(GPU_FAULT_STATUS), rd(GPU_FAULT_ADDRESS_HI), rd(GPU_FAULT_ADDRESS_LO), rd(JOB_INT_RAWSTAT));
	if (as >= 0) {
		uint32_t a = AS_BASE((uint32_t)as);

		IOLog("GPU: slot %u %s: AS %d FAULTSTATUS 0x%x at 0x%08x%08x, STATUS 0x%x\n", slot, when, as,
		    rd(a + AS_FAULTSTATUS), rd(a + AS_FAULTADDRESS_HI), rd(a + AS_FAULTADDRESS_LO), rd(a + AS_STATUS));
	}
}

void
MappleGPUAccelerator::jobTimer(OSObject *owner, IOTimerEventSource *src __unused)
{
	MappleGPUAccelerator *self = OSDynamicCast(MappleGPUAccelerator, owner);
	if (self == NULL) {
		return;
	}
	self->fTimerArmed = false;
	self->promiseTimeouts();
	bool promises = self->fPromises != NULL && self->fPromises->getCount() != 0;
	if (!self->fPowered) {
		if (promises) {
			self->fTimerArmed = true;
			self->fTimer->setTimeoutMS(TIMER_MS);
		}
		return;
	}
	self->fastSync();
	uint64_t now = nowMs();
	bool busy = promises;
	for (uint32_t slot = 0; slot < 2; slot++) {
		MappleGPUJob *job = self->fRunning[slot];
		if (job == NULL) {
			continue;
		}
		busy = true;
		if (self->fStopDeadline[slot] != 0 && now >= self->fStopDeadline[slot]) {
			self->logStuckJob(slot, "after its hard stop");
			self->resetGPU("a job ignored its hard stop");
			if (self->fPowered && (self->fRunning[0] || self->fRunning[1])) {
				self->fTimerArmed = true;
				self->fTimer->setTimeoutMS(TIMER_MS);
			}
			return;
		}
		uint64_t limit = (slot == 0 && self->fWatchFragment) ? JOB_TIMEOUT_AFTER_FAULT_MS : self->fJobTimeoutMs;
		if (self->fStopDeadline[slot] == 0 && now - job->startMs > limit) {
			self->fTimeouts++;
			IOLog("GPU: job on slot %u timed out after %llu ms (chain 0x%llx head 0x%08x%08x), hard stop\n",
			    slot, now - job->startMs, job->jc, self->rd(JS_BASE(slot) + JS_HEAD_HI),
			    self->rd(JS_BASE(slot) + JS_HEAD_LO));
			self->logStuckJob(slot, "at its timeout");
			self->wr(JS_BASE(slot) + JS_COMMAND, JS_CMD_HARD_STOP);
			self->fStopDeadline[slot] = now + JOB_STOP_GRACE_MS;
		}
	}
	if (busy) {
		self->fTimerArmed = true;
		self->fTimer->setTimeoutMS(TIMER_MS);
	}
}

/*
 * DVFS governor (mp_gpu_dvfs=1, default): every DVFS_WINDOW_MS, the share of time a chain was on the hardware.
 * Busier than DVFS_UP_PCT: straight to the fastest OPP (mp_gpu_freqmax); otherwise, no sooner than DVFS_HOLD_MS
 * after a step up, down to the slowest OPP that would still be at most DVFS_TARGET_PCT busy, never below
 * mp_gpu_freq. setOpp() steps the rails 50 mV at a time with jobs running (about 1 ms, gated). Idle at the floor:
 * the timer stops until the next job.
 */
void
MappleGPUAccelerator::dvfsTimer(OSObject *owner, IOTimerEventSource *src __unused)
{
	MappleGPUAccelerator *self = OSDynamicCast(MappleGPUAccelerator, owner);
	if (self == NULL) {
		return;
	}
	self->dvfsSample();
}

void
MappleGPUAccelerator::dvfsSample(void)
{
	fDvfsArmed = false;
	if (!fPowered || !fDvfs) {
		return;
	}
	uint64_t now = mach_absolute_time(), busy = fBusyNs, window = now - fWindowStart;
	if (fBusyStart != 0) {
		busy += now - fBusyStart;
		fBusyTotal += now - fBusyStart;
		fBusyStart = now;
	}
	fBusyNs = 0;
	fWindowStart = now;
	uint32_t pct = window ? (uint32_t)MIN(busy * 100 / window, 100ull) : 0;
	fLoadPct = pct;
	/* OPP indexes: 0 is the fastest; fOpp (mp_gpu_freq, "gpu freq") the slowest allowed, fDvfsFast the fastest */
	uint32_t fast = MIN(fDvfsFast, fOpp), cur = fClockOpp, want = cur;
	if (gMappleGPUCapMHz != 0) {
		fast = MIN(MAX(fast, oppForMHz(gMappleGPUCapMHz)), fOpp);   /* Low Power Mode ceiling */
	}
	uint64_t msec = nowMs();
	uint32_t waits = mapple_fb_gpu_waits;
	bool boost = fDvfsBoost && waits != fDvfsSeenWaits;
	fDvfsSeenWaits = waits;
	if (boost) {
		want = fast;            /* 0137: a frame missed its vsync waiting for us (bursty load averages low) */
		fDvfsBoosts++;
	}
	if (pct >= DVFS_UP_PCT || boost) {
		want = fast;
	} else if (msec - fDvfsUpMs >= DVFS_HOLD_MS) {
		uint32_t khz = (uint32_t)((uint64_t)gMappleOpp[cur].khz * pct / DVFS_TARGET_PCT);
		while (want + 1 < OPP_COUNT && gMappleOpp[want + 1].khz >= khz) {
			want++;         /* slower while still fast enough */
		}
	}
	want = MIN(MAX(want, fast), fOpp);
	if (want != cur) {
		if (gMappleOpp[want].khz > gMappleOpp[cur].khz) {
			fDvfsUps++;
			fDvfsUpMs = msec;
		} else {
			fDvfsDowns++;
		}
		uint32_t floor = fOpp;
		setOpp(want, NULL, NULL);
		fOpp = floor;
	}
	if (pct != 0 || fClockOpp != fOpp || fRunning[0] != NULL || fRunning[1] != NULL) {
		fDvfsArmed = true;
		fDvfsTimer->setTimeoutMS(DVFS_WINDOW_MS);
	}
}

/*
 * K0g, targeted wakeups. Before, every waiter slept on one event that every job completion, submit and syncobj
 * change woke: with ~100 jobs per frame each sleeping thread of every process (Mesa's WAIT_BO, the Metal driver's
 * completion threads, IOSurfaceLock) woke ~200 times per frame, took the gate, found nothing and slept again, and
 * submits queued behind them for the gate. Now a thread waiting for one fence sleeps on that fence, and the shared
 * event is only for waits on several syncobjs (or one without a fence yet), woken only when such waiters exist.
 */
static IOReturn
sleepResult(int r)
{
	return r == THREAD_TIMED_OUT ? kIOReturnTimeout : r == THREAD_INTERRUPTED ? kIOReturnAborted : kIOReturnSuccess;
}

IOReturn
MappleGPUAccelerator::sleepOn(MappleGPUFence *fence, uint64_t deadline)
{
	if (fence->signalled) {
		return kIOReturnSuccess;
	}
	fence->waiters++;
	int r = deadline ? fGate->commandSleep(fence, deadline, THREAD_ABORTSAFE) :
	    fGate->commandSleep(fence, THREAD_ABORTSAFE);
	fence->waiters--;
	fSleeps++;
	if (r == THREAD_AWAKENED && !fence->signalled) {
		fSleepsSpurious++;      /* dropped from a syncobj, or a generic wake */
	}
	return sleepResult(r);
}

IOReturn
MappleGPUAccelerator::sleepUntil(uint64_t deadline)
{
	fGenericWaiters++;
	int r = deadline ? fGate->commandSleep(&fFenceEvent, deadline, THREAD_ABORTSAFE) :
	    fGate->commandSleep(&fFenceEvent, THREAD_ABORTSAFE);
	fGenericWaiters--;
	fSleeps++;
	return sleepResult(r);
}

void
MappleGPUAccelerator::fenceWake(MappleGPUFence *fence)
{
	if (fence != NULL && fence->waiters != 0) {
		fWakeFence++;
		fGate->commandWakeup(fence, false);
	}
	wakeWaiters();
}

void
MappleGPUAccelerator::signalFence(MappleGPUFence *fence, IOReturn error, uint64_t at)
{
	if (fence == NULL || fence->signalled) {
		return;
	}
	fence->signalled = true;
	fence->error = error;
	fence->signalAt = at;
	OSSafeReleaseNULL(fence->deps);         /* K0j: only unsignalled fences keep what they waited for */
	if (fence->promise) {
		fK0jEpoch++;
		unsigned int idx = fPromises != NULL ? fPromises->getNextIndexOfObject(fence, 0) : (unsigned int)-1;
		if (idx != (unsigned int)-1) {
			fPromises->removeObject(idx);   /* fPromises = pending promises only (vmHasPromise stays short) */
		}
		/*
		 * 0135: the surfaces it covered may now be idle. A promise resolved with a fence that had already signalled
		 * (the replay's last job done before its TRANSFER) used to wake nobody: the display found the frame only at
		 * the next vsync tick and switched it one frame after that.
		 */
		mapple_fb_gpu_done();
	}
	fenceWake(fence);
	OSArray *followers = fence->followers;
	fence->followers = NULL;
	for (unsigned int i = 0; followers != NULL && i < followers->getCount(); i++) {
		signalFence((MappleGPUFence *)followers->getObject(i), error, at);     /* promises resolved by it */
	}
	OSSafeReleaseNULL(followers);
}

/*
 * K0i. A promise is a fence put on IOSurfaces before the jobs that write them exist (the Metal driver replays a
 * command buffer after commit returns): readers (IOSurfaceLock, the display, other processes' jobs) wait for it
 * instead of seeing the surface idle. It follows the fence that later replaces it in its syncobj.
 */
void
MappleGPUAccelerator::resolvePromise(MappleGPUFence *p, MappleGPUFence *to)
{
	if (p == NULL || !p->promise || p->signalled || p == to) {
		return;
	}
	if (to == NULL || to->signalled) {
		signalFence(p, to != NULL ? to->error : kIOReturnSuccess, mach_absolute_time());
		kick();                 /* K0j: jobs waiting for it can start now, not at the next submit */
		return;
	}
	if (to->followers == NULL) {
		to->followers = OSArray::withCapacity(2);
	}
	if (to->followers == NULL || !to->followers->setObject(p)) {
		signalFence(p, kIOReturnSuccess, mach_absolute_time());    /* no memory: never leave readers waiting */
		kick();
		return;
	}
	OSSafeReleaseNULL(p->deps);
	p->deps = OSArray::withObjects((const OSObject **)&to, 1);     /* K0j: it now waits for `to` */
	fK0jEpoch++;
}

MappleGPUFence *
MappleGPUAccelerator::makePromise(MappleGPUVM *vm, uint32_t timeoutMs, MappleSurface **surfaces, uint32_t count)
{
	MappleGPUFence *p = MappleGPUFence::create(false);
	if (p == NULL || (fPromises == NULL && (fPromises = OSArray::withCapacity(8)) == NULL) || !fPromises->setObject(p)) {
		OSSafeReleaseNULL(p);
		return NULL;
	}
	p->promise = true;
	p->owner = vm;
	fK0jEpoch++;
	clock_interval_to_deadline(timeoutMs, kMillisecondScale, &p->deadline);
	for (uint32_t i = 0; i < count; i++) {
		MappleSurface *s = surfaces[i];
		MappleGPUFence *last = OSDynamicCast(MappleGPUFence, s->gpuFence);
		if (last != NULL && last->promise && !last->signalled && last->owner == vm) {
			/* an earlier promise of ours still pending: ours come after it, its predecessor stays the base */
			last->retain();
			OSSafeReleaseNULL(s->gpuPrev);
			s->gpuPrev = last;
		} else {
			OSSafeReleaseNULL(s->gpuPrev);
			s->gpuPrev = s->gpuFence;           /* the reference moves */
			s->gpuFence = NULL;
		}
		OSSafeReleaseNULL(s->gpuFence);
		p->retain();
		s->gpuFence = p;
		s->gpuJobs++;
	}
	fPromisesMade++;
	if (!fTimerArmed) {
		fTimerArmed = true;
		fTimer->setTimeoutMS(TIMER_MS);
	}
	return p;
}

/* Gated (job timer): drop resolved promises, signal the ones past their deadline (their submit never came). */
void
MappleGPUAccelerator::promiseTimeouts(void)
{
	uint64_t now = mach_absolute_time();
	bool kicked = false;
	for (unsigned int i = 0; fPromises != NULL && i < fPromises->getCount();) {
		MappleGPUFence *p = (MappleGPUFence *)fPromises->getObject(i);
		if (p->signalled) {
			fPromises->removeObject(i);
			continue;
		}
		if (now >= p->deadline) {
			fPromisesTimedOut++;
			if (fPromisesTimedOut <= 8) {
				IOLog("GPU: surface promise not resolved in time, signalled (readers may see it unfinished)\n");
			}
			p->retain();
			signalFence(p, kIOReturnTimeout, now);  /* takes it out of fPromises */
			p->release();
			kicked = true;
			continue;
		}
		i++;
	}
	if (kicked) {
		kick();                 /* K0j: the jobs that waited for them start now (they used to stay queued) */
	}
}

/* ---- K0j: waits that must not wait for a connection's own pending promise ---- */

bool
MappleGPUAccelerator::vmHasPromise(MappleGPUVM *vm)
{
	for (unsigned int i = 0; vm != NULL && fPromises != NULL && i < fPromises->getCount(); i++) {
		MappleGPUFence *p = (MappleGPUFence *)fPromises->getObject(i);
		if (!p->signalled && p->owner == vm) {
			return true;
		}
	}
	return false;
}

/*
 * Gated: does unsignalled `f` wait, through job dependencies and resolved promises, for a pending promise of vm?
 * 0128: depth-first with an explicit stack. The recursive version went one 64-byte kernel frame per fence, and with
 * the asynchronous replay the chains of unfinished jobs passed 250 fences: "Invalid kernel stack pointer (probable
 * overflow)" with dependsOnPromiseOf all down the backtrace. Each node is entered once per walk (a DAG with shared
 * dependencies used to be walked once per path; a cycle, until the budget ran out). The budget counts distinct nodes.
 */
bool
MappleGPUAccelerator::dependsOnPromiseOf(MappleGPUFence *f, MappleGPUVM *vm, uint32_t *budget)
{
	if (f == NULL || f->signalled) {
		return false;
	}
	if (f->promise && f->owner == vm) {
		return true;
	}
	if (f->k0jEpoch == fK0jEpoch && f->k0jVM == vm) {
		fK0jCached++;
		return f->k0jDep;       /* nothing about promises changed since it was worked out */
	}
	if (*budget == 0) {
		fK0jCut++;
		return false;           /* too deep to tell: wait as before (the promise timeout is the backstop) */
	}
	if (fK0jStack == NULL && (fK0jStack = IONew(K0jFrame, kMappleK0jDepth)) == NULL) {
		return false;
	}
	uint64_t walk = ++fK0jWalk;
	uint32_t sp = 0;
	bool dep = false, cut = false;
	(*budget)--;
	fK0jVisits++;
	f->k0jWalk = walk;
	fK0jStack[0].f = f;
	fK0jStack[0].next = 0;
	fK0jStack[0].cond = false;
	sp = 1;
	while (sp != 0) {
		K0jFrame *top = &fK0jStack[sp - 1];
		MappleGPUFence *n = top->f;
		if (n->deps == NULL || top->next >= n->deps->getCount()) {
			/* all of n's dependencies said no */
			if (!top->cond) {
				n->k0jEpoch = fK0jEpoch;
				n->k0jVM = vm;
				n->k0jDep = false;
			}
			sp--;
			if (top->cond && sp != 0) {
				fK0jStack[sp - 1].cond = true;
			}
			continue;
		}
		MappleGPUFence *d = (MappleGPUFence *)n->deps->getObject(top->next++);
		if (d == NULL || d->signalled) {
			continue;
		}
		if (d->promise && d->owner == vm) {
			dep = true;
			break;
		}
		if (d->k0jEpoch == fK0jEpoch && d->k0jVM == vm) {
			fK0jCached++;
			if (d->k0jDep) {
				dep = true;
				break;
			}
			continue;
		}
		if (d->k0jWalk == walk) {
			top->cond = true;       /* on the path (a cycle) or already seen without a final answer */
			continue;
		}
		if (*budget == 0 || sp == kMappleK0jDepth) {
			top->cond = true;
			cut = true;
			continue;
		}
		(*budget)--;
		fK0jVisits++;
		d->k0jWalk = walk;
		fK0jStack[sp].f = d;
		fK0jStack[sp].next = 0;
		fK0jStack[sp].cond = false;
		sp++;
	}
	if (dep) {
		for (uint32_t i = 0; i < sp; i++) {     /* the path to the promise: all of it depends on it */
			fK0jStack[i].f->k0jEpoch = fK0jEpoch;
			fK0jStack[i].f->k0jVM = vm;
			fK0jStack[i].f->k0jDep = true;
		}
	} else if (cut) {
		fK0jCut++;
	}
	return dep;
}

/*
 * Gated: the fences connection `vm` has to wait for instead of `f`. A fence that depends on one of vm's pending
 * promises runs after vm's promised work (which is what vm is doing now), so waiting for it is a deadlock: vm waits
 * for what it waited for instead, recursively. vm's own promise: nothing (vm resolves it).
 */
void
MappleGPUAccelerator::collectWait(MappleGPUFence *f, MappleGPUVM *vm, OSArray *out, uint32_t depth)
{
	if (f == NULL || f->signalled || (f->promise && f->owner == vm)) {
		return;
	}
	if (depth == 0) {
		fK0jCollect++;
	} else if (f->k0jCollect == fK0jCollect) {
		return;                 /* 0128: reached already through another dependency (a diamond, not a new path) */
	}
	f->k0jCollect = fK0jCollect;
	uint32_t budget = 512;
	if (depth >= 16 || !dependsOnPromiseOf(f, vm, &budget)) {
		if (out->getNextIndexOfObject(f, 0) == (unsigned int)-1) {
			out->setObject(f);
		}
		return;
	}
	fPromiseBypass++;
	for (unsigned int i = 0; f->deps != NULL && i < f->deps->getCount(); i++) {
		collectWait((MappleGPUFence *)f->deps->getObject(i), vm, out, depth + 1);
	}
}

void
MappleGPUAccelerator::addImplicitDepFor(OSArray *deps, MappleGPUFence *f, MappleGPUFence *self, MappleGPUVM *vm)
{
	if (f == NULL || f->signalled) {
		return;
	}
	uint32_t budget = 512;
	OSArray *list = (vmHasPromise(vm) && dependsOnPromiseOf(f, vm, &budget)) ? OSArray::withCapacity(4) : NULL;
	if (list == NULL) {
		addImplicitDep(deps, f, self);  /* the common case: nothing to look past, no allocation */
		return;
	}
	collectWait(f, vm, list, 0);
	for (unsigned int i = 0; i < list->getCount(); i++) {
		addImplicitDep(deps, (MappleGPUFence *)list->getObject(i), self);
	}
	list->release();
}

MappleGPUFence *
MappleGPUAccelerator::boWaitFence(MappleGPUBO *bo)
{
	MappleGPUFence *first = boBusyFence(bo);
	if (first == NULL) {
		return NULL;
	}
	if (!vmHasPromise(bo->vm)) {
		first->retain();
		return first;
	}
	MappleGPUFence *sf0 = NULL;
	if (bo->kind == kMappleBOSurface && bo->surface != NULL) {
		sf0 = OSDynamicCast(MappleGPUFence, bo->surface->gpuFence);
		if (sf0 != NULL && sf0->promise && !sf0->signalled && sf0->owner == bo->vm) {
			sf0 = OSDynamicCast(MappleGPUFence, bo->surface->gpuPrev);
		}
	}
	uint32_t budget = 512;
	if (!dependsOnPromiseOf(bo->busy, bo->vm, &budget) && !dependsOnPromiseOf(sf0, bo->vm, &budget)) {
		first->retain();        /* nothing to look past: as before */
		return first;
	}
	OSArray *list = OSArray::withCapacity(4);
	if (list == NULL) {
		first->retain();
		return first;
	}
	/* the BO's last job and (an IOSurface) the surface's last fence, both through collectWait */
	collectWait(bo->busy, bo->vm, list, 0);
	if (bo->kind == kMappleBOSurface && bo->surface != NULL) {
		MappleGPUFence *sf = OSDynamicCast(MappleGPUFence, bo->surface->gpuFence);
		if (sf != NULL && sf->promise && !sf->signalled && sf->owner == bo->vm) {
			sf = OSDynamicCast(MappleGPUFence, bo->surface->gpuPrev);
		}
		collectWait(sf, bo->vm, list, 0);
	}
	MappleGPUFence *f = NULL;
	for (unsigned int i = 0; f == NULL && i < list->getCount(); i++) {
		MappleGPUFence *c = (MappleGPUFence *)list->getObject(i);
		if (!c->signalled) {
			f = c;
			f->retain();
		}
	}
	list->release();
	return f;
}

/* Gated: a queued job that will never run (its connection closed): out of its connection's list, fence signalled. */
void
MappleGPUAccelerator::discardJob(MappleGPUJob *job, IOReturn error)
{
	MappleGPUVM *vm = job->vm;
	if (job->vmPrev != NULL) {
		job->vmPrev->vmNext = job->vmNext;
	} else if (vm->jobHead == job) {
		vm->jobHead = job->vmNext;
	}
	if (job->vmNext != NULL) {
		job->vmNext->vmPrev = job->vmPrev;
	} else if (vm->jobTail == job) {
		vm->jobTail = job->vmPrev;
	}
	vm->seqUpdate();
	fK0jEpoch++;
	signalFence(job->fence, error, mach_absolute_time());
	OSSafeReleaseNULL(job->deps);
	OSSafeReleaseNULL(job->bos);
	OSSafeReleaseNULL(job->fence);
	OSSafeReleaseNULL(job->vm);
	IOFreeType(job, MappleGPUJob);
	fJobsLive--;
	fJobsDiscarded++;
}

/*
 * Gated, K0j: the connection is closing (its process may be dead: vctl kill backboardd). Its queued jobs would run
 * after its BOs and address space are gone and fault: drop them (their fences signal kIOReturnAborted, so other
 * processes' jobs waiting for them go on). Wait (up to 1 s, then the job timeout's reset) for its running jobs.
 */
void
MappleGPUAccelerator::abortVM(MappleGPUVM *vm)
{
	if (vm == NULL) {
		return;
	}
	fastSync();
	uint64_t dropped = 0;
	for (uint32_t slot = 0; slot < 2; slot++) {
		MappleGPUJob *prev = NULL, *job = fQueue[slot];
		while (job != NULL) {
			MappleGPUJob *next = job->next;
			if (job->vm == vm) {
				unlinkQueued(slot, job, prev);
				discardJob(job, kIOReturnAborted);
				dropped++;
			} else {
				prev = job;
			}
			job = next;
		}
	}
	if (dropped != 0) {
		IOLog("GPU: connection closed with %llu queued jobs: dropped\n", dropped);
		stateUpdate(mach_absolute_time());
	}
	kick();
	uint64_t deadline;
	clock_interval_to_deadline(1000, kMillisecondScale, &deadline);
	for (;;) {
		MappleGPUFence *f = NULL;
		for (uint32_t slot = 0; slot < 2 && f == NULL; slot++) {
			if (fRunning[slot] != NULL && fRunning[slot]->vm == vm) {
				f = fRunning[slot]->fence;
			}
		}
		if (f == NULL) {
			break;
		}
		f->retain();
		IOReturn r = sleepOn(f, deadline);
		f->release();
		if (r != kIOReturnSuccess) {
			IOLog("GPU: closed connection's job still running after 1 s (the job timeout handles it)\n");
			break;
		}
		fastSync();
	}
}

void
MappleGPUAccelerator::wakeWaiters(void)
{
	if (fGenericWaiters != 0) {
		fWakeGeneric++;
		fGate->commandWakeup(&fFenceEvent, false);
	}
}

IOReturn
MappleGPUAccelerator::waitFence(MappleGPUFence *fence, uint64_t deadline)
{
	while (!fence->signalled) {
		IOReturn r = sleepOn(fence, deadline);
		if (r != kIOReturnSuccess) {
			return fence->signalled ? kIOReturnSuccess : r;
		}
	}
	return kIOReturnSuccess;
}

/* ---- parameters ---- */

IOReturn
MappleGPUAccelerator::getParam(uint32_t param, uint64_t *value)
{
	switch (param) {
	case VGPU_PARAM_GPU_PROD_ID: *value = fGpuId >> 16; break;
	case VGPU_PARAM_GPU_REVISION: *value = fGpuId & 0xffff; break;
	case VGPU_PARAM_SHADER_PRESENT: *value = fShaderPresent; break;
	case VGPU_PARAM_TILER_PRESENT: *value = fTilerPresent; break;
	case VGPU_PARAM_L2_PRESENT: *value = fL2Present; break;
	case VGPU_PARAM_STACK_PRESENT: *value = fStackPresent; break;
	case VGPU_PARAM_AS_PRESENT: *value = fAsPresent; break;
	case VGPU_PARAM_JS_PRESENT: *value = fJsPresent; break;
	case VGPU_PARAM_L2_FEATURES: *value = fL2Features; break;
	case VGPU_PARAM_CORE_FEATURES: *value = fCoreFeatures; break;
	case VGPU_PARAM_TILER_FEATURES: *value = fTilerFeatures; break;
	case VGPU_PARAM_MEM_FEATURES: *value = fMemFeatures; break;
	case VGPU_PARAM_MMU_FEATURES: *value = fMmuFeatures; break;
	case VGPU_PARAM_THREAD_FEATURES: *value = fThreadFeatures; break;
	case VGPU_PARAM_MAX_THREADS: *value = fThreadMaxThreads; break;
	case VGPU_PARAM_THREAD_MAX_WORKGROUP_SZ: *value = fThreadMaxWg; break;
	case VGPU_PARAM_THREAD_MAX_BARRIER_SZ: *value = fThreadMaxBarrier; break;
	case VGPU_PARAM_COHERENCY_FEATURES: *value = fCoherencyFeatures; break;
	case VGPU_PARAM_TEXTURE_FEATURES0: case VGPU_PARAM_TEXTURE_FEATURES1:
	case VGPU_PARAM_TEXTURE_FEATURES2: case VGPU_PARAM_TEXTURE_FEATURES3:
		*value = fTextureFeatures[param - VGPU_PARAM_TEXTURE_FEATURES0];
		break;
	case VGPU_PARAM_NR_CORE_GROUPS: *value = (uint64_t)__builtin_popcountll(fL2Present); break;
	case VGPU_PARAM_THREAD_TLS_ALLOC: *value = fThreadTlsAlloc; break;
	case VGPU_PARAM_AFBC_FEATURES: *value = fAfbcFeatures; break;
	case VGPU_PARAM_SYSTEM_TIMESTAMP:
		if (fPowered) {         /* the GPU's copy of the system counter, what its jobs write */
			uint32_t hi, lo;
			do {
				hi = rd(GPU_TIMESTAMP_HI);
				lo = rd(GPU_TIMESTAMP_LO);
			} while (hi != rd(GPU_TIMESTAMP_HI));
			*value = (uint64_t)hi << 32 | lo;
		} else {
			*value = mach_absolute_time();
		}
		break;
	case VGPU_PARAM_SYSTEM_TIMESTAMP_FREQUENCY: *value = gPEClockFrequencyInfo.timebase_frequency_hz; break;
	case VGPU_PARAM_ALLOWED_JM_CTX_PRIORITIES: *value = 1u << 1; break;
	case VGPU_PARAM_SELECTED_COHERENCY: *value = VGPU_COHERENCY_NONE; break;
	default:
		if (param >= VGPU_PARAM_JS_FEATURES0 && param <= VGPU_PARAM_JS_FEATURES15) {
			*value = fJsFeatures[param - VGPU_PARAM_JS_FEATURES0];
			break;
		}
		return kIOReturnBadArgument;
	}
	return kIOReturnSuccess;
}

void
MappleGPUAccelerator::getVersion(struct vgpu_version *v)
{
	v->major = VGPU_UAPI_MAJOR;
	v->minor = VGPU_UAPI_MINOR;
	v->gpu_id = fGpuId;
	v->page_size = (uint32_t)page_size;
	snprintf(v->name, sizeof(v->name), "Mali-G57 MC%u", (unsigned)__builtin_popcountll(fShaderPresent));
}

/* ---- self test (USB link) ---- */

void
MappleGPUAccelerator::selftest(bool fault, uint32_t count, char **out, char *end)
{
	if (!fPowered) {
		say(out, end, "selftest: GPU off (gpu on first)\n");
		return;
	}
	if (fSelfVM == NULL && (fSelfVM = MappleGPUVM::create(this)) == NULL) {
		say(out, end, "selftest: no VM\n");
		return;
	}
	MappleGPUBO *bo = new MappleGPUBO;
	IOBufferMemoryDescriptor *mem = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,
	    kIODirectionInOut, page_size, page_size);
	if (bo == NULL || !bo->init() || mem == NULL) {
		OSSafeReleaseNULL(bo);
		OSSafeReleaseNULL(mem);
		say(out, end, "selftest: no memory\n");
		return;
	}
	bo->vm = fSelfVM;
	fSelfVM->retain();
	fSelfVM->bos++;
	bo->kind = kMappleBONormal;
	bo->flags = VGPU_BO_NOEXEC;             /* GPU-written: listed for invalidation (mp_gpu_lazy) */
	bo->memory = mem;
	bo->cacheMaint = bo->cacheable = true;  /* written and read through the kernel's cacheable mapping */
	bo->size = bo->vaSize = page_size;
	if (!chargeMemory(fSelfVM, page_size)) {
		bo->vaSize = 0;
		bo->release();
		say(out, end, "selftest: over the GPU memory limit\n");
		return;
	}
	bo->va = fSelfVM->allocVA(page_size, page_size);
	if (bo->va == 0 || !fSelfVM->map(bo->va, mem, 0, page_size, PTE_AP_UNPRIV | PTE_SH_IS | PTE_AF | PTE_NG)) {
		say(out, end, "selftest: cannot map\n");
		bo->release();
		return;
	}
	volatile uint32_t *w = (volatile uint32_t *)mem->getBytesNoCopy();
	uint64_t target = fault ? bo->va + (64ull << 20) : bo->va + 0x200;       /* fault: an unmapped address */
	uint32_t value = 0xc0ffee00u | (uint32_t)(fJobs & 0xff);
	for (int i = 0; i < 256; i++) {
		w[i] = 0;
	}
	w[4] = (2u << 1) | (1u << 16);          /* job header: type WRITE_VALUE, index 1, no next */
	w[8] = (uint32_t)target;                /* payload: address */
	w[9] = (uint32_t)(target >> 32);
	w[10] = 6;                              /* IMMEDIATE_32 */
	w[12] = value;
	__asm__ volatile ("dsb sy" : : : "memory");

	uint64_t t0 = nowMs(), mmu0 = fMmuFaults, resets0 = fResets;
	if (count > 1) {
		objectCounts(out, end, "before");
	}
	MappleGPUFence *fence = NULL;
	OSArray *bos = NULL;
	IOReturn ret = kIOReturnSuccess;
	/* count > 1: the same job again, each with its own fence and BO list as a client's submit has */
	for (uint32_t k = 0; k < count && ret == kIOReturnSuccess; k++) {
		OSSafeReleaseNULL(bos);
		OSSafeReleaseNULL(fence);
		fence = MappleGPUFence::create(false);
		bos = OSArray::withObjects((const OSObject **)&bo, 1);
		ret = (fence && bos) ? submit(fSelfVM, bo->va, 0, NULL, bos, fence) : kIOReturnNoMemory;
		if (ret == kIOReturnSuccess) {
			uint64_t deadline;
			clock_interval_to_deadline(1000, kMillisecondScale, &deadline);
			ret = waitFence(fence, deadline);
		}
		invalidatePending(fSelfVM, bo);         /* what userspace's wait does */
	}
	uint32_t got = w[0x200 / 4], exc = w[0];
	say(out, end, "selftest%s: submit/wait 0x%x after %llu ms, fence %s 0x%x, job status 0x%02x (%s), "
	    "value 0x%08x (expected 0x%08x), MMU faults +%llu\n", fault ? " fault" : "", ret, nowMs() - t0,
	    fence && fence->signalled ? "signalled" : "pending", fence ? fence->error : 0, exc & 0xff,
	    jsStatusName(exc & 0xff), got, fault ? 0 : value, fMmuFaults - mmu0);
	if (fault) {
		say(out, end, "selftest fault: %s, %s\n", (fence && fence->signalled && fence->error != kIOReturnSuccess &&
		    fMmuFaults > mmu0) ? "PASS (fault caught, GPU still alive: run gpu selftest)" : "FAIL",
		    fResets == resets0 ? "no reset" : "GPU was reset");
	} else {
		say(out, end, "selftest: %s\n", got == value ? "PASS" : "FAIL");
	}
	if (fence != NULL && !fence->signalled) {
		resetGPU("selftest timed out");
	}
	OSSafeReleaseNULL(bos);
	OSSafeReleaseNULL(fence);
	bo->release();
	if (count > 1) {
		say(out, end, "selftest: %u jobs in %llu ms\n", count, nowMs() - t0);
		objectCounts(out, end, "after");
	}
}

extern "C" unsigned int vm_page_wire_count;

/* Live driver objects and wired pages, for leak checks ("gpu jobs N", "gpu memory"). Gated. */
void
MappleGPUAccelerator::objectCounts(char **out, char *end, const char *when)
{
	say(out, end, "objects%s%s: wired %llu KiB; jobs %llu, fences %u, BOs %u, VMs %u, clients %u, surfaces %u, "
	    "OSArrays %u, OSData %u, memory maps %u, buffer descriptors %u; mappings dropped at close %llu\n", when ? " " : "",
	    when ? when : "", (uint64_t)vm_page_wire_count * (PAGE_SIZE >> 10),
	    fJobsLive, MappleGPUFence::metaClass->getInstanceCount(), MappleGPUBO::metaClass->getInstanceCount(),
	    MappleGPUVM::metaClass->getInstanceCount(), MappleGPUUserClient::metaClass->getInstanceCount(),
	    MappleSurface::metaClass->getInstanceCount(),
	    OSArray::metaClass->getInstanceCount(), OSData::metaClass->getInstanceCount(),
	    IOMemoryMap::metaClass->getInstanceCount(), IOBufferMemoryDescriptor::metaClass->getInstanceCount(), fMapsDropped);
}

/* ---- USB link: "gpu [status|mfg|on|off|regs|freq [MHz]|dvfs [on|off]|selftest [fault]|jobs N|probe|publish]" ---- */

void
MappleGPUAccelerator::linkCommand(const char *args, char *out, size_t outLen)
{
	__block char *p = out;
	char *end = out + outLen;
	out[0] = 0;
	while (*args == ' ') {
		args++;
	}
	fGate->runActionBlock(^IOReturn {
		char **o = &p;
		if (strncmp(args, "mfg", 3) == 0) {
			mfgDump(o, end);
		} else if (strncmp(args, "on", 2) == 0) {
			powerOn(o, end);
		} else if (strncmp(args, "off", 3) == 0) {
			powerOff(o, end);
		} else if (strncmp(args, "selftest", 8) == 0) {
			selftest(strncmp(args, "selftest fault", 14) == 0, 1, o, end);
		} else if (strncmp(args, "jobs ", 5) == 0) {
			uint32_t n = (uint32_t)strtoul(args + 5, NULL, 10);
			selftest(false, n < 1 ? 1 : n > 100000 ? 100000 : n, o, end);
		} else if (strncmp(args, "freq", 4) == 0) {
			const char *a = args + 4;
			while (*a == ' ') {
				a++;
			}
			if (*a >= '0' && *a <= '9') {
				setOpp(oppForMHz((uint32_t)strtoul(a, NULL, 10)), o, end);
			} else {
				uint32_t tvg, tvs;
				oppSels(fOpp, &tvg, &tvs);
				say(o, end, "OPP %u MHz (clock %u MHz), vgpu %u uV vsram_gpu %u uV, margin %d mV; table (MHz):",
				    gMappleOpp[fOpp].khz / 1000, gMappleOpp[fClockOpp].khz / 1000, UV(tvg), UV(tvs), fMarginMv);
				for (uint32_t i = 0; i < OPP_COUNT; i++) {
					say(o, end, " %u", gMappleOpp[i].khz / 1000);
				}
				say(o, end, "\n");
			}
		} else if (strncmp(args, "dvfs", 4) == 0) {
			if (strncmp(args, "dvfs on", 7) == 0) {
				fDvfs = true;
			} else if (strncmp(args, "dvfs off", 8) == 0) {
				fDvfs = false;
			}
			say(o, end, "dvfs %s: %u..%u MHz (mp_gpu_freq..mp_gpu_freqmax), now %u MHz, load %u%%, %llu up %llu down\n",
			    fDvfs ? "on" : "off", gMappleOpp[fOpp].khz / 1000, gMappleOpp[MIN(fDvfsFast, fOpp)].khz / 1000,
			    gMappleOpp[fClockOpp].khz / 1000, fLoadPct, fDvfsUps, fDvfsDowns);
		} else if (strncmp(args, "lat", 3) == 0) {
			if (strncmp(args, "lat reset", 9) == 0) {
				latencyReset();
				say(o, end, "latency counters reset\n");
			} else {
				latencyStats(o, end);
			}
		} else if (strncmp(args, "calls", 5) == 0) {
			if (strncmp(args, "calls reset", 11) == 0) {
				callReset();
				say(o, end, "call counters reset\n");
			} else {
				callStats(o, end);
			}
		} else if (strncmp(args, "clocks", 6) == 0) {
			clocksDump(o, end);
		} else if (strncmp(args, "kv", 2) == 0) {
			/* KA1 (gpu/docs/RICHIESTE-KERNEL.md): stable key=value lines for Viva Manager */
			uint32_t cur = fPowered ? fClockOpp : fOpp, tvg, tvs;
			oppSels(cur, &tvg, &tvs);
			uint64_t busy = fBusyTotal, bns;
			if (fBusyStart != 0) {
				busy += mach_absolute_time() - fBusyStart;
			}
			absolutetime_to_nanoseconds(busy, &bns);
			say(o, end, "powered=%d\nmode=%d\ngpu_id=0x%08x\nmhz=%u\nopp_min_mhz=%u\nopp_max_mhz=%u\ndvfs=%d\n",
			    fPowered ? 1 : 0, fMode, fGpuId, gMappleOpp[cur].khz / 1000, gMappleOpp[fOpp].khz / 1000,
			    gMappleOpp[MIN(fDvfsFast, fOpp)].khz / 1000, fDvfs ? 1 : 0);
			say(o, end, "load_pct=%u\ndvfs_ups=%llu\ndvfs_downs=%llu\ndvfs_boosts=%llu\njobs=%llu\nrunning=%d\n"
			    "queued=%d\n", fLoadPct, fDvfsUps, fDvfsDowns, fDvfsBoosts, fJobs,
			    (fRunning[0] != NULL) + (fRunning[1] != NULL), (fQueue[0] != NULL) + (fQueue[1] != NULL));
			say(o, end, "job_faults=%llu\nmmu_faults=%llu\ngpu_faults=%llu\ntimeouts=%llu\nresets=%llu\nheap_grows=%llu\n",
			    fJobFaults, fMmuFaults, fGpuFaults, fTimeouts, fResets, fHeapGrows);
			{
				int bon = 0;
				uint64_t bstarts = 0, bms = 0, bsig = 0;
				mt6781_cpufreq_boost_stats(&bon, &bstarts, &bms, &bsig);
				say(o, end, "cpu_boost=%d\ncpu_boosts=%llu\ncpu_boost_ms=%llu\ncpu_boost_signals=%llu\n", bon, bstarts,
				    bms, bsig);
			}
			say(o, end, "mem_kib=%lld\nmem_peak_kib=%lld\nmem_limit_mib=%lld\nmem_proc_mib=%lld\nmem_refusals=%llu\n"
			    "busy_ns=%llu\nvgpu_mv=%u\nvsram_mv=%u\n", fMemory >> 10, fMemPeak >> 10, fMemLimit >> 20,
			    fProcLimit >> 20, fMemRefused, bns, UV(tvg) / 1000, UV(tvs) / 1000);
			for (MappleGPUVM *v = fVMs; v != NULL; v = v->nextVM) {
				bool seen = false;
				for (MappleGPUVM *w = fVMs; w != v; w = w->nextVM) {
					seen = seen || w->pid == v->pid;
				}
				if (seen) {
					continue;
				}
				int64_t bytes = 0, imported = 0;
				uint32_t bos = 0;
				for (MappleGPUVM *w = v; w != NULL; w = w->nextVM) {
					if (w->pid == v->pid) {
						bytes += w->bytes;
						imported += w->imported;
						bos += w->bos;
					}
				}
				say(o, end, "proc.%d.name=%s\nproc.%d.kib=%lld\nproc.%d.bos=%u\nproc.%d.imported_kib=%lld\n", v->pid,
				    v->proc, v->pid, bytes >> 10, v->pid, bos, v->pid, imported >> 10);
			}
		} else if (strncmp(args, "fastkick", 8) == 0) {
			if (strncmp(args, "fastkick on", 11) == 0) {
				fFastKick = true;
				kick();
			} else if (strncmp(args, "fastkick off", 12) == 0) {
				fFastKick = false;
				fastSync();             /* drop what is armed */
			}
			say(o, end, "fast kick %s\n", fFastKick ? "on" : "off");
		} else if (strncmp(args, "probe", 5) == 0) {
			probeDump(o, end);
		} else if (strncmp(args, "publish", 7) == 0) {
			if (!fPowered) {
				say(o, end, "GPU off: gpu on first\n");
			} else if (!fPublished) {
				fPublished = true;
				registerService();
				say(o, end, "accelerator published (IOAcceleratorES, MetalPluginName MappleMTLDriver)\n");
			} else {
				say(o, end, "already published\n");
			}
		} else if (strncmp(args, "regs", 4) == 0) {
			if (!fPowered) {
				say(o, end, "GPU off: registers not readable\n");
				return kIOReturnSuccess;
			}
			say(o, end, "id 0x%08x status 0x%08x irq raw 0x%08x | job raw 0x%08x | mmu raw 0x%08x\n", rd(GPU_ID),
			    rd(GPU_STATUS), rd(GPU_IRQ_RAWSTAT), rd(JOB_INT_RAWSTAT), rd(MMU_INT_RAWSTAT));
			say(o, end, "ready: l2 0x%x shader 0x%x tiler 0x%x stack 0x%x | trans: shader 0x%x l2 0x%x\n",
			    rd(GPU_L2_READY_LO), rd(GPU_SHADER_READY_LO), rd(GPU_TILER_READY_LO), rd(GPU_STACK_READY_LO),
			    rd(GPU_SHADER_PWRTRANS_LO), rd(GPU_L2_PWRTRANS_LO));
			say(o, end, "features: l2 0x%08x core 0x%08x tiler 0x%08x mem 0x%08x mmu 0x%08x thread 0x%08x "
			    "max threads %u tls %u\n", fL2Features, fCoreFeatures, fTilerFeatures, fMemFeatures,
			    fMmuFeatures, fThreadFeatures, fThreadMaxThreads, fThreadTlsAlloc);
			for (uint32_t js = 0; js < 3; js++) {
				say(o, end, "js%u: features 0x%08x status 0x%02x head 0x%08x%08x config 0x%08x\n", js,
				    fJsFeatures[js], rd(JS_BASE(js) + JS_STATUS), rd(JS_BASE(js) + JS_HEAD_HI),
				    rd(JS_BASE(js) + JS_HEAD_LO), rd(JS_BASE(js) + JS_CONFIG));
			}
			for (uint32_t as = 0; as < fASCount; as++) {
				if (fAS[as] != NULL || rd(AS_BASE(as) + AS_FAULTSTATUS) != 0) {
					say(o, end, "as%u: %s transtab 0x%08x%08x transcfg 0x%08x status 0x%x fault 0x%08x at "
					    "0x%08x%08x\n", as, fAS[as] ? "bound" : "free", rd(AS_BASE(as) + AS_TRANSTAB_HI),
					    rd(AS_BASE(as) + AS_TRANSTAB_LO), rd(AS_BASE(as) + AS_TRANSCFG_LO),
					    rd(AS_BASE(as) + AS_STATUS), rd(AS_BASE(as) + AS_FAULTSTATUS),
					    rd(AS_BASE(as) + AS_FAULTADDRESS_HI), rd(AS_BASE(as) + AS_FAULTADDRESS_LO));
				}
			}
		} else {
			say(o, end, "gpu: %s, mode %d%s, id 0x%08x, OPP %u MHz\n", fPowered ? "on" : "off", fMode,
			    fPublished ? ", published" : "", fGpuId, gMappleOpp[fPowered ? fClockOpp : fOpp].khz / 1000);
			say(o, end, "jobs %llu (running %d/%d, queued %d/%d), job faults %llu, MMU faults %llu (%llu without "
			    "reset%s), heap grows %llu, GPU faults %llu, timeouts %llu, resets %llu\n", fJobs, fRunning[0] != NULL,
			    fRunning[1] != NULL, fQueue[0] != NULL, fQueue[1] != NULL, fJobFaults, fMmuFaults, fFaultsNoReset,
			    fWatchFragment ? ", watching fragment jobs" : "", fHeapGrows, fGpuFaults, fTimeouts, fResets);
			memoryDump(o, end);
			cacheStats(o, end);
			say(o, end, "dvfs %s (%u..%u MHz, load %u%%, %llu up %llu down, %llu display boosts%s), flush reduction "
			    "%s\n", fDvfs ? "on" : "off", gMappleOpp[fOpp].khz / 1000, gMappleOpp[MIN(fDvfsFast, fOpp)].khz / 1000,
			    fLoadPct, fDvfsUps, fDvfsDowns, fDvfsBoosts, fDvfsBoost ? "" : ", mp_gpu_boost=0",
			    fFlushReduce ? "on" : "off");
			latencyStats(o, end);   /* per-call costs: "gpu calls" (the reply holds 4000 bytes) */
			say(o, end, "usage: gpu [status|mfg|on|off|regs|freq [MHz]|dvfs [on|off]|lat [reset]|calls [reset]|clocks|"
			    "fastkick [on|off]|selftest [fault]|probe|publish]\n");
		}
		return kIOReturnSuccess;
	});
}

/* kern.mapple_gpu_surf_debug: where the GPU's page table puts the surface's last import (0: no live import). */
extern "C" uint64_t
mapple_gpu_surface_pa(MappleSurface *surface)
{
	MappleGPUAccelerator *gpu = gMappleGPU;
	__block uint64_t pa = 0;
	if (gpu == NULL || surface == NULL || surface->gpuVM == NULL || gpu->gate() == NULL) {
		return 0;
	}
	gpu->gate()->runActionBlock(^IOReturn {
		for (MappleGPUVM *v = gpu->firstVM(); v != NULL; v = v->nextVM) {
			if (v == surface->gpuVM) {
				pa = v->paOf(surface->gpuVA);
			}
		}
		return kIOReturnSuccess;
	});
	return pa;
}

/* K0c: the display (AppleCLCD) and IOSurfaceLock wait for the GPU's writes to a surface. */
bool
mapple_gpu_surface_idle(MappleSurface *surface, uint64_t *jobs)
{
	MappleGPUAccelerator *gpu = gMappleGPU;
	if (gpu == NULL || surface == NULL || gpu->gate() == NULL) {
		if (jobs != NULL) {
			*jobs = 0;
		}
		return true;
	}
	__block bool idle = true;
	__block uint64_t n = 0;
	gpu->gate()->runActionBlock(^IOReturn {
		idle = MappleGPUAccelerator::surfaceIdle(surface, &n);
		return kIOReturnSuccess;
	});
	if (jobs != NULL) {
		*jobs = n;
	}
	return idle;
}

IOReturn
mapple_gpu_surface_wait(MappleSurface *surface, uint32_t timeoutMs)
{
	MappleGPUAccelerator *gpu = gMappleGPU;
	if (gpu == NULL || surface == NULL || gpu->gate() == NULL) {
		return kIOReturnSuccess;
	}
	uint64_t deadline;
	clock_interval_to_deadline(timeoutMs, kMillisecondScale, &deadline);
	return gpu->gate()->runActionBlock(^IOReturn {
		while (!MappleGPUAccelerator::surfaceIdle(surface, NULL)) {
			MappleGPUFence *f = OSDynamicCast(MappleGPUFence, surface->gpuFence);     /* unsignalled: not idle */
			f->retain();            /* a newer job may replace it while we sleep */
			IOReturn r = gpu->sleepOn(f, deadline);
			f->release();
			if (r != kIOReturnSuccess) {
				return MappleGPUAccelerator::surfaceIdle(surface, NULL) ? kIOReturnSuccess : r;
			}
		}
		return kIOReturnSuccess;
	});
}

/* Low Power Mode (BQ27Z561Battery.cpp): DVFS never goes above `mhz` (0 = mp_gpu_freqmax again); applied at the next sample. */
extern "C" void
mapple_gpu_set_dvfs_cap(uint32_t mhz)
{
	gMappleGPUCapMHz = mhz;
}

#if DEVELOPMENT || DEBUG
extern "C" void mtk_gpu_probe_link(const char *args, char *out, size_t len);    /* osfmk/arm64/mtk_gpu_probe.c */
#endif

extern "C" void
mapple_gpu_link(const char *args, char *out, size_t outLen)
{
	if (gMappleGPU == NULL) {
		while (*args == ' ') {
			args++;
		}
#if DEVELOPMENT || DEBUG
		if (strncmp(args, "probe", 5) == 0 || strncmp(args, "power", 5) == 0) {
			mtk_gpu_probe_link(args, out, outLen);
			return;
		}
#endif
		snprintf(out, outLen, strncmp(args, "kv", 2) == 0 ? "powered=0\nmode=0\n" :
		    "gpu: driver not running (boot-arg mp_gpu=1 or 2)\n");
		return;
	}
	gMappleGPU->linkCommand(args, out, outLen);
}

#if DEVELOPMENT || DEBUG
#include <sys/sysctl.h>

/* gpu status's memory lines (per process: GPU memory, imports, pages wired for the GPU), for mstat */
static int
mapple_gpu_memory_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	const size_t cap = 32768;
	MappleGPUAccelerator *gpu = gMappleGPU;
	if (gpu == NULL) {
		return ENODEV;
	}
	char *buf = (char *)IOMallocData(cap);
	if (buf == NULL) {
		return ENOMEM;
	}
	__block char *o = buf;
	gpu->gate()->runActionBlock(^IOReturn {
		gpu->memoryDump(&o, buf + cap);
		return kIOReturnSuccess;
	});
	int err = SYSCTL_OUT(req, buf, (size_t)(o - buf) + 1);
	IOFreeData(buf, cap);
	return err;
}
SYSCTL_PROC(_kern, OID_AUTO, mapple_gpu_memory, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_ANYBODY | CTLFLAG_LOCKED,
    0, 0, mapple_gpu_memory_sysctl, "A", "GPU memory per process, imports and pages wired for the GPU");
#endif /* DEVELOPMENT || DEBUG */
#endif /* MT6781 */
