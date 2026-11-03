/*
 * viva (MT6781): Mali-G57 MC2 driver (see VivaGPU.h).
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
 * One fixed operating point for now (300 MHz, VGPU 0.65 V, VSRAM 0.85 V:
 * OPP 48 of the table plus 50 mV), no DVFS.
 *
 * Mali (job-manager Valhall), written from the register map of the
 * architecture; behaviour as Panfrost expects it from Linux:
 *   one AArch64 4K address space per connection (48-bit VA, leaves of
 *   4 KiB, four per XNU page), bound to a hardware AS on demand (LRU);
 *   slot 0 fragment, slot 1 vertex/tiler/compute, one chain per slot on
 *   the hardware, FIFO per slot, a chain starts when its input fences have
 *   signalled; GPU caches cleaned and invalidated around every chain;
 *   tiler heaps grow by 2 MiB on translation faults; any other fault fails
 *   the chain and disables its address space (never a panic); a chain that
 *   runs longer than 3 s is hard-stopped, a slot that ignores the stop
 *   resets the GPU.
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>
#include <libkern/OSAtomic.h>
#include <pexpert/pexpert.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>
#include <sys/proc.h>

#include "VivaGPU.h"
#include "VivaPMIC.h"
#include "SweetSurface.h"

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
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
#define TIMER_MS                250
#define HEAP_CHUNK              (2ull << 20)
#define GPU_MEMORY_LIMIT        (2048ll << 20)

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
#define VGPU_SEL                0x18    /* 0.65 V */
#define VSRAM_SEL               0x38    /* 0.85 V */
#define MFGPLL_CON1_VALUE       0x831713b1u     /* 300 MHz: posdiv /8, PCW 2400 MHz / 26 MHz << 14 */

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

static VivaGPUAccelerator *gVivaGPU;

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

/*
 * CPU cache maintenance (to PoC) by kernel virtual address. Always through a kernel mapping of the buffer:
 * on this port the physical aperture (phystokv) only covers RAM from 0xc0000000, and kernel allocations
 * come from below it too.
 */
static void
cacheVA(const void *p, uint64_t len, int op)
{
	uintptr_t end = (uintptr_t)p + len;
	for (uintptr_t line = (uintptr_t)p & ~(uintptr_t)63; line < end; line += 64) {
		if (op == kVivaCacheFlush) {
			__asm__ volatile ("dc civac, %0" : : "r"(line) : "memory");
		} else if (op == kVivaCacheInvalidate) {
			__asm__ volatile ("dc ivac, %0" : : "r"(line) : "memory");
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
	cacheVA(p, len, kVivaCacheClean);
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
OSDefineMetaClassAndStructors(VivaGPUFence, OSObject);

VivaGPUFence *
VivaGPUFence::create(bool isSignalled)
{
	VivaGPUFence *f = new VivaGPUFence;
	if (f != NULL && !f->init()) {
		f->release();
		return NULL;
	}
	if (f != NULL) {
		f->signalled = isSignalled;
		f->error = kIOReturnSuccess;
	}
	return f;
}

/* ---- jobs ---- */

struct VivaGPUJob {
	VivaGPUJob   *next;
	VivaGPUVM    *vm;
	uint64_t      jc;
	uint32_t      requirements;
	OSArray      *deps;     /* VivaGPUFence */
	OSArray      *bos;      /* VivaGPUBO */
	VivaGPUFence *fence;
	uint64_t      startMs;
};

/* ---- address spaces ---- */

OSDefineMetaClassAndStructors(VivaGPUVM, OSObject);

VivaGPUVM *
VivaGPUVM::create(VivaGPUAccelerator *g)
{
	VivaGPUVM *vm = new VivaGPUVM;
	if (vm == NULL || !vm->init()) {
		OSSafeReleaseNULL(vm);
		return NULL;
	}
	vm->gpu = g;
	vm->asSlot = -1;
	vm->fPool = OSArray::withCapacity(4);
	vm->fRangeCap = 64;
	vm->fRanges = IONewZeroData(Range, vm->fRangeCap);
	if (vm->fPool == NULL || vm->fRanges == NULL || (vm->rootPA = vm->newTable()) == 0) {
		vm->release();
		return NULL;
	}
	return vm;
}

void
VivaGPUVM::free(void)
{
	if (gpu != NULL && asSlot >= 0) {
		gpu->vmDestroyed(this);
	}
	if (fPool != NULL) {
		gpu->chargeMemory(-(int64_t)(fPool->getCount() * page_size));
	}
	OSSafeReleaseNULL(fPool);
	if (fRanges != NULL) {
		IODeleteData(fRanges, Range, fRangeCap);
	}
	OSObject::free();
}

/* A zeroed 4 KiB table (physical address), carved from 16 KiB wired pages. */
uint64_t
VivaGPUVM::newTable(void)
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
		if (fPoolCount == kPoolMax) {
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
		gpu->chargeMemory(page_size);
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
VivaGPUVM::tableKVA(uint64_t pa)
{
	uint64_t base = pa & ~(uint64_t)(page_size - 1);
	for (uint32_t i = 0; i < fPoolCount; i++) {
		if (fPoolPA[i] == base) {
			return (uint64_t *)(fPoolKVA[i] + (pa - base));
		}
	}
	panic("VIVA GPU: page table 0x%llx not in the pool", pa);
}

/* The level 3 table for `va`, created on the way if `create`. */
uint64_t *
VivaGPUVM::tableFor(uint64_t va, bool create)
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

bool
VivaGPUVM::map(uint64_t va, IOMemoryDescriptor *md, uint64_t offset, uint64_t size, uint64_t pte)
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
VivaGPUVM::unmap(uint64_t va, uint64_t size)
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
VivaGPUVM::leafAttrs(bool exec)
{
	return PTE_AP_UNPRIV | PTE_SH_IS | PTE_AF | PTE_NG | (exec ? 0 : PTE_XN);
}

uint64_t
VivaGPUVM::allocVA(uint64_t size, uint64_t align)
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
VivaGPUVM::freeVA(uint64_t va)
{
	for (uint32_t i = 0; i < fRangeCount; i++) {
		if (fRanges[i].start == va) {
			bcopy(&fRanges[i + 1], &fRanges[i], (fRangeCount - i - 1) * sizeof(Range));
			fRangeCount--;
			return;
		}
	}
}

/* ---- buffer objects ---- */

OSDefineMetaClassAndStructors(VivaGPUBO, OSObject);

void
VivaGPUBO::free(void)
{
	if (vm != NULL && va != 0) {
		vm->unmap(va, vaSize);
		vm->freeVA(va);
	}
	if (kind == kVivaBOHeap && vm != NULL) {
		vm->gpu->heapRemove(this);
	}
	if (chunks != NULL) {
		for (uint32_t i = 0; i < chunkCount; i++) {
			if (chunks[i] != NULL) {
				chunks[i]->release();
				vm->gpu->chargeMemory(-(int64_t)HEAP_CHUNK);
			}
		}
		IODelete(chunks, IOBufferMemoryDescriptor *, chunkCount);
	}
	if (memory != NULL) {
		if (prepared) {
			memory->complete();
		}
		if (kind == kVivaBONormal && vm != NULL) {
			vm->gpu->chargeMemory(-(int64_t)vaSize);
		}
		memory->release();
	}
	OSSafeReleaseNULL(kmap);
	OSSafeReleaseNULL(surface);
	OSSafeReleaseNULL(busy);
	OSSafeReleaseNULL(vm);
	OSObject::free();
}

bool
VivaGPUBO::growHeap(uint64_t addr)
{
	if (kind != kVivaBOHeap || addr < va || addr >= va + vaSize) {
		return false;
	}
	uint32_t idx = (uint32_t)((addr - va) / HEAP_CHUNK);
	if (idx >= chunkCount) {
		return false;
	}
	if (chunks[idx] != NULL) {
		return true;            /* another unit faulted on it before the flush: already mapped */
	}
	if (!vm->gpu->chargeMemory(HEAP_CHUNK)) {
		return false;
	}
	uint64_t len = MIN(HEAP_CHUNK, vaSize - (uint64_t)idx * HEAP_CHUNK);
	IOBufferMemoryDescriptor *c = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task,
	    kIODirectionInOut, len, page_size);         /* cacheable: see createBO; GPU-only afterwards */
	if (c == NULL) {
		vm->gpu->chargeMemory(-(int64_t)HEAP_CHUNK);
		return false;
	}
	cacheBuffer(c, 0, len, kVivaCacheFlush);
	if (!vm->map(va + (uint64_t)idx * HEAP_CHUNK, c, 0, len, PTE_AP_UNPRIV | PTE_SH_IS | PTE_AF | PTE_NG | PTE_XN)) {
		c->release();
		vm->gpu->chargeMemory(-(int64_t)HEAP_CHUNK);
		return false;
	}
	chunks[idx] = c;
	return true;
}

void
VivaGPUBO::cacheOp(uint64_t offset, uint64_t len, int op)
{
	if (memory == NULL) {
		return;
	}
	if (len == 0 || offset + len > size) {
		len = offset < size ? size - offset : 0;
	}
	IOBufferMemoryDescriptor *buf = OSDynamicCast(IOBufferMemoryDescriptor, memory);
	if (buf != NULL) {
		cacheBuffer(buf, memOffset + offset, len, op);
		return;
	}
	if (kmap == NULL && (kmap = memory->map()) == NULL) {        /* user memory, physical surfaces */
		return;
	}
	cacheVA((const uint8_t *)kmap->getVirtualAddress() + memOffset + offset, len, op);
}

/* ---- accelerator ---- */

OSDefineMetaClassAndStructors(VivaGPUAccelerator, IOAcceleratorES);

IOWorkLoop *
VivaGPUAccelerator::getWorkLoop(void) const
{
	return fWorkLoop;
}

bool
VivaGPUAccelerator::start(IOService *provider)
{
	int mode = 0;
	if (!PE_parse_boot_argn("vivagpu", &mode, sizeof(mode)) || mode <= 0) {
		IOLog("VIVA GPU: off (boot-arg vivagpu=1 for bring-up, 2 to power at boot and publish)\n");
		return false;
	}
	fMode = mode;
	fProbeMode = 1;
	PE_parse_boot_argn("vivagpuprobe", &fProbeMode, sizeof(fProbeMode));
	fProbeLock = IOLockAlloc();
	fWorkLoop = IOWorkLoop::workLoop();
	if (fWorkLoop == NULL || fProbeLock == NULL || !IOAcceleratorES::start(provider)) {
		return false;
	}
	for (int i = 0; i < 6; i++) {
		fMaps[i] = provider->mapDeviceMemoryWithIndex(i);
		if (fMaps[i] == NULL) {
			IOLog("VIVA GPU: cannot map register range %d\n", i);
			return false;
		}
	}
	fMali = (vm_offset_t)fMaps[kMapMali]->getVirtualAddress();
	fMfgcfg = (vm_offset_t)fMaps[kMapMfgcfg]->getVirtualAddress();
	fSpm = (vm_offset_t)fMaps[kMapSpm]->getVirtualAddress();
	fTopck = (vm_offset_t)fMaps[kMapTopck]->getVirtualAddress();
	fApmixed = (vm_offset_t)fMaps[kMapApmixed]->getVirtualAddress();
	fInfra = (vm_offset_t)fMaps[kMapInfra]->getVirtualAddress();

	fPwrap = VivaPwrap::wait();
	if (fPwrap == NULL) {
		IOLog("VIVA GPU: no PMIC wrapper\n");
		return false;
	}
	fGate = IOCommandGate::commandGate(this);
	fTimer = IOTimerEventSource::timerEventSource(this, &VivaGPUAccelerator::jobTimer);
	IOInterruptEventSource::Action actions[3] = {
		&VivaGPUAccelerator::gpuInterrupt, &VivaGPUAccelerator::mmuInterrupt, &VivaGPUAccelerator::jobInterrupt,
	};
	if (fGate == NULL || fTimer == NULL || fWorkLoop->addEventSource(fGate) != kIOReturnSuccess ||
	    fWorkLoop->addEventSource(fTimer) != kIOReturnSuccess) {
		return false;
	}
	for (int i = 0; i < 3; i++) {
		fIrq[i] = IOInterruptEventSource::interruptEventSource(this, actions[i], provider, i);
		if (fIrq[i] == NULL || fWorkLoop->addEventSource(fIrq[i]) != kIOReturnSuccess) {
			IOLog("VIVA GPU: no interrupt %d\n", i);
			return false;
		}
	}
	setProperty("MetalPluginName", "VivaMTLDriver");
	setProperty("MetalPluginClassName", "VMDevice");
	gVivaGPU = this;
	IOLog("VIVA GPU: driver started (vivagpu=%d)\n", fMode);

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
			IOLog("VIVA GPU: accelerator published (MetalPluginName VivaMTLDriver)\n");
		}
	}
	return true;
}

void
VivaGPUAccelerator::free(void)
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
	if (fGate != NULL) {
		fWorkLoop->removeEventSource(fGate);
		fGate->release();
	}
	for (int i = 0; i < 6; i++) {
		OSSafeReleaseNULL(fMaps[i]);
	}
	if (fHeapList != NULL) {
		IODelete(fHeapList, VivaGPUBO *, fHeapCap);
	}
	if (fProbeLock != NULL) {
		IOLockFree(fProbeLock);
	}
	OSSafeReleaseNULL(fWorkLoop);
	IOAcceleratorES::free();
}

IOReturn
VivaGPUAccelerator::newUserClient(task_t owningTask, void *securityID, UInt32 type, IOUserClient **handler)
{
	if (type != VGPU_CLIENT_TYPE) {
		char proc[20];
		proc_selfname(proc, sizeof(proc));
		probeNote(proc, type, ~0u, 0, 0);
		IOLog("VIVA GPU: %s[%d] opens user client type 0x%x (not vgpu): %s\n", proc, proc_selfpid(), type,
		    fProbeMode ? "probe client" : "refused");
		if (!fProbeMode) {
			return kIOReturnUnsupported;
		}
		VivaGPUProbeClient *probe = new VivaGPUProbeClient;
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
	VivaGPUUserClient *client = new VivaGPUUserClient;
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

bool
VivaGPUAccelerator::chargeMemory(int64_t bytes)
{
	if (bytes > 0 && fMemory + bytes > GPU_MEMORY_LIMIT) {
		return false;
	}
	fMemory += bytes;
	return true;
}

/* ---- unknown user clients and property reads (K1: what IOGPU/QuartzCore/Metal ask for) ---- */

#define PROBE_LOG_MAX   48      /* log lines per client; "gpu probe" keeps counting */
/* selector tags in the probe table (high 16 bits) for calls that are not external methods */
#define PROBE_TRAP      0x74720000u
#define PROBE_MEMORY    0x6d650000u
#define PROBE_NOTIFY    0x6e6f0000u

/* One table line per (process, type, selector); selector ~0 = the open itself. */
void
VivaGPUAccelerator::probeNote(const char *proc, uint32_t type, uint32_t selector, uint32_t inSize, uint32_t outSize)
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
VivaGPUAccelerator::propertyRead(const char *key, bool found)
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
		IOLog("VIVA GPU: %s[%d] reads property \"%s\": %s\n", proc, proc_selfpid(), key, found ? "found" : "MISSING");
	}
}

void
VivaGPUAccelerator::probeDump(char **out, char *end)
{
	IOLockLock(fProbeLock);
	say(out, end, "unknown user clients (%u calls dropped, vivagpuprobe=%d):\n", fProbeDropped, fProbeMode);
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
viva_gpu_property_read(IORegistryEntry *entry, const char *key, bool found)
{
	VivaGPUAccelerator *gpu = gVivaGPU;
	if (gpu != NULL && entry == gpu) {
		gpu->propertyRead(key, found);
	}
}

/* ---- probe user client ---- */

OSDefineMetaClassAndStructors(VivaGPUProbeClient, IOUserClient);

bool
VivaGPUProbeClient::initWithTask(task_t owningTask, void *securityID, UInt32 type)
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
VivaGPUProbeClient::start(IOService *provider)
{
	fGPU = OSDynamicCast(VivaGPUAccelerator, provider);
	return fGPU != NULL && IOUserClient::start(provider);
}

void
VivaGPUProbeClient::note(const char *what, uint32_t selector, uint32_t inSize, uint32_t outSize, const char *detail)
{
	fGPU->probeNote(fProc, fType, selector, inSize, outSize);
	if (fLogged < PROBE_LOG_MAX) {
		fLogged++;
		IOLog("VIVA GPU: probe %s[%d] type 0x%x %s %u%s -> unsupported\n", fProc, fPid, fType, what,
		    selector & 0xffff, detail);
	}
}

IOReturn
VivaGPUProbeClient::clientClose(void)
{
	if (fLogged < PROBE_LOG_MAX) {
		IOLog("VIVA GPU: probe %s[%d] type 0x%x closed\n", fProc, fPid, fType);
	}
	terminate();
	return kIOReturnSuccess;
}

IOReturn
VivaGPUProbeClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
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
VivaGPUProbeClient::clientMemoryForType(UInt32 type, __unused IOOptionBits *options, __unused IOMemoryDescriptor **memory)
{
	note("memory type", PROBE_MEMORY | (type & 0xffff), 0, 0, "");
	return kIOReturnUnsupported;
}

IOReturn
VivaGPUProbeClient::registerNotificationPort(mach_port_t port, UInt32 type, io_user_reference_t refCon)
{
	note("notification port type", PROBE_NOTIFY | (type & 0xffff), 0, 0, "");
	return IOUserClient::registerNotificationPort(port, type, refCon);
}

IOExternalTrap *
VivaGPUProbeClient::getTargetAndTrapForIndex(__unused IOService **targetP, UInt32 index)
{
	note("trap", PROBE_TRAP | (index & 0xffff), 0, 0, "");
	return NULL;
}

IOReturn
VivaGPUProbeClient::setProperties(OSObject *properties)
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
VivaGPUProbeClient::connectClient(IOUserClient *client)
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
VivaGPUAccelerator::protRelease(int from)
{
	for (int i = from; i >= 0; i--) {
		wr32(fInfra, gMfgProt[i].clr, gMfgProt[i].mask);
	}
}

bool
VivaGPUAccelerator::mtcmosOn(uint32_t n, char **out, char *end)
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
VivaGPUAccelerator::mtcmosOff(uint32_t n, char **out, char *end)
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

/* Set a rail's VOSEL and enable it (VivaPwrap only lets the GPU rails through, within limits). */
bool
VivaGPUAccelerator::regulatorSet(uint32_t enAddr, uint32_t voselAddr, uint32_t sel)
{
	if (fPwrap->writeRegulator(voselAddr, 0x7f, sel) != kIOReturnSuccess) {
		return false;
	}
	return fPwrap->writeRegulator(enAddr, 0x1, 1) == kIOReturnSuccess;
}

/* MFGPLL at 300 MHz, mfg_sel parked on mainpll_d5 while it changes (gpufreq's frequency switch). */
void
VivaGPUAccelerator::mfgClock(char **out, char *end)
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
	wr32(fApmixed, PLL_MFG_CON1, MFGPLL_CON1_VALUE);
	set32(fApmixed, PLL_MFG_CON0, 1u << 0);
	IODelay(20);
	if (cfg & CK_MFG_PDN) {
		wr32(fTopck, CK_CFG_0_CLR, CK_MFG_PDN);
	}
	wr32(fTopck, CK_CFG_0_CLR, CK_MFG_SEL_MASK);
	wr32(fTopck, CK_CFG_0_SET, CK_MFG_MFGPLL << CK_MFG_SEL_SHIFT);
	wr32(fTopck, CK_CFG_UPDATE, CK_CFG_UPDATE_MFG);
	IODelay(20);
	say(out, end, "clock: mfgpll con0 0x%08x con1 0x%08x pwr 0x%08x, clk_cfg_0 0x%08x (was 0x%08x)\n",
	    rd32(fApmixed, PLL_MFG_CON0), rd32(fApmixed, PLL_MFG_CON1), rd32(fApmixed, PLL_MFG_PWR_CON0),
	    rd32(fTopck, CK_CFG_0), cfg);
}

bool
VivaGPUAccelerator::mfgOn(char **out, char *end)
{
	uint32_t vgEn = 0, vgSel = 0, vsEn = 0, vsSel = 0;
	fPwrap->readPMIC(PMIC_VGPU_DA_EN, &vgEn);
	fPwrap->readPMIC(PMIC_VGPU_DA_VOSEL, &vgSel);
	fPwrap->readPMIC(PMIC_VSRAM_DA_EN, &vsEn);
	fPwrap->readPMIC(PMIC_VSRAM_DA_VOSEL, &vsSel);
	vgEn &= 1;
	vsEn &= 1;
	vgSel &= 0x7f;
	vsSel = (vsSel >> 8) & 0x7f;
	say(out, end, "rails before: vgpu %s %u uV, vsram_gpu %s %u uV\n", vgEn ? "on" : "off", UV(vgSel),
	    vsEn ? "on" : "off", UV(vsSel));

	/*
	 * Lowering (the preloader left the rails higher): clock down first, then VGPU, then VSRAM.
	 * Raising: VSRAM, VGPU, then the clock. VSRAM >= VGPU + 0.1 V throughout.
	 */
	bool ok, lowering = vgEn && vgSel > VGPU_SEL;
	if (lowering) {
		mfgClock(out, end);
	}
	if (lowering) {
		ok = regulatorSet(PMIC_VGPU_EN, PMIC_VGPU_VOSEL, VGPU_SEL);
		IODelay(300);
		ok = ok && regulatorSet(PMIC_VSRAM_EN, PMIC_VSRAM_VOSEL, VSRAM_SEL);
		IODelay(300);
	} else {
		ok = regulatorSet(PMIC_VSRAM_EN, PMIC_VSRAM_VOSEL, VSRAM_SEL);
		IODelay(300);       /* 240 us enable ramp + settle */
		ok = ok && regulatorSet(PMIC_VGPU_EN, PMIC_VGPU_VOSEL, VGPU_SEL);
		IODelay(300);       /* 200 us enable ramp + settle */
	}
	fPwrap->readPMIC(PMIC_VGPU_DA_EN, &vgEn);
	fPwrap->readPMIC(PMIC_VGPU_DA_VOSEL, &vgSel);
	fPwrap->readPMIC(PMIC_VSRAM_DA_EN, &vsEn);
	fPwrap->readPMIC(PMIC_VSRAM_DA_VOSEL, &vsSel);
	vgEn &= 1;
	vsEn &= 1;
	vgSel &= 0x7f;
	vsSel = (vsSel >> 8) & 0x7f;
	say(out, end, "rails: vgpu %s %u uV, vsram_gpu %s %u uV%s\n", vgEn ? "on" : "off", UV(vgSel),
	    vsEn ? "on" : "off", UV(vsSel), ok ? "" : " (PMIC write refused or failed)");
	if (!ok || !vgEn || !vsEn) {
		return false;
	}

	if (!lowering) {
		mfgClock(out, end);     /* raising: voltage first, then the clock */
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
VivaGPUAccelerator::mfgOff(char **out, char *end)
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
		fPwrap->writeRegulator(PMIC_VGPU_EN, 0x1, 0);
		IODelay(50);
		fPwrap->writeRegulator(PMIC_VSRAM_EN, 0x1, 0);
	}
	say(out, end, "MFG off: status 0x%08x/0x%08x, mfgpll con0 0x%08x\n", rd32(fSpm, SPM_PWR_STATUS),
	    rd32(fSpm, SPM_PWR_STATUS_2ND), rd32(fApmixed, PLL_MFG_CON0));
}

/* Read-only: what LK/preloader left (registers outside the MFG domain; mfgcfg only if MFG0/1 are on). */
void
VivaGPUAccelerator::mfgDump(char **out, char *end)
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
VivaGPUAccelerator::softReset(void)
{
	wr(GPU_IRQ_MASK, 0);
	wr(GPU_IRQ_CLEAR, 0xffffffff);
	wr(GPU_COMMAND, GPU_CMD_SOFT_RESET);
	bool ok = pollUs(100000, [&]{ return (rd(GPU_IRQ_RAWSTAT) & GPU_IRQ_RESET_COMPLETED) != 0; });
	if (!ok) {
		IOLog("VIVA GPU: soft reset timed out, hard reset\n");
		wr(GPU_COMMAND, GPU_CMD_HARD_RESET);
		ok = pollUs(100000, [&]{ return (rd(GPU_IRQ_RAWSTAT) & GPU_IRQ_RESET_COMPLETED) != 0; });
	}
	wr(GPU_IRQ_CLEAR, 0xffffffff);
	return ok;
}

void
VivaGPUAccelerator::readFeatures(void)
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
VivaGPUAccelerator::powerCores(void)
{
	uint32_t l2 = (uint32_t)fL2Present, sh = (uint32_t)fShaderPresent, st = (uint32_t)fStackPresent;
	wr(GPU_L2_PWRON_LO, l2);
	if (!pollUs(20000, [&]{ return (rd(GPU_L2_READY_LO) & l2) == l2; })) {
		IOLog("VIVA GPU: L2 not ready (0x%x)\n", rd(GPU_L2_READY_LO));
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
		IOLog("VIVA GPU: shader cores not ready (0x%x of 0x%x)\n", rd(GPU_SHADER_READY_LO), sh);
		return false;
	}
	pollUs(20000, [&]{ return (rd(GPU_TILER_READY_LO) & (uint32_t)fTilerPresent) == (uint32_t)fTilerPresent; });
	return true;
}

bool
VivaGPUAccelerator::maliInit(char **out, char *end)
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
	say(out, end, "VIVA GPU: id 0x%08x (product 0x%04x r%up%u) cores %u (0x%llx) l2 on (0x%llx), "
	    "as 0x%x js 0x%x mmu 0x%08x coherency 0x%x\n", fGpuId, fGpuId >> 16, (fGpuId >> 12) & 0xf,
	    (fGpuId >> 4) & 0xff, cores, fShaderPresent, fL2Present, fAsPresent, fJsPresent, fMmuFeatures,
	    fCoherencyFeatures);
	return true;
}

IOReturn
VivaGPUAccelerator::powerOn(char **out, char *end)
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
	IOLog("VIVA GPU: id 0x%08x cores 0x%llx l2 on\n", fGpuId, fShaderPresent);
	fPowered = true;
	for (int i = 0; i < 3; i++) {
		fIrq[i]->enable();
	}
	return kIOReturnSuccess;
}

void
VivaGPUAccelerator::powerOff(char **out, char *end)
{
	if (!fPowered) {
		mfgOff(out, end);
		return;
	}
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
VivaGPUAccelerator::asWait(int as)
{
	bool ok = pollUs(100000, [&]{ return !(rd(AS_BASE(as) + AS_STATUS) & AS_STATUS_ACTIVE); });
	if (!ok) {
		IOLog("VIVA GPU: AS%d stuck active\n", as);
	}
	return ok;
}

void
VivaGPUAccelerator::asCommand(int as, uint32_t cmd)
{
	asWait(as);
	wr(AS_BASE(as) + AS_COMMAND, cmd);
}

/* Lock the naturally aligned power-of-two region covering [va, va + size) (32 KiB minimum). */
void
VivaGPUAccelerator::asLock(int as, uint64_t va, uint64_t size)
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
VivaGPUAccelerator::asProgram(int as, VivaGPUVM *vm)
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
VivaGPUAccelerator::asDisable(int as)
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
VivaGPUAccelerator::asBind(VivaGPUVM *vm)
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
VivaGPUAccelerator::vmFlush(VivaGPUVM *vm, uint64_t va, uint64_t size)
{
	if (!fPowered || vm->asSlot < 0) {
		return;         /* binding the AS later flushes everything */
	}
	asLock(vm->asSlot, va, size);
	asCommand(vm->asSlot, AS_CMD_FLUSH_PT);
	asWait(vm->asSlot);
}

void
VivaGPUAccelerator::vmDestroyed(VivaGPUVM *vm)
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
		return kIOReturnSuccess;
	});
}

void
VivaGPUAccelerator::heapAdd(VivaGPUBO *bo)
{
	if (fHeapCount == fHeapCap) {
		uint32_t cap = fHeapCap ? fHeapCap * 2 : 16;
		VivaGPUBO **list = IONew(VivaGPUBO *, cap);
		if (list == NULL) {
			return;
		}
		if (fHeapList != NULL) {
			bcopy(fHeapList, list, fHeapCount * sizeof(*list));
			IODelete(fHeapList, VivaGPUBO *, fHeapCap);
		}
		fHeapList = list;
		fHeapCap = cap;
	}
	fHeapList[fHeapCount++] = bo;
}

void
VivaGPUAccelerator::heapRemove(VivaGPUBO *bo)
{
	for (uint32_t i = 0; i < fHeapCount; i++) {
		if (fHeapList[i] == bo) {
			fHeapList[i] = fHeapList[--fHeapCount];
			return;
		}
	}
}

VivaGPUBO *
VivaGPUAccelerator::findHeapBO(VivaGPUVM *vm, uint64_t va)
{
	for (uint32_t i = 0; i < fHeapCount; i++) {
		VivaGPUBO *bo = fHeapList[i];
		if (bo->vm == vm && va >= bo->va && va < bo->va + bo->vaSize) {
			return bo;
		}
	}
	return NULL;
}

/* ---- jobs ---- */

IOReturn
VivaGPUAccelerator::submit(VivaGPUVM *vm, uint64_t jc, uint32_t requirements, OSArray *deps, OSArray *bos,
    VivaGPUFence *fence)
{
	if (!fPowered) {
		return kIOReturnNotReady;
	}
	VivaGPUJob *job = IOMallocType(VivaGPUJob);
	job->vm = vm;
	vm->retain();
	job->jc = jc;
	job->requirements = requirements;
	job->deps = deps;
	if (deps != NULL) {
		deps->retain();
	}
	job->bos = bos;
	if (bos != NULL) {
		bos->retain();
		for (unsigned int i = 0; i < bos->getCount(); i++) {
			VivaGPUBO *bo = (VivaGPUBO *)bos->getObject(i);
			fence->retain();
			OSSafeReleaseNULL(bo->busy);
			bo->busy = fence;
			if (bo->cacheMaint) {
				bo->cacheOp(0, 0, kVivaCacheFlush);     /* CPU writes out before the GPU reads */
			}
		}
	}
	job->fence = fence;
	fence->retain();
	uint32_t slot = (requirements & VGPU_JD_REQ_FS) ? 0 : 1;
	if (fQueueTail[slot] != NULL) {
		fQueueTail[slot]->next = job;
	} else {
		fQueue[slot] = job;
	}
	fQueueTail[slot] = job;
	kick();
	return kIOReturnSuccess;
}

void
VivaGPUAccelerator::kick(void)
{
	if (!fPowered) {
		return;
	}
	for (uint32_t slot = 0; slot < 2; slot++) {
		VivaGPUJob *job = fQueue[slot];
		if (fRunning[slot] != NULL || job == NULL) {
			continue;
		}
		bool ready = true;
		for (unsigned int i = 0; job->deps != NULL && i < job->deps->getCount(); i++) {
			if (!((VivaGPUFence *)job->deps->getObject(i))->signalled) {
				ready = false;
				break;
			}
		}
		if (!ready || !asBind(job->vm)) {
			continue;
		}
		fQueue[slot] = job->next;
		if (fQueue[slot] == NULL) {
			fQueueTail[slot] = NULL;
		}
		job->next = NULL;
		startJob(slot, job);
	}
}

void
VivaGPUAccelerator::startJob(uint32_t slot, VivaGPUJob *job)
{
	uint32_t js = JS_BASE(slot);
	fRunning[slot] = job;
	fStopDeadline[slot] = 0;
	job->vm->running++;
	job->startMs = nowMs();
	fJobs++;
	wr(js + JS_HEAD_NEXT_LO, (uint32_t)job->jc);
	wr(js + JS_HEAD_NEXT_HI, (uint32_t)(job->jc >> 32));
	wr(js + JS_AFFINITY_NEXT_LO, (uint32_t)fShaderPresent);
	wr(js + JS_AFFINITY_NEXT_HI, (uint32_t)(fShaderPresent >> 32));
	wr(js + JS_CONFIG_NEXT, (uint32_t)job->vm->asSlot | JS_CFG_THREAD_PRI(8) | JS_CFG_START_FLUSH_CLEAN_INV |
	    JS_CFG_END_FLUSH_CLEAN_INV);
	wr(js + JS_COMMAND_NEXT, JS_CMD_START);
	if (!fTimerArmed) {
		fTimerArmed = true;
		fTimer->setTimeoutMS(TIMER_MS);
	}
}

void
VivaGPUAccelerator::completeJob(uint32_t slot, IOReturn error, uint32_t status)
{
	VivaGPUJob *job = fRunning[slot];
	if (job == NULL) {
		return;
	}
	fRunning[slot] = NULL;
	fStopDeadline[slot] = 0;
	job->vm->running--;
	if (error != kIOReturnSuccess) {
		fJobFaults++;
		if (fJobFaults <= 32) {
			IOLog("VIVA GPU: job on slot %u failed: status 0x%02x (%s), chain 0x%llx, AS%d, after %llu ms\n",
			    slot, status, jsStatusName(status), job->jc, job->vm->asSlot, nowMs() - job->startMs);
		}
	}
	for (unsigned int i = 0; job->bos != NULL && i < job->bos->getCount(); i++) {
		VivaGPUBO *bo = (VivaGPUBO *)job->bos->getObject(i);
		if (bo->cacheMaint) {
			bo->cacheOp(0, 0, kVivaCacheInvalidate);        /* drop CPU lines: the CPU sees what the GPU wrote */
		}
	}
	job->fence->signalled = true;
	job->fence->error = error;
	OSSafeReleaseNULL(job->deps);
	OSSafeReleaseNULL(job->bos);
	OSSafeReleaseNULL(job->fence);
	OSSafeReleaseNULL(job->vm);
	IOFreeType(job, VivaGPUJob);
	wakeWaiters();
}

/* Fail the jobs on the hardware (reset, power off); queued jobs stay queued (power off fails them too). */
void
VivaGPUAccelerator::failAll(IOReturn error)
{
	for (uint32_t slot = 0; slot < 2; slot++) {
		completeJob(slot, error, 0x04);
	}
	if (error == kIOReturnOffline) {
		for (uint32_t slot = 0; slot < 2; slot++) {
			while (fQueue[slot] != NULL) {
				VivaGPUJob *job = fQueue[slot];
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
VivaGPUAccelerator::resetGPU(const char *why)
{
	fResets++;
	IOLog("VIVA GPU: reset (%s)\n", why);
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
		IOLog("VIVA GPU: reset failed, GPU unusable until gpu off/on: %s", buf);
		for (int i = 0; i < 3; i++) {
			fIrq[i]->disable();
		}
		failAll(kIOReturnOffline);
		fPowered = false;
		return;
	}
	kick();
}

void
VivaGPUAccelerator::gpuInterrupt(OSObject *owner, IOInterruptEventSource *src __unused, int count __unused)
{
	VivaGPUAccelerator *self = OSDynamicCast(VivaGPUAccelerator, owner);
	if (self == NULL || !self->fPowered) {
		return;
	}
	uint32_t stat = self->rd(GPU_IRQ_STATUS);
	if (stat & (GPU_IRQ_FAULT | GPU_IRQ_MULTIPLE_FAULT)) {
		self->fGpuFaults++;
		if (self->fGpuFaults <= 16) {
			IOLog("VIVA GPU: GPU fault: status 0x%08x address 0x%08x%08x\n", self->rd(GPU_FAULT_STATUS),
			    self->rd(GPU_FAULT_ADDRESS_HI), self->rd(GPU_FAULT_ADDRESS_LO));
		}
	}
	self->wr(GPU_IRQ_CLEAR, stat);
}

void
VivaGPUAccelerator::mmuInterrupt(OSObject *owner, IOInterruptEventSource *src __unused, int count __unused)
{
	VivaGPUAccelerator *self = OSDynamicCast(VivaGPUAccelerator, owner);
	if (self == NULL || !self->fPowered) {
		return;
	}
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
		VivaGPUVM *vm = self->fAS[as];
		bool handled = false;
		if (vm != NULL && (bits & (1u << as)) && (fs & 0xf8) == 0xc0) {
			VivaGPUBO *heap = self->findHeapBO(vm, addr);
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
				IOLog("VIVA GPU: MMU %s on AS%u at 0x%llx: status 0x%08x (%s, %s access, source 0x%04x)\n",
				    (bits & (1u << as)) ? "fault" : "bus error", as, addr, fs, jsStatusName(fs & 0xff),
				    ((fs >> 8) & 3) == 1 ? "execute" : ((fs >> 8) & 3) == 2 ? "read" :
				    ((fs >> 8) & 3) == 3 ? "write" : "atomic", fs >> 16);
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
VivaGPUAccelerator::jobInterrupt(OSObject *owner, IOInterruptEventSource *src __unused, int count __unused)
{
	VivaGPUAccelerator *self = OSDynamicCast(VivaGPUAccelerator, owner);
	if (self == NULL || !self->fPowered) {
		return;
	}
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
			if (slot < 2) {
				self->completeJob(slot, kIOReturnIOError, status);
			}
			if ((status & 0xff) == 0x48) {
				self->wr(JOB_INT_CLEAR, stat & (done | fail));
				self->resetGPU("job bus fault (TTRX_3076: later fragment jobs may hang)");
				return;
			}
		} else if (slot < 2) {
			self->completeJob(slot, kIOReturnSuccess, 0x01);
		}
		self->wr(JOB_INT_CLEAR, stat & (done | fail));
	}
	self->kick();
}

void
VivaGPUAccelerator::jobTimer(OSObject *owner, IOTimerEventSource *src __unused)
{
	VivaGPUAccelerator *self = OSDynamicCast(VivaGPUAccelerator, owner);
	if (self == NULL) {
		return;
	}
	self->fTimerArmed = false;
	if (!self->fPowered) {
		return;
	}
	uint64_t now = nowMs();
	bool busy = false;
	for (uint32_t slot = 0; slot < 2; slot++) {
		VivaGPUJob *job = self->fRunning[slot];
		if (job == NULL) {
			continue;
		}
		busy = true;
		if (self->fStopDeadline[slot] != 0 && now >= self->fStopDeadline[slot]) {
			self->resetGPU("a job ignored its hard stop");
			if (self->fPowered && (self->fRunning[0] || self->fRunning[1])) {
				self->fTimerArmed = true;
				self->fTimer->setTimeoutMS(TIMER_MS);
			}
			return;
		}
		if (self->fStopDeadline[slot] == 0 && now - job->startMs > JOB_TIMEOUT_MS) {
			self->fTimeouts++;
			IOLog("VIVA GPU: job on slot %u timed out after %llu ms (chain 0x%llx head 0x%08x%08x), hard stop\n",
			    slot, now - job->startMs, job->jc, self->rd(JS_BASE(slot) + JS_HEAD_HI),
			    self->rd(JS_BASE(slot) + JS_HEAD_LO));
			self->wr(JS_BASE(slot) + JS_COMMAND, JS_CMD_HARD_STOP);
			self->fStopDeadline[slot] = now + JOB_STOP_GRACE_MS;
		}
	}
	if (busy) {
		self->fTimerArmed = true;
		self->fTimer->setTimeoutMS(TIMER_MS);
	}
}

IOReturn
VivaGPUAccelerator::sleepUntil(uint64_t deadline)
{
	IOReturn r = deadline ? fGate->commandSleep(&fFenceEvent, deadline, THREAD_ABORTSAFE) :
	    fGate->commandSleep(&fFenceEvent, THREAD_ABORTSAFE);
	return r == THREAD_TIMED_OUT ? kIOReturnTimeout : r == THREAD_INTERRUPTED ? kIOReturnAborted : kIOReturnSuccess;
}

IOReturn
VivaGPUAccelerator::waitFence(VivaGPUFence *fence, uint64_t deadline)
{
	while (!fence->signalled) {
		IOReturn r = sleepUntil(deadline);
		if (r != kIOReturnSuccess) {
			return fence->signalled ? kIOReturnSuccess : r;
		}
	}
	return kIOReturnSuccess;
}

/* ---- parameters ---- */

IOReturn
VivaGPUAccelerator::getParam(uint32_t param, uint64_t *value)
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
VivaGPUAccelerator::getVersion(struct vgpu_version *v)
{
	v->major = VGPU_UAPI_MAJOR;
	v->minor = VGPU_UAPI_MINOR;
	v->gpu_id = fGpuId;
	v->page_size = (uint32_t)page_size;
	snprintf(v->name, sizeof(v->name), "Mali-G57 MC%u", (unsigned)__builtin_popcountll(fShaderPresent));
}

/* ---- self test (USB link) ---- */

void
VivaGPUAccelerator::selftest(bool fault, char **out, char *end)
{
	if (!fPowered) {
		say(out, end, "selftest: GPU off (gpu on first)\n");
		return;
	}
	if (fSelfVM == NULL && (fSelfVM = VivaGPUVM::create(this)) == NULL) {
		say(out, end, "selftest: no VM\n");
		return;
	}
	VivaGPUBO *bo = new VivaGPUBO;
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
	bo->kind = kVivaBONormal;
	bo->memory = mem;
	bo->cacheMaint = true;  /* written and read through the kernel's cacheable mapping */
	bo->size = bo->vaSize = page_size;
	chargeMemory(page_size);
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

	VivaGPUFence *fence = VivaGPUFence::create(false);
	OSArray *bos = OSArray::withObjects((const OSObject **)&bo, 1);
	uint64_t t0 = nowMs(), mmu0 = fMmuFaults;
	IOReturn ret = (fence && bos) ? submit(fSelfVM, bo->va, 0, NULL, bos, fence) : kIOReturnNoMemory;
	if (ret == kIOReturnSuccess) {
		uint64_t deadline;
		clock_interval_to_deadline(1000, kMillisecondScale, &deadline);
		ret = waitFence(fence, deadline);
	}
	uint32_t got = w[0x200 / 4], exc = w[0];
	say(out, end, "selftest%s: submit/wait 0x%x after %llu ms, fence %s 0x%x, job status 0x%02x (%s), "
	    "value 0x%08x (expected 0x%08x), MMU faults +%llu\n", fault ? " fault" : "", ret, nowMs() - t0,
	    fence && fence->signalled ? "signalled" : "pending", fence ? fence->error : 0, exc & 0xff,
	    jsStatusName(exc & 0xff), got, fault ? 0 : value, fMmuFaults - mmu0);
	if (fault) {
		say(out, end, "selftest fault: %s\n", (fence && fence->signalled && fence->error != kIOReturnSuccess &&
		    fMmuFaults > mmu0) ? "PASS (fault caught, GPU still alive: run gpu selftest)" : "FAIL");
	} else {
		say(out, end, "selftest: %s\n", got == value ? "PASS" : "FAIL");
	}
	if (fence != NULL && !fence->signalled) {
		resetGPU("selftest timed out");
	}
	OSSafeReleaseNULL(bos);
	OSSafeReleaseNULL(fence);
	bo->release();
}

/* ---- USB link: "gpu [status|mfg|on|off|regs|selftest [fault]|probe|publish]" ---- */

void
VivaGPUAccelerator::linkCommand(const char *args, char *out, size_t outLen)
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
			selftest(strncmp(args, "selftest fault", 14) == 0, o, end);
		} else if (strncmp(args, "probe", 5) == 0) {
			probeDump(o, end);
		} else if (strncmp(args, "publish", 7) == 0) {
			if (!fPowered) {
				say(o, end, "GPU off: gpu on first\n");
			} else if (!fPublished) {
				fPublished = true;
				registerService();
				say(o, end, "accelerator published (IOAcceleratorES, MetalPluginName VivaMTLDriver)\n");
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
			say(o, end, "gpu: %s, mode %d%s, id 0x%08x, memory %lld KiB\n", fPowered ? "on" : "off", fMode,
			    fPublished ? ", published" : "", fGpuId, fMemory >> 10);
			say(o, end, "jobs %llu (running %d/%d, queued %d/%d), job faults %llu, MMU faults %llu, heap grows "
			    "%llu, GPU faults %llu, timeouts %llu, resets %llu\n", fJobs, fRunning[0] != NULL,
			    fRunning[1] != NULL, fQueue[0] != NULL, fQueue[1] != NULL, fJobFaults, fMmuFaults, fHeapGrows,
			    fGpuFaults, fTimeouts, fResets);
			say(o, end, "usage: gpu [status|mfg|on|off|regs|selftest [fault]|probe|publish]\n");
		}
		return kIOReturnSuccess;
	});
}

extern "C" void
viva_gpu_link(const char *args, char *out, size_t outLen)
{
	if (gVivaGPU == NULL) {
		snprintf(out, outLen, "gpu: driver not running (boot-arg vivagpu=1 or 2)\n");
		return;
	}
	gVivaGPU->linkCommand(args, out, outLen);
}
#endif /* SWEET */
