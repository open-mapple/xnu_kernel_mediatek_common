/*
 * MT6781: memory for CONNSYS (bsd/dev/mt6631_wifi.c does the rest, in C).
 *
 * The CONNSYS MCU sees a 5 MiB DRAM window ("EMI", consys-reserve-memory on Android) at 0xF0000000 through the
 * remap register infracfg_ao 0x10001380[19:0] = PA >> 16: one physically contiguous range, below 4 GiB (the DT's
 * alloc-ranges keep it 32-bit), 16 MiB aligned like the DT asks (64 KiB, the remap granularity, as a fallback).
 * Cacheable; mt6631_wifi_emi_flush() pushes the CPU's writes out before the MCU reads them.
 * mt6631_wifi_dma_alloc(): the WFDMA rings and buffers (any contiguous range below 4 GiB; the WFDMA takes 36 bits).
 * mt6631_wifi_reserve(): both taken at boot (MTKPwrap::start), before the contiguous scanout surfaces (mp_fb_scanout=1)
 * fragment the memory below 4 GiB; boot-arg mp_wifi=0 skips it (then "wifi on" tries later, and may fail).
 */
#include <pexpert/arm64/board_config.h>

#if defined(MT6781)
#include <IOKit/IOLib.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <pexpert/pexpert.h>

static IOBufferMemoryDescriptor *gMappleWiFiEMI;

extern "C" void *
mt6631_wifi_emi_alloc(uint32_t size, uint64_t *pa)
{
	if (gMappleWiFiEMI == NULL) {
		static const uint64_t masks[] = { 0x00000000ff000000ull, 0x00000000ffff0000ull };
		for (unsigned i = 0; i < 2 && gMappleWiFiEMI == NULL; i++) {
			gMappleWiFiEMI = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
			    kIODirectionInOut | kIOMemoryPhysicallyContiguous, size, masks[i]);
		}
		if (gMappleWiFiEMI == NULL) {
			return NULL;
		}
		bzero(gMappleWiFiEMI->getBytesNoCopy(), size);
		gMappleWiFiEMI->performOperation(kIOMemoryIncoherentIOFlush, 0, size);
	}
	if (gMappleWiFiEMI->getLength() < size) {
		return NULL;
	}
	*pa = gMappleWiFiEMI->getPhysicalSegment(0, NULL, kIOMemoryMapperNone);
	return gMappleWiFiEMI->getBytesNoCopy();
}

static IOBufferMemoryDescriptor *gMappleWiFiDMA;

extern "C" void *
mt6631_wifi_dma_alloc(uint32_t size, uint64_t *pa)
{
	if (gMappleWiFiDMA == NULL) {
		gMappleWiFiDMA = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
		    kIODirectionInOut | kIOMemoryPhysicallyContiguous, size, 0x00000000fffff000ull);
		if (gMappleWiFiDMA == NULL) {
			return NULL;
		}
	}
	if (gMappleWiFiDMA->getLength() < size) {
		return NULL;
	}
	*pa = gMappleWiFiDMA->getPhysicalSegment(0, NULL, kIOMemoryMapperNone);
	return gMappleWiFiDMA->getBytesNoCopy();
}

#define MT6631_WIFI_EMI_SIZE      0x500000        /* EMI_SIZE and DMA_SIZE in bsd/dev/mt6631_wifi.c */
#define MT6631_WIFI_DMA_SIZE      0x48000

extern "C" void
mt6631_wifi_reserve(void)
{
	uint32_t enable = 1;
	uint64_t emi = 0, dma = 0;
	PE_parse_boot_argn_alias("mp_wifi", "vivawifi", &enable, sizeof(enable));
	if (enable == 0) {
		return;
	}
	void *e = mt6631_wifi_emi_alloc(MT6631_WIFI_EMI_SIZE, &emi);
	void *d = mt6631_wifi_dma_alloc(MT6631_WIFI_DMA_SIZE, &dma);
	IOLog("WIFI: reserved EMI %s 0x%llx, DMA %s 0x%llx\n", e ? "at" : "FAILED", emi, d ? "at" : "FAILED", dma);
}

/* The DMA memory mapped write-combined (normal uncached), for the ring descriptors (mt6631_wifi.c desc()). */
static IOMemoryMap *gMappleWiFiDMAUncached;

extern "C" void *
mt6631_wifi_dma_uncached(void)
{
	if (gMappleWiFiDMAUncached == NULL && gMappleWiFiDMA != NULL) {
		gMappleWiFiDMAUncached = gMappleWiFiDMA->createMappingInTask(kernel_task, 0,
		    kIOMapAnywhere | kIOMapWriteCombineCache, 0, 0);
	}
	return gMappleWiFiDMAUncached ? (void *)gMappleWiFiDMAUncached->getVirtualAddress() : NULL;
}

/* Clean + invalidate a range of the DMA memory (before the device reads it, and before the CPU reads its writes). */
extern "C" void
mt6631_wifi_dma_flush(uint32_t offset, uint32_t len)
{
	if (gMappleWiFiDMA != NULL) {
		gMappleWiFiDMA->performOperation(kIOMemoryIncoherentIOFlush, offset, len);
	}
}

/* CPU writes out to DRAM (clean + invalidate) before CONNSYS reads; also before the CPU reads what it wrote. */
extern "C" void
mt6631_wifi_emi_flush(uint32_t offset, uint32_t len)
{
	if (gMappleWiFiEMI != NULL) {
		gMappleWiFiEMI->performOperation(kIOMemoryIncoherentIOFlush, offset, len);
	}
}
#endif /* MT6781 */
