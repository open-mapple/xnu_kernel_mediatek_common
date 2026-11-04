/*
 * viva (MT6781): memory for CONNSYS (bsd/dev/viva_wifi.c does the rest, in C).
 *
 * The CONNSYS MCU sees a 5 MiB DRAM window ("EMI", consys-reserve-memory on Android) at 0xF0000000 through the
 * remap register infracfg_ao 0x10001380[19:0] = PA >> 16: one physically contiguous range, below 4 GiB (the DT's
 * alloc-ranges keep it 32-bit), 16 MiB aligned like the DT asks (64 KiB, the remap granularity, as a fallback).
 * Cacheable; viva_wifi_emi_flush() pushes the CPU's writes out before the MCU reads them.
 * viva_wifi_dma_alloc(): the WFDMA rings and buffers (any contiguous range below 4 GiB; the WFDMA takes 36 bits).
 * viva_wifi_reserve(): both taken at boot (VivaPwrap::start), before the contiguous scanout surfaces (vivascanout=1)
 * fragment the memory below 4 GiB; boot-arg vivawifi=0 skips it (then "wifi on" tries later, and may fail).
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <pexpert/pexpert.h>

static IOBufferMemoryDescriptor *gVivaWiFiEMI;

extern "C" void *
viva_wifi_emi_alloc(uint32_t size, uint64_t *pa)
{
	if (gVivaWiFiEMI == NULL) {
		static const uint64_t masks[] = { 0x00000000ff000000ull, 0x00000000ffff0000ull };
		for (unsigned i = 0; i < 2 && gVivaWiFiEMI == NULL; i++) {
			gVivaWiFiEMI = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
			    kIODirectionInOut | kIOMemoryPhysicallyContiguous, size, masks[i]);
		}
		if (gVivaWiFiEMI == NULL) {
			return NULL;
		}
		bzero(gVivaWiFiEMI->getBytesNoCopy(), size);
		gVivaWiFiEMI->performOperation(kIOMemoryIncoherentIOFlush, 0, size);
	}
	if (gVivaWiFiEMI->getLength() < size) {
		return NULL;
	}
	*pa = gVivaWiFiEMI->getPhysicalSegment(0, NULL, kIOMemoryMapperNone);
	return gVivaWiFiEMI->getBytesNoCopy();
}

static IOBufferMemoryDescriptor *gVivaWiFiDMA;

extern "C" void *
viva_wifi_dma_alloc(uint32_t size, uint64_t *pa)
{
	if (gVivaWiFiDMA == NULL) {
		gVivaWiFiDMA = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
		    kIODirectionInOut | kIOMemoryPhysicallyContiguous, size, 0x00000000fffff000ull);
		if (gVivaWiFiDMA == NULL) {
			return NULL;
		}
	}
	if (gVivaWiFiDMA->getLength() < size) {
		return NULL;
	}
	*pa = gVivaWiFiDMA->getPhysicalSegment(0, NULL, kIOMemoryMapperNone);
	return gVivaWiFiDMA->getBytesNoCopy();
}

#define VIVA_WIFI_EMI_SIZE      0x500000        /* EMI_SIZE and DMA_SIZE in bsd/dev/viva_wifi.c */
#define VIVA_WIFI_DMA_SIZE      0x48000

extern "C" void
viva_wifi_reserve(void)
{
	uint32_t enable = 1;
	uint64_t emi = 0, dma = 0;
	PE_parse_boot_argn("vivawifi", &enable, sizeof(enable));
	if (enable == 0) {
		return;
	}
	void *e = viva_wifi_emi_alloc(VIVA_WIFI_EMI_SIZE, &emi);
	void *d = viva_wifi_dma_alloc(VIVA_WIFI_DMA_SIZE, &dma);
	IOLog("VIVA WIFI: reserved EMI %s 0x%llx, DMA %s 0x%llx\n", e ? "at" : "FAILED", emi, d ? "at" : "FAILED", dma);
}

/* The DMA memory mapped write-combined (normal uncached), for the ring descriptors (viva_wifi.c desc()). */
static IOMemoryMap *gVivaWiFiDMAUncached;

extern "C" void *
viva_wifi_dma_uncached(void)
{
	if (gVivaWiFiDMAUncached == NULL && gVivaWiFiDMA != NULL) {
		gVivaWiFiDMAUncached = gVivaWiFiDMA->createMappingInTask(kernel_task, 0,
		    kIOMapAnywhere | kIOMapWriteCombineCache, 0, 0);
	}
	return gVivaWiFiDMAUncached ? (void *)gVivaWiFiDMAUncached->getVirtualAddress() : NULL;
}

/* Clean + invalidate a range of the DMA memory (before the device reads it, and before the CPU reads its writes). */
extern "C" void
viva_wifi_dma_flush(uint32_t offset, uint32_t len)
{
	if (gVivaWiFiDMA != NULL) {
		gVivaWiFiDMA->performOperation(kIOMemoryIncoherentIOFlush, offset, len);
	}
}

/* CPU writes out to DRAM (clean + invalidate) before CONNSYS reads; also before the CPU reads what it wrote. */
extern "C" void
viva_wifi_emi_flush(uint32_t offset, uint32_t len)
{
	if (gVivaWiFiEMI != NULL) {
		gVivaWiFiEMI->performOperation(kIOMemoryIncoherentIOFlush, offset, len);
	}
}
#endif /* SWEET */
