/*
 * viva (MT6781): memory for CONNSYS (bsd/dev/viva_wifi.c does the rest, in C).
 *
 * The CONNSYS MCU sees a 5 MiB DRAM window ("EMI", consys-reserve-memory on Android) at 0xF0000000 through the
 * remap register infracfg_ao 0x10001380[19:0] = PA >> 16: one physically contiguous range, below 4 GiB (the DT's
 * alloc-ranges keep it 32-bit), 16 MiB aligned like the DT asks (64 KiB, the remap granularity, as a fallback).
 * Cacheable; viva_wifi_emi_flush() pushes the CPU's writes out before the MCU reads them.
 * viva_wifi_dma_alloc(): the WFDMA rings and buffers (any contiguous range below 4 GiB; the WFDMA takes 36 bits).
 */
#include <pexpert/arm64/board_config.h>

#if defined(SWEET)
#include <IOKit/IOLib.h>
#include <IOKit/IOBufferMemoryDescriptor.h>

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
