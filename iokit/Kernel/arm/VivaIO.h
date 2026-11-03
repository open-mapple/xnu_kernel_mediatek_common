/*
 * viva (MT6781): GPIO and I2C bus drivers.
 *
 * VivaGPIO  (/arm-io/gpio, "mediatek,mt6781-gpio"): pin input/output for
 *           drivers; only the pins listed in the node's "output-pins" can be
 *           driven, everything else is refused.
 * VivaI2C   (/arm-io/i2c0, "mediatek,mt6781-i2c"): MediaTek I2C v2
 *           controller in FIFO mode (8 bytes per direction and transfer),
 *           interrupt driven; publishes its device-tree children as
 *           VivaDTNub nubs (property "i2c-address").
 */
#ifndef _IOKIT_ARM_VIVAIO_H
#define _IOKIT_ARM_VIVAIO_H

#include <IOKit/IOService.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOCommandGate.h>
#include <IOKit/IOInterruptEventSource.h>

#include "VivaPMIC.h"   /* VivaDTNub */

class VivaGPIO : public IOService {
	OSDeclareDefaultStructors(VivaGPIO);
public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	IOReturn setOutput(uint32_t pin, bool high);
	IOReturn getInput(uint32_t pin, bool *high);
	/* The started instance, waiting for it if needed (NULL after timeout). */
	static VivaGPIO *wait(void);

private:
	bool outputAllowed(uint32_t pin);
	inline uint32_t rd32(uint32_t off) { return *(volatile uint32_t *)(fRegs + off); }
	inline void wr32(uint32_t off, uint32_t v) { *(volatile uint32_t *)(fRegs + off) = v; }

	IOMemoryMap *fMap;
	vm_offset_t  fRegs;
	OSData      *fOutputs;
};

class VivaI2C : public IOService {
	OSDeclareDefaultStructors(VivaI2C);
public:
	enum { kFIFOSize = 8 };

	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOWorkLoop *getWorkLoop(void) const APPLE_KEXT_OVERRIDE;

	/*
	 * One transaction to 7-bit `addr`: write `wlen` bytes, then (if rlen)
	 * a repeated start and read `rlen` bytes. wlen, rlen <= kFIFOSize.
	 * Sleeps: not from interrupt context.
	 */
	IOReturn transfer(uint8_t addr, const uint8_t *wbuf, uint32_t wlen, uint8_t *rbuf, uint32_t rlen);

private:
	static void interruptOccurred(OSObject *owner, IOInterruptEventSource *src, int count);
	IOReturn transferGated(uint8_t addr, const uint8_t *wbuf, uint32_t wlen, uint8_t *rbuf, uint32_t rlen);
	void initHW(void);
	bool calcSpeed(uint32_t srcHz, uint32_t speedHz, uint32_t *step, uint32_t *sample);
	inline uint16_t rd16(uint32_t off) { return *(volatile uint16_t *)(fRegs + off); }
	inline void wr16(uint32_t off, uint16_t v) { *(volatile uint16_t *)(fRegs + off) = v; }

	IOMemoryMap            *fMap;
	vm_offset_t             fRegs;
	IOWorkLoop             *fWorkLoop;
	IOCommandGate          *fGate;
	IOInterruptEventSource *fInterrupt;
	uint32_t                fSrcHz, fSpeedHz, fClockDiv;
	uint16_t                fTiming, fLTiming, fHWTimeout, fExtConf;
	volatile uint16_t       fIrqStat;
	bool                    fDone;
	uint32_t                fErrors;
};

#endif /* _IOKIT_ARM_VIVAIO_H */
