/*
 * MT6781: GPIO and I2C bus drivers.
 *
 * MTKGPIO  (/arm-io/gpio, "mediatek,mt6781-gpio"): pin input/output for
 *           drivers; only the pins listed in the node's "output-pins" can be
 *           driven, everything else is refused.
 * MTKI2C   (/arm-io/i2c0, /arm-io/i2c3, "mediatek,mt6781-i2c"): MediaTek
 *           I2C v2 controller in FIFO mode (8 bytes per direction and
 *           transfer), interrupt driven; publishes its device-tree children
 *           as MappleDTNub nubs (property "i2c-address"). "bus-id" = the SoC's
 *           I2C number; optional reg 1 + "imp-cg-bit" = its IMP_IIC_WRAP
 *           clock gate, opened at start if closed.
 */
#ifndef _IOKIT_ARM_MTKIO_H
#define _IOKIT_ARM_MTKIO_H

#include <IOKit/IOService.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOCommandGate.h>
#include <IOKit/IOInterruptEventSource.h>

#include "MT6358PMIC.h"   /* MappleDTNub */

class MTKGPIO : public IOService {
	OSDeclareDefaultStructors(MTKGPIO);
public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	IOReturn setOutput(uint32_t pin, bool high);
	IOReturn getInput(uint32_t pin, bool *high);
	/* The started instance, waiting for it if needed (NULL after timeout). */
	static MTKGPIO *wait(void);

private:
	bool outputAllowed(uint32_t pin);
	inline uint32_t rd32(uint32_t off) { return *(volatile uint32_t *)(fRegs + off); }
	inline void wr32(uint32_t off, uint32_t v) { *(volatile uint32_t *)(fRegs + off) = v; }

	IOMemoryMap *fMap;
	vm_offset_t  fRegs;
	OSData      *fOutputs;
};

class MTKI2C : public IOService {
	OSDeclareDefaultStructors(MTKI2C);
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
	/* The started controller with this bus-id, waiting for it if needed (NULL after timeout). */
	static MTKI2C *wait(uint32_t busId);
	uint32_t busId(void) const { return fBusId; }

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
	uint32_t                fSrcHz, fSpeedHz, fClockDiv, fBusId;
	uint16_t                fTiming, fLTiming, fHWTimeout, fExtConf;
	volatile uint16_t       fIrqStat;
	bool                    fDone;
	uint32_t                fErrors;
};

#endif /* _IOKIT_ARM_MTKIO_H */
