/*
 * MT6781: the PMIC bus. MTKPwrap (the SoC's PMIC wrapper) carries
 * PMIC register access; MT6358PMIC (the MT6366 behind it, MT6358 register
 * layout) owns the PMIC interrupt line and hands PMIC interrupt groups to
 * its children (MT6358Keys, later charger/RTC/...).
 *
 * Device tree: /arm-io/pwrap/pmic/keys; each level publishes its children
 * as MappleDTNub nubs, so drivers match them by "compatible" as usual.
 */
#ifndef _IOKIT_ARM_MT6358PMIC_H
#define _IOKIT_ARM_MT6358PMIC_H

#include <IOKit/IOService.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOInterruptEventSource.h>

/* A device-tree child of a bus driver (the platform expert's IOPlatformDevice needs the PE as provider). */
class MappleDTNub : public IOService {
	OSDeclareDefaultStructors(MappleDTNub);
public:
	virtual bool compareName(OSString *name, OSString **matched = NULL) const APPLE_KEXT_OVERRIDE;
	/* Publish every device-tree child of `dtEntry` as a nub of `parent`. */
	static void publishChildren(IOService *parent, IORegistryEntry *dtEntry);
};

class MTKPwrap : public IOService {
	OSDeclareDefaultStructors(MTKPwrap);
public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	IOReturn readPMIC(uint32_t addr, uint32_t *value);
	/* Only the PMIC interrupt enable/status registers (writeAllowed) can be written. */
	IOReturn writePMIC(uint32_t addr, uint32_t value);
	static bool writeAllowed(uint32_t addr);
	/*
	 * GPU rails only (VGPU, VSRAM_GPU: enable bit and VOSEL, with a voltage ceiling):
	 * read-modify-write of the bits in `mask`, read back. Everything else is refused.
	 */
	IOReturn writeRegulator(uint32_t addr, uint32_t mask, uint32_t value);
	/* The fuel gauge's latch and the AUXADC request bits only (gMappleGaugeRegs): read-modify-write, not read back. */
	IOReturn writeGauge(uint32_t addr, uint32_t mask, uint32_t value);
	/* The codec's speaker path and the VIBR rail (gMappleAudioRegs), gains and voltage capped: RMW, read back. */
	IOReturn writeAudio(uint32_t addr, uint32_t mask, uint32_t value);
	/* The started instance, waiting for it if needed (NULL after timeout). */
	static MTKPwrap *wait(void);
	/* MT6358PowerOff.h's sequence; returns only when the phone is still on. */
	void powerOff(void);

private:
	bool waitFSM(uint32_t fsm);
	static int poRead(uint32_t addr, uint32_t *value);
	static int poWrite(uint32_t addr, uint32_t value);
	inline uint32_t rd32(uint32_t off) { return *(volatile uint32_t *)(fRegs + off); }
	inline void wr32(uint32_t off, uint32_t v) { *(volatile uint32_t *)(fRegs + off) = v; }

	IOMemoryMap *fMap;
	vm_offset_t  fRegs;
	IOLock      *fLock;
	uint32_t     fTimeouts;
};

/* MapplePlatformExpert::haltRestart: MTKPwrap::powerOff; returns only when the phone is still on. */
extern "C" void mtk_pmic_poweroff(void);

/* PMIC interrupt groups (MT6358 "sp_top"), in TOP_INT_STATUS0 bit order. */
enum {
	kMapplePMICGroupBUCK = 0, kMapplePMICGroupLDO = 1, kMapplePMICGroupPSC = 2, kMapplePMICGroupSCK = 3,
	kMapplePMICGroupBM = 4, kMapplePMICGroupHK = 5, kMapplePMICGroupAUD = 7, kMapplePMICGroupMISC = 8,
	kMapplePMICGroupCount = 9,
};

/*
 * Per group (include/linux/mfd/mt6358/registers.h, Xiaomi viva-r-oss): register n of a
 * group has INT_CON = con0 + 6n (SET +2, CLR +4) and INT_STATUS = sta0 + 2n (write 1 to clear).
 * nregs from the DT's mediatek,pmic-irqs. con0 == 0: no such group (bit 6 is XPP, unused).
 */
struct MT6358PMICGroupRegs {
	uint16_t con0, sta0, nregs;
	const char *name;
};
static const MT6358PMICGroupRegs gMapplePMICGroups[kMapplePMICGroupCount] = {
	{ 0x1318, 0x1324, 1, "BUCK" }, { 0x1a50, 0x1a68, 2, "LDO" }, { 0x0910, 0x091c, 1, "PSC" },
	{ 0x052e, 0x053a, 1, "SCK" },  { 0x0c32, 0x0c4a, 2, "BM" },  { 0x0f92, 0x0f9e, 1, "HK" },
	{ 0, 0, 0, "XPP" },            { 0x2228, 0x2234, 1, "AUD" }, { 0x0188, 0x0194, 1, "MISC" },
};
#define MT6358_PMIC_TOP_INT_STATUS0 0x19e
#define MT6358_PMIC_HWCID         0x008

/* PSC group bits */
#define MT6358_PMIC_PSC_PWRKEY    (1u << 0)
#define MT6358_PMIC_PSC_HOMEKEY   (1u << 1)
#define MT6358_PMIC_PSC_PWRKEY_R  (1u << 2)
#define MT6358_PMIC_PSC_HOMEKEY_R (1u << 3)

#define MT6358_PMIC_TOPSTATUS     0x028   /* bit 1 PWRKEY_DEB, bit 3 HOMEKEY_DEB (0 = pressed) */

class MT6358PMIC : public IOService {
	OSDeclareDefaultStructors(MT6358PMIC);
public:
	/* Work-loop context: `status` = the group's status register `index` (already acknowledged). */
	typedef void (*Handler)(OSObject *target, uint32_t group, uint32_t index, uint32_t status);

	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOWorkLoop *getWorkLoop(void) const APPLE_KEXT_OVERRIDE;

	IOReturn read(uint32_t addr, uint32_t *value) { return fPwrap->readPMIC(addr, value); }
	/* Enable `bits` of a group's interrupt register `index` and send them to `handler`. */
	IOReturn registerGroupInterrupt(uint32_t group, uint32_t index, uint32_t bits, OSObject *target, Handler handler);

private:
	static void interruptOccurred(OSObject *owner, IOInterruptEventSource *src, int count);
	void service(void);

	MTKPwrap              *fPwrap;
	IOWorkLoop             *fWorkLoop;
	IOInterruptEventSource *fInterrupt;
	OSObject               *fTarget[kMapplePMICGroupCount];
	Handler                 fHandler[kMapplePMICGroupCount];
	uint32_t                fInterrupts, fStray;
};

#endif /* _IOKIT_ARM_MT6358PMIC_H */
