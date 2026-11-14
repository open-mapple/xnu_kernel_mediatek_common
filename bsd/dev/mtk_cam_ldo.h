/*
 * The realme 8i's camera LDO chip (WL2864C type) on I2C2 at 0x29, for bsd/dev/mtk_cam.c and tools/mtk_cam_ldo_test.c.
 * Facts from realme's camera_ldo.c/.h and Android's log of this phone: LDO1-7 outputs at 0x03-0x09, LDO1-2 0.6 V
 * and LDO3-7 1.2 V plus 12.5 mV a step, enables at 0x0E (bit n-1 for LDOn). The S5K JN1 runs from LDO7 (DOVDD
 * 1.8 V), LDO2 (DVDD 1.05 V), LDO3 (AVDD 2.9 V) and LDO5 (AF 2.9 V).
 */
#ifndef MTK_CAM_LDO_H
#define MTK_CAM_LDO_H
#include <stdbool.h>
#include <stdint.h>

#define CAM_LDO_BUS             2
#define CAM_LDO_ADDR            0x29
#define CAM_LDO_REGS            16
#define CAM_LDO_OUT(n)          (0x02 + (n))
#define CAM_LDO_EN              0x0e
#define CAM_LDO_JN1_RAILS       ((1u << 6) | (1u << 1) | (1u << 2))     /* DOVDD, DVDD, AVDD */

static inline uint32_t
cam_ldo_mv(uint32_t n, uint8_t out)
{
	return (n <= 2 ? 600u : 1200u) + out * 25u / 2;
}

static inline bool
cam_ldo_jn1_on(uint8_t en)
{
	return (en & CAM_LDO_JN1_RAILS) == CAM_LDO_JN1_RAILS;
}
#endif
