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
#define CAM_LDO_BIT(n)          (1u << ((n) - 1))
#define CAM_LDO_JN1_RAILS       (CAM_LDO_BIT(7) | CAM_LDO_BIT(2) | CAM_LDO_BIT(3))      /* DOVDD, DVDD, AVDD */
#define CAM_LDO_JN1_ALL         (CAM_LDO_JN1_RAILS | CAM_LDO_BIT(5))

static inline uint32_t
cam_ldo_mv(uint32_t n, uint8_t out)
{
	return (n <= 2 ? 600u : 1200u) + out * 25u / 2;
}

/* The output code for mv, as camera_ldo_set_ldo_value computes it; -1 unless it is exact. */
static inline int
cam_ldo_code(uint32_t n, uint32_t mv)
{
	uint32_t base = n <= 2 ? 600u : 1200u;

	if (n < 1 || n > 7 || mv < base || (mv - base) * 2 % 25 != 0 || (mv - base) * 2 / 25 > 0xff) {
		return -1;
	}
	return (int)((mv - base) * 2 / 25);
}

static inline bool
cam_ldo_jn1_on(uint8_t en)
{
	return (en & CAM_LDO_JN1_RAILS) == CAM_LDO_JN1_RAILS;
}

/*
 * The JN1's power-up, realme's sensor_power_sequence_21690 (imgsensor_hwcfg_mt6781.c), each step followed by its
 * delay; off in reverse, each with the same delay (imgsensor_hw_power_sequence): RST low, MCLK off, LDOs disabled.
 */
enum { CAM_SEQ_RST, CAM_SEQ_LDO, CAM_SEQ_MCLK };

static const struct cam_seq {
	uint8_t kind;
	uint8_t n;              /* LDOn, or RST's level */
	uint16_t mv;
	uint8_t ms;
} cam_jn1_seq[] = {
	{ CAM_SEQ_RST, 0, 0, 1 },
	{ CAM_SEQ_LDO, 7, 1800, 1 },
	{ CAM_SEQ_LDO, 2, 1050, 1 },
	{ CAM_SEQ_LDO, 3, 2900, 5 },
	{ CAM_SEQ_LDO, 5, 2900, 0 },
	{ CAM_SEQ_RST, 1, 0, 2 },
	{ CAM_SEQ_MCLK, 0, 0, 2 },
};

#define CAM_JN1_SEQ_N           (sizeof(cam_jn1_seq) / sizeof(cam_jn1_seq[0]))
#endif
