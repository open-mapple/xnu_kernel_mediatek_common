/*
 * bsd/dev/mtk_cam_ldo.h against Android's log of the realme 8i (the JN1's rails at the HAL's boot probe) and realme's
 * power sequence for it.
 *   cc -Wall tools/mtk_cam_ldo_test.c -o cam_ldo_test && ./cam_ldo_test
 */
#include <assert.h>
#include <stdio.h>
#include "../bsd/dev/mtk_cam_ldo.h"

int
main(void)
{
	/* "ldo=6,value=1800,Ldo_out:48,regaddr=0x9" etc. (the driver's ldo is LDOn - 1) */
	assert(CAM_LDO_OUT(7) == 0x09 && cam_ldo_mv(7, 48) == 1800);
	assert(CAM_LDO_OUT(2) == 0x04 && cam_ldo_mv(2, 36) == 1050);
	assert(CAM_LDO_OUT(3) == 0x05 && cam_ldo_mv(3, 136) == 2900);
	assert(CAM_LDO_OUT(5) == 0x07 && cam_ldo_mv(5, 136) == 2900);
	assert(CAM_LDO_OUT(1) == 0x03 && cam_ldo_mv(1, 0) == 600 && cam_ldo_mv(4, 128) == 2800);
	assert(cam_ldo_mv(3, 255) == 4387);

	/* enables in Android's order: 0x40, 0x42, 0x46 (rails up), 0x56 (AF too); off 0x46, 0x42, 0x40, 0 */
	assert(!cam_ldo_jn1_on(0x40) && !cam_ldo_jn1_on(0x42) && cam_ldo_jn1_on(0x46) && cam_ldo_jn1_on(0x56));
	assert(!cam_ldo_jn1_on(0x06) && !cam_ldo_jn1_on(0x44) && !cam_ldo_jn1_on(0));

	/* camera_ldo_set_ldo_value's codes; nothing that is not exact */
	assert(cam_ldo_code(7, 1800) == 48 && cam_ldo_code(2, 1050) == 36 && cam_ldo_code(3, 2900) == 136);
	assert(cam_ldo_code(5, 2900) == 136 && cam_ldo_code(5, 2800) == 128 && cam_ldo_code(1, 600) == 0);
	assert(cam_ldo_code(2, 1055) == -1 && cam_ldo_code(3, 1100) == -1 && cam_ldo_code(8, 1800) == -1);
	assert(cam_ldo_code(0, 1800) == -1 && cam_ldo_code(3, 4400) == -1 && cam_ldo_code(3, 4387) == -1);
	for (uint32_t n = 1; n <= 7; n++) {
		for (uint32_t c = 0; c <= 0xff; c++) {
			/* odd codes are half a millivolt off a whole one */
			assert(cam_ldo_code(n, cam_ldo_mv(n, (uint8_t)c)) == (c % 2 ? -1 : (int)c));
		}
	}

	/* the sequence: RST low, LDO7, 2, 3, 5 at Android's voltages, RST high, MCLK; delays 1 1 1 5 0 2 2 */
	{
		static const uint8_t kind[] = { CAM_SEQ_RST, CAM_SEQ_LDO, CAM_SEQ_LDO, CAM_SEQ_LDO, CAM_SEQ_LDO, CAM_SEQ_RST,
			                        CAM_SEQ_MCLK };
		static const uint8_t n[] = { 0, 7, 2, 3, 5, 1, 0 }, ms[] = { 1, 1, 1, 5, 0, 2, 2 };
		static const uint8_t on[] = { 0x40, 0x42, 0x46, 0x56 }, off[] = { 0x46, 0x42, 0x40, 0 };
		uint8_t en = 0;
		uint32_t i, j = 0, total = 0;

		assert(CAM_JN1_SEQ_N == 7);
		for (i = 0; i < CAM_JN1_SEQ_N; i++) {
			assert(cam_jn1_seq[i].kind == kind[i] && cam_jn1_seq[i].n == n[i] && cam_jn1_seq[i].ms == ms[i]);
			total += cam_jn1_seq[i].ms;
			if (cam_jn1_seq[i].kind == CAM_SEQ_LDO) {
				assert(cam_ldo_code(cam_jn1_seq[i].n, cam_jn1_seq[i].mv) >= 0);
				en |= CAM_LDO_BIT(cam_jn1_seq[i].n);
				assert(en == on[j++]);
			}
		}
		assert(j == 4 && en == CAM_LDO_JN1_ALL && total == 12);
		for (i = CAM_JN1_SEQ_N, j = 0; i-- > 0;) {
			if (cam_jn1_seq[i].kind == CAM_SEQ_LDO) {
				en &= ~CAM_LDO_BIT(cam_jn1_seq[i].n);
				assert(en == off[j++]);
			}
		}
		assert(j == 4);
		assert(cam_jn1_seq[1].mv == 1800 && cam_jn1_seq[2].mv == 1050 && cam_jn1_seq[3].mv == 2900 &&
		    cam_jn1_seq[4].mv == 2900);
	}
	printf("mtk_cam_ldo: ok\n");
	return 0;
}
