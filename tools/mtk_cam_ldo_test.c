/*
 * bsd/dev/mtk_cam_ldo.h against Android's log of the realme 8i (the JN1's rails at the HAL's boot probe).
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
	printf("mtk_cam_ldo: ok\n");
	return 0;
}
