/*
 * bsd/dev/mtk_audio_level.h and the kernel's clamp of Darwin's volume, on this machine: the line-out never louder than
 * Android's 0 dB, the split always adding up, the total bounded. Run by tools/speaker-guard-test.sh.
 */
#include <stdio.h>
#include "../bsd/dev/mtk_audio_level.h"

#define AUDIO_BELOW_MAX (48 * 8)        /* sys/mapple_audio.h MAPPLE_AUDIO_BELOW_MAX */
#define BELOW_MAX       (90 * 8)        /* mtk_audio.c: mp_audio_below's ceiling */

int
main(void)
{
	int failures = 0;

	for (uint32_t base = 0; base <= BELOW_MAX; base++) {
		for (uint32_t darwin = 0; darwin <= 0xffffu; darwin += darwin < 1024 ? 1 : 997) {
			uint32_t d = darwin > AUDIO_BELOW_MAX ? AUDIO_BELOW_MAX : darwin, below = base + d;
			uint32_t code = mtk_audio_lol_code(below), digital = mtk_audio_digital(below);

			if (code < LOL_ANDROID || code > LOL_QUIETEST || (code - LOL_ANDROID) * 8 + digital != below ||
			    (code < LOL_QUIETEST && digital >= 8) || below > BELOW_MAX + AUDIO_BELOW_MAX) {
				if (failures++ < 5) {
					printf("FAILED: below %u: code %u, digital %u\n", below, code, digital);
				}
			}
		}
	}
	if (mtk_audio_lol_code(0) != 8 || mtk_audio_lol_code(160) != 18 || mtk_audio_digital(160) != 80 ||
	    mtk_audio_lol_code(23) != 10 || mtk_audio_digital(23) != 7) {
		printf("FAILED: the examples\n");
		failures++;
	}
	printf("%s\n", failures ? "mtk_audio_level_test: FAILED" : "mtk_audio_level_test: ok");
	return failures != 0;
}
