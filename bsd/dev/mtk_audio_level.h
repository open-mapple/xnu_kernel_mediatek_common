/*
 * The speaker's level on the MT6366: `below' eighths of a dB under Android's (the line-out at 0 dB, its code 8), split
 * between the line-out gain (1 dB a step down to -10 dB, code 18; lower codes are louder and never used) and the rest,
 * applied to the samples before the guard. tools/mtk_audio_level_test.c checks it.
 */
#ifndef _MTK_AUDIO_LEVEL_H_
#define _MTK_AUDIO_LEVEL_H_

#include <stdint.h>

#define LOL_ANDROID     8       /* 0 dB: the codec driver's default, the loudest we set */
#define LOL_QUIETEST    18      /* -10 dB */
#define LOL_MUTE        0x1f    /* -40 dB */

static inline uint32_t
mtk_audio_lol_code(uint32_t below)
{
	uint32_t db = below / 8;

	return LOL_ANDROID + (db > LOL_QUIETEST - LOL_ANDROID ? LOL_QUIETEST - LOL_ANDROID : db);
}

/* what is left for the samples, eighths of a dB */
static inline uint32_t
mtk_audio_digital(uint32_t below)
{
	return below - (mtk_audio_lol_code(below) - LOL_ANDROID) * 8;
}

#endif
