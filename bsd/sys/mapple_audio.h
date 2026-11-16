/*
 * /dev/mapple-audio: the sound Darwin plays, on its way to whatever can make it heard.
 *
 * Darwin expects its output devices from Apple's audio drivers through IOAudio2; neither exists
 * here. Instead the audio server (mediaserverd) is given a plug-in of ours that presents one
 * stereo output device, and the plug-in writes what the server has mixed for that device
 * here. In the kernel the samples go to a sink -- a driver for something that plays them --
 * or, with none, nowhere (bsd/dev/mapple_audio.c).
 *
 *   write(): whole frames in the format below. Never blocks: the writer is the audio server's
 *            real-time thread, which paces itself (48000 frames a second while anything
 *            plays, nothing at all otherwise). What the sink cannot take is dropped.
 */
#ifndef _SYS_MAPPLE_AUDIO_H_
#define _SYS_MAPPLE_AUDIO_H_

#include <stdint.h>

#define MAPPLE_AUDIO_DEVICE    "/dev/mapple-audio"

/* The one format: 16-bit signed little-endian samples, left then right. */
#define MAPPLE_AUDIO_RATE      48000
#define MAPPLE_AUDIO_CHANNELS  2
#define MAPPLE_AUDIO_FRAME     4       /* bytes */

/*
 * ioctl(MAPPLE_AUDIO_SET_VOLUME): the speakers' volume, as Darwin sets it on the plug-in's volume
 * control -- how many eighths of a dB under the loudest they may play, and whether they are
 * muted (Darwin's volume at 0). The loudest is the kernel's to say (on the phone: Android's own
 * level less mp_audio_below, bsd/dev/mtk_audio.c); the plug-in only says how far under it.
 */
#include <sys/ioccom.h>

struct mapple_audio_volume {
	uint32_t below;                 /* eighths of a dB */
	uint32_t muted;
};
#define MAPPLE_AUDIO_SET_VOLUME _IOW('r', 1, struct mapple_audio_volume)

/* ioctl(MAPPLE_AUDIO_SET_ROUTE): where what is written plays, the speaker or the earpiece (the port Darwin has chosen). */
#define MAPPLE_AUDIO_ROUTE_SPEAKER      0
#define MAPPLE_AUDIO_ROUTE_RECEIVER     1
#define MAPPLE_AUDIO_SET_ROUTE  _IOW('r', 2, uint32_t)

#ifdef KERNEL
#include <stdbool.h>

#define MAPPLE_AUDIO_BELOW_MAX  (48 * 8)        /* the plug-in's whole range: what is asked for beyond it is taken as this */

/*
 * For drivers. A sink is offered frames as they are written and says how many it took (the
 * rest are lost); it is called from the writer's thread and must not wait.
 */
typedef uint32_t (*mapple_audio_sink_t)(const int16_t *frames, uint32_t count);

void mapple_audio_set_sink(const char *name, mapple_audio_sink_t sink);
/* The volume Darwin has set last (MAPPLE_AUDIO_SET_VOLUME): false if it has set none; *generation counts the settings. */
bool mapple_audio_volume(uint32_t *below, bool *muted, uint32_t *generation);
uint32_t mapple_audio_route(void);     /* MAPPLE_AUDIO_SET_ROUTE's last, MAPPLE_AUDIO_ROUTE_SPEAKER until one */
/* /dev/mapple-mic: a source gives up to `count' samples (48 kHz, one channel) and says how many; it must not wait. */
typedef uint32_t (*mapple_audio_source_t)(int16_t *samples, uint32_t count);
void mapple_audio_set_source(const char *name, mapple_audio_source_t source);
bool mapple_audio_mic_open(void);       /* whether /dev/mapple-mic is open: the mic is wanted */
/* /dev/mapple-haptic: each write's loudest sample and its length in frames (48 kHz, one channel); same rules as a sink */
typedef void (*mapple_haptic_sink_t)(uint32_t peak, uint32_t frames);
void mapple_audio_set_haptic(mapple_haptic_sink_t sink);
void mapple_audio_init(void);          /* the /dev nodes: bsd_init, once devfs is there */

/*
 * The speakers' guard (bsd/dev/speaker_guard.c): what the samples go through in the feeder on their
 * way to the amplifier, for want of a speaker protection. Levels in eighths of a dB.
 */
#define SPEAKER_GUARD_SILENT    0xffffffffu     /* the amplifiers are off: nothing reaches the speakers */
struct speaker_guard_report {
	uint32_t blocks, rms_blocks, fast_blocks, slow_blocks;  /* blocks done, and in how many each average held the level down */
	uint32_t limited_frames;        /* frames in which the peak limiter turned the level down */
	uint32_t peak_in, peak_out;     /* the largest sample that came in and that went out */
	uint32_t least_peak_gain;       /* the peak limiter's least gain, in 1/65536 */
	uint64_t least_gain, gain;      /* the power guard's (with the monitor's) least gain and the latest, Q30 */
	uint64_t rms, fast, slow;       /* the averages now, the louder speaker's: Q30 of a full-scale square wave at Android's level */
};
#define SPEAKER_GUARD_MAX_GAIN  (12 * 8)        /* the most gain before the guard: 12 dB */
void speaker_guard_configure(bool hpf, uint32_t gain);         /* the high-pass filter; `gain' before everything */
void speaker_guard_attenuation(uint32_t below);                 /* the amplifiers' level under Android's (or SPEAKER_GUARD_SILENT):
                                                                 * what the limits are reckoned from */
void speaker_guard_monitor(uint32_t below);                     /* a further reduction (an amplifier's own watch, if it has one) */
void speaker_guard_process(int16_t *frames, uint32_t count);    /* stereo frames, in place */
void speaker_guard_take_report(struct speaker_guard_report *report);   /* what happened since the last call */
int speaker_guard_decibels(uint64_t ratio);                     /* 10 log10 of a Q30 ratio, in tenths of a dB */
uint64_t speaker_guard_amplitude(uint32_t below);               /* below eighths of a dB, as a Q30 factor of amplitude */
#endif

#endif /* _SYS_MAPPLE_AUDIO_H_ */
