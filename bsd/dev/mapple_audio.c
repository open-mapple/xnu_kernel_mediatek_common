/*
 * /dev/mapple-audio, /dev/mapple-mic and /dev/mapple-haptic: the kernel's end of the audio plug-in (mapple_userland,
 * audio/audio_plugin.c). With no backend they drop what is written and give silence, so the plug-in, and through it
 * iOS's speaker route, its volume and its haptics, run without touching any hardware. A backend takes the samples
 * with mapple_audio_set_sink() and the haptics with mapple_audio_set_haptic(), and gives the mic's with
 * mapple_audio_set_source() (bsd/dev/mtk_audio.c).
 *
 * The nodes are mediaserverd's (user mobile), mode 0600, and each is open once at a time (EBUSY). Said: who opens a
 * node, the speaker's first frames and then, while there is sound, its loudest sample once a second (the first 30).
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <libkern/libkern.h>
#include <mach/mach_time.h>
#include <miscfs/devfs/devfs.h>
#include <sys/conf.h>
#include <sys/errno.h>
#include <sys/kauth.h>
#include <sys/proc.h>
#include <sys/systm.h>
#include <sys/uio.h>
#include <sys/mapple_audio.h>

#define MOBILE_ID       501

enum { NODE_AUDIO, NODE_MIC, NODE_HAPTIC, NODES };
static const char *const node_names[NODES] = { "mapple-audio", "mapple-mic", "mapple-haptic" };

static LCK_GRP_DECLARE(audio_lck_grp, "mapple-audio");
static LCK_MTX_DECLARE(audio_lock, &audio_lck_grp);
static bool opened[NODES];
static mapple_audio_sink_t sink;
static const char *sink_name = "nothing";
static mapple_haptic_sink_t haptic;
static mapple_audio_source_t source;
static const char *source_name = "silence";
static uint32_t volume_below, volume_generation;
static bool volume_muted, volume_set;
static uint32_t route;                  /* MAPPLE_AUDIO_ROUTE_*: the plug-in's real-time thread sets it, so no lock */
static uint64_t frames_in, frames_taken, haptic_writes, mic_frames, mic_heard;
static unsigned haptic_said, sound_said;
static uint32_t second_peak;
static uint64_t second_start;

void
mapple_audio_set_sink(const char *name, mapple_audio_sink_t s)
{
	lck_mtx_lock(&audio_lock);
	sink = s;
	sink_name = s != NULL ? name : "nothing";
	lck_mtx_unlock(&audio_lock);
	printf("audio: the sound goes to %s\n", s != NULL ? name : "nothing");
}

void
mapple_audio_set_source(const char *name, mapple_audio_source_t s)
{
	lck_mtx_lock(&audio_lock);
	source = s;
	source_name = s != NULL ? name : "silence";
	lck_mtx_unlock(&audio_lock);
	printf("audio: the mic's samples come from %s\n", s != NULL ? name : "nothing (silence)");
}

bool
mapple_audio_mic_open(void)
{
	return __atomic_load_n(&opened[NODE_MIC], __ATOMIC_RELAXED);
}

void
mapple_audio_set_haptic(mapple_haptic_sink_t s)
{
	lck_mtx_lock(&audio_lock);
	haptic = s;
	lck_mtx_unlock(&audio_lock);
}

bool
mapple_audio_volume(uint32_t *below, bool *muted, uint32_t *generation)
{
	bool set;

	lck_mtx_lock(&audio_lock);
	*below = volume_below;
	*muted = volume_muted;
	*generation = volume_generation;
	set = volume_set;
	lck_mtx_unlock(&audio_lock);
	return set;
}

static int
audio_open(dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	unsigned n = (unsigned)minor(dev);
	int error = 0;

	if (n >= NODES) {
		return ENXIO;
	}
	lck_mtx_lock(&audio_lock);
	if (opened[n]) {
		error = EBUSY;
	} else {
		opened[n] = true;
		frames_in = n == NODE_AUDIO ? 0 : frames_in;
		frames_taken = n == NODE_AUDIO ? 0 : frames_taken;
		sound_said = n == NODE_AUDIO ? 0 : sound_said;
		mic_frames = n == NODE_MIC ? 0 : mic_frames;
		mic_heard = n == NODE_MIC ? 0 : mic_heard;
	}
	lck_mtx_unlock(&audio_lock);
	{
		char name[32];

		proc_selfname(name, sizeof(name));
		printf("audio: /dev/%s %s by %s (pid %d, uid %u)\n", node_names[n], error ? "is already open (EBUSY): refused" : "opened",
		    name, proc_selfpid(), kauth_getuid());
	}
	return error;
}

static int
audio_close(dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	unsigned n = (unsigned)minor(dev);

	if (n >= NODES) {
		return ENXIO;
	}
	lck_mtx_lock(&audio_lock);
	opened[n] = false;
	lck_mtx_unlock(&audio_lock);
	if (n == NODE_AUDIO) {
		printf("audio: /dev/mapple-audio closed: %llu frames written, %llu taken by %s\n", frames_in, frames_taken, sink_name);
	} else if (n == NODE_MIC) {
		printf("audio: /dev/mapple-mic closed: %llu frames given, %llu of them from %s\n", mic_frames, mic_heard, source_name);
	} else {
		printf("audio: /dev/mapple-haptic closed: %llu writes\n", haptic_writes);
	}
	return 0;
}

/* While there is sound, its loudest sample once a second (Darwin's volume shows in it: the speaker is in software volume). */
static void
say_level(const int16_t *frames, uint32_t count)
{
	uint64_t now = mach_absolute_time(), second;

	for (uint32_t i = 0; i < 2 * count; i++) {
		uint32_t v = frames[i] < 0 ? (uint32_t)-frames[i] : (uint32_t)frames[i];

		second_peak = v > second_peak ? v : second_peak;
	}
	nanoseconds_to_absolutetime(NSEC_PER_SEC, &second);
	if (now - second_start >= second) {
		if (second_peak > 0 && sound_said < 30) {
			sound_said++;
			printf("audio: the speaker: peak %u of 32768 in the last second\n", second_peak);
		}
		second_peak = 0;
		second_start = now;
	}
}

/* The speaker's samples (whole frames) to the sink; what it does not take is dropped. Never waits. */
static int
audio_write_samples(struct uio *uio)
{
	int16_t buffer[1024];

	while (uio_resid(uio) >= MAPPLE_AUDIO_FRAME) {
		user_ssize_t want = uio_resid(uio) - uio_resid(uio) % MAPPLE_AUDIO_FRAME;
		uint32_t bytes = (uint32_t)(want < (user_ssize_t)sizeof(buffer) ? want : sizeof(buffer)), count = bytes / MAPPLE_AUDIO_FRAME;
		int error = uiomove((caddr_t)buffer, (int)bytes, uio);

		if (error) {
			return error;
		}
		lck_mtx_lock(&audio_lock);
		if (frames_in == 0) {
			printf("audio: the speaker's first frames (%u)\n", count);
		}
		frames_in += count;
		frames_taken += sink != NULL ? sink(buffer, count) : 0;
		lck_mtx_unlock(&audio_lock);
		say_level(buffer, count);
	}
	uio_update(uio, uio_resid(uio));        /* a tail of less than a frame is dropped */
	return 0;
}

/* The haptics: each write's loudest sample and length, to the motor's driver; the first ones said. */
static int
audio_write_haptic(struct uio *uio)
{
	int16_t buffer[1024];
	uint32_t peak = 0, frames = 0;

	while (uio_resid(uio) >= 2) {
		user_ssize_t want = uio_resid(uio) & ~(user_ssize_t)1;
		uint32_t bytes = (uint32_t)(want < (user_ssize_t)sizeof(buffer) ? want : sizeof(buffer));
		int error = uiomove((caddr_t)buffer, (int)bytes, uio);

		if (error) {
			return error;
		}
		for (uint32_t i = 0; i < bytes / 2; i++) {
			uint32_t v = buffer[i] < 0 ? (uint32_t)-buffer[i] : (uint32_t)buffer[i];

			peak = v > peak ? v : peak;
		}
		frames += bytes / 2;
	}
	uio_update(uio, uio_resid(uio));
	lck_mtx_lock(&audio_lock);
	haptic_writes++;
	if (haptic != NULL) {
		haptic(peak, frames);
	}
	lck_mtx_unlock(&audio_lock);
	if (peak > 0 && haptic_said < 12) {
		haptic_said++;
		printf("audio: haptic: %u frames, peak %u of 32768%s\n", frames, peak, haptic != NULL ? "" : " (no motor driver: not felt)");
	}
	return 0;
}

static int
audio_write(dev_t dev, struct uio *uio, __unused int ioflag)
{
	switch (minor(dev)) {
	case NODE_AUDIO:
		return audio_write_samples(uio);
	case NODE_HAPTIC:
		return audio_write_haptic(uio);
	default:
		return ENODEV;
	}
}

/* The microphone: what its source has (never waiting for more), or with none, silence, as much as is asked for. */
static int
audio_read(dev_t dev, struct uio *uio, __unused int ioflag)
{
	static const int16_t zeros[512];
	int16_t samples[256];
	mapple_audio_source_t s;

	if (minor(dev) != NODE_MIC) {
		return ENODEV;
	}
	lck_mtx_lock(&audio_lock);
	s = source;
	lck_mtx_unlock(&audio_lock);
	while (s != NULL && uio_resid(uio) >= 2) {
		user_ssize_t want = uio_resid(uio) / 2;
		uint32_t n = s(samples, (uint32_t)(want < 256 ? want : 256));
		int error;

		if (n == 0) {
			return 0;
		}
		if ((error = uiomove((caddr_t)samples, (int)n * 2, uio)) != 0) {
			return error;
		}
		mic_frames += n;
		mic_heard += n;
	}
	while (uio_resid(uio) >= 2) {
		user_ssize_t want = uio_resid(uio) & ~(user_ssize_t)1;
		int bytes = (int)(want < (user_ssize_t)sizeof(zeros) ? want : sizeof(zeros)), error;

		if ((error = uiomove((caddr_t)(uintptr_t)zeros, bytes, uio)) != 0) {
			return error;
		}
		mic_frames += (uint64_t)bytes / 2;
	}
	return 0;
}

uint32_t
mapple_audio_route(void)
{
	return __atomic_load_n(&route, __ATOMIC_RELAXED);
}

static int
audio_ioctl(dev_t dev, u_long cmd, caddr_t data, __unused int flag, __unused struct proc *p)
{
	struct mapple_audio_volume v;
	uint32_t below, to;
	bool muted, changed;

	if (minor(dev) == NODE_AUDIO && cmd == MAPPLE_AUDIO_SET_ROUTE) {
		bcopy(data, &to, sizeof(to));
		if (to != MAPPLE_AUDIO_ROUTE_SPEAKER && to != MAPPLE_AUDIO_ROUTE_RECEIVER) {
			return EINVAL;
		}
		__atomic_store_n(&route, to, __ATOMIC_RELAXED);
		return 0;
	}
	if (minor(dev) != NODE_AUDIO || cmd != MAPPLE_AUDIO_SET_VOLUME) {
		return ENOTTY;
	}
	bcopy(data, &v, sizeof(v));
	below = v.below > MAPPLE_AUDIO_BELOW_MAX ? MAPPLE_AUDIO_BELOW_MAX : v.below;
	muted = v.muted != 0;
	lck_mtx_lock(&audio_lock);
	changed = !volume_set || below != volume_below || muted != volume_muted;
	volume_below = below;
	volume_muted = muted;
	volume_set = true;
	volume_generation++;
	lck_mtx_unlock(&audio_lock);
	if (changed) {
		printf("audio: volume %s%u.%u dB under the loudest%s\n", muted ? "muted, " : "", below / 8, below % 8 * 10 / 8,
		    v.below > MAPPLE_AUDIO_BELOW_MAX ? " (asked for more)" : "");
	}
	return 0;
}

static int
audio_select(__unused dev_t dev, __unused int which, __unused void *wql, __unused struct proc *p)
{
	return 1;
}

static const struct cdevsw audio_cdevsw = {
	.d_open = audio_open, .d_close = audio_close, .d_read = audio_read, .d_write = audio_write,
	.d_ioctl = audio_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_select = audio_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0
};

void
mapple_audio_init(void)
{
	int major = cdevsw_add(-1, &audio_cdevsw);
	bool made = major >= 0;

	for (unsigned i = 0; made && i < NODES; i++) {
		made = devfs_make_node(makedev(major, i), DEVFS_CHAR, MOBILE_ID, MOBILE_ID, 0600, "%s", node_names[i]) != NULL;
	}
	printf("audio: /dev/mapple-audio, -mic, -haptic %s\n", made ? "made" : "NOT MADE");
}
