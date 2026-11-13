/*
 * /dev/mapple-audio, /dev/mapple-mic and /dev/mapple-haptic: the kernel's end of the audio plug-in (mapple_userland,
 * audio/audio_plugin.c). With no backend they drop what is written and give silence, so the plug-in, and through it
 * iOS's speaker route, its volume and its haptics, run without touching any hardware. A backend takes the samples
 * with mapple_audio_set_sink() and the haptics with mapple_audio_set_haptic() (bsd/dev/mtk_audio.c).
 *
 * The nodes are mediaserverd's (user mobile), mode 0600, and each is open once at a time (EBUSY).
 */
#include <kern/locks.h>
#include <libkern/libkern.h>
#include <miscfs/devfs/devfs.h>
#include <sys/conf.h>
#include <sys/errno.h>
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
static uint32_t volume_below, volume_generation;
static bool volume_muted, volume_set;
static uint64_t frames_in, frames_taken, haptic_writes, mic_frames;
static unsigned haptic_said;

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
		mic_frames = n == NODE_MIC ? 0 : mic_frames;
	}
	lck_mtx_unlock(&audio_lock);
	printf("audio: /dev/%s %s\n", node_names[n], error ? "is already open (EBUSY)" : "opened");
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
		printf("audio: /dev/mapple-mic closed: %llu frames of silence given\n", mic_frames);
	} else {
		printf("audio: /dev/mapple-haptic closed: %llu writes\n", haptic_writes);
	}
	return 0;
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
		frames_in += count;
		frames_taken += sink != NULL ? sink(buffer, count) : 0;
		lck_mtx_unlock(&audio_lock);
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

/* The microphone: silence, as much as is asked for. */
static int
audio_read(dev_t dev, struct uio *uio, __unused int ioflag)
{
	static const int16_t zeros[512];

	if (minor(dev) != NODE_MIC) {
		return ENODEV;
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

static int
audio_ioctl(dev_t dev, u_long cmd, caddr_t data, __unused int flag, __unused struct proc *p)
{
	struct mapple_audio_volume v;
	uint32_t below;
	bool muted, changed;

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
