/*
 * A safe boot's command line is the payload's after mtkloader's safe_boot_args: " mp_bootrec=0 mp_mem4g=0" added, the
 * single digit of mp_gpu=2 made 1 and those of mp_wifi_steps and mp_bt_steps made 0, in place (the thermal and DVFS
 * arguments come after it). mapple_safeline_payload() undoes that: of the at most 200 lines it could have been, the
 * payload's is the one whose FNV-1a 64 is base, the boot-args record's base_hash. With no match the record belongs
 * to another payload. tools/mapple_safeline_test.c checks it on the host.
 */
#ifndef _BSD_DEV_MAPPLE_SAFELINE_H
#define _BSD_DEV_MAPPLE_SAFELINE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MAPPLE_SAFELINE_MAX     608
#define MAPPLE_SAFELINE_ADDED   " mp_bootrec=0 mp_mem4g=0"

static inline uint64_t
mapple_safeline_hash(const char *s)
{
	uint64_t h = 0xcbf29ce484222325ULL;
	for (unsigned i = 0; i < MAPPLE_SAFELINE_MAX && s[i]; i++) {
		h ^= (uint8_t)s[i];
		h *= 0x100000001b3ULL;
	}
	return h;
}

/* The single digits of key's values: at a word's start, one digit, then a space or the end. At most max. */
static inline int
mapple_safeline_digits(char *line, const char *key, char **at, int max)
{
	size_t k = 0;
	int n = 0;
	while (key[k]) {
		k++;
	}
	for (char *p = line; *p && n < max; p++) {
		size_t i = 0;
		if (p != line && p[-1] != ' ') {
			continue;
		}
		while (i < k && p[i] == key[i]) {
			i++;
		}
		if (i == k && p[k] >= '0' && p[k] <= '9' && (p[k + 1] == ' ' || p[k + 1] == 0)) {
			at[n++] = p + k;
		}
	}
	return n;
}

static inline bool
mapple_safeline_payload(const char *running, uint64_t base, char *out, size_t len)
{
	char line[MAPPLE_SAFELINE_MAX], *gpu[4], *wifi[4], *bt[4], *added = NULL;
	const char *a = MAPPLE_SAFELINE_ADDED;
	size_t n = 0, alen = sizeof(MAPPLE_SAFELINE_ADDED) - 1;

	while (n < sizeof(line) - 1 && running[n]) {
		line[n] = running[n];
		n++;
	}
	line[n] = 0;
	for (char *p = line; *p && !added; p++) {
		size_t i = 0;
		while (i < alen && p[i] == a[i]) {
			i++;
		}
		if (i == alen) {
			added = p;
		}
	}
	if (!added) {
		return false;
	}
	for (char *p = added;; p++) {
		p[0] = p[alen];
		if (!p[0]) {
			break;
		}
	}
	int ng = mapple_safeline_digits(line, "mp_gpu=", gpu, 4);
	int nw = mapple_safeline_digits(line, "mp_wifi_steps=", wifi, 4);
	int nb = mapple_safeline_digits(line, "mp_bt_steps=", bt, 4);
	bool gpu_down = ng > 0;
	for (int i = 0; i < ng; i++) {
		gpu_down = gpu_down && *gpu[i] == '1';
	}
	for (int g = 0; g < (gpu_down ? 2 : 1); g++) {
		for (int w = 0; w < (nw ? 10 : 1); w++) {
			for (int b = 0; b < (nb ? 10 : 1); b++) {
				for (int i = 0; gpu_down && i < ng; i++) {
					*gpu[i] = g ? '2' : '1';
				}
				for (int i = 0; i < nw; i++) {
					*wifi[i] = (char)('0' + w);
				}
				for (int i = 0; i < nb; i++) {
					*bt[i] = (char)('0' + b);
				}
				if (mapple_safeline_hash(line) == base) {
					size_t i = 0;
					for (; i + 1 < len && line[i]; i++) {
						out[i] = line[i];
					}
					out[i] = 0;
					return true;
				}
			}
		}
	}
	return false;
}

#endif /* _BSD_DEV_MAPPLE_SAFELINE_H */
