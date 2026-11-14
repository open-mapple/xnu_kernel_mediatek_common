/*
 * bsd/dev/mapple_safeline.h: a safe boot's line back to the payload's, as mtkloader's safe_boot_args changes it.
 *   cc -Wall tools/mapple_safeline_test.c -o safeline_test && ./safeline_test
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../bsd/dev/mapple_safeline.h"

/* mtkloader: safe_boot_args, then the thermal argument */
static void
safe(const char *payload, const char *after, char *out)
{
	char line[MAPPLE_SAFELINE_MAX];
	snprintf(line, sizeof(line), "%s" MAPPLE_SAFELINE_ADDED, payload);
	for (char *p = line; *p; p++) {
		if (p != line && p[-1] != ' ') {
			continue;
		}
		if (!strncmp(p, "mp_gpu=2", 8) && (p[8] == ' ' || !p[8])) {
			p[7] = '1';
		}
		if (!strncmp(p, "mp_wifi_steps=", 14) && p[14] >= '0' && p[14] <= '9') {
			p[14] = '0';
		}
		if (!strncmp(p, "mp_bt_steps=", 12) && p[12] >= '0' && p[12] <= '9') {
			p[12] = '0';
		}
	}
	snprintf(out, MAPPLE_SAFELINE_MAX, "%s%s", line, after);
}

static void
check(const char *payload)
{
	const char *after = " mp_lvts_golden=1234 mp_lvts_r=r1.2.3";
	char normal[MAPPLE_SAFELINE_MAX], running[MAPPLE_SAFELINE_MAX], out[MAPPLE_SAFELINE_MAX];
	snprintf(normal, sizeof(normal), "%s%s", payload, after);
	safe(payload, after, running);
	assert(mapple_safeline_payload(running, mapple_safeline_hash(normal), out, sizeof(out)));
	assert(strcmp(out, normal) == 0);
	assert(!mapple_safeline_payload(running, mapple_safeline_hash("another payload"), out, sizeof(out)));
}

int
main(void)
{
	char out[MAPPLE_SAFELINE_MAX];
	check("rootrw=1 mp_gpu=2 mp_wifi_steps=5 mp_bt_steps=2 mp_ufsrw=1");
	check("rootrw=1 mp_gpu=1 mp_wifi_steps=0 mp_mem4g=1");
	check("rootrw=1 mp_bt_steps=9");
	check("rootrw=1");
	check("mp_gpu=2 mp_gpu=2 mp_wifi_steps=7 xmp_bt_steps=3");
	assert(!mapple_safeline_payload("rootrw=1 mp_gpu=1", mapple_safeline_hash("rootrw=1 mp_gpu=2"), out, sizeof(out)));
	printf("safeline_test: ok\n");
	return 0;
}
