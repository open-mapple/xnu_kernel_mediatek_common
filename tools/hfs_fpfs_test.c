/*
 * bsd/hfs_staged/core/hfs_fpfs.h on this machine: the fsctl numbers as iOS 16.7.16's FileProvider passes them, and the
 * nearest sync root walk over a small tree.
 *   cc -Wall tools/hfs_fpfs_test.c -o hfs_fpfs_test && ./hfs_fpfs_test
 */
#include <assert.h>
#include <stdio.h>
#include "../bsd/hfs_staged/core/hfs_fpfs.h"

/* parent of each file ID (1 is above the root folder 2); 9 points at itself, a damaged catalog */
static const uint32_t up[] = { 0, 0, 1, 2, 3, 4, 5, 3, 7, 9 };
static int mark[10];

static int
marked(void *arg, uint32_t id, int *is)
{
	(void)arg;
	if (id >= 10) {
		return EIO;
	}
	*is = mark[id];
	return 0;
}

static int
parent(void *arg, uint32_t id, uint32_t *p)
{
	(void)arg;
	if (id >= 10) {
		return ENOENT;
	}
	*p = up[id];
	return 0;
}

static uint64_t
nearest(uint32_t item, int expect)
{
	uint64_t found = 99;
	assert(hfs_fpfs_nearest_sync_root(up[item], NULL, marked, parent, &found) == expect);
	return found;
}

int
main(void)
{
	assert(FPFS_SET_SYNC_ROOT == 0x80044a48);
	assert(FPFS_GET_SYNC_ROOT == 0x40044a48);
	assert(FPFS_NEAREST_SYNC_ROOT == 0x40084a4a);
	assert(APFSIOC_MAINTAIN_DIR_STATS == 0x80084a02);
	assert(APFSIOC_DIR_STATS_OP == 0xc0284a20);
	assert(FPFS_MARK_PURGEABLE == 0xc0084a44);
	assert(FPFS_GET_PURGEABLE_FLAGS == 0x40084a47);

	/* nothing marked: 0 up to the root folder, and for the root folder itself */
	assert(nearest(6, 0) == 0);
	assert(nearest(2, 0) == 0);
	/* 4 marked: its descendants see 4, 4 itself and its parent do not */
	mark[4] = 1;
	assert(nearest(6, 0) == 4);
	assert(nearest(5, 0) == 4);
	assert(nearest(4, 0) == 0);
	assert(nearest(8, 0) == 0);
	/* the nearest wins over a marked folder further up */
	mark[3] = 1;
	assert(nearest(6, 0) == 4);
	assert(nearest(8, 0) == 3);
	assert(nearest(4, 0) == 3);
	/* the root folder can be the one */
	mark[3] = mark[4] = 0;
	mark[2] = 1;
	assert(nearest(6, 0) == 2);
	assert(nearest(3, 0) == 2);
	mark[2] = 0;
	/* a lookup error stops the walk; a cycle ends in ELOOP */
	uint64_t found;
	assert(hfs_fpfs_nearest_sync_root(12, NULL, marked, parent, &found) == EIO && found == 0);
	assert(hfs_fpfs_nearest_sync_root(9, NULL, marked, parent, &found) == ELOOP && found == 0);
	printf("hfs_fpfs_test: ok\n");
	return 0;
}
