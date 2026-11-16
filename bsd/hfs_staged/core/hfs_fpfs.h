/*
 * The APFS fsctls FileProvider's local storage (FileProvider.framework's fpfs_* calls, LocalStorageFileProvider)
 * issues on its volume, answered on HFS+. Numbers as iOS 16.7.16's FileProvider passes them to ffsctl/fsctl; the
 * names are ours where Apple's are not public. Each answer is behind a bit of the mp_fsfp boot-arg (default 0: ENOTTY,
 * as before). No kernel headers here: tools/hfs_fpfs_test.c checks it on the host.
 */
#ifndef _HFS_FPFS_H_
#define _HFS_FPFS_H_

#include <stdint.h>
#include <sys/errno.h>
#include <sys/ioccom.h>

#define FPFS_SET_SYNC_ROOT          _IOW('J', 72, uint32_t)     /* nonzero marks the item a sync root, 0 clears it */
#define FPFS_GET_SYNC_ROOT          _IOR('J', 72, uint32_t)
#define FPFS_NEAREST_SYNC_ROOT      _IOR('J', 74, uint64_t)     /* file ID of the nearest marked ancestor, or 0 */
#define APFSIOC_MAINTAIN_DIR_STATS  _IOW('J', 2, uint64_t)
#define FPFS_MARK_PURGEABLE         _IOWR('J', 68, uint64_t)    /* 0 unmarks */
#define FPFS_GET_PURGEABLE_FLAGS    _IOR('J', 71, uint64_t)

#define MP_FSFP_SYNC_ROOT   0x1     /* 'J' 72 and 74, the mark kept in HFS_SYNC_ROOT_XATTR */
#define MP_FSFP_DIR_STATS   0x2     /* 'J' 2 answers 0 for a folder, EINVAL for anything else */
#define MP_FSFP_PURGEABLE   0x4     /* 'J' 71 answers "not purgeable", 'J' 68 unmarking answers 0 */
#define MP_FSFP_RENAME_SWAP 0x8     /* RENAME_SWAP as three renames (hfs_vnop_renamex) */

/* Hidden from listxattr and refused to setxattr/removexattr by the kernel's com.apple.system. rule. */
#define HFS_SYNC_ROOT_XATTR "com.apple.system.mapple.syncroot"

#ifdef KERNEL
extern uint32_t hfs_mp_fsfp;
#endif

#define HFS_FPFS_ROOT_FOLDER_ID 2
#define HFS_FPFS_MAX_DEPTH      1024

/*
 * 'J' 74: walking up from the item's parent 'id', the first folder marked a sync root (the item itself never counts:
 * FileProvider asks 'J' 72 for that), 0 when none is marked up to the root folder.
 */
static inline int
hfs_fpfs_nearest_sync_root(uint32_t id, void *arg, int (*marked)(void *, uint32_t, int *),
    int (*parent)(void *, uint32_t, uint32_t *), uint64_t *found)
{
	*found = 0;
	for (int depth = 0; depth < HFS_FPFS_MAX_DEPTH; depth++) {
		int is = 0, error;

		if (id < HFS_FPFS_ROOT_FOLDER_ID) {
			return 0;
		}
		if ((error = marked(arg, id, &is)) != 0) {
			return error;
		}
		if (is) {
			*found = id;
			return 0;
		}
		if (id == HFS_FPFS_ROOT_FOLDER_ID) {
			return 0;
		}
		if ((error = parent(arg, id, &id)) != 0) {
			return error;
		}
	}
	return ELOOP;
}

#endif /* _HFS_FPFS_H_ */
