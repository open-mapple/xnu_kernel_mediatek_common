/*
 * Compile Apple's HFS kext sources inside MT6781 the way the kext is built:
 * with the kext (KPI) view of the kernel headers, and with only HFS's own
 * kext-config.h deciding features. The kernel's -DCONFIG_* options are not
 * visible to a kext build, so the ones HFS tests are cleared first. MT6781
 * has no AppleKeyStore, so content protection stays off.
 */
#ifndef HFS_STAGED_COMPAT_H
#define HFS_STAGED_COMPAT_H
#undef BSD_KERNEL_PRIVATE
#undef XNU_KERNEL_PRIVATE
#undef CONFIG_PROTECT
#undef CONFIG_SEARCHFS
#undef CONFIG_HFS_DIRLINK
#undef CONFIG_SECLUDED_RENAME
#undef CONFIG_MACF
#undef CONFIG_HFS_RBTREE
#undef QUOTA
#undef NAMEDSTREAMS
#undef FIFO
#undef HFS_COMPRESSION
#include "core/kext-config.h"

/* Built in, not a loadable kext (see hfs_staged_iokit.c). */
#define OSKextGetCurrentLoadTag hfs_staged_current_load_tag
#endif
