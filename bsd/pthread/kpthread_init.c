/* Built-in provider: Apple libpthread-514.100.2, paired with xnu-8796.141.3. */
#if defined(ARM64_BOARD_CONFIG_MT6781)
#include "kpthread_compat.h"
#include "libpthread_staged/kern/kern_init.c"

/* Keep the provider's full layout within the kernel's reserved storage. */
_Static_assert(sizeof(struct ksyn_waitq_element) == 56,
    "update MT6781 ksyn storage in sys/pthread_internal.h");

/* Restore the startup interface after compiling the provider's kext code. */
#define XNU_KERNEL_PRIVATE 1
#include <kern/startup.h>
#include <pexpert/pexpert.h>

__startup_func
static void
kpthread_register(void)
{
    kern_return_t kr = kpthread_start(NULL, NULL);
    if (kr != KERN_SUCCESS || kpthread_kern == NULL) {
        panic("pthread provider registration failed: %d", kr);
    }
    kprintf("MT6781 pthread: libpthread-514.100.2 registered\n");
}
STARTUP(EARLY_BOOT, STARTUP_RANK_SECOND, kpthread_register);
#endif /* ARM64_BOARD_CONFIG_MT6781 */
