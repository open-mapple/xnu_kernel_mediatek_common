/* Screen diagnostics through bootstrap TTBR0 and the final kernel TTBR1. */
#ifndef _ARM64_EARLY_CONSOLE_H_
#define _ARM64_EARLY_CONSOLE_H_

#include <pexpert/arm64/board_config.h>
#include <stdarg.h>
#include <stdint.h>

/* viva only: on spaced 0x7bc10000 is TEE memory (secure only), so a spaced build leaves this out */
#if defined(MT6781) && defined(VIVA_EARLY_CONSOLE)
/*
 * MT6781: LK's video-mode framebuffer, ARGB8888, pitch 1088 px. It is
 * not 32MB-aligned, so the early console maps only the L2 block holding its
 * start and uses the rows that fit inside that block. The block also covers
 * part of the TEE carve-out (0x7aa00000-), so it is mapped as Device memory:
 * no speculative reads into secure DRAM.
 */
#define EARLY_CONSOLE_FB_PHYS     0x7bc10000ULL
#define EARLY_CONSOLE_FB_BLOCK    0x7a000000ULL
#define EARLY_CONSOLE_FB_ROWBYTES 4352u
#define EARLY_CONSOLE_FB_ROWS     948u
void early_console_mark(unsigned int stage, const char *label);
void early_console_putc(char c);
void early_console_panic(const char *fmt, va_list *args);
void early_console_relocate(uint64_t base);
void early_console_disable(void);
#else
static inline void early_console_mark(unsigned int stage, const char *label)
{
	(void)stage;
	(void)label;
}
static inline void early_console_disable(void) {}
#endif

#endif
