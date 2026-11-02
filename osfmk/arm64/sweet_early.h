/* Screen diagnostics through bootstrap TTBR0 and the final kernel TTBR1. */
#ifndef _ARM64_SWEET_EARLY_H_
#define _ARM64_SWEET_EARLY_H_

#include <pexpert/arm64/board_config.h>
#include <stdarg.h>
#include <stdint.h>

#if defined(SWEET)
/*
 * viva (MT6781): LK's video-mode framebuffer, ARGB8888, pitch 1088 px. It is
 * not 32MB-aligned, so the early console maps only the L2 block holding its
 * start and uses the rows that fit inside that block. The block also covers
 * part of the TEE carve-out (0x7aa00000-), so it is mapped as Device memory:
 * no speculative reads into secure DRAM.
 */
#define SWEET_EARLY_FB_PHYS     0x7bc10000ULL
#define SWEET_EARLY_FB_BLOCK    0x7a000000ULL
#define SWEET_EARLY_FB_ROWBYTES 4352u
#define SWEET_EARLY_FB_ROWS     948u
void sweet_early_mark(unsigned int stage, const char *label);
void sweet_early_putc(char c);
void sweet_early_panic(const char *fmt, va_list *args);
void sweet_early_relocate(uint64_t base);
void sweet_early_disable(void);
#else
static inline void sweet_early_mark(unsigned int stage, const char *label)
{
	(void)stage;
	(void)label;
}
static inline void sweet_early_disable(void) {}
#endif

#endif
