/*
 * The ILI7807S's touch reports and mtkloader's copy of its firmware, for
 * ILI7807S.cpp. Plain C with no IOKit in it, so it also builds on a host.
 *
 * A report is the read command 0x83 and 45 bytes back: two (the second the
 * firmware's ACK 0xA3), then the 43-byte demo packet: 0x5A, ten fingers of
 * four bytes (x high nibble | y high nibble, x low, y low, pressure; FF FF FF
 * is no finger), and a checksum that makes the packet's byte sum zero.
 */
#ifndef _IOKIT_ARM_HID_MAPPLEILIPROTOCOL_H
#define _IOKIT_ARM_HID_MAPPLEILIPROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define ILI_FINGERS     10
#define ILI_PACKET      43
#define ILI_RX_LEN      (3 + ILI_PACKET)        /* clocked in while 0x83 goes out, two bytes, the packet */
#define ILI_ACK         0xa3
#define ILI_DEMO        0x5a
#define ILI_NO_ACK      (-1)
#define ILI_BAD         (-2)

#define ILI_BLOCKS      3                       /* AP, DATA, TUNING */
#define ILI_BLOCK_MAX   (160u << 10)
#define ILI_MEM_END     0x30000

struct ili_point {
	uint32_t x, y, pressure;
};

/* mtkloader's struct ili_block (mkilifw.py) as it lies in memory */
struct ili_loader_block {
	uint64_t name;
	uint32_t mem, len, crc, pad;
	uint64_t data;
};

struct ili_fw_block {
	uint32_t mem, len, crc, got;
	const volatile uint8_t *data;
};

/* Maps len bytes at physical address pa; NULL on failure. */
typedef const volatile uint8_t *(*ili_map_fn)(void *ctx, uint64_t pa, uint32_t len);

/*
 * One read's bytes: the fingers it names, one bit each, their places in pt
 * (grid units); ILI_NO_ACK when the firmware does not answer (it is gone),
 * ILI_BAD when the rest is not a report.
 */
static inline int
ili_parse(const uint8_t *rx, struct ili_point *pt)
{
	const uint8_t *p = rx + 3;
	uint8_t sum = 0;
	int map = 0;

	if (rx[1] != ILI_ACK) {
		return ILI_NO_ACK;
	}
	for (int i = 0; i < ILI_PACKET; i++) {
		sum += p[i];
	}
	if (p[0] != ILI_DEMO || sum != 0) {
		return ILI_BAD;
	}
	for (int f = 0; f < ILI_FINGERS; f++) {
		const uint8_t *q = p + 1 + 4 * f;

		if (q[0] == 0xff && q[1] == 0xff && q[2] == 0xff) {
			continue;
		}
		pt[f].x = (uint32_t)(q[0] & 0xf0) << 4 | q[1];
		pt[f].y = (uint32_t)(q[0] & 0x0f) << 8 | q[2];
		pt[f].pressure = q[3];
		map |= 1 << f;
	}
	return map;
}

/* 0..grid_max on the chip's grid to 0..pixels - 1 on the panel */
static inline uint32_t
ili_scale(uint32_t v, uint32_t grid_max, uint32_t pixels)
{
	uint32_t px = v * (pixels - 1) / grid_max;

	return px < pixels ? px : pixels - 1;
}

/* CRC32 MSB first, polynomial 0x04C11DB7, start 0xFFFFFFFF, no final xor: each block carries it in its last 4 bytes */
static inline uint32_t
ili_crc32(const volatile uint8_t *p, uint32_t len)
{
	uint32_t crc = 0xffffffff;

	for (uint32_t i = 0; i < len; i++) {
		crc ^= (uint32_t)p[i] << 24;
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x80000000) ? (crc << 1) ^ 0x04c11db7 : crc << 1;
		}
	}
	return crc;
}

/*
 * mtkloader's table into fw[], each block mapped and its CRC checked.
 * ILI_BLOCKS when all are good; otherwise the first bad block's index, with
 * its data NULL when the table entry does not look like one, or its got !=
 * crc.
 */
static inline int
ili_fw_walk(const volatile struct ili_loader_block *t, struct ili_fw_block *fw, ili_map_fn map, void *ctx)
{
	for (int i = 0; i < ILI_BLOCKS; i++) {
		fw[i].mem = t[i].mem;
		fw[i].len = t[i].len;
		fw[i].crc = t[i].crc;
		fw[i].got = 0;
		fw[i].data = NULL;
		if (fw[i].len < 8 || fw[i].len > ILI_BLOCK_MAX || fw[i].mem >= ILI_MEM_END || t[i].data == 0) {
			return i;
		}
		fw[i].data = map(ctx, t[i].data, fw[i].len);
		if (fw[i].data == NULL) {
			return i;
		}
		fw[i].got = ili_crc32(fw[i].data, fw[i].len - 4);
		if (fw[i].got != fw[i].crc) {
			return i;
		}
	}
	return ILI_BLOCKS;
}

#endif /* _IOKIT_ARM_HID_MAPPLEILIPROTOCOL_H */
