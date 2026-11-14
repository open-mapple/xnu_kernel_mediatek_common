/*
 * iokit/Kernel/arm/hid/MappleILIProtocol.h on this machine: a report, a lost firmware, a version answer, the
 * firmware's host-download bit and the chip ID.
 *   cc -Wall tools/ili_touch_test.c -o ili_touch_test && ./ili_touch_test
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../iokit/Kernel/arm/hid/MappleILIProtocol.h"

/* a demo packet with one finger at grid x, y, its checksum set */
static void
report(uint8_t *rx, uint32_t x, uint32_t y)
{
	uint8_t *p = rx + 3, sum = 0;

	memset(rx, 0, ILI_RX_LEN);
	rx[1] = ILI_ACK;
	memset(p + 1, 0xff, 4 * ILI_FINGERS);
	p[0] = ILI_DEMO;
	p[1] = (uint8_t)((x >> 4 & 0xf0) | (y >> 8 & 0x0f));
	p[2] = (uint8_t)x;
	p[3] = (uint8_t)y;
	p[4] = 40;
	for (int i = 0; i < ILI_PACKET - 1; i++) {
		sum += p[i];
	}
	p[ILI_PACKET - 1] = (uint8_t)-sum;
}

int
main(void)
{
	uint8_t rx[ILI_RX_LEN], ap[0x14000];
	struct ili_point pt[ILI_FINGERS];

	report(rx, 1023, 2000);
	assert(ili_parse(rx, pt) == 1 && pt[0].x == 1023 && pt[0].y == 2000 && pt[0].pressure == 40);
	rx[10] ^= 1;
	assert(ili_parse(rx, pt) == ILI_BAD);

	rx[1] = 0x83;                                   /* the firmware gone: "firmware GONE (ACK 83)" */
	assert(ili_parse(rx, pt) == ILI_NO_ACK);

	/* mtkloader's version answer "a3 .. 21 04 2d 0c 00" (FACTS, ILI_DOWNLOAD) */
	memset(rx, 0, sizeof(rx));
	rx[1] = ILI_ACK;
	rx[3] = 0x21;
	rx[4] = 0x04;
	rx[5] = 0x2d;
	rx[6] = 0x0c;
	assert(ili_parse(rx, pt) == ILI_ANSWER);
	rx[1] = 0x00;
	assert(ili_parse(rx, pt) == ILI_NO_ACK);

	/* the phone's AP block: info block byte 0 is 0xea (host download set, SPI slave set, not I2C) */
	memset(ap, 0, sizeof(ap));
	ap[sizeof(ap) - 0x50] = 0xea;
	assert(ili_fw_host_download(ap, sizeof(ap)));
	ap[sizeof(ap) - 0x50] = 0xe8;
	assert(!ili_fw_host_download(ap, sizeof(ap)));
	assert(!ili_fw_host_download(ap, 0x50));

	assert(ili_chip_ok(0x78071f05));                /* mtkloader's ILI_PROBE */
	assert(!ili_chip_ok(0x98820000) && !ili_chip_ok(0) && !ili_chip_ok(0xffffffff));

	printf("ili_touch_test: ok\n");
	return 0;
}
