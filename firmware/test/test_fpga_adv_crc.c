/**
 * Host-native test of fpga_adv_crc8 (src/fpga_adv_crc.h): CRC-8/ATM, polynomial 0x07,
 * init 0, no reflection. Checked against the standard check value and an independent
 * table-driven implementation, so the bitwise loop the firmware ships is the one tested.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "fpga_adv_crc.h"

static int failures = 0;

#define CHECK(cond, ...)                                                       \
	do {                                                                       \
		if (cond) { printf("  ok   - "); }                                     \
		else { printf("  FAIL - "); failures++; }                              \
		printf(__VA_ARGS__);                                                   \
		printf("\n");                                                          \
	} while (0)

static uint8_t table[256];

static void build_table(void)
{
	for (int i = 0; i < 256; i++) {
		uint8_t crc = (uint8_t)i;
		for (int bit = 0; bit < 8; bit++) {
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
		}
		table[i] = crc;
	}
}

static uint8_t reference(const uint8_t *data, uint8_t length)
{
	uint8_t crc = 0;
	for (uint8_t i = 0; i < length; i++) {
		crc = table[crc ^ data[i]];
	}
	return crc;
}

int main(void)
{
	build_table();

	const uint8_t check[] = "123456789";
	CHECK(fpga_adv_crc8(check, 9) == 0xF4, "CRC-8/ATM of \"123456789\" is 0xF4 (got 0x%02X)", fpga_adv_crc8(check, 9));
	CHECK(fpga_adv_crc8(check, 0) == 0x00, "an empty buffer gives 0x00");

	int bad_single = 0;
	for (int value = 0; value < 256; value++) {
		uint8_t byte = (uint8_t)value;
		if (fpga_adv_crc8(&byte, 1) != reference(&byte, 1)) bad_single++;
	}
	CHECK(bad_single == 0, "all 256 single-byte inputs match the table implementation (%d differ)", bad_single);

	// Frame shapes the link uses: 3-byte replies and longer buffers, pseudo-random content.
	int bad_buffers = 0;
	uint32_t state = 0x12345678u;
	for (int round = 0; round < 2000; round++) {
		uint8_t buffer[18];
		uint8_t length = (uint8_t)(1 + round % 18);
		for (int i = 0; i < length; i++) {
			state = state * 1664525u + 1013904223u;
			buffer[i] = (uint8_t)(state >> 24);
		}
		if (fpga_adv_crc8(buffer, length) != reference(buffer, length)) bad_buffers++;
	}
	CHECK(bad_buffers == 0, "2000 buffers of 1..18 bytes match the table implementation (%d differ)", bad_buffers);

	// A frame followed by its CRC checks to zero (the property the receiver relies on).
	uint8_t frame[4] = { 0xC1, 0x14, 0x01, 0 };
	frame[3] = fpga_adv_crc8(frame, 3);
	CHECK(fpga_adv_crc8(frame, 4) == 0x00, "a frame plus its CRC checks to zero");

	printf(failures ? "FAILED\n" : "all passed\n");
	return failures != 0;
}
