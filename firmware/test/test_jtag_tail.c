/**
 * Host-native test of where a scan's bit-banged tail stores TDO (jtag_tail.h).
 *
 * The tail's destination was jtag_in_buffer + slow_offset even when the scan discards TDO; with
 * slow_offset up to the chunk length and jtag_in_buffer only the second 512 bytes of the 1024-byte
 * region, a final chunk over 513 bytes wrote past the region (692 bytes landed on the USB control
 * struct's callback pointer).
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stddef.h>

#include "jtag_tail.h"

static int failures = 0;

#define CHECK(cond, ...)                                                       \
	do {                                                                       \
		if (cond) { printf("  ok   - "); }                                     \
		else { printf("  FAIL - "); failures++; }                              \
		printf(__VA_ARGS__);                                                   \
		printf("\n");                                                          \
	} while (0)

int main(void)
{
	static uint8_t region[1024];
	uint8_t *in_buffer = region + 512;

	CHECK(jtag_tail_dest(true, in_buffer, 691) == NULL, "a scan that discards TDO has no tail destination");
	CHECK(jtag_tail_dest(true, in_buffer, 0) == NULL, "also at offset 0");
	CHECK(jtag_tail_dest(false, in_buffer, 100) == in_buffer + 100, "a capturing scan stores at the offset");
	CHECK(jtag_tail_dest(false, in_buffer, 511) < region + sizeof(region),
	      "a capturing scan's last allowed offset (511, JTAG_READ_BUFFER_SIZE - 1) stays inside the region");

	printf(failures ? "FAILED\n" : "all passed\n");
	return failures != 0;
}
