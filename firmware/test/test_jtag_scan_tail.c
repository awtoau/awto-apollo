/**
 * Host-native regression test of the real jtag.c: where a scan's bit-banged tail stores TDO.
 *
 * jtag.c is compiled in (so its statics are visible) against stub headers and fake SPI / TAP
 * functions. The fake jtag_tap_shift records the output pointer it is given. Every pointer must be
 * NULL (scan discards TDO) or inside the 1024-byte `comms` region.
 *
 * Before the fix the destination was jtag_in_buffer + (bytes - 1) for every scan: a 692-byte final
 * chunk stored past the region, onto the USB control struct's callback pointer on the board.
 * JTAG_SRC selects the file under test (default src/jtag.c); the test is run against the pre-fix
 * file to show it fails there.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>

#ifndef JTAG_SRC
#define JTAG_SRC "../src/jtag.c"
#endif
#include JTAG_SRC

static int failures = 0;

#define CHECK(cond, ...)                                                       \
	do {                                                                       \
		if (cond) { printf("  ok   - "); }                                     \
		else { printf("  FAIL - "); failures++; }                              \
		printf(__VA_ARGS__);                                                   \
		printf("\n");                                                          \
	} while (0)

// --- fakes ---------------------------------------------------------------------------------

static uint8_t *tap_output;
static uint32_t tap_bits;
static int tap_calls;

void jtag_tap_shift(uint8_t *input_data, uint8_t *output_data, uint32_t data_bits, bool must_end)
{
	(void)input_data; (void)must_end;
	tap_output = output_data;
	tap_bits = data_bits;
	tap_calls++;
}

uint32_t board_millis(void) { return 0; }
void fpga_set_online(bool online) { (void)online; }
uint8_t jtag_current_state(void) { return 0; }
void jtag_deinit(void) {}
void jtag_init(void) {}
void jtag_go_to_state(unsigned state) { (void)state; }
void jtag_wait_time(uint32_t microseconds) { (void)microseconds; }
void led_set_override(uint8_t value) { (void)value; }
void spi_configure_pinmux(spi_target_t target) { (void)target; }
void spi_release_pinmux(spi_target_t target) { (void)target; }
void spi_init(spi_target_t target, bool lsb_first, bool configure_pinmux, uint8_t baud_divider,
              uint8_t clock_polarity, uint8_t clock_phase)
{ (void)target; (void)lsb_first; (void)configure_pinmux; (void)baud_divider; (void)clock_polarity; (void)clock_phase; }
static uint8_t *spi_rx;
static size_t spi_rx_len;
void spi_send(spi_target_t port, void *data_to_send, void *data_received, size_t length)
{ (void)port; (void)data_to_send; spi_rx = data_received; spi_rx_len = length; }
bool spi_send_async(spi_target_t port, void *data_to_send, void *data_received, size_t length)
{ (void)port; (void)data_to_send; spi_rx = data_received; spi_rx_len = length; return true; }
bool spi_send_done(void) { return true; }
bool tud_control_xfer(uint8_t rhport, tusb_control_request_t const *request, void *buffer, uint16_t len)
{ (void)rhport; (void)request; (void)buffer; (void)len; return true; }

// --- the test ------------------------------------------------------------------------------

static bool inside_comms(uint8_t *p, uint32_t bits)
{
	uint8_t *base = comms.jtag_tx;
	size_t bytes = (bits + 7) / 8;
	return p >= base && p + bytes <= base + sizeof(comms);
}

int main(void)
{
	int bad_discard = 0, bad_capture = 0, scans = 0;
	size_t first_bad = 0;

	// A final chunk: every length the host can send in the primary buffer, advancing state.
	for (size_t bytes = 1; bytes <= JTAG_BUFFER_SIZE; bytes++) {
		tap_calls = 0; tap_output = (uint8_t *)1;
		memset(comms.jtag_tx, 0, sizeof(comms.jtag_tx));
		bool ok = jtag_scan_from(comms.jtag_tx, bytes * 8, true, false, true);
		if (!ok) continue;
		scans++;
		if (tap_calls && tap_output != NULL && !inside_comms(tap_output, tap_bits)) {
			if (!bad_discard) first_bad = bytes;
			bad_discard++;
		}
	}
	CHECK(scans == JTAG_BUFFER_SIZE, "%d discarding final chunks of 1..%d bytes were scanned", scans, JTAG_BUFFER_SIZE);
	CHECK(bad_discard == 0, "no discarding final chunk stores outside comms (%d do; first at %zu bytes)",
	      bad_discard, first_bad);

	// A capturing scan is limited to the read half; its tail must land inside the region.
	for (size_t bytes = 1; bytes <= JTAG_READ_BUFFER_SIZE; bytes++) {
		tap_calls = 0; tap_output = (uint8_t *)1;
		if (!jtag_scan_from(comms.jtag_tx, bytes * 8, true, false, false)) continue;
		if (tap_calls && tap_output != NULL && !inside_comms(tap_output, tap_bits)) bad_capture++;
	}
	CHECK(bad_capture == 0, "no capturing final chunk stores outside comms (%d do)", bad_capture);

	// The case from the board: a 692-byte final chunk.
	tap_calls = 0; tap_output = (uint8_t *)1;
	(void)jtag_scan_from(comms.jtag_tx, 692 * 8, true, false, true);
	CHECK(tap_calls == 1 && tap_output == NULL, "a 692-byte discarding final chunk has no tail destination");

	// Every bit count, every flag combination: neither the DMA receive nor the tail may leave comms.
	int bad_any = 0;
	size_t first_bits = 0;
	for (int capture = 0; capture <= 1; capture++) {
		for (int advance = 0; advance <= 1; advance++) {
			for (uint32_t bits = 1; bits <= 8u * JTAG_BUFFER_SIZE + 64u; bits++) {
				tap_calls = 0; tap_output = NULL; spi_rx = NULL; spi_rx_len = 0;
				if (!jtag_scan_from(comms.jtag_tx, bits, advance, false, !capture)) continue;
				bool rx_bad = spi_rx && (spi_rx < comms.jtag_tx || spi_rx + spi_rx_len > comms.jtag_tx + sizeof(comms));
				bool tail_bad = tap_calls && tap_output && !inside_comms(tap_output, tap_bits);
				if (rx_bad || tail_bad) {
					if (!bad_any) first_bits = bits;
					bad_any++;
				}
			}
		}
	}
	CHECK(bad_any == 0, "no scan of any bit count (capture or discard, advance or not) writes outside comms (%d do; first at %zu bits)",
	      bad_any, first_bits);

	// FORCE_BITBANG sends every bit through the tail: the tail must be told the full count.
	int bad_bits = 0;
	uint32_t first_bitbang = 0;
	for (uint32_t bits = 1; bits <= 8u * JTAG_READ_BUFFER_SIZE; bits++) {
		tap_calls = 0; tap_bits = 0;
		if (!jtag_scan_from(comms.jtag_tx, bits, false, true, true)) continue;
		if (tap_bits != bits) {
			if (!bad_bits) first_bitbang = bits;
			bad_bits++;
		}
	}
	CHECK(bad_bits == 0, "a bitbang scan shifts every bit it was asked for (%d counts truncated; first at %u bits)",
	      bad_bits, first_bitbang);

	// The benchmark (0xB9) receives into jtag_in_buffer: its chunk must fit the read half.
	int bad_bench = 0;
	for (unsigned units = 1; units <= 255; units++) {
		tusb_control_request_t request = { .bRequest = 0xB9, .wValue = 1, .wIndex = (uint16_t)(units | 0xFF00), .wLength = 12 };
		spi_rx = NULL; spi_rx_len = 0;
		(void)handle_jtag_benchmark(0, &request);
		if (spi_rx && (spi_rx < comms.jtag_tx || spi_rx + spi_rx_len > comms.jtag_tx + sizeof(comms))) bad_bench++;
	}
	CHECK(bad_bench == 0, "no benchmark chunk receives outside comms (%d chunk sizes do)", bad_bench);

	printf(failures ? "FAILED\n" : "all passed\n");
	return failures != 0;
}
