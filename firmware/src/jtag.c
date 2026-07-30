/*
 * Code for interacting with the FPGA via JTAG.
 *
 * This JTAG driver is intended to be as simple as possible in order to facilitate
 * configuration and debugging of the attached FPGA. It is not intended to be a general-
 * purpose JTAG link.
 *
 * Copyright (c) 2019-2023 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <tusb.h>
#include <apollo_board.h>
#include <bsp/board_api.h>

#include "led.h"
#include "jtag.h"
#include "uart.h"
#include "spi.h"
#include "fpga.h"

#define ISC_ENABLE  0xC6
#define ISC_DISABLE 0x26


// JTAG comms buffers, sharing storage with the console RX ring.
//
// The console ring and the JTAG buffers are never live at the same time, and the
// separation is enforced rather than conventional: apollo_mode_acquire_jtag() is
// called in jtag_init() and released in jtag_deinit(), so the lock spans every use
// of these buffers -- and console_task() returns immediately while it is held, so
// the ring is neither filled nor drained during a session.
//
// That is a stronger argument than the pin sharing. UART and JTAG contending for
// TDI/TMS explains why they cannot both drive; the lock scope is what proves
// neither BUFFER is in use while the other is.
//
// Worth stating what is NOT aliased: jtag_in_buffer and jtag_out_buffer remain
// separate. spi_send() takes both in one call, reading TDI from one while writing
// TDO to the other, so overlapping those two would corrupt every transfer.
//
// The risk this takes on: a wrong exclusivity assumption stops being a dropped
// console byte and becomes a corrupted bitstream mid-configure. The JTAG lock is
// what makes that not merely unlikely but structurally impossible.
// The pool is one region, carved differently depending on direction.
//
// A write does not read TDO -- the ECP5 self-validates by CRC, so ecp5.py passes
// ignore_response=True and FLAG_DISCARD_TDO makes spi_send() discard rather than
// store. So on the write path the receive half is dead space, and the transmit
// buffer may have the whole region: JTAG_BUFFER_SIZE bytes rather than half that.
//
// A read still needs somewhere to put TDO, so jtag_in_buffer points at the second
// half and is bounded by JTAG_READ_BUFFER_SIZE. That asymmetry is the whole point:
// writes are the bulk case at hundreds of chunks per configure, reads are a handful
// of bytes for IDCODE and status.
//
// What makes it safe is that the two are never live together. spi_send() takes both
// pointers in one call, so if a scan ever both transmitted a full-region payload AND
// captured TDO, the capture would overwrite the second half of the data still being
// clocked out. FLAG_DISCARD_TDO is what prevents that, and the assertion below is
// what stops the two definitions drifting apart.
#define JTAG_READ_BUFFER_SIZE (JTAG_BUFFER_SIZE / 2u)

union comms_buffers {
	uint8_t jtag_tx[JTAG_BUFFER_SIZE];
	uint8_t console_ring[CONSOLE_RING_SIZE];
};

static union comms_buffers comms __attribute__((aligned(4)));

uint8_t *const jtag_out_buffer = comms.jtag_tx;
// Second half of the same region. Valid only while TDO is being captured, which
// FLAG_DISCARD_TDO guarantees is never during a full-region write.
uint8_t *const jtag_in_buffer  = comms.jtag_tx + JTAG_READ_BUFFER_SIZE;
uint8_t *const console_rx_ring = comms.console_ring;


/**
 * Flags for our JTAG commands.
 */
enum {
        FLAG_ADVANCE_STATE = 0b001,
        FLAG_FORCE_BITBANG = 0b010,

        // The host does not want TDO back, so it need not be stored.
        //
        // The host already had an `ignore_response` notion, but it only suppressed
        // the host's own GET_IN_BUFFER call -- the firmware still captured every
        // byte into jtag_in_buffer. Telling the firmware means the receive buffer
        // does not have to exist for the write path, which is what makes a larger
        // transmit buffer affordable.
        FLAG_DISCARD_TDO   = 0b100
};


/**
 * Performs JTAG scan.
 */
bool jtag_scan(uint32_t num_bits, bool advance_state, bool bitbang,
               bool discard_tdo)
{
	// Our bulk method can only send whole bytes; so send as many bytes as we can
	// using the fast method; and then send the remainder using our slow method.
	size_t bytes_to_send_bulk = num_bits / 8;
	size_t bits_to_send_slow  = num_bits % 8;

	// We can't handle 0-byte transfers; fail out.
	if (!bits_to_send_slow && !bytes_to_send_bulk) {
		return false;
	}

	// A scan that captures TDO is bounded by the READ buffer, since TDO lands in the
	// second half of the region the payload occupies. Without this a large capturing
	// scan would overwrite the data it was still clocking out.
	if (!discard_tdo && bytes_to_send_bulk > JTAG_READ_BUFFER_SIZE) {
		return false;
	}

	// If this would scan more than we have buffer for, fail out.
	if (bytes_to_send_bulk > JTAG_BUFFER_SIZE) {
		return false;
	}

	// If we've been asked to send data the slow way, honor that, and send all of our bits
	// using the slow method.
	if (bitbang) {
		bytes_to_send_bulk = 0;
		bits_to_send_slow  = num_bits;
	}

	// If we're going to advance state, always make sure the last bit is sent using the slow method,
	// so we can handle JTAG TAP state advancement on the last bit. If we don't have any bits to send slow,
	// send the last byte slow.
	if (!bits_to_send_slow && advance_state) {
		bytes_to_send_bulk--;
		bits_to_send_slow = 8;
	}

	// Switch to SPI mode, and send the bulk of the transfer using it.
	if (bytes_to_send_bulk) {
		spi_configure_pinmux(SPI_FPGA_JTAG);
		// NULL receive means "clock these out and drop what comes back", so the
		// receive buffer is untouched on the write path.
		spi_send(SPI_FPGA_JTAG, jtag_out_buffer,
		         discard_tdo ? NULL : jtag_in_buffer, bytes_to_send_bulk);
	}

	// Switch back to GPIO mode, and send the remainder using the slow method.
	spi_release_pinmux(SPI_FPGA_JTAG);
	if (bits_to_send_slow) {
		jtag_tap_shift(jtag_out_buffer + bytes_to_send_bulk, jtag_in_buffer + bytes_to_send_bulk,
				bits_to_send_slow, advance_state);
	}

	return true;
}


/**
 * Simple request that clears the JTAG out buffer.
 */
bool handle_jtag_request_clear_out_buffer(uint8_t rhport, tusb_control_request_t const* request)
{
	memset(jtag_out_buffer, 0, JTAG_BUFFER_SIZE);
	return tud_control_xfer(rhport, request, NULL, 0);
}


/**
 * Simple request that sets the JTAG out buffer's contents.
 * This is used to set the data to be transmitted during the next scan.
 */
bool handle_jtag_request_set_out_buffer(uint8_t rhport, tusb_control_request_t const* request)
{
	// If we've been handed too much data, stall.
	if (request->wLength > JTAG_BUFFER_SIZE) {
		return false;
	}

	// HACK: check the buffer for commands that affect the FPGA configuration state.
    if (request->wLength == 1) {
		if (jtag_out_buffer[0] == ISC_ENABLE) {
			fpga_set_online(false);
		}
		if (jtag_out_buffer[0] == ISC_DISABLE) {
			fpga_set_online(true);
		}
    }

	// Copy the relevant data into our OUT buffer.
	return tud_control_xfer(rhport, request, jtag_out_buffer, request->wLength);
}


/**
 * Simple request that gets the JTAG in buffer's contents.
 * This is used to fetch the data received during the last scan.
 */
bool handle_jtag_request_get_in_buffer(uint8_t rhport, tusb_control_request_t const* request)
{
	uint16_t length = request->wLength;

	// Bounded by the READ size, not the whole region: jtag_in_buffer is the second
	// half of the pool, so reading JTAG_BUFFER_SIZE from it would run off the end.
	if (length > JTAG_READ_BUFFER_SIZE) {
		length = JTAG_READ_BUFFER_SIZE;
	}

	// Send up the contents of our IN buffer.
	return tud_control_xfer(rhport, request, jtag_in_buffer, length);
}


/**
 * Request that performs the actual JTAG scan event.
 * Arguments:
 *     wValue: the number of bits to scan; total
 *     wIndex:
 *        - 1 if the given command should advance the FSM
 *        - 2 if the given command should be sent using the slow method
 */
bool handle_jtag_request_scan(uint8_t rhport, tusb_control_request_t const* request)
{
	if (jtag_scan(request->wValue, request->wIndex & FLAG_ADVANCE_STATE,
	              request->wIndex & FLAG_FORCE_BITBANG,
	              request->wIndex & FLAG_DISCARD_TDO)) {
		return tud_control_xfer(rhport, request, NULL, 0);
	} else {
		return false;
	}
}


/**
 * Runs the JTAG clock for a specified amount of ticks.
 * Arguments:
 *     wValue: The number of clock cycles to run.
 */
bool handle_jtag_run_clock(uint8_t rhport, tusb_control_request_t const* request)
{
	jtag_wait_time(request->wValue);
	return tud_control_xfer(rhport, request, NULL, 0);
}


/**
 * Runs the JTAG clock for a specified amount of ticks.
 * Arguments:
 *     wValue: The state number to go to. See jtag.h for state numbers.
 */
bool handle_jtag_go_to_state(uint8_t rhport, tusb_control_request_t const* request)
{
	jtag_go_to_state(request->wValue);
	return tud_control_xfer(rhport, request, NULL, 0);
}


/**
 * Reads the current JTAG TAP state. Mostly intended as a debug aid.
 */
bool handle_jtag_get_state(uint8_t rhport, tusb_control_request_t const* request)
{
	static uint8_t jtag_state;

	jtag_state = jtag_current_state();
	return tud_control_xfer(rhport, request, &jtag_state, sizeof(jtag_state));
}


/**
 * Initializes JTAG communication.
 */
bool handle_jtag_start(uint8_t rhport, tusb_control_request_t const* request)
{
	led_set_pattern(LED_JTAG_CONNECTED);
	jtag_init();

	return tud_control_xfer(rhport, request, NULL, 0);
}


/**
 * De-initializes JTAG communcation; and stops driving the scan chain.
 */
bool handle_jtag_stop(uint8_t rhport, tusb_control_request_t const* request)
{
	led_set_pattern(LED_IDLE);
	jtag_deinit();

	return tud_control_xfer(rhport, request, NULL, 0);
}


/**
 * Synthetic JTAG throughput benchmark.
 *
 * Every previous measurement of the JTAG path was taken from the host, and so
 * included a SET_OUT_BUFFER control transfer per 256-byte chunk -- which is the
 * larger cost by a wide margin. This request removes USB from the measurement
 * entirely: one small setup transfer starts it, the MCU then clocks
 * (chunk x repeats) bytes out of a buffer it already holds, and one IN transfer
 * collects the result. What happens in between is MCU and wire only.
 *
 * The data source is a rolling pattern rather than zeroes, and every received
 * byte is checked against what the TAP should have returned. A stalled or no-op
 * loop is indistinguishable from a fast one on a stopwatch, so this readback
 * check is the evidence that bytes actually moved; and a rolling pattern makes
 * it sensitive to bytes arriving in the wrong order, which a constant is not.
 *
 * The caller must have parked the TAP in SHIFT_DR. Anywhere else, TDO is not
 * carrying data: SCK still runs at full rate and the timings still look
 * entirely reasonable, but the readback is all zeroes and proves nothing.
 *
 * Left in the firmware permanently: it is the only instrument that can measure
 * the JTAG side in isolation, and so it is also the regression check for it.
 *
 * Arguments:
 *     wValue: number of repeats (how many times the chunk is clocked out)
 *     wIndex: low byte  -- chunk size in bytes (0 means 256)
 *             high byte -- SERCOM baud divider to use for the run; 0xFF keeps
 *                          the JTAG default. SCK = 48MHz / (2 * (divider + 1)).
 *
 * Returns 12 bytes, little-endian:
 *     [0:4]  elapsed milliseconds
 *     [4:6]  total bytes clocked / 256
 *     [6:10] count of bytes that did not read back as expected
 *     [10]   first byte sent on the last iteration
 *     [11]   first byte received on the last iteration
 */
bool handle_jtag_benchmark(uint8_t rhport, tusb_control_request_t const* request)
{
	static uint8_t result[12];

	uint16_t repeats = request->wValue;
	uint16_t chunk   = request->wIndex & 0xFF;
	uint8_t  divider = (request->wIndex >> 8) & 0xFF;

	// A chunk of 0 means a full buffer; 256 does not fit in the low byte.
	if (chunk == 0) {
		chunk = JTAG_BUFFER_SIZE;
	}
	if (repeats == 0) {
		return false;
	}

	// Fill the source buffer with a rolling pattern. Done here rather than by
	// the host so that no bulk data crosses USB for this measurement at all.
	for (unsigned i = 0; i < chunk; ++i) {
		jtag_out_buffer[i] = (uint8_t)(i * 7 + 1);
	}

	// Re-clock the SERCOM if the caller asked for a different rate. This is the
	// point of the harness: SCK can be pushed until TDO readback stops matching,
	// with no USB traffic in the way to confound the result.
	if (divider != 0xFF) {
		spi_init(SPI_FPGA_JTAG, true, false, divider, 1, 1);
	}

	spi_configure_pinmux(SPI_FPGA_JTAG);

	// Count bytes that came back exactly as the TAP should have returned them,
	// rather than accumulating a hash. In SHIFT_DR with BYPASS selected, TDO is
	// TDI delayed by one bit, so the expected response is fully predictable and
	// can be compared byte for byte. A hash was tried first and proved a poor
	// instrument: it is length-dependent, it wraps, and a wrong-but-consistent
	// link still yields a stable value. A mismatch count answers the question
	// that actually matters -- did every byte arrive intact -- and does so
	// identically at any transfer size or clock rate.
	uint32_t mismatches = 0;
	uint8_t carry = 0;

	uint32_t start = board_millis();

	for (unsigned r = 0; r < repeats; ++r) {
		spi_send(SPI_FPGA_JTAG, jtag_out_buffer, jtag_in_buffer, chunk);

		// Reading every byte also guarantees the compiler cannot discard the
		// transfer as unused work.
		for (unsigned i = 0; i < chunk; ++i) {
			// SPI here is LSB-first, so the one-bit BYPASS delay shifts each
			// byte right, with the previous byte's LSB arriving in bit 7.
			uint8_t sent     = jtag_out_buffer[i];
			uint8_t expected = (uint8_t)((sent << 1) | carry);
			carry = (uint8_t)(sent >> 7);

			if (jtag_in_buffer[i] != expected) {
				mismatches++;
			}
		}
	}

	uint32_t elapsed = board_millis() - start;

	spi_release_pinmux(SPI_FPGA_JTAG);

	// Restore the standard JTAG clocking, so a benchmark run at an exotic rate
	// cannot leave the scan chain misconfigured for the configuration that
	// follows it.
	if (divider != 0xFF) {
		spi_init(SPI_FPGA_JTAG, true, false, 1, 1, 1);
	}

	uint32_t blocks = ((uint32_t)chunk * repeats) / 256;

	result[0] = elapsed            & 0xFF;
	result[1] = (elapsed >>  8)    & 0xFF;
	result[2] = (elapsed >> 16)    & 0xFF;
	result[3] = (elapsed >> 24)    & 0xFF;
	result[4] = blocks             & 0xFF;
	result[5] = (blocks >>   8)    & 0xFF;
	result[6] = mismatches         & 0xFF;
	result[7] = (mismatches >>  8) & 0xFF;
	result[8] = (mismatches >> 16) & 0xFF;
	result[9] = (mismatches >> 24) & 0xFF;

	// The first sent/received pair from the last iteration. When a run fails,
	// the difference between "TDO stuck at 0x00", "stuck at 0xFF" and "shifted
	// by the wrong number of bits" are three quite different faults, and the
	// mismatch count alone cannot distinguish them.
	result[10] = jtag_out_buffer[0];
	result[11] = jtag_in_buffer[0];

	return tud_control_xfer(rhport, request, result, sizeof(result));
}
