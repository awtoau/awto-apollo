/**
 * Console handling code.
 *
 * This file is part of Apollo.
 *
 * Copyright (c) 2019-2024 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <tusb.h>

#include <stdbool.h>

#include "led.h"
#include "uart.h"
#include "jtag.h"
#include "apollo_mode.h"


extern bool uart_active;


// UART RX interrupt can fire while TinyUSB is in critical sections.
// Buffer bytes here and flush them from console_task() in thread context.
#define UART_RX_RING_SIZE CONSOLE_RING_SIZE

// The ring lives in the union shared with the JTAG buffers, declared in jtag.c.
// Safe because console_task() returns immediately while the JTAG lock is held, so
// the ring is neither filled nor drained during a JTAG session -- see the union's
// comment for the full argument.
#define uart_rx_ring ((volatile uint8_t *)console_rx_ring)
static volatile uint16_t uart_rx_head = 0;
static volatile uint16_t uart_rx_tail = 0;


static inline uint16_t uart_ring_next(uint16_t v)
{
	return (uint16_t)((v + 1u) & (UART_RX_RING_SIZE - 1u));
}


static inline bool uart_ring_is_empty(void)
{
	return uart_rx_head == uart_rx_tail;
}


static inline void uart_ring_push(uint8_t byte)
{
	uint16_t next = uart_ring_next(uart_rx_head);

	// Drop oldest byte on overflow to preserve newest console data.
	if (next == uart_rx_tail) {
		uart_rx_tail = uart_ring_next(uart_rx_tail);
	}

	uart_rx_ring[uart_rx_head] = byte;
	uart_rx_head = next;
}


static inline bool uart_ring_pop(uint8_t *byte)
{
	if (uart_ring_is_empty()) {
		return false;
	}

	*byte = uart_rx_ring[uart_rx_tail];
	uart_rx_tail = uart_ring_next(uart_rx_tail);
	return true;
}


/**
 * Pass any data received via UART directly up to the host.
 */
void uart_byte_received_cb(uint8_t byte)
{
	uart_ring_push(byte);
}


/**
 * Main task that handles console I/O.
 */
void console_task(void)
{
	if (!tud_cdc_connected()) {
		return;
	}

	// The console must be entirely inert while JTAG programming is in progress.
	//
	// UART and JTAG share pins on this part -- TDI is PA14, TMS is PA11 -- so
	// they cannot both own them. uart_initialize() already refuses while the
	// JTAG lock is held, which is what stops the console stealing the pins
	// mid-programming. But refusing is not the same as not asking: the main loop
	// is an unthrottled while(1), so without this check console_task() calls
	// uart_initialize() on every iteration for the whole duration of a
	// programming operation, purely to be turned away. JTAG programming is the
	// one operation here that must not be disturbed, and a half-programmed FPGA
	// is the failure being prevented.
	//
	// Bailing out entirely also leaves the ring buffer alone, so nothing is
	// drained to CDC while the FPGA cannot be sending anything anyway.
	if (apollo_mode_jtag_active()) {
		return;
	}

	// Opportunistically initialize UART if callbacks were missed.
	if (!uart_active) {
		uart_initialize(true, 115200);
	}

	// Flush UART->CDC data in task context (TinyUSB-safe context).
	if (tud_cdc_write_available()) {
		uint8_t byte;
		while (tud_cdc_write_available() && uart_ring_pop(&byte)) {
			tud_cdc_write_char(byte);
		}
		tud_cdc_write_flush();
	}

	// We can send data to the FPGA over UART iff:
	//  - there's data waiting for us to send, and
	//  - the UART has room in its FIFO
	//
	// If both conditions are met, send data.
	while (uart_ready_for_write() && tud_cdc_available()) {
		uint8_t byte = tud_cdc_read_char();
		uart_nonblocking_write(byte);
	}

}

//
// We defer initializing our UART until we get a CDC connection.
//
// This prevents contention if the FPGA lines are used for something else,
// but makes everything seem to Just Work (TM) once the user starts using
// the CDC-ACM connection.
//


/**
 * Call-back issued when the host's line-coding changes.
 */
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* coding)
{
	// Refuse to (re)initialize the UART while JTAG owns the shared pins. On d11
	// the console UART lives on the JTAG pins (PA11/PA14), so a host line-coding
	// change arriving mid-flash would otherwise repinmux them and corrupt an
	// in-flight JTAG program/configure. See apollo_mode.h / #65.
	if (apollo_mode_jtag_active()) {
		return;
	}
	uart_initialize(true, coding->bit_rate);
}


/**
 * Other callbacks: if our UART isn't active, initialize it.
 * These are also gated on JTAG mode for the same pin-ownership reason.
 */

void tud_cdc_rx_wanted_cb(uint8_t itf, char wanted_char)
{
	if (apollo_mode_jtag_active()) {
		return;
	}
	if (!uart_active) {
		uart_initialize(true, 115200);
	}
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	if (apollo_mode_jtag_active()) {
		return;
	}
	if (!uart_active) {
		uart_initialize(true, 115200);
	}
}
