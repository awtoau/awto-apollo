/**
 * FPGA advertisement pin handling code.
 *
 * This file is part of Apollo.
 *
 * Copyright (c) 2023 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __FPGA_ADV_H__
#define __FPGA_ADV_H__

#include <stdint.h>

/**
 * How the FPGA advertises that it wants the shared USB port.
 *
 * FPGA_ADV (PA09) muxes to either EIC/EXTINT7 or SERCOM1/PAD3 -- one pin, one
 * peripheral -- so these are mutually exclusive and switching means re-muxing.
 *
 * EIC is the power-on default deliberately: a host that never selects a mode
 * behaves exactly as older firmware did, which is the right failure mode for
 * the path that decides who owns the USB port.
 */
typedef enum {
	// Count rising edges in a time window (original mechanism).
	FPGA_ADV_MODE_EIC  = 0,

	// Parse a framed heartbeat from a 230400-baud receive-only UART.
	//
	// 230400, not the 1 Mbaud this once said: at 1 us bits there is no margin
	// left for a USB interrupt to stretch a bit-banged transmit past the
	// receiver's sample point. See ADV_UART_BAUD in the cynthion_d11 board file
	// for the measurements. The responder gateware must be built at the same
	// rate -- a mismatch kills the link silently rather than degrading it.
	FPGA_ADV_MODE_UART = 1,
} fpga_adv_mode_t;

/**
 * Initialize FPGA_ADV receive-only pin
 */
void fpga_adv_init(void);

/**
 * Select how the FPGA advertisement is received.
 *
 * Re-muxes PA09 and reconfigures the relevant peripheral. Any evidence
 * gathered by the previous mode is discarded, so a switch cannot report a
 * port request derived from the other mechanism.
 *
 * Returns false if the mode is not recognised.
 */
bool fpga_adv_set_mode(fpga_adv_mode_t mode);

/**
 * The advertisement mode currently in effect.
 *
 * Worth having: without it a host cannot tell which regime it is in, and
 * diagnosing a handoff problem becomes guesswork.
 */
fpga_adv_mode_t fpga_adv_get_mode(void);

/**
 * Issue a command on FPGA_ADV and collect the fixed-length response.
 *
 * Response length is a property of the command, agreed by both sides at
 * compile time, so nothing is parsed to discover it. See
 * docs/apollo_samd11_mcu/fpga-adv-sideband.md.
 *
 * Only valid in UART mode. Returns the number of bytes collected, or 0 on
 * timeout -- which means the FPGA is not there, not that the protocol failed.
 */
uint8_t fpga_adv_command(uint8_t command, uint8_t *buffer, uint8_t length);

/**
 * CRC-8/ATM over a buffer, matching the responder gateware.
 */
uint8_t fpga_adv_crc8(const uint8_t *data, uint8_t length);

/**
 * Report link health since the last call: responses OK, CRC failures, and
 * timeouts. Reading clears the counters.
 */
void fpga_adv_get_stats(uint8_t *ok, uint8_t *crc_fail, uint8_t *timeout);

/**
 * True once the FPGA has completed configuration (its DONE pin is high).
 *
 * The sideband link cannot work before this: the FPGA's I/O are tri-stated
 * until configuration completes, so FPGA_ADV floats and there is no responder.
 */
bool fpga_configuration_done(void);

/**
 * Diagnostic: drive FPGA_ADV as a 10 Hz square wave, bypassing UART framing.
 *
 * Isolates "can Apollo drive this pin at all" from "does the transmit path
 * work". Off by default.
 */
void fpga_adv_set_toggle(bool enable);

/**
 * Task for things related with the advertisement pin
 */
void fpga_adv_task(void);

/**
 * Allow FPGA takeover of the USB port
 */
void allow_fpga_takeover_usb(bool allow);

/**
 * True if we received an advertisement message within the last time window.
 */
bool fpga_requesting_port(void);

#endif
