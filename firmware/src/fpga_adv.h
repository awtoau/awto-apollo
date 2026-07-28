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

	// Parse a framed heartbeat from a 1 Mbaud receive-only UART.
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
