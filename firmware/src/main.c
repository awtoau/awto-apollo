/*
 * Copyright (c) 2019-2023 Great Scott Gadgets <info@greatscottgadgets.com>
 * Copyright (c) 2019 Katherine J. Temkin <kate@ktemkin.com>
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <tusb.h>
#include <bsp/board_api.h>
#include <apollo_board.h>

#include "board_rev.h"
#include "led.h"
#include "stack_probe.h"
#include "jtag.h"
#include "fpga.h"
#include "console.h"
#include "debug_spi.h"
#include "usb_switch.h"
#include "button.h"
#include "fpga_adv.h"



/**
 * Main round-robin 'scheduler' for the execution tasks.
 */
int main(void)
{
	// Paint before anything else runs, so as little stack usage as possible is
	// excluded from the measurement. See stack_probe.c.
	stack_probe_paint();

	board_init();
	detect_hardware_revision();
	tusb_init();

	fpga_io_init();
	led_init();
	debug_spi_init();
	allow_fpga_takeover_usb(true);

	if (button_pressed()) {
		/*
		 * Interrupted start-up: Force the FPGA offline and take
		 * control of the USB port.
		 */
		force_fpga_offline();
		take_over_usb();

		/*
		 * Now that the FPGA is being held offline, release the
		 * mechanism that prevented the FPGA from configuring itself at
		 * startup.
		 */
		permit_fpga_configuration(true);
	} else {
		/*
		 * Normal start-up: Reconfigure FPGA from flash and hand off
		 * the USB port to the FPGA. This effectively makes the RESET
		 * button reset both the microcontroller and the FPGA.
		 */
		permit_fpga_configuration(true);
		trigger_fpga_reconfiguration();
		hand_off_usb();
	}

	fpga_adv_init();

	while (1) {
		tud_task(); // tinyusb device task
		// Clock out any scan the host has queued.
		//
		// Placed immediately after tud_task() and before everything else so the
		// SERCOM starts as soon as possible after the SCAN request that queued it.
		// The clocking is what overlaps with the next chunk's USB transfer, so every
		// microsecond between the request arriving and the clocking starting is a
		// microsecond of overlap lost.
		//
		// It does NOT run to completion. The clocking is handed to the DMAC and this
		// returns immediately, so tud_task() above keeps servicing USB while the bytes
		// are on the wire; later passes poll one DMAC flag and finish the scan when it
		// retires. That is the point of calling it every iteration rather than once.
		//
		// The earlier version spun on the SERCOM flags for the whole chunk -- about
		// 700 us for 1024 bytes -- during which tud_task() could not run at all and
		// the device NAKed the host's next request until the loop came round. Measured
		// at ~98 us of NAK per USB transaction; removing it is worth about 20% of the
		// configure time.
		jtag_scan_task();
		console_task();
		led_task();
		button_task();
		fpga_adv_task();
	}

	return 0;
}
