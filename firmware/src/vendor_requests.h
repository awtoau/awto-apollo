/*
 * Apollo vendor request IDs and the JTAG session policy that depends only on them.
 *
 * Split out of vendor.c so a host test can include it without the USB stack.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __VENDOR_REQUESTS_H__
#define __VENDOR_REQUESTS_H__

#include <stdbool.h>
#include <stdint.h>

// Supported vendor requests.
enum {
	VENDOR_REQUEST_GET_ID                  = 0xa0,
	// 0xa1 was SET_LED_PATTERN, removed -- do not reuse (see led.c).
	VENDOR_REQUEST_GET_FIRMWARE_VERSION    = 0xa2,
	VENDOR_REQUEST_GET_USB_API_VERSION     = 0xa3,
	VENDOR_REQUEST_GET_ADC_READING         = 0xa4,

	// Stack high-water mark, for right-sizing the reservation (#74). Grouped
	// with the other read-only status requests rather than given a number of its
	// own elsewhere; 0xa5 was the next free one.
	VENDOR_REQUEST_GET_STACK_USAGE         = 0xa5,

	//
	// JTAG requests.
	//
	VENDOR_REQUEST_JTAG_START              = 0xbf,
	VENDOR_REQUEST_JTAG_STOP               = 0xbe,

	VENDOR_REQUEST_JTAG_CLEAR_OUT_BUFFER   = 0xb0,
	VENDOR_REQUEST_JTAG_SET_OUT_BUFFER     = 0xb1,
	VENDOR_REQUEST_JTAG_GET_IN_BUFFER      = 0xb2,
	VENDOR_REQUEST_JTAG_SCAN               = 0xb3,
	VENDOR_REQUEST_JTAG_RUN_CLOCK          = 0xb4,
	VENDOR_REQUEST_JTAG_GOTO_STATE         = 0xb5,
	VENDOR_REQUEST_JTAG_GET_STATE          = 0xb6,
	// Reserved upstream for a bulk JTAG scan that was never implemented: there is
	// no handler and no case in the dispatch switch, here or in stock v1.1.1.
	//
	// Do NOT add it to is_jtag_request() or is_jtag_programming_request(). It was
	// in both until this comment replaced them, which armed an upstream
	// placeholder: 0xb7 escalated the session to "programming in flight" and then
	// fell through to `return false`, leaving the JTAG lock held by a request
	// that cannot complete. Gating a request that cannot be dispatched has no
	// meaning. Kept as a constant so the ID is not silently reused.
	VENDOR_REQUEST_JTAG_BULK_SCAN          = 0xb7,
	VENDOR_REQUEST_JTAG_GET_INFO           = 0xb8,
	VENDOR_REQUEST_JTAG_BENCHMARK          = 0xb9,

	// General programming requests.
	VENDOR_REQUEST_TRIGGER_RECONFIGURATION = 0xc0,
	VENDOR_REQUEST_FORCE_FPGA_OFFLINE      = 0xc1,
	VENDOR_REQUEST_ALLOW_FPGA_TAKEOVER_USB = 0xc2,
	VENDOR_REQUEST_FPGA_ADV_MODE           = 0xc3,
	VENDOR_REQUEST_GET_FPGA_STATUS_PINS    = 0xc4,
	VENDOR_REQUEST_BOOT_TO_DFU             = 0xed,


	//
	// Debug SPI requests
	//
	VENDOR_REQUEST_DEBUG_SPI_SEND          = 0x50,
	VENDOR_REQUEST_DEBUG_SPI_READ_RESPONSE = 0x51,
	VENDOR_REQUEST_FLASH_SPI_SEND          = 0x52,
	VENDOR_REQUEST_TAKE_FLASH_LINES        = 0x53,
	VENDOR_REQUEST_RELEASE_FLASH_LINES     = 0x54,

	//
	// Self-test requests.
	//
	VENDOR_REQUEST_GET_RAIL_VOLTAGE        = 0xe0,

	//
	// Control-plane escape hatch.
	//
	VENDOR_REQUEST_EMERGENCY_RESET         = 0xec,

	//
	// Microsoft WCID descriptor request
	//
	VENDOR_REQUEST_GET_MS_DESCRIPTOR       = 0xee,
};


/**
 * JTAG requests that actually drive the scan chain. The first of these in a
 * session escalates it to "programming in flight", after which conflicting
 * control-plane requests are refused.
 */
static inline bool is_jtag_programming_request(uint8_t request)
{
	switch (request) {
		case VENDOR_REQUEST_JTAG_CLEAR_OUT_BUFFER:
		case VENDOR_REQUEST_JTAG_SET_OUT_BUFFER:
		case VENDOR_REQUEST_JTAG_SCAN:
		case VENDOR_REQUEST_JTAG_RUN_CLOCK:
		case VENDOR_REQUEST_JTAG_GOTO_STATE:
		case VENDOR_REQUEST_JTAG_BENCHMARK:
			return true;
		default:
			return false;
	}
}


/**
 * Session policy for a request: JTAG_GATE_REFUSE, JTAG_GATE_PASS, or JTAG_GATE_ESCALATE
 * (run it, and the session becomes "programming in flight").
 *
 * The scan buffers alias the console ring, so without a JTAG_START a request that
 * writes them would corrupt live console data.
 */
enum { JTAG_GATE_REFUSE = -1, JTAG_GATE_PASS = 0, JTAG_GATE_ESCALATE = 1 };

static inline int jtag_request_gate(bool session_open, uint8_t request)
{
	bool programming = is_jtag_programming_request(request);
	return (programming && !session_open) ? JTAG_GATE_REFUSE : programming;
}

#endif
