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

#ifndef __JTAG_H__
#define __JTAG_H__

#include <stdint.h>

/**
 * Size of each JTAG scan buffer, in bytes.
 *
 * Declared here rather than in jtag.c because the buffers are used outside it:
 * fpga.c stages ISC_ENABLE / ISC_DISABLE into jtag_out_buffer directly. It had
 * its own `extern uint8_t jtag_out_buffer[256];` with the 256 written out a
 * second time, so changing the size in one place left the other silently
 * disagreeing -- and an array bound that disagrees with its definition is the
 * kind of mismatch a compiler is under no obligation to mention.
 *
 * Raised from 256 to 512, which is a real speed change rather than tidying. The
 * host does NOT hardcode the chunk: apollo_fpga/jtag.py:188 reads the buffer size
 * from the GET_INFO request and sizes its transactions from that, falling back to
 * 256 only when GET_INFO is unimplemented -- which it was here, so every host
 * silently used the fallback. Implementing GET_INFO and doubling this halves the
 * number of chunks in a configure, and per-request overhead dominates that path:
 * 612 ms of 953 ms measured, across 394 chunks at ~215 us of fixed cost each.
 *
 * 512 rather than 1024 because of RAM, not the protocol. Two buffers at 1024 puts
 * static RAM at 96.5% of 4 KB, and the stack high-water mark is measured at 344
 * bytes of a 1024-byte reservation -- comfortable, but not with 144 bytes of slack
 * on a part that has no MPU to catch an overflow. At 512 the total is 83.98%.
 */
#define JTAG_BUFFER_SIZE 512

extern uint8_t jtag_in_buffer[JTAG_BUFFER_SIZE];
extern uint8_t jtag_out_buffer[JTAG_BUFFER_SIZE];

typedef enum e_TAPState
{
	STATE_TEST_LOGIC_RESET =  0,
	STATE_RUN_TEST_IDLE    =  1,
	STATE_SELECT_DR_SCAN   =  2,
	STATE_CAPTURE_DR       =  3,
	STATE_SHIFT_DR         =  4,
	STATE_EXIT1_DR         =  5,
	STATE_PAUSE_DR         =  6,
	STATE_EXIT2_DR         =  7,
	STATE_UPDATE_DR        =  8,
	STATE_SELECT_IR_SCAN   =  9, 
	STATE_CAPTURE_IR       = 10,
	STATE_SHIFT_IR         = 11,
	STATE_EXIT1_IR         = 12,
	STATE_PAUSE_IR         = 13,
	STATE_EXIT2_IR         = 14,
	STATE_UPDATE_IR        = 15
} jtag_tap_state_t;


/**
 * Performs the start-of-day tasks necessary to talk JTAG to our FPGA.
 */
void jtag_init(void);


/**
 * De-inits the JTAG connection, so the JTAG chain. is no longer driven.
 */
void jtag_deinit(void);


/**
 * Performs JTAG scan.
 */
bool jtag_scan(uint32_t num_bits, bool advance_state, bool bitbang);


/**
 * Moves to a given JTAG state.
 */
void jtag_goto_state(int state);


/**
 * Performs a raw TAP scan.
 */
void jtag_tap_shift(
	uint8_t *input_data,
	uint8_t *output_data,
	uint32_t data_bits,
	bool must_end);


void jtag_wait_time(uint32_t microseconds);

void jtag_go_to_state(unsigned state);

uint8_t jtag_current_state(void);

/**
 * Simple handler that clears the JTAG out buffer.
 */
bool handle_jtag_request_clear_out_buffer(uint8_t rhport, tusb_control_request_t const* request);


/**
 * Simple request that sets the JTAG out buffer's contents.
 * This is used to set the data to be transmitted during the next scan.
 */
bool handle_jtag_request_set_out_buffer(uint8_t rhport, tusb_control_request_t const* request);


/**
 * Simple request that gets the JTAG in buffer's contents.
 * This is used to fetch the data received during the last scan.
 */
bool handle_jtag_request_get_in_buffer(uint8_t rhport, tusb_control_request_t const* request);


/**
 * Request that performs the actual JTAG scan event.
 * Arguments:
 *     wValue: the number of bits to scan; total
 */
bool handle_jtag_request_scan(uint8_t rhport, tusb_control_request_t const* request);


/**
 * Runs the JTAG clock for a specified amount of ticks.
 * Arguments:
 *     wValue: The number of clock cycles to run.
 *     wIndex: If non-zero, TMS will be held high during the relevant cycles..
 */
bool handle_jtag_run_clock(uint8_t rhport, tusb_control_request_t const* request);


/**
 * Initializes JTAG communication.
 */
bool handle_jtag_start(uint8_t rhport, tusb_control_request_t const* request);


/**
 * Runs the JTAG clock for a specified amount of ticks.
 * Arguments:
 *     wValue: The state number to go to. See jtag.h for state numbers.
 */
bool handle_jtag_go_to_state(uint8_t rhport, tusb_control_request_t const* request);


/**
 * De-initializes JTAG communcation; and stops driving the scan chain.
 */
bool handle_jtag_stop(uint8_t rhport, tusb_control_request_t const* request);


/**
 * Reads the current JTAG TAP state. Mostly intended as a debug aid.
 */
bool handle_jtag_get_state(uint8_t rhport, tusb_control_request_t const* request);


/**
 * Synthetic JTAG throughput benchmark: clocks generated data out over JTAG with
 * no bulk USB traffic, and reports elapsed time plus a TDO checksum.
 *
 * Arguments:
 *     wValue: number of repeats
 *     wIndex: low byte -- chunk size (0 means 256); high byte -- SERCOM baud
 *             divider, or 0xFF to keep the JTAG default.
 */
bool handle_jtag_benchmark(uint8_t rhport, tusb_control_request_t const* request);

#endif
