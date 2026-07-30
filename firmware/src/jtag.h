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
 * fpga.c stages ISC_ENABLE / ISC_DISABLE through jtag_fill_buffer() directly. It
 * had its own `extern uint8_t jtag_out_buffer[256];` with the 256 written out a
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
#define JTAG_BUFFER_SIZE 1024

/**
 * Size of the SECOND transmit buffer, which exists so that a USB transfer filling
 * one buffer can overlap with the SERCOM clocking the other.
 *
 * 512 rather than 1024, and the asymmetry is forced by RAM rather than chosen. A
 * second full-size buffer costs +1024 bytes; the part has 1216 unallocated, so it
 * physically fits -- but it puts static RAM at 95.3% and
 * scripts/apollo_budget_check.py fails anything above 85%. Under that ceiling
 * there are 601 bytes to spend, so 512 is the largest power of two available.
 *
 * Losing nothing by it is not an assumption: scripts/jtag_cost_split.py measures
 * the two halves separately at 1024 B/chunk and finds staging 339.7 ms against
 * clocking 107.4 ms. The second buffer only ever has to hide CLOCKING behind
 * staging, and 512 bytes of clocking (~42 ms) is far less than 1024 bytes of
 * staging (~340 ms), so the smaller buffer still covers the whole overlap. Making
 * it 1024 would buy no additional overlap and cost 512 bytes of RAM.
 *
 * The primary buffer stays 1024 so the negotiated chunk size, and therefore the
 * chunk count in a configure, is unchanged.
 */
#define JTAG_ALT_BUFFER_SIZE 512

/**
 * Bytes the console RX ring needs. Declared here because it shares storage with
 * the JTAG buffers -- see the union in jtag.c for why that is safe.
 */
#define CONSOLE_RING_SIZE 256

// Pointers into a shared union, NOT arrays. sizeof() on these yields the pointer
// size, so use JTAG_BUFFER_SIZE for bounds -- a check against sizeof() would
// silently become a 4-byte limit, which on a request handler is a bound that
// passes everything real and truncates it.
/**
 * Limit for a scan that CAPTURES TDO, as opposed to one that discards it.
 *
 * The buffers overlap: transmit gets the whole region and TDO lands in its second
 * half, so a capturing scan larger than this would overwrite data still being
 * clocked out. jtag_scan() refuses one, and GET_INFO reports this separately so the
 * host can discover the boundary rather than meeting it as a failed request.
 */
#define JTAG_READ_BUFFER_SIZE (JTAG_BUFFER_SIZE / 2u)

extern uint8_t *const jtag_in_buffer;
extern uint8_t *const console_rx_ring;

/**
 * The transmit buffer currently being FILLED, which is what a caller staging data
 * wants. No longer a fixed pointer: with two buffers alternating, the one being
 * filled changes every chunk, so a `uint8_t *const` cannot name it.
 *
 * fpga.c stages single opcode bytes through this, and does so between scans rather
 * than during one, so it always sees an idle buffer.
 */
uint8_t *jtag_fill_buffer(void);

/**
 * True if a scan is queued or in progress, so the buffer being filled is not the
 * one being clocked.
 */
bool jtag_scan_pending(void);

/**
 * Whether a SET_OUT_BUFFER (is_scan false) or SCAN (is_scan true) of these
 * parameters can be handled in USB interrupt context, i.e. without reaching the
 * DMAC's shared CHID window.
 *
 * See the definition in jtag.c for the reasoning. A false answer is always safe: the
 * request falls back to the queued path and costs the latency it always did.
 */
bool jtag_request_isr_safe(bool is_scan, uint16_t wValue, uint16_t wIndex,
                           uint16_t wLength);

/**
 * Advances a scan queued by handle_jtag_request_scan(), if there is one.
 *
 * Called from the main loop rather than from the request handler: the handler
 * returns as soon as the scan is queued, which frees the host to stage the next
 * chunk over USB while this clocks the previous one. That overlap is the entire
 * point -- see the comment on jtag_queue_scan() in jtag.c.
 *
 * Must be called repeatedly, and does NOT clock a whole scan per call. The first
 * call hands the transfer to the DMAC and returns immediately, so tud_task() keeps
 * running while the bytes are on the wire; subsequent calls read one DMAC flag and
 * finish the scan once it retires.
 */
void jtag_scan_task(void);

/**
 * Blocks until any queued scan has finished clocking.
 *
 * Bounded by the work already queued -- at most JTAG_BUFFER_SIZE bytes at 12 MHz
 * SCK, about 700 us -- and not by a timeout, because completion is signalled by the
 * DMAC's own transfer-complete flag, which the SERCOM's clock guarantees will arrive.
 * A transfer that could not complete raises TERR, which also ends the wait. On return
 * the previously queued scan is complete and both buffers are idle.
 */
void jtag_scan_drain(void);

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
bool jtag_scan(uint32_t num_bits, bool advance_state, bool bitbang,
               bool discard_tdo);


/**
 * Performs a JTAG scan out of an explicitly named buffer.
 *
 * The source is no longer implied now that two transmit buffers alternate: the
 * overlapped path clocks the buffer filled one chunk ago, not the one being filled
 * now.
 */
bool jtag_scan_from(uint8_t *out_buffer, uint32_t num_bits, bool advance_state,
                    bool bitbang, bool discard_tdo);


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
 * Called once a SET_OUT_BUFFER's data has actually landed in the buffer.
 *
 * Returning from the setup handler does not mean the bytes arrived -- the USB DMA
 * engine was only handed the address. Dispatched from CONTROL_STAGE_DATA, the first
 * point at which the buffer contents are known good.
 */
bool handle_jtag_request_set_out_buffer_complete(uint8_t rhport, tusb_control_request_t const* request);


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
 *     wIndex: low byte -- chunk size in 64-BYTE UNITS (so 8 means 512 bytes);
 *             high byte -- SERCOM baud divider, or 0xFF to keep the JTAG default.
 *
 * Chunk was previously a raw byte count with "0 means full buffer", which broke
 * silently once the buffer exceeded 256 bytes: 512 truncated to 0, read as "full
 * buffer", and a 512 x 240 request clocked 245760 bytes in one uninterruptible
 * loop -- overrunning the host timeout and dropping the device off the bus.
 *
 * Total work is now bounded at 65536 bytes (about 44 ms of clocking at 12 MHz), so
 * a mis-sized request is refused rather than wedging the board.
 */
bool handle_jtag_benchmark(uint8_t rhport, tusb_control_request_t const* request);

#endif
