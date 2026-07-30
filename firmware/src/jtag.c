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
union comms_buffers {
	uint8_t jtag_tx[JTAG_BUFFER_SIZE];
	uint8_t console_ring[CONSOLE_RING_SIZE];
};

static union comms_buffers comms __attribute__((aligned(4)));

// The second transmit buffer, which is what makes the overlap possible.
//
// Deliberately NOT in the union above. The exclusivity argument for that union is
// the JTAG lock: console_task() returns immediately while the lock is held, so the
// ring is neither filled nor drained during a JTAG session. That argument is about
// JTAG versus console, and it still holds for this buffer -- but this buffer must
// also be live at the same time as comms.jtag_tx, which is the whole point of
// having it. Putting it in the same union would alias the two transmit buffers
// against each other and corrupt every overlapped transfer.
//
// Smaller than the primary buffer; see JTAG_ALT_BUFFER_SIZE in jtag.h for why 512
// costs nothing here.
static uint8_t jtag_tx_alt[JTAG_ALT_BUFFER_SIZE] __attribute__((aligned(4)));

// Second half of the same region. Valid only while TDO is being captured, which
// FLAG_DISCARD_TDO guarantees is never during a full-region write.
uint8_t *const jtag_in_buffer  = comms.jtag_tx + JTAG_READ_BUFFER_SIZE;
uint8_t *const console_rx_ring = comms.console_ring;


// Which buffer the host is currently staging into. Alternates on every scan, so
// that the scan just queued clocks out of one while the next SET_OUT_BUFFER fills
// the other.
//
// `volatile` because handle_jtag_request_set_out_buffer() runs from USB interrupt
// context via tud_task() while jtag_scan_task() runs from the main loop, and both
// read it. The SAMD11 is single-core with no store buffer, so volatile is
// sufficient here -- there is no reordering to fence against, only a compiler that
// would otherwise cache the value in a register across a scan.
static volatile bool fill_is_alt = false;

// A scan the host has asked for but which has not been clocked yet.
//
// The request handler records the parameters and returns immediately; the main
// loop notices `pending` and does the clocking. Everything the scan needs must be
// captured here, because by the time it runs the control request that described it
// is long gone.
// Field order is for packing: the uint16_t leads so its 2-byte alignment does not
// insert a pad after a leading bool, which is what a natural reading order costs.
//
// There is no `discard_tdo` field. Only discarding scans are ever deferred -- a
// capturing scan is run inline by handle_jtag_request_scan(), because the host's very
// next request reads TDO back and a deferred scan would answer it with the previous
// scan's data. So the deferred path's discard_tdo is a constant true, and storing it
// would only invite someone to set it to something else.
static struct {
	// wValue is 16 bits on the wire, so a wider field could not hold a larger request
	// even in principle -- and the buffer bound is 1024 bytes regardless.
	uint16_t num_bits;
	volatile bool     pending;
	bool     advance_state;
	bool     bitbang;
	// Which buffer this scan clocks OUT of. Latched at queue time rather than
	// derived when it runs: fill_is_alt has already flipped by then, so reading it
	// at run time would clock the buffer the host is currently filling.
	bool     from_alt;
	// Whether the scan has been armed on the SERCOM but not yet finished.
	// Distinguishes "the host asked and nothing has happened yet" from "the DMAC is
	// clocking it right now", which the task needs in order to know whether to arm or
	// to poll.
	bool     started;
} queued_scan;

// Whether the buffer currently being filled has had data land in it.
//
// Set at CONTROL_STAGE_DATA, which is the first moment the DMA engine's write is
// known complete; cleared when a scan consumes it. A SCAN that arrives without this
// set is scanning stale buffer contents, which on the configuration path means a
// corrupt bitstream -- so it is worth being able to tell the two apart rather than
// assuming USB always delivers the two requests in order.
static volatile bool     staged_valid = false;
static volatile uint16_t staged_bytes = 0;


/**
 * The buffer the host is currently staging into.
 */
uint8_t *jtag_fill_buffer(void)
{
	return fill_is_alt ? jtag_tx_alt : comms.jtag_tx;
}


/**
 * How much room the buffer currently being filled has.
 *
 * The two buffers differ in size, so a bound taken from JTAG_BUFFER_SIZE would
 * overrun the smaller one by 512 bytes. This is the only correct bound for a
 * SET_OUT_BUFFER, and getting it wrong writes past the end of .bss on a part with
 * no MPU to catch it.
 */
static uint16_t jtag_fill_capacity(void)
{
	return fill_is_alt ? JTAG_ALT_BUFFER_SIZE : JTAG_BUFFER_SIZE;
}


bool jtag_scan_pending(void)
{
	return queued_scan.pending;
}


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
	// Clocks out of whichever buffer is being filled. This is the path fpga.c and
	// the benchmark take: they stage a byte and scan it immediately, with no
	// pipelining, so "the buffer being filled" is also the one to send.
	return jtag_scan_from(jtag_fill_buffer(), num_bits, advance_state, bitbang,
	                      discard_tdo);
}


// The bit-level tail of a scan whose bulk is currently being clocked by DMA.
//
// jtag_scan_start() arms the DMA and returns, so the remaining work -- releasing the
// pinmux and bitbanging any leftover bits -- has to survive until a later pass of the
// main loop sees the transfer finish. Captured here rather than recomputed, because
// the buffer alternation means the source pointer is not derivable after the fact.
// Field order is chosen for packing, not for reading order: the pointer must be
// 4-aligned, so the four sub-word fields are grouped ahead of it and fill exactly one
// word. Put the uint16_t first, ahead of the two bytes, or its own 2-byte alignment
// inserts a pad and the struct rounds up to 12 -- which is what a natural reading order
// produces, and on a part with a few hundred spare bytes of RAM those four matter.
//
// No "is the DMA running" flag of its own: spi_send_done() already tracks that, and
// returns true when nothing is outstanding, so asking it unconditionally is correct for
// both the DMA'd and the polled-fallback case.
static struct {
	// How far into jtag_in_buffer the tail's received bits go -- the same value as the
	// bulk byte count, kept as an offset because that is its only use. Bounded by
	// JTAG_BUFFER_SIZE, so 16 bits is ample.
	uint16_t       slow_offset;
	// At most 8 bits, so one byte.
	uint8_t        slow_bits;
	bool           advance_state;
	// Where the leftover bits come from: the byte just past the DMA'd block, in
	// whichever of the two transmit buffers this scan came from. Stored already-offset
	// rather than as a buffer plus a bulk length, so this is one pointer instead of a
	// pointer plus a count.
	uint8_t       *slow_source;
} scan_tail;


/**
 * Starts a scan, handing the bulk of it to DMA and returning before it completes.
 *
 * The caller must drive jtag_scan_finish_step() until it reports done before touching
 * either buffer or starting another scan.
 *
 * @return true if the scan was started; false if the request was invalid, in which
 *         case nothing was armed and no tail is owed.
 */
static bool jtag_scan_start(uint8_t *out_buffer, uint32_t num_bits,
                            bool advance_state, bool bitbang, bool discard_tdo)
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
	//
	// Bounded by the size of the buffer actually being clocked, which the two
	// buffers no longer agree on: the alternate one is 512 bytes. A bound of
	// JTAG_BUFFER_SIZE would pass a 1024-byte scan out of a 512-byte buffer and
	// clock 512 bytes of whatever follows it in .bss.
	size_t capacity = (out_buffer == jtag_tx_alt) ? JTAG_ALT_BUFFER_SIZE
	                                              : JTAG_BUFFER_SIZE;
	if (bytes_to_send_bulk > capacity) {
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

	scan_tail.slow_source   = out_buffer + bytes_to_send_bulk;
	scan_tail.slow_offset   = bytes_to_send_bulk;
	scan_tail.slow_bits     = bits_to_send_slow;
	scan_tail.advance_state = advance_state;

	// Switch to SPI mode, and hand the bulk of the transfer to the DMAC.
	if (bytes_to_send_bulk) {
		spi_configure_pinmux(SPI_FPGA_JTAG);
		// NULL receive means "clock these out and drop what comes back", so the
		// receive buffer is untouched on the write path.
		//
		// The return value is discarded: it distinguishes "armed, poll for it" from
		// "already done on the polled fallback", and spi_send_done() answers both.
		(void)spi_send_async(SPI_FPGA_JTAG, out_buffer,
		                     discard_tdo ? NULL : jtag_in_buffer,
		                     bytes_to_send_bulk);
	}

	return true;
}


/**
 * Completes a scan started by jtag_scan_start(), if the DMA has finished.
 *
 * @return true when the scan is fully complete -- including the bitbanged tail -- and
 *         false while the DMA still has bytes on the wire.
 *
 * Not a wait: one register read, then either the tail work or an immediate return. The
 * caller decides whether to spin (jtag_scan_drain()) or come back next loop pass
 * (jtag_scan_task()).
 */
static bool jtag_scan_finish_step(void)
{
	if (!spi_send_done()) {
		return false;
	}

	// Switch back to GPIO mode, and send the remainder using the slow method. This
	// must happen AFTER the DMA has drained: releasing the pinmux mid-transfer would
	// pull TCK away from the SERCOM with bytes still queued.
	spi_release_pinmux(SPI_FPGA_JTAG);
	if (scan_tail.slow_bits) {
		jtag_tap_shift(scan_tail.slow_source,
		               jtag_in_buffer + scan_tail.slow_offset,
		               scan_tail.slow_bits, scan_tail.advance_state);
		// Cleared so a second finish step cannot re-shift these bits.
		scan_tail.slow_bits = 0;
	}

	return true;
}


/**
 * Performs a JTAG scan out of an explicitly named buffer, returning only once the
 * whole scan has been clocked.
 *
 * Split out from jtag_scan() because with two buffers alternating, the source is no
 * longer implied. The overlapped path must clock the buffer that was filled one
 * chunk ago, not the one being filled now, and a function that consulted
 * fill_is_alt itself could not express that.
 *
 * Implemented as start-then-spin over the same two steps the deferred path uses, so
 * there is one copy of the bit splitting and the buffer bounds rather than two that
 * can drift apart. The spin is bounded by the DMAC's transfer-complete flag, which
 * the SERCOM's own clock guarantees will arrive -- ~700 us for a full 1024 bytes --
 * and a transfer that could not complete raises TERR, which also ends the spin.
 */
bool jtag_scan_from(uint8_t *out_buffer, uint32_t num_bits, bool advance_state,
                    bool bitbang, bool discard_tdo)
{
	if (!jtag_scan_start(out_buffer, num_bits, advance_state, bitbang,
	                     discard_tdo)) {
		return false;
	}

	while (!jtag_scan_finish_step());

	return true;
}


/**
 * Simple request that clears the JTAG out buffer.
 */
bool handle_jtag_request_clear_out_buffer(uint8_t rhport, tusb_control_request_t const* request)
{
	// Any queued scan is clocking out of one of these buffers, so zeroing them
	// underneath it would corrupt the transfer in flight.
	jtag_scan_drain();

	memset(comms.jtag_tx, 0, JTAG_BUFFER_SIZE);
	memset(jtag_tx_alt, 0, JTAG_ALT_BUFFER_SIZE);
	return tud_control_xfer(rhport, request, NULL, 0);
}


/**
 * Simple request that sets the JTAG out buffer's contents.
 * This is used to set the data to be transmitted during the next scan.
 */
bool handle_jtag_request_set_out_buffer(uint8_t rhport, tusb_control_request_t const* request)
{
	// This is where the pipeline synchronises, and it is why the host needs no
	// explicit "is the scan done" request.
	//
	// A poll-per-chunk would cost about 145 us of control transfer each way and
	// would eat most of the 107 ms the overlap wins. Instead the host simply stages
	// the next chunk, and this request blocks -- if it must -- until the buffer it
	// is about to fill is free. In the steady state it must NOT: the previous scan
	// is clocking the OTHER buffer, so this returns immediately and the fill
	// proceeds concurrently with that clocking, which is the overlap.
	//
	// It blocks only when the host runs more than one chunk ahead, which the
	// alternation makes impossible for a host that scans between every stage.
	// A chunk too large for the alternate buffer goes in the primary one instead,
	// rather than being refused.
	//
	// The alternation must NOT be a contract the host has to track. It cannot
	// reliably: several requests drain a queued scan, any of them shifts the parity,
	// and a host that guessed wrong would get a stall for a chunk size the firmware
	// is perfectly able to hold. Worse, the parity used to survive across sessions,
	// so a fresh connection's first 1024-byte chunk failed or succeeded depending on
	// what the previous process had done -- observed as exactly that, a pipe error on
	// the first request of a new process.
	//
	// Sizing the decision on the request instead makes it self-correcting: the host
	// sends whatever chunk it likes and the firmware puts it somewhere it fits. The
	// cost is a drain when the large buffer is the one busy, which the host avoids by
	// alternating sizes -- an optimisation on its side rather than a requirement.
	if (request->wLength > JTAG_ALT_BUFFER_SIZE && fill_is_alt) {
		fill_is_alt = false;
	}

	// If we've been handed too much data for either buffer, stall.
	if (request->wLength > JTAG_BUFFER_SIZE) {
		return false;
	}

	// Wait only if the buffer about to be filled is the one being clocked. In the
	// steady state it is not -- the queued scan is on the other buffer -- so this
	// returns immediately and the fill overlaps the clocking, which is the overlap
	// this whole change exists to create.
	if (queued_scan.pending && fill_is_alt == queued_scan.from_alt) {
		jtag_scan_drain();
	}

	uint8_t *fill = jtag_fill_buffer();

	// HACK: check the buffer for commands that affect the FPGA configuration state.
	//
	// Reads the PREVIOUS contents of the buffer, which is what upstream did and is
	// preserved deliberately: the new data has not arrived yet at this point. The
	// USB DMA engine is handed the address by tud_control_xfer() below and fills it
	// asynchronously, so inspecting it here sees the last chunk, not this one. That
	// oddity predates this change; noted so it is not mistaken for a new bug.
    if (request->wLength == 1) {
		if (fill[0] == ISC_ENABLE) {
			fpga_set_online(false);
		}
		if (fill[0] == ISC_DISABLE) {
			fpga_set_online(true);
		}
    }

	// Hand the buffer to the USB DMA engine, which fills it without the CPU.
	//
	// That asynchrony is what makes the overlap possible rather than being an
	// obstacle to it: dcd_samd.c sets bank->ADDR.reg to this pointer and the
	// peripheral moves the bytes in on its own, so the CPU is free to return here
	// and then arm the DMAC to clock the other buffer. The fill completing is
	// observed at CONTROL_STAGE_DATA -- see handle_jtag_request_set_out_buffer_complete().
	return tud_control_xfer(rhport, request, fill, request->wLength);
}


/**
 * Called once a SET_OUT_BUFFER's data has actually landed in the buffer.
 *
 * Returning from the setup handler above does NOT mean the bytes arrived -- the DMA
 * engine was merely given the address. This is dispatched from CONTROL_STAGE_DATA,
 * which is the first point at which the buffer's contents are known good, and so it
 * is the only safe place to declare the staged chunk ready to clock.
 */
bool handle_jtag_request_set_out_buffer_complete(uint8_t rhport, tusb_control_request_t const* request)
{
	(void)rhport;

	// The number of bytes now sitting in the buffer that was being filled. The
	// subsequent SCAN carries its own bit count, so this is not used to size the
	// scan; it exists so that a scan arriving for a buffer that was never filled
	// can be distinguished from one that was.
	staged_bytes = request->wLength;
	staged_valid = true;
	return true;
}


/**
 * Simple request that gets the JTAG in buffer's contents.
 * This is used to fetch the data received during the last scan.
 */
bool handle_jtag_request_get_in_buffer(uint8_t rhport, tusb_control_request_t const* request)
{
	uint16_t length = request->wLength;

	// A queued scan may still be clocking, and although a deferred scan never
	// captures TDO, letting it finish first keeps the ordering the host expects:
	// everything it asked for before this read has happened by the time the read
	// returns.
	jtag_scan_drain();

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
	bool discard_tdo = request->wIndex & FLAG_DISCARD_TDO;

	// A scan that CAPTURES TDO cannot be deferred, and this is the reason.
	//
	// The host's next action after such a scan is GET_IN_BUFFER, to collect what was
	// captured. If the scan had not run yet, that request would return the previous
	// scan's data -- silently, and looking entirely plausible. Deferring is only
	// safe when nothing observes the result until the next synchronisation point,
	// which is true for the write path and false for the read path.
	//
	// The write path is the one that matters for speed: a configure is hundreds of
	// discarding chunks and a handful of capturing reads.
	if (!discard_tdo) {
		// Anything already queued must finish first, so the two scans reach the TAP
		// in the order the host asked for.
		jtag_scan_drain();

		if (jtag_scan_from(jtag_fill_buffer(), request->wValue,
		                   request->wIndex & FLAG_ADVANCE_STATE,
		                   request->wIndex & FLAG_FORCE_BITBANG, false)) {
			staged_valid = false;
			return tud_control_xfer(rhport, request, NULL, 0);
		}
		return false;
	}

	// Only one scan may be queued at a time. The host reaches this only by issuing
	// two SCANs with no SET_OUT_BUFFER between them, which would in any case scan
	// the same bytes twice; draining keeps the queue depth at one rather than
	// needing a ring.
	if (queued_scan.pending) {
		jtag_scan_drain();
	}

	// Validate now rather than in the task. The task runs from the main loop with no
	// control transfer to fail, so a bad request discovered there could only be
	// dropped silently; here it can still stall and tell the host.
	//
	// Against the buffer the data was actually staged into, which is the one
	// SET_OUT_BUFFER chose by size. Deriving the capacity from fill_is_alt is the
	// same thing, since that request may have flipped it, but the ordering matters:
	// this must run AFTER the stage, which it always does -- the host cannot scan a
	// chunk it has not sent.
	size_t bytes = request->wValue / 8;
	if (bytes > jtag_fill_capacity() || request->wValue == 0) {
		return false;
	}

	queued_scan.num_bits      = request->wValue;
	queued_scan.advance_state = request->wIndex & FLAG_ADVANCE_STATE;
	queued_scan.bitbang       = request->wIndex & FLAG_FORCE_BITBANG;
	// Latch which buffer to clock BEFORE flipping, so the scan reads the buffer the
	// host just filled rather than the one it is about to fill next.
	queued_scan.from_alt      = fill_is_alt;
	queued_scan.pending       = true;

	// Hand the other buffer to the next SET_OUT_BUFFER. From here the host may
	// stage into it while jtag_scan_task() clocks the one above.
	fill_is_alt  = !fill_is_alt;
	staged_valid = false;

	// Complete immediately. This is the change that creates the overlap: the host is
	// released now rather than after the clocking, so its next SET_OUT_BUFFER
	// travels while the SERCOM is still busy.
	return tud_control_xfer(rhport, request, NULL, 0);
}


/**
 * Clocks out a queued scan. Called from the main loop.
 */
void jtag_scan_task(void)
{
	if (!queued_scan.pending) {
		return;
	}

	// First visit for this scan: arm it and get straight back to the main loop, so
	// tud_task() runs while the DMAC clocks. That return is the whole point -- the
	// old code spun here for the entire transfer and the device NAKed throughout.
	if (!queued_scan.started) {
		if (!jtag_scan_start(queued_scan.from_alt ? jtag_tx_alt : comms.jtag_tx,
		                     queued_scan.num_bits, queued_scan.advance_state,
		                     queued_scan.bitbang, true)) {
			// Invalid request, already validated at queue time, so this should be
			// unreachable. Drop it rather than retrying forever: the main loop has
			// no control transfer left to stall.
			queued_scan.pending = false;
			return;
		}
		queued_scan.started = true;
	}

	// Subsequent visits: one register read to ask whether the wire is clear. Cheap
	// enough to do on every loop pass, which is what replaces the 700 us spin.
	if (!jtag_scan_finish_step()) {
		return;
	}

	queued_scan.started = false;
	// Cleared last, so that a SET_OUT_BUFFER arriving during the clocking above sees
	// the scan as still pending and waits if it targets this buffer.
	queued_scan.pending = false;
}


/**
 * Blocks until any queued scan has finished clocking.
 */
void jtag_scan_drain(void)
{
	// Not a wait on a duration, and deliberately not a timeout. The only work that
	// can be outstanding is one transfer of at most JTAG_BUFFER_SIZE bytes -- about
	// 700 us at 12 MHz SCK for a full 1024-byte buffer -- whose completion is
	// signalled by the DMAC's own transfer-complete flag (or by the SERCOM's DRE/RXC
	// flags on the polled fallback). There is nothing to time out ON: the hardware
	// clock guarantees the flag arrives, and a transfer that could not complete
	// raises TERR, which spi_send_done() also reports as finished.
	while (queued_scan.pending) {
		jtag_scan_task();
	}
}


/**
 * Runs the JTAG clock for a specified amount of ticks.
 * Arguments:
 *     wValue: The number of clock cycles to run.
 */
bool handle_jtag_run_clock(uint8_t rhport, tusb_control_request_t const* request)
{
	// Clocking the TAP while a queued scan has not yet been shifted would insert
	// these cycles BEFORE data the host sent first, corrupting the sequence.
	jtag_scan_drain();

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
	// A state change must not overtake a scan the host queued earlier -- moving the
	// TAP out of SHIFT-DR before the data is shifted would lose the whole chunk.
	// This is the request that ends every configure, so it is also the point at
	// which the final queued chunk is guaranteed to have been clocked.
	jtag_scan_drain();

	jtag_go_to_state(request->wValue);
	return tud_control_xfer(rhport, request, NULL, 0);
}


/**
 * Reads the current JTAG TAP state. Mostly intended as a debug aid.
 */
bool handle_jtag_get_state(uint8_t rhport, tusb_control_request_t const* request)
{
	static uint8_t jtag_state;

	// Report the state after any queued scan, not before it: a scan with
	// FLAG_ADVANCE_STATE moves the TAP, so answering early would describe a state
	// the host has already asked to leave.
	jtag_scan_drain();

	jtag_state = jtag_current_state();
	return tud_control_xfer(rhport, request, &jtag_state, sizeof(jtag_state));
}


/**
 * Initializes JTAG communication.
 */
bool handle_jtag_start(uint8_t rhport, tusb_control_request_t const* request)
{
	// No LED pattern call here: led_task() reads apollo_mode_jtag_active() and
	// apollo_mode_programming_active() directly, and both are maintained for the
	// whole session. See led.c.
	//
	// wValue carried nothing before, so it now takes an optional LED override --
	// bit 5 arms it, bits 0-4 are the LEDs. Zero (what every existing host sends)
	// means "report live state", so this is backward compatible by construction.
	led_set_override((uint8_t)(request->wValue & 0xffu));

	// Reset the buffer alternation for the new session.
	//
	// Without this the parity survives across host sessions, because these are
	// statics and nothing else clears them. The host then has no way to know which
	// buffer it is filling: a fresh connection whose first chunk is 1024 bytes gets
	// a stall if the previous session happened to leave the 512-byte buffer in the
	// fill position. That is exactly the failure seen in testing -- the first
	// SET_OUT_BUFFER of a new process failing with a pipe error, depending on what
	// the last one did.
	//
	// JTAG_START is the right place: the host issues exactly one per session, and
	// the alternation is meaningless outside a session.
	//
	// Drained rather than simply discarded. The scan is now clocked by the DMAC, so a
	// pending scan may have channels armed and writing into one of these buffers; just
	// clearing the flag would leave that transfer running into a buffer the new session
	// believes it owns. Letting it finish costs at most one 1024-byte transfer.
	jtag_scan_drain();

	fill_is_alt         = false;
	staged_valid        = false;
	staged_bytes        = 0;

	jtag_init();

	return tud_control_xfer(rhport, request, NULL, 0);
}


/**
 * De-initializes JTAG communcation; and stops driving the scan chain.
 */
bool handle_jtag_stop(uint8_t rhport, tusb_control_request_t const* request)
{
	// Nothing to reset: the JTAG LED follows apollo_mode_jtag_active(), which
	// jtag_deinit() clears below.

	// Stop driving the chain only once the last queued scan has been shifted out.
	// jtag_deinit() releases the JTAG lock and the pins; doing that with a chunk
	// still queued would drop it silently at the very end of a configure.
	jtag_scan_drain();

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

	// Chunk size in 64-byte units, and the SERCOM divider, packed into wIndex.
	//
	// It was previously a raw byte count in the low byte with "0 means full
	// buffer". That silently broke once the buffer exceeded 256: asking for 512
	// truncated to 0, which the firmware read as "use the whole buffer" -- so a
	// request for 512 x 240 clocked 1024 x 240 = 245760 bytes in one
	// uninterruptible loop, overran the host's control-transfer timeout, and took
	// the device off the bus entirely. A physical replug was needed.
	//
	// 64-byte units reach 16320 bytes in a byte, which is far beyond any buffer
	// this part will hold, and there is no magic zero to misread. 0 is simply
	// invalid.
	uint16_t chunk   = (uint16_t)(request->wIndex & 0xFF) * 64u;
	uint8_t  divider = (request->wIndex >> 8) & 0xFF;

	if (chunk == 0 || chunk > JTAG_BUFFER_SIZE) {
		return false;
	}

	// Bound the total work, so a large repeats cannot exceed the host's
	// control-transfer timeout the way the truncation bug did. At 12 MHz SCK,
	// 65536 bytes is about 44 ms of clocking -- comfortably inside a 500 ms
	// timeout, and far more than any single measurement needs.
	if ((uint32_t)chunk * repeats > 65536u) {
		return false;
	}
	if (repeats == 0) {
		return false;
	}

	// This harness measures the SERCOM in isolation, so it always uses the primary
	// buffer and never the alternate one. Taken directly rather than via
	// jtag_fill_buffer() so that a benchmark result cannot depend on which buffer
	// the pipeline happened to leave in the fill position -- the two differ in size,
	// and a run that silently used the 512-byte buffer would refuse larger chunks
	// for reasons having nothing to do with the clocking being measured.
	uint8_t *source = comms.jtag_tx;

	// Any queued scan clocks out of one of these buffers; let it finish before
	// overwriting them with the test pattern.
	jtag_scan_drain();

	// Fill the source buffer with a rolling pattern. Done here rather than by
	// the host so that no bulk data crosses USB for this measurement at all.
	for (unsigned i = 0; i < chunk; ++i) {
		source[i] = (uint8_t)(i * 7 + 1);
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
		spi_send(SPI_FPGA_JTAG, source, jtag_in_buffer, chunk);

		// Reading every byte also guarantees the compiler cannot discard the
		// transfer as unused work.
		for (unsigned i = 0; i < chunk; ++i) {
			// SPI here is LSB-first, so the one-bit BYPASS delay shifts each
			// byte right, with the previous byte's LSB arriving in bit 7.
			uint8_t sent     = source[i];
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
	result[10] = source[0];
	result[11] = jtag_in_buffer[0];

	return tud_control_xfer(rhport, request, result, sizeof(result));
}
