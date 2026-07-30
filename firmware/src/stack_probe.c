/**
 * Stack high-water measurement, by painting and reading back.
 *
 * The d11 reserves 1024 bytes of stack and has never measured how much is used,
 * so free RAM is margin over a guess rather than spare capacity. That blocks two
 * real changes: growing the console RX ring, and growing the JTAG chunk past 256
 * bytes -- one of the two levers on the USB overhead that dominates FPGA
 * configuration time.
 *
 * This makes the number a measurement.
 *
 * Method: fill the stack region with a known pattern early in main(), then read
 * back the lowest address still holding it. Everything below that mark has been
 * written by something, so the difference from _estack is the depth reached.
 *
 * Why not -fstack-usage: the firmware is built -flto=auto -flto-partition=one, so
 * LTO inlines across translation units and per-function frame sizes no longer
 * correspond to the frames in the final binary. The numbers come out individually
 * plausible and collectively wrong. Disabling LTO to obtain them is worse -- it
 * reclaims 2968 bytes on a part that is otherwise 568 bytes from its ROM ceiling.
 *
 * Why the firmware reports it rather than a debugger reading it: there is no SWD
 * debugger in this workflow. Reporting through a vendor request means the
 * measurement works on any board already running this firmware, with no extra
 * hardware.
 *
 * This file is part of Apollo.
 *
 * Copyright (c) 2026 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stddef.h>

#include "stack_probe.h"

// Provided by the linker script, which bounds the region explicitly.
extern uint32_t _sstack;
extern uint32_t _estack;

/**
 * The paint value.
 *
 * Not zero and not 0xff: .bss is already zeroed and erased flash reads as 0xff,
 * so either would be indistinguishable from memory that was never painted, and
 * the measurement would silently read as "nothing used" or "everything used".
 * 0xDEADBEEF is also unlikely to be written coincidentally by real code, which
 * matters because a single matching word is what the scan below stops on.
 */
#define STACK_PAINT 0xDEADBEEFul


void stack_probe_paint(void)
{
	// Paint from the bottom of the region up towards the current stack
	// pointer, and stop short of it.
	//
	// The live frame belongs to this function and its callers -- main() and
	// whatever called it -- and overwriting that is an immediate crash rather
	// than a measurement. `&here` is the address of a local, so it sits inside
	// the current frame, and stopping below it keeps the paint strictly in the
	// unused region.
	//
	// The consequence for accuracy is worth stating: whatever depth was already
	// in use at this point is invisible to the measurement, so the result is a
	// lower bound on total usage. Painting is done early in main() to keep that
	// unmeasured portion as small as possible.
	volatile uint32_t here;
	uint32_t *limit = (uint32_t *)((uintptr_t)&here - sizeof(uint32_t) * 4);

	for (uint32_t *word = &_sstack; word < limit; word++) {
		*word = STACK_PAINT;
	}
}


uint32_t stack_probe_high_water(void)
{
	// Scan up from the bottom for the first word still painted. Everything
	// below it has been written, so that address is the deepest point reached.
	//
	// Scanning up rather than down matters: the stack grows down on this part,
	// so the deepest excursion leaves its mark at the LOWEST address. Searching
	// from the top would find the boundary of the currently-live frame, which is
	// shallower and would understate usage.
	uint32_t *word = &_sstack;
	while (word < &_estack && *word != STACK_PAINT) {
		word++;
	}

	// Bytes between the deepest point reached and the top of the region.
	return (uint32_t)((uintptr_t)&_estack - (uintptr_t)word);
}


uint32_t stack_probe_size(void)
{
	return (uint32_t)((uintptr_t)&_estack - (uintptr_t)&_sstack);
}


bool stack_probe_overflowed(void)
{
	// No paint left at the very bottom means the deepest excursion reached or
	// passed _sstack. On a part with no MPU that is not a fault, it is silent
	// corruption of whatever .bss sits below -- so this is the difference
	// between "used a lot" and "already broke something".
	return _sstack != STACK_PAINT;
}
