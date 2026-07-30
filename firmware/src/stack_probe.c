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
 * This file is compiled WITHOUT LTO (see firmware/Makefile). With LTO enabled the
 * painting and scanning functions get inlined into callers in different objects,
 * and the two resolved `&_sstack` to different places: the probe reported
 * high_water == the full region size while simultaneously reporting no overflow,
 * which are contradictory. Excluding one small file costs nothing measurable and
 * keeps both halves agreeing on where the region is.
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
 * The region bounds, read through volatile pointers.
 *
 * This indirection is load-bearing rather than style. With LTO the painting and
 * scanning functions get inlined into callers in different objects, and the two
 * resolved `&_sstack` to different addresses -- the probe reported
 * high_water == the whole region while simultaneously reporting no overflow,
 * which cannot both be true. Forcing the address through a volatile makes every
 * caller read the same linker symbol at run time instead of letting the optimiser
 * bake in its own answer per copy.
 */
static uint8_t *const volatile stack_bottom = (uint8_t *)&_sstack;
static uint8_t *const volatile stack_top    = (uint8_t *)&_estack;

/**
 * The fill byte.
 *
 * A single repeated byte rather than a 32-bit word, following FreeRTOS's
 * prvTaskCheckFreeStackSpace: it makes the scan byte-granular, so alignment
 * cannot cause a miss, and it removes a failure mode the word version has. A word
 * scan that stops at the first match is defeated by ONE coincidental 0xDEADBEEF
 * anywhere in the used region -- it would report far less usage than really
 * occurred, which is the dangerous direction to be wrong in.
 *
 * 0xA5 is not 0x00 and not 0xFF: .bss is already zeroed and erased flash reads as
 * 0xFF, so either would be indistinguishable from memory that was never painted.
 */
#define STACK_FILL_BYTE 0xA5u


__attribute__((noinline)) void stack_probe_paint(void)
{
	// Paint from the bottom of the region up towards the current stack pointer,
	// stopping short of it. The live frame belongs to this function and its
	// callers, and overwriting that is a crash rather than a measurement.
	//
	// `&here` is a local, so it sits in the current frame. The margin below it
	// covers this function's own remaining frame plus anything the compiler
	// spills after this point.
	//
	// Consequence for accuracy, worth stating: whatever depth was in use when
	// this ran is invisible, so the result is a LOWER BOUND. Called as early in
	// main() as possible to keep that unmeasured portion small.
	volatile uint8_t here;
	uint8_t *bottom = stack_bottom;
	uint8_t *limit  = (uint8_t *)&here - 64;

	while (bottom < limit) {
		*bottom++ = (uint8_t)STACK_FILL_BYTE;
	}
}


__attribute__((noinline)) uint32_t stack_probe_high_water(void)
{
	// Count up from the bottom while the fill survives, exactly as FreeRTOS
	// does. The first byte that differs is the deepest point reached.
	//
	// Counting up rather than down is essential: the stack grows down, so the
	// deepest excursion leaves its mark at the LOWEST address. Searching from the
	// top would find the edge of the currently-live frame, which is shallower and
	// would understate usage.
	const uint8_t *byte = stack_bottom;
	const uint8_t *top  = stack_top;
	uint32_t unused = 0;

	while (byte < top && *byte == (uint8_t)STACK_FILL_BYTE) {
		byte++;
		unused++;
	}

	// Used = region size minus the run of surviving fill at the bottom.
	return (uint32_t)((uintptr_t)top - (uintptr_t)stack_bottom) - unused;
}


uint32_t stack_probe_size(void)
{
	return (uint32_t)((uintptr_t)stack_top - (uintptr_t)stack_bottom);
}


bool stack_probe_overflowed(void)
{
	// No paint left at the very bottom means the deepest excursion reached or
	// passed _sstack. On a part with no MPU that is not a fault, it is silent
	// corruption of whatever .bss sits below -- so this is the difference
	// between "used a lot" and "already broke something".
	return *stack_bottom != (uint8_t)STACK_FILL_BYTE;
}
