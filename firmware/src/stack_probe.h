/**
 * Stack high-water measurement.
 *
 * See stack_probe.c for the method and its limits. Summary: the stack region is
 * painted with a known pattern early in main(), and the lowest address still
 * holding it marks the deepest point execution reached.
 *
 * The result is a LOWER BOUND on usage, for two reasons worth knowing before
 * sizing anything from it. Whatever depth was already in use when painting ran is
 * invisible, and one run does not prove a worst case -- an interrupt arriving at
 * the deepest point of the deepest call path may simply not have happened. Size
 * buffers with margin, not to the measured figure.
 *
 * This file is part of Apollo.
 *
 * Copyright (c) 2026 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __STACK_PROBE_H__
#define __STACK_PROBE_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * Fills the unused stack region with a known pattern.
 *
 * Call once, as early in main() as practical: everything already on the stack at
 * that point is excluded from the measurement.
 */
void stack_probe_paint(void);

/**
 * Bytes between the deepest point reached and the top of the stack region.
 *
 * Meaningless unless stack_probe_paint() ran first, in which case it returns the
 * full region size -- indistinguishable from "used everything". Callers that
 * cannot guarantee painting should check stack_probe_overflowed() too.
 */
uint32_t stack_probe_high_water(void);

/** Total size of the stack region, from the linker symbols. */
uint32_t stack_probe_size(void);

/**
 * True if the deepest excursion reached the bottom of the region.
 *
 * This part has no MPU, so an overflow is not a fault -- it is silent corruption
 * of whatever sits below the stack. This distinguishes "used a lot" from "already
 * broke something", which a high-water figure alone cannot.
 */
bool stack_probe_overflowed(void);

#endif
