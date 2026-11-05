/**
 * Where a scan's bit-banged tail stores the TDO it captures. Pure logic, host-testable.
 *
 * A scan that discards TDO has no destination. The tail used to store at jtag_in_buffer +
 * slow_offset regardless; with slow_offset up to the chunk length and jtag_in_buffer only the second
 * 512 bytes of the 1024-byte region, a final chunk over 513 bytes wrote past the region.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __JTAG_TAIL_H__
#define __JTAG_TAIL_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static inline uint8_t *jtag_tail_dest(bool discard_tdo, uint8_t *in_buffer, uint16_t offset)
{
	return discard_tdo ? NULL : in_buffer + offset;
}

#endif
