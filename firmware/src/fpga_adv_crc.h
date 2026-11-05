/*
 * CRC-8 for FPGA_ADV frames.
 *
 * Own header so a host test can include it without the hardware.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __FPGA_ADV_CRC_H__
#define __FPGA_ADV_CRC_H__

#include <stdint.h>

/**
 * CRC-8/ATM over a buffer: polynomial 0x07, init 0x00, no reflection.
 *
 * Matches the responder gateware. Bitwise: a 256-byte table costs more flash than it
 * saves on a link that moves 18 bytes at a time (the Makefile sets -fno-optimize-crc).
 */
static inline uint8_t fpga_adv_crc8(const uint8_t *data, uint8_t length)
{
	uint8_t crc = 0;
	for (uint8_t i = 0; i < length; i++) {
		crc ^= data[i];
		for (uint8_t bit = 0; bit < 8; bit++) {
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
			                   : (uint8_t)(crc << 1);
		}
	}
	return crc;
}

#endif
