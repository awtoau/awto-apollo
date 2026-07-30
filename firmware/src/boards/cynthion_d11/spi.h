/*
 * SPI driver code.
 *
 * This file is part of Apollo.
 *
 * Copyright (c) 2020-2024 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __SPI_H__
#define __SPI_H__

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum {
	 SPI_FPGA_JTAG,
	 SPI_FPGA_DEBUG
 } spi_target_t;


/**
 * Configures the relevant SPI target's pins to be used for SPI.
 */
void spi_configure_pinmux(spi_target_t target);


/**
 * Returns the relevant SPI target's pins to being used for GPIO.
 */
void spi_release_pinmux(spi_target_t target);


/**
 * Configures the provided target to be used as an SPI port via the SERCOM.
 */
void spi_init(spi_target_t target, bool lsb_first, bool configure_pinmux, uint8_t baud_divider,
	 uint8_t clock_polarity, uint8_t clock_phase);


/**
 * Synchronously send a single byte on the given SPI bus.
 * Does not manage the SSEL line.
 */
uint8_t spi_send_byte(spi_target_t port, uint8_t data);


/**
 * Sends a block of data over the SPI bus.
 * 
 * @param port The port on which to perform the SPI transaction.
 * @param data_to_send The data to be transferred over the SPI bus.
 * @param data_received Any data received during the SPI transaction.
 * @param length The total length of the data to be exchanged, in bytes.
 */
void spi_send(spi_target_t port, void *data_to_send, void *data_received, size_t length);


/**
 * Sends a block of data over the SPI bus WITHOUT waiting for it to finish.
 *
 * The polled spi_send() above spins on the SERCOM flags for the whole transfer --
 * about 700 us for 1024 bytes -- during which tud_task() cannot run and the device
 * NAKs the host. This hands the transfer to the DMAC and returns, so the main loop
 * keeps servicing USB while the bytes are on the wire.
 *
 * @return true if a DMA transfer was armed, in which case the caller MUST poll
 *         spi_send_done() before touching either buffer or starting another transfer.
 *         false if the transfer went down the polled path instead -- short transfers
 *         and non-JTAG targets -- and is therefore already complete on return.
 */
bool spi_send_async(spi_target_t port, void *data_to_send, void *data_received, size_t length);


/**
 * Whether the transfer armed by spi_send_async() has finished.
 *
 * A single register read, cheap enough to call on every pass of the main loop. Returns
 * true when nothing is outstanding, so it is safe to call unconditionally.
 */
bool spi_send_done(void);

#endif
