/*
 * SPI driver code.
 *
 * This file is part of Apollo.
 *
 * Copyright (c) 2020-2024 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <sam.h>

#include <hpl/pm/hpl_pm_base.h>
#include <hpl/gclk/hpl_gclk_base.h>
#include <hal/include/hal_gpio.h>

#include "spi.h"
#include "led.h"


// Hide the ugly Atmel Sercom object name.
typedef Sercom sercom_t;

/**
 * Returns the SERCOM object associated with the given target.
 */
static sercom_t *sercom_for_target(spi_target_t target)
{
	switch (target) {
		case SPI_FPGA_JTAG:  return SERCOM0; // Alternatively, SERCOM2.
		default:             return NULL;
	}
}


/**
 * Pinmux the relevent pins so the can be used for SERCOM SPI.
 */
static void _spi_configure_pinmux(spi_target_t target, bool use_for_spi)
{
	switch (target) {

		// FPGA JTAG connection -- configure PA08 (TDI), PA09 (TCK), and PA10 (TDO).
		case SPI_FPGA_JTAG:
			if (use_for_spi) {
				gpio_set_pin_function(PIN_PA14, MUX_PA14C_SERCOM0_PAD0);
				gpio_set_pin_function(PIN_PA15, MUX_PA15C_SERCOM0_PAD1);
				gpio_set_pin_function(PIN_PA10, MUX_PA10C_SERCOM0_PAD2);
			} else {
				gpio_set_pin_function(PIN_PA14, GPIO_PIN_FUNCTION_OFF);
				gpio_set_pin_function(PIN_PA15, GPIO_PIN_FUNCTION_OFF);
				gpio_set_pin_function(PIN_PA10, GPIO_PIN_FUNCTION_OFF);
			}
			break;

		default:
			// TODO
			break;
	}
}


/**
 * Configures the relevant SPI target's pins to be used for SPI.
 */
void spi_configure_pinmux(spi_target_t target)
{
	_spi_configure_pinmux(target, true);
}


/**
 * Returns the relevant SPI target's pins to being used for GPIO.
 */
void spi_release_pinmux(spi_target_t target)
{
	_spi_configure_pinmux(target, false);
}


/**
 * Configures the clocking for the relevant SERCOM peripheral.
 */
static void spi_set_up_clocking(spi_target_t target)
{
	switch (target) {

		case SPI_FPGA_JTAG:
			_pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
			_gclk_enable_channel(SERCOM0_GCLK_ID_CORE, GCLK_CLKCTRL_GEN_GCLK0_Val);
			break;

		case SPI_FPGA_DEBUG:
			_pm_enable_bus_clock(PM_BUS_APBC, SERCOM2);
			_gclk_enable_channel(SERCOM2_GCLK_ID_CORE, GCLK_CLKCTRL_GEN_GCLK0_Val);
			break;
	}

	// Wait for the clock to be ready.
	while(GCLK->STATUS.bit.SYNCBUSY);
}


/**
 * Configures the provided target to be used as an SPI port via the SERCOM.
 */
void spi_init(spi_target_t target, bool lsb_first, bool configure_pinmux, uint8_t baud_divider,
	 uint8_t clock_polarity, uint8_t clock_phase)
{
	volatile sercom_t *sercom = sercom_for_target(target);

	// Disable the SERCOM before configuring it, to 1) ensure we're not transacting
	// during configuration; and 2) as many of the registers are R/O when the SERCOM is enabled.
	while(sercom->SPI.SYNCBUSY.bit.ENABLE);
	sercom->SPI.CTRLA.bit.ENABLE = 0;

	// Software reset the SERCOM to restore initial values.
	while(sercom->SPI.SYNCBUSY.bit.SWRST);
	sercom->SPI.CTRLA.bit.SWRST = 1;

	// The SWRST bit becomes accessible again once the software reset is
	// complete -- we'll use this to wait for the reset to be finshed.
	while(sercom->SPI.SYNCBUSY.bit.SWRST);

	// Ensure we can work with the full SERCOM.
	while(sercom->SPI.SYNCBUSY.bit.SWRST || sercom->SPI.SYNCBUSY.bit.ENABLE);

	// Pinmux the relevant pins to be used for the SERCOM.
	if (configure_pinmux) {
		spi_configure_pinmux(target);
	}

	// Set up clocking for the SERCOM peripheral.
	spi_set_up_clocking(target);

	// Configure the SERCOM for SPI master mode.
	sercom->SPI.CTRLA.reg =
		SERCOM_SPI_CTRLA_MODE_SPI_MASTER  |  // SPI master
		SERCOM_SPI_CTRLA_DOPO(0)          |  // use our first pin as MOSI, and our second at SCK
		SERCOM_SPI_CTRLA_DIPO(2)          |  // use our third pin as MISO
		(lsb_first ? SERCOM_SPI_CTRLA_DORD : 0);   // SPI byte order

	// Set the clock polarity and phase.
	sercom->SPI.CTRLA.bit.CPOL = clock_polarity;
	sercom->SPI.CTRLA.bit.CPHA = clock_phase;

	// Use the SPI transceiver.
	while(sercom->SPI.SYNCBUSY.bit.CTRLB);
	sercom->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN;

	// Set the baud divider for the relevant channel.
	sercom->SPI.BAUD.reg = baud_divider;

	// Finally, enable the SPI controller.
	sercom->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;
	while(sercom->SPI.SYNCBUSY.bit.ENABLE);
}


/**
 * Synchronously send a single byte on the given SPI bus.
 * Does not manage the SSEL line.
 */
uint8_t spi_send_byte(spi_target_t port, uint8_t data)
{
	volatile sercom_t *sercom = sercom_for_target(port);

	// Send the relevant data...
	while(sercom->SPI.INTFLAG.bit.DRE == 0);
	sercom->SPI.DATA.reg = data;

	// ... and receive the response.
	while(sercom->SPI.INTFLAG.bit.RXC == 0);
	return (uint8_t)sercom->SPI.DATA.reg;
}


/**
 * Sends a block of data over the SPI bus.
 *
 * @param port The port on which to perform the SPI transaction.
 * @param data_to_send The data to be transferred over the SPI bus.
 * @param data_received Any data received during the SPI transaction.
 * @param length The total length of the data to be exchanged, in bytes.
 */
// data_received may be NULL, meaning "clock these bytes out and discard what comes
// back". SPI is inherently bidirectional -- every byte sent clocks one in -- so the
// hardware hands over a byte whether the caller wants it or not, and previously that
// byte was written to the caller's pointer unconditionally. Passing NULL wrote to
// address 0.
//
// This matters for FPGA configuration, which never reads TDO: the ECP5 self-validates
// by CRC, so ecp5.py passes ignore_response=True and the host never fetches the
// receive buffer. Discarding rather than storing means a 512-byte receive buffer does
// not have to exist for the write path.
void spi_send(spi_target_t port, void *data_to_send, void *data_received, size_t length)
{
	volatile sercom_t *sercom = sercom_for_target(port);
	uint8_t *to_send  = data_to_send;
	uint8_t *received = data_received;

	if (!length) {
		return;
	}

	// Pipelined transfer. The SERCOM has a separate TX data-register-empty (DRE)
	// flag and receive-complete (RXC) flag, so once a byte has moved from DATA into
	// the shift register we may queue the next one while the first is still on the
	// wire. The naive loop (write, wait RXC, write, ...) leaves SCK idle between
	// bytes because it will not queue byte N+1 until byte N has fully returned.
	//
	// Ordering is what makes this correct: we prime the first byte, then for each
	// subsequent byte we wait for DRE and queue it BEFORE draining the previous
	// byte's RXC. That keeps exactly one byte queued behind the one in flight, and
	// because every queued byte is matched by exactly one blocking RXC read, no
	// received byte is ever dropped. (An earlier version polled DRE and RXC in a
	// single racy loop and corrupted TDO; the ECP5 then reported an all-zero
	// status register. Hence the strict ordering here.)
	while (sercom->SPI.INTFLAG.bit.DRE == 0);
	sercom->SPI.DATA.reg = to_send[0];

	for (size_t i = 1; i < length; ++i) {
		// Queue the next byte as soon as the shift register has taken the last one.
		while (sercom->SPI.INTFLAG.bit.DRE == 0);
		sercom->SPI.DATA.reg = to_send[i];

		// Now collect the byte that finished while we were queueing.
		//
		// DATA must be read even when the caller does not want it: RXC stays set
		// until DATA is read, and the loop above waits on RXC, so skipping the
		// read hangs on the next iteration rather than merely discarding a byte.
		while (sercom->SPI.INTFLAG.bit.RXC == 0);
		uint8_t got = (uint8_t)sercom->SPI.DATA.reg;
		if (received) {
			received[i - 1] = got;
		}
	}

	// Drain the final byte, which has no successor to overlap with.
	while (sercom->SPI.INTFLAG.bit.RXC == 0);
	uint8_t last = (uint8_t)sercom->SPI.DATA.reg;
	if (received) {
		received[length - 1] = last;
	}
}
