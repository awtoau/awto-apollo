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
 * DMA support for the FPGA JTAG SERCOM.
 *
 * The point is NOT per-byte speed. The pipelined polled loop below already runs at
 * about 0.77 us/byte against a 0.67 us/byte wire, so there is no CPU bottleneck left
 * in the clocking itself -- an earlier synchronous DMA attempt measured slower, and
 * was retired for exactly that reason.
 *
 * What DMA buys is that the CPU is not *in* the transfer. The polled loop spins on
 * SERCOM flags for the whole chunk -- about 700 us at 1024 bytes -- and `tud_task()`
 * cannot run for any of it, so the device NAKs the host's next request until the main
 * loop comes round. Arming the DMAC and returning lets the main loop keep servicing
 * USB while the bytes are on the wire, which is where the measured ~98 us per
 * transaction of NAK time lives.
 *
 * Two channels are needed because SPI is full duplex: every byte clocked out clocks
 * one in, and if nothing drains DATA the RX path overruns and the transfer stalls. So
 * TX is fed from the caller's buffer and RX is drained into it, both by DMA.
 *
 * The RX channel is given the higher priority (lower channel number wins at equal
 * priority level on this part) so a received byte is always collected before the next
 * TX beat is serviced.
 */
#define SPI_DMA_CHANNEL_RX  0
#define SPI_DMA_CHANNEL_TX  1

// Transfers shorter than this stay on the polled path: DMA setup is a fixed cost of a
// few dozen register writes, which is not worth paying for a handful of bytes, and a
// short transfer does not block tud_task() long enough to matter.
#define SPI_DMA_MIN_LENGTH  8

// Descriptor and write-back sections, as one allocation. The DMAC requires both to be
// 128-bit aligned and indexes each by channel number, so each needs one 16-byte slot
// per channel we use -- 64 bytes in total.
//
// One array rather than two because two separately 16-byte-aligned arrays make the
// linker pad between them; a single aligned block of four slots is the same 64 bytes
// with nothing wasted, and the DMAC neither knows nor cares that BASEADDR and WRBADDR
// point into the same object. The first two slots are the descriptors, indexed by
// channel; the last two are the write-back area, which only the DMAC ever touches.
static DmacDescriptor spi_dma_slots[4] __attribute__((aligned(16)));

#define spi_dma_descriptors  (spi_dma_slots)
#define spi_dma_writeback    (spi_dma_slots + 2)

// A sink for transfers whose response the caller does not want -- the NULL
// data_received case, which is the whole FPGA write path. Keeping the destination
// address fixed (DSTINC clear) means a single byte suffices, and nothing has to
// pretend a receive buffer exists.
static uint8_t spi_dma_scratch;

static bool spi_dma_ready = false;

// Whether a DMA transfer armed by spi_send_async() is still on the wire.
static bool spi_dma_active = false;


/**
 * Brings up the DMAC for SERCOM SPI transfers. Idempotent.
 */
static void spi_dma_init(void)
{
	if (spi_dma_ready) {
		return;
	}

	// The DMAC needs its bus clocks running before any register will stick.
	// Unlike the SERCOMs it lives on the AHB as well as the APB.
	PM->AHBMASK.reg  |= PM_AHBMASK_DMAC;
	PM->APBBMASK.reg |= PM_APBBMASK_DMAC;

	// No SWRST and no memset of the descriptor sections. Both would be dead work: the
	// DMAC is untouched by anything else in this firmware and comes out of reset
	// disabled, and the descriptor arrays live in .bss, which the startup code has
	// already zeroed. Every descriptor field this driver depends on is written
	// explicitly in spi_dma_arm() before each transfer.
	DMAC->BASEADDR.reg = (uint32_t)spi_dma_descriptors;
	DMAC->WRBADDR.reg  = (uint32_t)spi_dma_writeback;

	// Enable the DMAC and all four priority levels.
	DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN0 | DMAC_CTRL_LVLEN1 |
			 DMAC_CTRL_LVLEN2 | DMAC_CTRL_LVLEN3;

	spi_dma_ready = true;
}


/**
 * Arms a DMA transfer and returns immediately, without waiting for it.
 *
 * See spi_send_async() for the contract; this is the part that touches the DMAC.
 */
static bool spi_dma_arm(volatile sercom_t *sercom, const uint8_t *to_send,
		uint8_t *received, size_t length)
{
	spi_dma_init();

	// Drain any byte left sitting in the receive register, so the first thing
	// the RX channel stores is the response to our first transmitted byte
	// rather than a stale leftover.
	while (sercom->SPI.INTFLAG.bit.RXC) {
		(void)sercom->SPI.DATA.reg;
	}

	uint32_t data_address = (uint32_t)&sercom->SPI.DATA.reg;

	// Receive descriptor: SERCOM DATA -> caller's buffer, or into the scratch byte
	// when the caller passed NULL. RX must run either way: RXC stays set until DATA
	// is read, so a transfer with nothing draining DATA stalls rather than merely
	// discarding bytes.
	//
	// DSTADDR must point one beat PAST the end of the destination block: the
	// DMAC decrements the address before each write when DSTINC is set. Getting
	// this wrong silently writes one byte outside the buffer. With DSTINC clear
	// -- the scratch case -- the address is used as-is and must NOT be offset.
	DmacDescriptor *rx = &spi_dma_descriptors[SPI_DMA_CHANNEL_RX];
	rx->BTCTRL.reg  = DMAC_BTCTRL_VALID | DMAC_BTCTRL_BEATSIZE_BYTE |
			  (received ? DMAC_BTCTRL_DSTINC : 0);
	rx->BTCNT.reg   = length;
	rx->SRCADDR.reg = data_address;
	rx->DSTADDR.reg = received ? (uint32_t)(received + length)
				   : (uint32_t)&spi_dma_scratch;

	// Transmit descriptor: caller's buffer -> SERCOM DATA. Same end-address
	// rule applies to SRCADDR when SRCINC is set.
	DmacDescriptor *tx = &spi_dma_descriptors[SPI_DMA_CHANNEL_TX];
	tx->BTCTRL.reg  = DMAC_BTCTRL_VALID | DMAC_BTCTRL_BEATSIZE_BYTE |
			  DMAC_BTCTRL_SRCINC;
	tx->BTCNT.reg   = length;
	tx->SRCADDR.reg = (uint32_t)(to_send + length);
	tx->DSTADDR.reg = data_address;

	// DESCADDR stays zero for both -- single block, no linked list. Never written
	// non-zero, so the .bss zero it starts with is the value it keeps.

	// Configure the receive channel first, and start it before the transmit
	// channel: it must already be armed when the first byte completes, or that
	// byte is lost and the transfer deadlocks waiting for a beat that never
	// arrives. The loop below runs RX (channel 0) then TX (channel 1) for exactly
	// that reason, so the channel numbering is load-bearing and not cosmetic.
	//
	// RX also takes the higher priority level, so a received byte is always
	// collected before the next TX beat is serviced.
	//
	// Note the CHID indirection -- channel registers are a single window
	// selected by CHID, so this sequence is not re-entrant and must not be
	// touched from an interrupt. That is why completion is polled from the main
	// loop rather than handled in a DMA ISR: an ISR that re-pointed CHID would
	// corrupt whichever setup sequence it interrupted.
	static const uint32_t channel_ctrlb[2] = {
		[SPI_DMA_CHANNEL_RX] = DMAC_CHCTRLB_TRIGSRC(SERCOM0_DMAC_ID_RX) |
				       DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_LVL(3),
		[SPI_DMA_CHANNEL_TX] = DMAC_CHCTRLB_TRIGSRC(SERCOM0_DMAC_ID_TX) |
				       DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_LVL(0),
	};

	for (unsigned channel = 0; channel < 2; ++channel) {
		DMAC->CHID.reg = channel;
		DMAC->CHCTRLA.reg = 0;
		while (DMAC->CHCTRLA.bit.ENABLE);
		DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
		while (DMAC->CHCTRLA.bit.SWRST);
		DMAC->CHCTRLB.reg = channel_ctrlb[channel];
		DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL | DMAC_CHINTFLAG_TERR;
		DMAC->CHCTRLA.reg = DMAC_CHCTRLA_ENABLE;
	}

	spi_dma_active = true;
	return true;
}


/**
 * Whether a transfer armed by spi_send_async() has finished.
 *
 * One register read, so the caller can afford to ask on every pass of the main loop.
 * Reports done when nothing is outstanding, so this is safe to call unconditionally.
 *
 * Not a duration and not a timeout: the flag is set by the DMAC when the block
 * transfer count retires, which the SERCOM's own clock guarantees will happen. A
 * transfer that could not complete would show TERR, which this also reports as done
 * so the caller is never wedged.
 */
bool spi_send_done(void)
{
	if (!spi_dma_active) {
		return true;
	}

	// The RX channel completes last by construction -- the final byte cannot be
	// received until after it has been transmitted -- so this one flag covers the
	// whole transaction, and means the last byte is in memory when it is set.
	DMAC->CHID.reg = SPI_DMA_CHANNEL_RX;
	if (!(DMAC->CHINTFLAG.reg & (DMAC_CHINTFLAG_TCMPL | DMAC_CHINTFLAG_TERR))) {
		return false;
	}

	// Disable and clear both channels, so the next arm starts from a known state.
	for (unsigned channel = 0; channel < 2; ++channel) {
		DMAC->CHID.reg = channel;
		DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL | DMAC_CHINTFLAG_TERR;
		DMAC->CHCTRLA.reg = 0;
	}

	spi_dma_active = false;
	return true;
}


/**
 * Sends a block of data over the SPI bus WITHOUT waiting for it to finish.
 *
 * @return true if a DMA transfer was armed, in which case the caller must poll
 *         spi_send_done() before touching either buffer or issuing another transfer.
 *         false if the transfer was performed synchronously on the polled path and is
 *         already complete.
 */
bool spi_send_async(spi_target_t port, void *data_to_send, void *data_received,
		size_t length)
{
	volatile sercom_t *sercom = sercom_for_target(port);

	// The DMAC's block transfer count is 16 bits, and very short transfers are not
	// worth the setup cost. Only the FPGA JTAG SERCOM has its trigger sources wired
	// up, so everything else takes the polled path.
	if ((length < SPI_DMA_MIN_LENGTH) || (length > 0xFFFF) ||
	    (port != SPI_FPGA_JTAG)) {
		spi_send(port, data_to_send, data_received, length);
		return false;
	}

	return spi_dma_arm(sercom, data_to_send, data_received, length);
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
