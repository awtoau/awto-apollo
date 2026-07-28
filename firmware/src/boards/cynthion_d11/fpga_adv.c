/**
 * FPGA advertisement pin handling code.
 *
 * This file is part of Apollo.
 *
 * Copyright (c) 2023-2024 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdbool.h>
#include "fpga_adv.h"
#include "usb_switch.h"
#include "apollo_board.h"
#include <hal/include/hal_gpio.h>

#include <bsp/board_api.h>
#include <hpl/pm/hpl_pm_base.h>
#include <hpl/gclk/hpl_gclk_base.h>
#include <peripheral_clk_config.h>

#ifdef BOARD_HAS_USB_SWITCH

// Switching the shared USB port to the FPGA is allowed.
static bool fpga_usb_allowed = false;

// Duration of the time window (in milliseconds).
#define WINDOW_PERIOD_MS 200UL

// Store the timestamp of the last time window update.
static uint32_t last_update = 0;

// Counter of edges detected within the last time window.
static uint32_t window_edges = 0;

// Counter of edges detected since the last time window update.
static volatile uint32_t edge_counter = 0;

// How the advertisement is currently received. EIC is the power-on default so
// that firmware with this change behaves identically to firmware without it
// until a host explicitly asks for UART mode.
static fpga_adv_mode_t adv_mode = FPGA_ADV_MODE_EIC;

//
// UART mode state.
//
// The FPGA sends a fixed 4-byte pattern; seeing it in full is what counts as
// an advertisement. A partial match is not evidence of anything, so the
// matcher tracks position rather than accumulating a score.
//
static const uint8_t HEARTBEAT_PATTERN[] = { 0xC1, 0x14, 0x01, 0xA5 };
#define HEARTBEAT_LENGTH (sizeof(HEARTBEAT_PATTERN))

// How far into the pattern the receiver currently is.
static volatile uint8_t pattern_position = 0;

// Timestamp of the last complete heartbeat frame.
static volatile uint32_t last_heartbeat = 0;

// A heartbeat older than this means the FPGA is no longer asking for the port.
// Longer than the 200 ms EIC window because a UART frame can be lost to a
// single bit error, and one dropped frame should not surrender the port.
#define HEARTBEAT_TIMEOUT_MS 300UL

// 115200 matches the FPGA responder. Software transmit is bit-banged from a
// timer interrupt; at 115200 a bit is 8.68 us, so a USB ISR preempting one
// still leaves the receiver sampling mid-bit. At 1 Mbaud (1 us bits) that
// margin is gone, which is why the command protocol runs at the lower rate.
#define ADV_UART_BAUD 115200UL

//
// Transmit state, shared between the foreground and TC1_Handler.
//
// tx_frame holds start bit + 8 data + stop, shifted out LSB first.
// tx_bits_left doubles as the "transmitting" flag: zero means idle.
//
static volatile uint16_t tx_frame     = 0;
static volatile uint8_t  tx_bits_left = 0;

// TC1 is the transmit bit clock. A TC rather than SysTick because SysTick's
// interrupt already belongs to board_millis() and cannot also drive a per-bit
// ISR; TC0/TC1/TC2/TCC0 are unused by this firmware and the TinyUSB BSP.
#define TX_TC         TC1
#define TX_TC_IRQn    TC1_IRQn
// TC1 and TC2 share one GCLK channel on this part, hence the combined ID.
#define TX_TC_GCLK_ID GCLK_CLKCTRL_ID_TC1_TC2

// Response buffer. Sized for the largest reply in the protocol: CMD_POWER
// returns status + 16 payload + CRC8.
#define ADV_RESPONSE_MAX 18
static volatile uint8_t  response[ADV_RESPONSE_MAX];
static volatile uint8_t  response_len = 0;
static volatile uint8_t  response_want = 0;

#endif

/**
 * Initialize FPGA_ADV receive-only pin
 */
void fpga_adv_init(void)
{
#ifdef BOARD_HAS_USB_SWITCH
	fpga_adv_set_mode(FPGA_ADV_MODE_EIC);
#endif
}

#ifdef BOARD_HAS_USB_SWITCH

/**
 * Configure PA09 as EIC/EXTINT7 and count rising edges.
 */
static void fpga_adv_init_eic(void)
{
	// Enable the APB clock for EIC (External Interrupt Controller).
	_pm_enable_bus_clock(PM_BUS_APBA, EIC);

	// Configure GCLK for EIC.
	_gclk_enable_channel(GCLK_CLKCTRL_ID_EIC_Val, GCLK_CLKCTRL_GEN_GCLK0_Val);
	while (GCLK->STATUS.bit.SYNCBUSY);

	// Configure FPGA_ADV as an input with function A (external interrupt).
	gpio_set_pin_direction(FPGA_ADV, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(FPGA_ADV, GPIO_PULL_UP);
	gpio_set_pin_function(FPGA_ADV, MUX_PA09A_EIC_EXTINT7);

	// Disable EIC.
	EIC->CTRL.bit.ENABLE = 0;
	while (EIC->STATUS.bit.SYNCBUSY);

	// Configure EIC to trigger on rising edge.
	EIC->CONFIG[0].reg &= ~EIC_CONFIG_SENSE7_Msk;
	EIC->CONFIG[0].reg |= EIC_CONFIG_SENSE7_RISE;

	// Enable External Interrupt.
	EIC->INTENSET.reg = EIC_INTENSET_EXTINT(1 << 7);

	// Enable EIC.
	EIC->CTRL.bit.ENABLE = 1;
	while (EIC->STATUS.bit.SYNCBUSY);

	// Enable IRQ.
	NVIC_EnableIRQ(EIC_IRQn);
}

/**
 * Configure TC1 as the transmit bit clock. Does not start it.
 */
static void fpga_adv_tx_timer_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TX_TC);
	_gclk_enable_channel(TX_TC_GCLK_ID, GCLK_CLKCTRL_GEN_GCLK0_Val);

	// Reset first: CTRLA is write-protected while enabled, and this can be
	// re-entered after a mode switch.
	TX_TC->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while (TX_TC->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

	// 16-bit, no prescaler, match-frequency so CC0 is the period.
	TX_TC->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16
	                         | TC_CTRLA_WAVEGEN_MFRQ
	                         | TC_CTRLA_PRESCALER_DIV1;
	TX_TC->COUNT16.CC[0].reg = (CONF_CPU_FREQUENCY / ADV_UART_BAUD) - 1;
	while (TX_TC->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

	TX_TC->COUNT16.INTENSET.reg = TC_INTENSET_MC0;

	// Below USB. The SAMD11 BSP never calls NVIC_SetPriority, so USB sits at
	// the default 0. If this ISR could preempt USB, bit-banging would delay
	// USB service -- the problem an interrupt-driven transmit exists to avoid.
	NVIC_SetPriority(TX_TC_IRQn, 3);
	NVIC_EnableIRQ(TX_TC_IRQn);
}

/**
 * Configure PA09 as SERCOM1/PAD3 and receive a framed heartbeat.
 *
 * Receive only: this pin is an input, and Apollo has nothing to say back to
 * the FPGA over it.
 */
static void fpga_adv_init_uart(void)
{
	Sercom *sercom = SERCOM1;

	// Stop the EIC from also driving the pin.
	NVIC_DisableIRQ(EIC_IRQn);
	EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(1 << 7);

	gpio_set_pin_direction(FPGA_ADV, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(FPGA_ADV, GPIO_PULL_UP);
	gpio_set_pin_function(FPGA_ADV, MUX_PA09C_SERCOM1_PAD3);

	// Set up clocking for the SERCOM peripheral.
	_pm_enable_bus_clock(PM_BUS_APBC, sercom);
	_gclk_enable_channel(SERCOM1_GCLK_ID_CORE, GCLK_CLKCTRL_GEN_GCLK0_Val);

	// Reset before configuring: this may be a re-entry after a mode switch,
	// and CTRLA is write-protected while enabled.
	sercom->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
	while (sercom->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_SWRST);

	sercom->USART.CTRLA.reg =
		SERCOM_USART_CTRLA_DORD            |  // LSB first
		SERCOM_USART_CTRLA_RXPO(3)         |  // RX on PA09 (PAD[3])
		SERCOM_USART_CTRLA_SAMPR(0)        |  // 16x oversampling
		SERCOM_USART_CTRLA_RUNSTDBY        |  // don't autosuspend the clock
		SERCOM_USART_CTRLA_MODE_USART_INT_CLK;

	// Baud divisor, using the same multiply-and-shift as uart.c: it avoids
	// pulling in soft division, which matters on a part this close to full.
	const uint32_t m1 =  (1ULL << 32) / CONF_CPU_FREQUENCY;
	const uint32_t m2 = ((1ULL << 42) / CONF_CPU_FREQUENCY) & 0x3FF;
	const uint32_t m3 = ((1ULL << 52) / CONF_CPU_FREQUENCY) & 0x3FF;
	const uint32_t m4 = ((1ULL << 62) / CONF_CPU_FREQUENCY) & 0x3FF;
	const uint32_t op4 = (ADV_UART_BAUD * m4 -   1) >> 10;
	const uint32_t op3 = (ADV_UART_BAUD * m3 + op4) >> 10;
	const uint32_t op2 = (ADV_UART_BAUD * m2 + op3) >> 10;
	const uint32_t op1 = (ADV_UART_BAUD * m1 + op2) >> 12;
	sercom->USART.BAUD.reg = 65535 - op1;

	// Receiver only, 8-N-1.
	sercom->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN;
	while (sercom->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_CTRLB);

	sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;

	sercom->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
	while (sercom->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_ENABLE);

	NVIC_EnableIRQ(SERCOM1_IRQn);

	fpga_adv_tx_timer_init();
}

#endif

/**
 * Select how the FPGA advertisement is received.
 */
bool fpga_adv_set_mode(fpga_adv_mode_t mode)
{
#ifdef BOARD_HAS_USB_SWITCH
	if (mode != FPGA_ADV_MODE_EIC && mode != FPGA_ADV_MODE_UART) {
		return false;
	}

	// Discard whatever the outgoing mode observed. Without this a switch could
	// report a port request derived from the other mechanism -- e.g. edges
	// counted from UART traffic, which is not an advertisement at all.
	NVIC_DisableIRQ(EIC_IRQn);
	NVIC_DisableIRQ(SERCOM1_IRQn);
	edge_counter     = 0;
	window_edges     = 0;
	pattern_position = 0;
	last_heartbeat   = 0;
	last_update      = board_millis();

	adv_mode = mode;

	if (mode == FPGA_ADV_MODE_UART) {
		fpga_adv_init_uart();
	} else {
		// Leaving UART mode: stop the receiver before the pin is re-muxed,
		// or a partial frame can raise RXC against a pin no longer wired to it.
		SERCOM1->USART.CTRLA.reg &= ~SERCOM_USART_CTRLA_ENABLE;
		while (SERCOM1->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_ENABLE);
		fpga_adv_init_eic();
	}

	return true;
#else
	(void)mode;
	return false;
#endif
}

#ifdef BOARD_HAS_USB_SWITCH
/**
 * Queue one byte for transmission on FPGA_ADV.
 *
 * Bit-banged but hardware-timed: TXPO selects only PAD0 or PAD2 (datasheet
 * Table 25-9) and PA09 is PAD3 on both SERCOM1 and SERCOM0, so the SERCOM
 * cannot transmit here. One bit per TC1 interrupt, so interrupts are never
 * disabled and USB service is never delayed.
 */
static void fpga_adv_tx_byte(uint8_t byte)
{
	// Wait out any byte still going. Spins with interrupts enabled, so USB
	// keeps being serviced.
	while (tx_bits_left != 0);

	gpio_set_pin_function(FPGA_ADV, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_direction(FPGA_ADV, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(FPGA_ADV, true);

	// Start bit (0), eight data bits LSB first, stop bit (1).
	tx_frame     = ((uint16_t)byte << 1) | 0x200;
	tx_bits_left = 10;

	// Start from zero so the first bit gets a full period.
	TX_TC->COUNT16.COUNT.reg = 0;
	TX_TC->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
	while (TX_TC->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
}
#endif

/**
 * Issue a command and collect the fixed-length response.
 *
 * Response length is a property of the command, known to both sides at compile
 * time, so there is no length field to parse. Returns the number of bytes
 * collected, or 0 on timeout.
 */
uint8_t fpga_adv_command(uint8_t command, uint8_t *buffer, uint8_t length)
{
#ifdef BOARD_HAS_USB_SWITCH
	if (adv_mode != FPGA_ADV_MODE_UART || length > ADV_RESPONSE_MAX) {
		return 0;
	}

	// Arm the collector before transmitting: the FPGA starts replying as soon
	// as it has the command byte, so arming afterwards can miss the first byte.
	response_len  = 0;
	response_want = length;

	fpga_adv_tx_byte(command);

	// The one timeout in the protocol. It does not guard a state machine --
	// the FPGA is stateless between commands -- it guards against the FPGA not
	// being there at all: unconfigured, wedged, or running gateware without
	// this protocol. No framing can fix that.
	//
	// Derived from the baud rate rather than hard-coded, so it stays correct
	// if the rate changes. Ten character times per expected byte, plus a
	// fixed allowance for the FPGA to turn the line around.
	//
	// Per-byte cost is folded to a compile-time constant (rounded up, so the
	// deadline is never short) rather than dividing at runtime. `length` is a
	// variable, so a runtime `/ ADV_UART_BAUD` would pull in __udivsi3 -- 266
	// bytes of soft-division helper on this Cortex-M0+, for one division.
	#define ADV_MS_PER_BYTE (((10UL * 10UL * 1000UL) + ADV_UART_BAUD - 1) \
	                         / ADV_UART_BAUD)
	const uint32_t deadline = board_millis()
	                        + 1 + (uint32_t)(length + 2) * ADV_MS_PER_BYTE;

	while (response_len < response_want) {
		if ((int32_t)(board_millis() - deadline) >= 0) {
			response_want = 0;
			return 0;
		}
	}

	response_want = 0;
	for (uint8_t i = 0; i < length; i++) {
		buffer[i] = response[i];
	}
	return length;
#else
	(void)command; (void)buffer; (void)length;
	return 0;
#endif
}

/**
 * CRC-8/ATM over a buffer: polynomial 0x07, init 0x00, no reflection.
 *
 * Matches the responder gateware. Bitwise rather than table-driven: a 256-byte
 * table would cost more flash than this saves in cycles, on a link that moves
 * 18 bytes at a time.
 */
uint8_t fpga_adv_crc8(const uint8_t *data, uint8_t length)
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

/**
 * The advertisement mode currently in effect.
 */
fpga_adv_mode_t fpga_adv_get_mode(void)
{
#ifdef BOARD_HAS_USB_SWITCH
	return adv_mode;
#else
	return FPGA_ADV_MODE_EIC;
#endif
}

/**
 * Task for things related with the advertisement pin
 */
void fpga_adv_task(void)
{
#ifdef BOARD_HAS_USB_SWITCH
	if (adv_mode == FPGA_ADV_MODE_EIC) {
		// Wait for the defined time window.
		if (board_millis() - last_update < WINDOW_PERIOD_MS) return;

		// Update edge counts inside time window.
		//
		// Mask the EIC interrupt across the read/clear pair: EIC_Handler increments
		// edge_counter on every advertisement edge, so an interrupt landing between
		// the two statements would have its edge silently dropped. That under-counts
		// the window, feeding fpga_requesting_port() (threshold > 2) and potentially
		// missing an FPGA USB-takeover request.
		NVIC_DisableIRQ(EIC_IRQn);
		window_edges = edge_counter;
		edge_counter = 0;
		NVIC_EnableIRQ(EIC_IRQn);
		last_update  = board_millis();
	}
	// In UART mode there is no window to service: the handler timestamps each
	// complete frame and fpga_requesting_port() compares against that directly.

    // Take over USB if the FPGA is not requesting the port.
	if (fpga_requesting_port() == false) {
		take_over_usb();
	} else if (fpga_usb_allowed) {
		hand_off_usb();
	}
#endif
}

/**
 * Allow FPGA takeover of the USB port
 */
void allow_fpga_takeover_usb(bool allow)
{
#ifdef BOARD_HAS_USB_SWITCH
	fpga_usb_allowed = allow;
#else
	/*
	 * Boards without a USB switch also lack the advertising channel used
	 * by the FPGA to request the USB port. On those platforms we
	 * immediately hand off the port to the FPGA.
	 */
	hand_off_usb();
#endif
}

/**
 * True if we received an advertisement message within the last time window.
 */
bool fpga_requesting_port(void)
{
#ifdef BOARD_HAS_USB_SWITCH
	if (adv_mode == FPGA_ADV_MODE_UART) {
		// True iff a complete heartbeat frame arrived recently enough. Zero
		// means none has been seen since the mode was selected, which is not
		// the same as "one arrived at time zero".
		if (last_heartbeat == 0) return false;
		return (board_millis() - last_heartbeat) < HEARTBEAT_TIMEOUT_MS;
	}

	// True iff the number of edge counts surpasses the defined threshold.
	return window_edges > 2;
#else
	return false;
#endif
}


#ifdef BOARD_HAS_USB_SWITCH
/**
 * FPGA_ADV interrupt handler.
 */
void EIC_Handler(void) {
  // Clear the interrupt flag.
  EIC->INTFLAG.reg = EIC_INTFLAG_EXTINT(1 << 7);

  // Increment our edge counter.
  edge_counter++;
}

/**
 * FPGA_ADV UART receive handler.
 *
 * Deliberately minimal: read the byte, advance the pattern matcher, timestamp
 * a completed frame. This runs on the path that decides USB port ownership, so
 * anything beyond that belongs in the task.
 */
void SERCOM1_Handler(void) {
  // Reading DATA clears RXC. Errors are cleared by writing STATUS back; a
  // framing error means this byte is unusable, so drop it and resynchronise
  // rather than feeding garbage to the matcher.
  bool corrupt = (SERCOM1->USART.STATUS.reg &
                  (SERCOM_USART_STATUS_FERR | SERCOM_USART_STATUS_BUFOVF |
                   SERCOM_USART_STATUS_PERR)) != 0;

  uint8_t byte = (uint8_t)SERCOM1->USART.DATA.reg;

  if (corrupt) {
    SERCOM1->USART.STATUS.reg = SERCOM_USART_STATUS_FERR |
                                SERCOM_USART_STATUS_BUFOVF |
                                SERCOM_USART_STATUS_PERR;
    pattern_position = 0;
    return;
  }

  // A command is in flight: these bytes are its response, not heartbeat
  // pattern candidates. Checked first so a reply containing pattern bytes
  // cannot be mistaken for an advertisement.
  if (response_want != 0) {
    if (response_len < ADV_RESPONSE_MAX) {
      response[response_len++] = byte;
    }
    return;
  }

  if (byte == HEARTBEAT_PATTERN[pattern_position]) {
    pattern_position++;
    if (pattern_position == HEARTBEAT_LENGTH) {
      last_heartbeat   = board_millis();
      pattern_position = 0;
    }
  } else {
    // Restart the match, but allow this byte to begin a new frame -- otherwise
    // a stream of pattern bytes offset by one would never resynchronise.
    pattern_position = (byte == HEARTBEAT_PATTERN[0]) ? 1 : 0;
  }
}

/**
 * Transmit bit clock: drives one bit per interrupt.
 *
 * Deliberately minimal. Every cycle here is jitter on the bit being driven,
 * and this runs on the path that decides USB port ownership.
 */
void TC1_Handler(void) {
  TX_TC->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;

  if (tx_bits_left == 0) {
    // Frame done: stop the counter and hand the pin back to the receiver, so
    // the FPGA's reply is not missed.
    TX_TC->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;

    gpio_set_pin_direction(FPGA_ADV, GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(FPGA_ADV, GPIO_PULL_UP);
    gpio_set_pin_function(FPGA_ADV, MUX_PA09C_SERCOM1_PAD3);
    return;
  }

  gpio_set_pin_level(FPGA_ADV, tx_frame & 1);
  tx_frame >>= 1;
  tx_bits_left--;
}
#endif
