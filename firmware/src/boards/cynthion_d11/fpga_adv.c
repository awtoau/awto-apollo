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

// 1 Mbaud, matching the FPGA side.
#define ADV_UART_BAUD 1000000UL

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
#endif
