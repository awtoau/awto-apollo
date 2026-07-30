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
// For tud_task(), pumped from the command wait so a slow or absent FPGA cannot
// stall USB service for the duration.
#include <tusb.h>
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

// 230400 matches the FPGA responder.
//
// Measured better than 115200 on r1.4, which is counter-intuitive but has a
// clear cause: transmit is bit-banged, so a byte occupies the CPU for 10 bit
// periods. At 115200 that is 86.8 us, at 230400 only 43.4 us -- halving the
// window in which a USB interrupt can preempt a bit and stretch it past the
// receiver's sample point. 100/100 at 230400 against 97/100 at 115200.
//
// 460800 fails hard (1/100, with real CRC corruption rather than timeouts):
// 104 CPU cycles per bit leaves nothing once M0+ ISR entry and exit are
// subtracted. Software transmit is bit-banged from a
// timer interrupt; at 115200 a bit is 8.68 us, so a USB ISR preempting one
// still leaves the receiver sampling mid-bit. At 1 Mbaud (1 us bits) that
// margin is gone, which is why the command protocol runs at the lower rate.
#define ADV_UART_BAUD 230400UL

//
// Transmit state, shared between the foreground and TC1_Handler.
//
// tx_frame holds start bit + 8 data + stop, shifted out LSB first.
// tx_bits_left doubles as the "transmitting" flag: zero means idle.
//
// Diagnostic square-wave state; see fpga_adv_set_toggle().
static bool     toggle_enabled = false;
static bool     toggle_level   = false;
static uint32_t toggle_last    = 0;

static volatile uint16_t tx_frame     = 0;
static volatile uint8_t  tx_bits_left = 0;

// TC1 is the transmit bit clock. A TC rather than SysTick because SysTick's
// interrupt already belongs to board_millis() and cannot also drive a per-bit
// ISR; TC0/TC1/TC2/TCC0 are unused by this firmware and the TinyUSB BSP.
#define TX_TC         TC1
#define TX_TC_IRQn    TC1_IRQn
// TC1 and TC2 share one GCLK channel on this part, hence the combined ID.
#define TX_TC_GCLK_ID GCLK_CLKCTRL_ID_TC1_TC2

//
// Open-drain transmit on FPGA_ADV (PA09).
//
// FPGA_ADV is one wire with a CMOS driver at each end. Driving it push-pull at
// both ends means any timing slip that enables both drivers at once puts a
// driven high against a driven low -- a low-impedance path through two output
// stages, tens to hundreds of milliamps, and a hardware-damage risk rather than
// a link-reliability one. Simulation of the retry-overlap case measured up to
// 30 us of exactly that per collision with both ends push-pull, and zero with
// both ends open-drain (scripts/sideband_contention_probe.py in the workspace).
//
// The SAMD11 has no hardware open-drain mode, so it is emulated: OUT for PA09 is
// parked LOW once, and each bit only toggles DIR. Low means drive low, high
// means release and let the pull-ups raise the line. Two open-drain drivers
// pulling low together is just a low.
//
// DIRSET/DIRCLR on PORT_IOBUS rather than gpio_set_pin_direction(), for two
// reasons. First cost: the HAL call is three register writes including two
// WRCONFIG, and this runs once per bit -- at 230400 baud a bit is 43.4 us of
// wall time but only ~4.3 us of ISR budget before jitter starts eating the
// receiver's sampling margin. IOBUS is single-cycle and DIRSET/DIRCLR are one
// write each. Second correctness: the HAL's GPIO_DIRECTION_IN path writes
// WRCONFIG without PULLEN, so it *clears the pull-up* -- a release through it
// would float the line rather than let it rise, which for an open-drain
// transmitter is the difference between a stop bit and a break.
#define FPGA_ADV_PIN_MASK (1UL << 9)
#define FPGA_ADV_PORT     0

// Release the line: input, pull-up holds it high.
#define FPGA_ADV_RELEASE() \
	(PORT_IOBUS->Group[FPGA_ADV_PORT].DIRCLR.reg = FPGA_ADV_PIN_MASK)

// Pull the line low: output, and OUT is already parked at 0.
#define FPGA_ADV_PULL_LOW() \
	(PORT_IOBUS->Group[FPGA_ADV_PORT].DIRSET.reg = FPGA_ADV_PIN_MASK)

// Response buffer. Sized for the largest reply in the protocol: CMD_POWER
// returns status + 16 payload + CRC8.
#define ADV_RESPONSE_MAX 18
static volatile uint8_t  response[ADV_RESPONSE_MAX];
static volatile uint8_t  response_len = 0;
static volatile uint8_t  response_want = 0;

// Link health, reported to the host so corruption is visible rather than
// silent. Saturating rather than wrapping: a count stuck at 255 still says
// "this link is bad", where a wrapped counter can read as healthy.
static uint8_t stat_ok      = 0;   // responses that arrived with a valid CRC
static uint8_t stat_crc     = 0;   // arrived complete, CRC did not match
static uint8_t stat_timeout = 0;   // did not arrive at all

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

	// OVF, not MC0. In MFRQ mode CC0 is the TOP value and the counter wraps at
	// it (datasheet 30.6.2.5), so the period event is an overflow -- there is
	// no compare match to catch. Enabling MC0 here produced a timer that ran
	// but never interrupted, so nothing was ever transmitted.
	TX_TC->COUNT16.INTENSET.reg = TC_INTENSET_OVF;

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

	// Stop the transmit timer before anything else.
	//
	// TC1_Handler re-muxes PA09 back to SERCOM1 when a frame finishes. Switching
	// modes with a byte still in flight let that ISR fire *after* the EIC
	// configuration below had been applied, quietly undoing it -- a 43 us window
	// at 230400 baud, reachable by switching modes mid-command. Disabling the
	// counter and its interrupt, then clearing the in-flight state, closes it:
	// there is no pending overflow left to service.
	//
	// The byte being transmitted is abandoned rather than completed. That is the
	// right trade: the mode is changing, so nothing is waiting for the reply, and
	// a truncated frame is rejected by the responder's stop-bit check.
	NVIC_DisableIRQ(TX_TC_IRQn);
	TX_TC->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
	TX_TC->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
	tx_bits_left = 0;
	tx_frame     = 0;

	// Leave the line released, not driven. An abandoned frame must not leave
	// PA09 pulling low: the FPGA would read a permanent break, and with the pin
	// about to be re-muxed nothing would ever release it.
	FPGA_ADV_RELEASE();

	// Any diagnostic square wave also stops here, for the same reason -- it
	// drives the same pin from the task loop and would fight whichever
	// peripheral this mode selects.
	toggle_enabled = false;

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

	// Take the pin off the SERCOM, but arrive at open-drain rather than at a
	// driven output.
	//
	// Order matters. The pull-up and the parked-low OUT are established while
	// the pin is still an input, so the line is never driven high even for a
	// cycle -- if it were, that instant would be push-pull against an FPGA that
	// may still be finishing a reply, which is the collision this scheme exists
	// to make impossible. Only after that does the first bit assert DIR.
	gpio_set_pin_function(FPGA_ADV, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_direction(FPGA_ADV, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(FPGA_ADV, GPIO_PULL_UP);

	// Park OUT low, once. Every bit from here is a DIR toggle only, and OUT is
	// never written again, so a low bit cannot be a driven high by accident.
	PORT->Group[FPGA_ADV_PORT].OUTCLR.reg = FPGA_ADV_PIN_MASK;
	FPGA_ADV_RELEASE();

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
	// Refuse before the FPGA has configured.
	//
	// Until DONE goes high the FPGA's I/O are tri-stated, so FPGA_ADV floats,
	// held only by our pull-up. There is no responder to answer, and a
	// floating CMOS input can pick up noise the receiver frames as spurious
	// start bits. Returning 0 is honest -- the link does not exist yet -- and
	// distinguishes "FPGA not ready" from "FPGA did not reply", which a
	// timeout alone cannot.
	//
	// Checked here, before anything is armed: the GPIO accesses it performs
	// are latency on a transmit path whose turnaround is already tight.
	if (!fpga_configuration_done()) {
		return 0;
	}

	if (adv_mode != FPGA_ADV_MODE_UART || length > ADV_RESPONSE_MAX) {
		return 0;
	}

	// Refuse while the diagnostic square wave is running.
	//
	// The toggle owns PA09 from the task loop and knows nothing about the
	// transmit path. With both live, fpga_adv_task() re-muxes and re-levels the
	// pin every 50 ms underneath a frame TC1_Handler is shifting out, and the
	// pin's direction depends on which ran last. Returning 0 makes that
	// combination a refusal rather than a corrupted frame -- the host asked for
	// two mutually exclusive things and gets told, instead of getting garbage
	// that looks like a link fault.
	if (toggle_enabled) {
		return 0;
	}

	// Drain the receiver before arming, so a retry cannot inherit the tail of
	// the previous reply.
	//
	// This is the failure the CRC was papering over. If a command times out
	// while the FPGA is still transmitting, the bytes still on their way land in
	// the SERCOM after this call returned. The next command re-arms the collector
	// and the ISR fills the buffer with those stale bytes -- lengths are fixed
	// and there is no framing, so response_len reaches response_want, the length
	// test passes, and a full-length reply assembled from the *previous*
	// transaction is handed back as valid. The CRC usually catches it, but it is
	// only counted; the bytes still go to the caller and up to the host, which
	// has no way to know.
	//
	// Reading DATA clears RXC, and the SERCOM has a two-deep receive buffer, so a
	// bounded three reads empties it. Bounded rather than a while loop: this runs
	// on the USB control path and a stuck RXC must not be able to hang it.
	for (uint8_t drain = 0; drain < 3; drain++) {
		if ((SERCOM1->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC) == 0) break;
		(void)SERCOM1->USART.DATA.reg;
	}
	// Clear any error left by the noise a half-finished handover can produce, so
	// the first real byte is not dropped as corrupt.
	SERCOM1->USART.STATUS.reg = SERCOM_USART_STATUS_FERR |
	                            SERCOM_USART_STATUS_BUFOVF |
	                            SERCOM_USART_STATUS_PERR;

	// Transmit first, then arm the collector.
	//
	// The SERCOM receiver stays enabled while we bit-bang the pin, so it sees
	// our own command byte echoed back. Arming beforehand captured that echo
	// as response byte 0, pushing every real byte one position along and
	// leaving response_len short -- which showed up as complete-or-nothing
	// replies, mostly nothing.
	//
	// Arming afterwards is safe because the FPGA cannot begin replying until
	// it has the whole command byte, and the pin handover in TC1_Handler
	// happens before this line runs.
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

	// Pump the USB stack while waiting.
	//
	// Without this the spin blocks tud_task() for the whole wait: about 1 ms for
	// a POWER command and up to ~21 ms on a timeout. This runs inside a control
	// transfer's SETUP stage, so a 21 ms stall risks the host's own control
	// timeout, and it starves the console and LED tasks as well. The stall
	// appears exactly when the FPGA is absent or slow -- which is when a host is
	// most likely to be polling hard.
	//
	// tud_task() is re-entrant with respect to this path: the response arrives
	// via SERCOM1_Handler, which touches only response[]/response_len, and no
	// USB callback calls back into fpga_adv_command().
	while (response_len < response_want) {
		tud_task();
		if ((int32_t)(board_millis() - deadline) >= 0) {
			response_want = 0;
			if (stat_timeout < 255) stat_timeout++;
			return 0;
		}
	}

	response_want = 0;
	for (uint8_t i = 0; i < length; i++) {
		buffer[i] = response[i];
	}

	// Verify here rather than leaving it to the host: a corrupted payload is
	// otherwise indistinguishable from a good one, and the whole point of
	// carrying a CRC is that nothing downstream has to trust the wire. The
	// bytes are still returned so the caller can inspect what arrived.
	if (length >= 2 && fpga_adv_crc8(buffer, length - 1) != buffer[length - 1]) {
		if (stat_crc < 255) stat_crc++;
	} else {
		if (stat_ok < 255) stat_ok++;
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
 * Report link health: responses OK, CRC failures, timeouts.
 *
 * Counters saturate at 255 and are cleared by reading, so successive reads
 * give the count since the last look rather than an ever-growing total.
 */
void fpga_adv_get_stats(uint8_t *ok, uint8_t *crc_fail, uint8_t *timeout)
{
#ifdef BOARD_HAS_USB_SWITCH
	*ok = stat_ok; *crc_fail = stat_crc; *timeout = stat_timeout;
	stat_ok = stat_crc = stat_timeout = 0;
#else
	*ok = *crc_fail = *timeout = 0;
#endif
}

/**
 * True once the FPGA has completed configuration.
 *
 * DONE is driven by the FPGA and read-only here, so this cannot disturb
 * configuration. Note it latches: it goes high at first configuration and
 * stays high across force-offline, so it answers "has the FPGA configured
 * since power-up" rather than "is the FPGA running right now".
 */
bool fpga_configuration_done(void)
{
#ifdef BOARD_HAS_USB_SWITCH
	gpio_set_pin_direction(FPGA_DONE, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(FPGA_DONE, GPIO_PULL_OFF);
	return gpio_get_pin_level(FPGA_DONE);
#else
	return true;
#endif
}

/**
 * Diagnostic: drive FPGA_ADV as a raw square wave.
 *
 * Bypasses the UART framing entirely -- no TC1, no shift register, just a GPIO
 * toggled from the main loop. If the FPGA sees edges from this but not from
 * fpga_adv_tx_byte(), the fault is in the transmit path rather than in the pin
 * configuration or the wire.
 *
 * Enabled by fpga_adv_set_toggle(); disabled by default so it cannot interfere
 * with normal operation.
 */
void fpga_adv_set_toggle(bool enable)
{
#ifdef BOARD_HAS_USB_SWITCH
	toggle_enabled = enable;
	if (enable) {
		// Take the pin off the SERCOM and drive it directly -- open-drain, for
		// the same reason as the transmit path: a diagnostic must not be able to
		// short the FPGA's driver. Pull-up and a parked-low OUT, then DIR
		// toggles in fpga_adv_task().
		gpio_set_pin_function(FPGA_ADV, GPIO_PIN_FUNCTION_OFF);
		gpio_set_pin_direction(FPGA_ADV, GPIO_DIRECTION_IN);
		gpio_set_pin_pull_mode(FPGA_ADV, GPIO_PULL_UP);
		PORT->Group[FPGA_ADV_PORT].OUTCLR.reg = FPGA_ADV_PIN_MASK;
		FPGA_ADV_RELEASE();
	} else {
		gpio_set_pin_direction(FPGA_ADV, GPIO_DIRECTION_IN);
		gpio_set_pin_pull_mode(FPGA_ADV, GPIO_PULL_UP);
		gpio_set_pin_function(FPGA_ADV, MUX_PA09C_SERCOM1_PAD3);
	}
#else
	(void)enable;
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
	// Diagnostic square wave: 10 Hz, so 50 ms per half period. Checked before
	// the mode logic because it deliberately overrides normal operation.
	if (toggle_enabled) {
		if (board_millis() - toggle_last >= 50) {
			toggle_last  = board_millis();
			toggle_level = !toggle_level;
			// Open-drain, matching the transmit path: release for high, pull for
			// low. The rising edge is the pull-ups rather than a driver, which is
			// still plainly visible on a scope at 10 Hz.
			if (toggle_level) {
				FPGA_ADV_RELEASE();
			} else {
				FPGA_ADV_PULL_LOW();
			}
		}
		return;
	}

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
  TX_TC->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;

  if (tx_bits_left == 0) {
    // Frame done: stop the counter and hand the pin back to the receiver, so
    // the FPGA's reply is not missed.
    TX_TC->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;

    // Release first, then re-mux. The line is already released -- the stop bit
    // above is a release -- so this is belt and braces against arriving here
    // with DIR still set, and it guarantees the pin is not an output for any
    // part of the handover.
    FPGA_ADV_RELEASE();
    gpio_set_pin_direction(FPGA_ADV, GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(FPGA_ADV, GPIO_PULL_UP);
    gpio_set_pin_function(FPGA_ADV, MUX_PA09C_SERCOM1_PAD3);
    return;
  }

  // Open-drain: pull low for a 0, release for a 1. A DIR toggle only -- OUT was
  // parked low in fpga_adv_tx_byte() and is never written here, so this can
  // never source current into an FPGA that is pulling the same wire low.
  if (tx_frame & 1) {
    FPGA_ADV_RELEASE();
  } else {
    FPGA_ADV_PULL_LOW();
  }
  tx_frame >>= 1;
  tx_bits_left--;
}
#endif
