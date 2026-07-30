/*
 * LED control abstraciton code.
 *
 * This file is part of Apollo.
 *
 * Copyright (c) 2019-2024 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __LED_H__
#define __LED_H__

#include <apollo_board.h>

/**
 * Sets up each of the LEDs for use.
 */
void led_init(void);


/**
 * Turns the provided LED on.
 */
void led_on(led_t led);


/**
 * Turns the provided LED off.
 */
void led_off(led_t led);


/**
 * Turns off all of the device's LEDs.
 */
void leds_off(void);


/**
 * Toggles the provided LED.
 */
void led_toggle(led_t led);


/**
 * Sets whether a given led is on.
 */
void led_set(led_t led, bool on);


/** Set in an override byte to arm it; bits 0-4 are the LEDs, lowest bit first. */
#define LED_OVERRIDE_ACTIVE 0x20u


/**
 * Overrides the LED display, or restores live state when passed 0.
 *
 * Arrives in wValue of JTAG_START, which carried nothing before. Intended for
 * bringing up a board -- putting a known value on the LEDs -- not for encoding
 * operations, which is what the removed pattern system did badly.
 */
void led_set_override(uint8_t value);


/**
 * Drives one LED per subsystem from live state.
 *
 * There are no patterns and no host control: each LED is a direct readout, so
 * nothing needs decoding. See led.c for the mapping.
 */
void led_task(void);

#endif
