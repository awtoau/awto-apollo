/*
 * LED control abstraciton code.
 *
 * This file is part of Apollo.
 *
 * Copyright (c) 2020-2024 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <tusb.h>
#include <sam.h>
#include <bsp/board_api.h>
#include <hal/include/hal_gpio.h>


#include "led.h"
#include "fpga.h"
#include "fpga_adv.h"
#include "usb_switch.h"
#include "apollo_mode.h"


/**
 * The LED pins, in display order.
 *
 * File-scope and const so the table lives in flash. It was previously spelled
 * out as a non-const local in four separate functions, which made the compiler
 * emit code to rebuild it on the stack at each call site -- four copies of the
 * same constant data plus the stores to construct it. On a part this close to
 * full that is not affordable. See awtoau/cynthion-workspace#73.
 */
static const led_t led_pins[LED_COUNT] = { LED_A, LED_B, LED_C, LED_D, LED_E };


/**
 * Sets up each of the LEDs for use.
 */
void led_init(void)
{
    // Default each LED to an output and _off_.
    for (unsigned i = 0; i < LED_COUNT; ++i) {
        gpio_set_pin_direction(led_pins[i], GPIO_DIRECTION_OUT);
        gpio_set_pin_level(led_pins[i], true);
    }
}


/**
 * Turns the provided LED on.
 */
void led_on(led_t led)
{
    gpio_set_pin_level(led, false);
}


/**
 * Turns the provided LED off.
 */
void led_off(led_t led)
{
    gpio_set_pin_level(led, true);
}


/**
 * Toggles the provided LED.
 */
void led_toggle(led_t led)
{
    gpio_toggle_pin_level(led);
}


/**
 * Sets whether a given led is on.
 */
void led_set(led_t led, bool on)
{
    gpio_set_pin_level(led, !on);
}


/**
 * Turns off all of the device's LEDs.
 */
void leds_off(void)
{
  for (unsigned i = 0; i < LED_COUNT; ++i) {
    led_off(led_pins[i]);
  }
}


/**
 * Turns off all of the device's LEDs.
 */
void leds_on(void)
{
  for (unsigned i = 0; i < LED_COUNT; ++i) {
    led_on(led_pins[i]);
  }
}


/**
 * Drives one LED per subsystem, straight from live state.
 *
 * There are no patterns, no animation and no host control. Every LED is a direct
 * readout of a condition the firmware can already answer, so none of them needs
 * decoding and none of them carries state of its own.
 *
 * What this replaced: a pattern state machine, a host request (0xa1) to select
 * patterns, board_millis() scheduling, sweep and blink animations, and a
 * bit-position display helper -- all to encode operations as coded patterns across
 * five LEDs. An indicator whose meaning has to be looked up conveys nothing at a
 * glance. This part had under 500 bytes spare and that budget went to DMA-driven
 * JTAG clocking instead, worth 1.26x (see the JTAG doc).
 *
 * The mapping, which needs no legend:
 *
 *   LED_A (blue)   power -- on whenever Apollo is running
 *   LED_B (pink)   JTAG  -- the uC is driving the JTAG lines, keep the header off
 *   LED_C (white)  FPGA has requested the CONTROL port
 *   LED_D (pink)   FPGA is online and holding the USB port
 *   LED_E (blue)   reserved for fault indication
 *
 * Being stateless is the point: recomputing five levels per iteration costs a few
 * GPIO stores, and it is what let the scheduling, the animation state, the pattern
 * variable and the USB request all go.
 */
void led_task(void)
{
  led_set(LED_A, true);
  led_set(LED_B, apollo_mode_jtag_active());
  led_set(LED_C, fpga_requesting_port());
  led_set(LED_D, fpga_is_online() && fpga_controls_usb_port());
  led_set(LED_E, false);
}
