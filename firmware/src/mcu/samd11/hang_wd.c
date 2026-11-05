/**
 * Main-loop hang watchdog, SAMD11. Timing and rationale: hang_wd.h.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <sam.h>
#include "hang_wd.h"

void hang_wd_enable(void)
{
	// GCLK_WDT runs at 32.768 kHz out of reset (as dfu.c relies on): 16384 clocks = 500 ms.
	WDT->CONFIG.reg = WDT_CONFIG_PER_16K;
	WDT->CTRL.reg = WDT_CTRL_ENABLE;
}

void hang_wd_kick(void)
{
	WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
}
