/**
 * Main-loop hang watchdog, RP2040: not enabled. Timing and rationale: hang_wd.h.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hang_wd.h"

void hang_wd_enable(void) {}
void hang_wd_kick(void) {}
