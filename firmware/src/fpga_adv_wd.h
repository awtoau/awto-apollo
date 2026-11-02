/**
 * FPGA_ADV watchdog: liveness decision from PING polls. Pure logic, host-testable.
 * Periods and bounds: docs/chips/cynone-sideband.md section 14.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __FPGA_ADV_WD_H__
#define __FPGA_ADV_WD_H__

#include <stdbool.h>
#include <stdint.h>

#define FPGA_ADV_WD_LINK_DEAD 1
#define FPGA_ADV_WD_CPU_DEAD  2

// 3 misses = link dead; 2 repeats of the tick byte = 3 equal polls = CPU dead.
typedef struct { uint8_t miss, same, tx; } fpga_adv_wd_t;

static inline void fpga_adv_wd_update(fpga_adv_wd_t *w, bool ok, uint8_t tx)
{
	if (!ok) {
		w->same = 0;
		if (w->miss < 255) w->miss++;
		return;
	}
	w->miss = 0;
	if (tx != w->tx) { w->tx = tx; w->same = 0; }
	else if (w->same < 255) w->same++;
}

static inline uint8_t fpga_adv_wd_health(const fpga_adv_wd_t *w)
{
	if (w->miss >= 3) return FPGA_ADV_WD_LINK_DEAD;
	return w->same >= 2 ? FPGA_ADV_WD_CPU_DEAD : 0;
}

// 10 ms poll; 100 ms while the link is dead.
static inline uint32_t fpga_adv_wd_period_ms(const fpga_adv_wd_t *w)
{
	return w->miss >= 3 ? 100 : 10;
}

// True 99..101 ms after the last advertiser frame: the next one is due.
static inline bool fpga_adv_wd_skip(uint32_t since_frame_ms)
{
	return since_frame_ms - 99 < 3;
}

#endif
