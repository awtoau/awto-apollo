/**
 * Host-native test of the FPGA_ADV watchdog decision (fpga_adv_wd.h).
 *
 * Models Apollo's poll loop against a SoC whose liveness byte is the 1 ms tick
 * count: the poll cadence, the 3 ms reply timeout and the 100 ms back-off are
 * the firmware's own constants. See docs/chips/cynone-sideband.md section 14.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "fpga_adv_wd.h"

static int failures = 0;

#define CHECK(cond, ...)                                                       \
	do {                                                                       \
		if (cond) { printf("  ok   - "); }                                     \
		else { printf("  FAIL - "); failures++; }                              \
		printf(__VA_ARGS__);                                                   \
		printf("\n");                                                          \
	} while (0)

// Wire costs, microseconds: PING exchange (section 6) and the reply timeout.
#define EXCHANGE_US 214UL
#define TIMEOUT_US  3000UL
#define BOUND_US    37500UL   // 1.25 x 30 ms
#define FRAME_US    174UL     // advertiser frame

typedef struct {
	uint32_t cpu_dead_at_us;   // tick byte stops advancing here
	uint32_t link_dead_at_us;  // no reply from here
	bool     poll_enabled;     // false: negative control, the firmware never polls
} scenario_t;

// Run until `end_us`; returns the first time a wanted health bit is set, or 0.
static uint32_t run(const scenario_t *s, uint32_t end_us, uint8_t want,
                    uint8_t *flagged_ever)
{
	fpga_adv_wd_t wd = {0};
	uint32_t now = 0, next_ms = 0;
	*flagged_ever = 0;
	while (now < end_us) {
		uint32_t now_ms = now / 1000;
		if (s->poll_enabled && (int32_t)(now_ms - next_ms) >= 0) {
			bool ok = now < s->link_dead_at_us;
			uint32_t t = now < s->cpu_dead_at_us ? now : s->cpu_dead_at_us;
			fpga_adv_wd_update(&wd, ok, (uint8_t)(t / 1000));
			next_ms = now_ms + fpga_adv_wd_period_ms(&wd);
			now += ok ? EXCHANGE_US : TIMEOUT_US;
			uint8_t h = fpga_adv_wd_health(&wd);
			*flagged_ever |= h;
			if (h & want) return now;
			continue;
		}
		now += 100;   // main-loop pass granularity
	}
	return 0;
}

static void test_cpu_dead_detected_in_bound(void)
{
	printf("test_cpu_dead_detected_in_bound\n");
	uint32_t worst = 0;
	int missed = 0;
	for (uint32_t fault = 50000; fault < 60000; fault += 97) {   // every phase
		scenario_t s = { fault, UINT32_MAX, true };
		uint8_t ever;
		uint32_t at = run(&s, fault + 200000, FPGA_ADV_WD_CPU_DEAD, &ever);
		if (!at || at - fault > BOUND_US) missed++;
		if (at - fault > worst) worst = at - fault;
	}
	CHECK(missed == 0, "dead tick flagged within %lu us for every fault phase "
	      "(worst %lu us)", (unsigned long)BOUND_US, (unsigned long)worst);
}

static void test_link_dead_detected_in_bound(void)
{
	printf("test_link_dead_detected_in_bound\n");
	uint32_t worst = 0;
	int missed = 0;
	for (uint32_t fault = 50000; fault < 60000; fault += 97) {
		scenario_t s = { UINT32_MAX, fault, true };
		uint8_t ever;
		uint32_t at = run(&s, fault + 200000, FPGA_ADV_WD_LINK_DEAD, &ever);
		if (!at || at - fault > BOUND_US) missed++;
		if (at - fault > worst) worst = at - fault;
	}
	CHECK(missed == 0, "dead link flagged within %lu us for every fault phase "
	      "(worst %lu us)", (unsigned long)BOUND_US, (unsigned long)worst);
}

static void test_live_never_flagged(void)
{
	printf("test_live_never_flagged\n");
	scenario_t s = { UINT32_MAX, UINT32_MAX, true };
	uint8_t ever;
	(void)run(&s, 10000000UL, 0xFF, &ever);   // 10 s
	CHECK(ever == 0, "live SoC over 10 s: health stays 0 (got %u)", ever);
}

static void test_one_lost_reply_is_not_a_fault(void)
{
	printf("test_one_lost_reply_is_not_a_fault\n");
	fpga_adv_wd_t wd = {0};
	fpga_adv_wd_update(&wd, true, 1);
	fpga_adv_wd_update(&wd, false, 0);
	fpga_adv_wd_update(&wd, true, 11);
	fpga_adv_wd_update(&wd, false, 0);
	fpga_adv_wd_update(&wd, false, 0);
	CHECK(fpga_adv_wd_health(&wd) == 0, "two misses are not a dead link");
	fpga_adv_wd_update(&wd, false, 0);
	CHECK(fpga_adv_wd_health(&wd) == FPGA_ADV_WD_LINK_DEAD, "the third is");
	fpga_adv_wd_update(&wd, true, 21);
	CHECK(fpga_adv_wd_health(&wd) == 0, "first good reply clears it");
}

static void test_cpu_dead_needs_three_equal_polls(void)
{
	printf("test_cpu_dead_needs_three_equal_polls\n");
	fpga_adv_wd_t wd = {0};
	fpga_adv_wd_update(&wd, true, 7);
	fpga_adv_wd_update(&wd, true, 7);
	CHECK(fpga_adv_wd_health(&wd) == 0, "two equal polls: not yet");
	fpga_adv_wd_update(&wd, true, 7);
	CHECK(fpga_adv_wd_health(&wd) == FPGA_ADV_WD_CPU_DEAD, "three equal polls");
	fpga_adv_wd_update(&wd, true, 8);
	CHECK(fpga_adv_wd_health(&wd) == 0, "a changed byte clears it");
}

static void test_period_and_backoff(void)
{
	printf("test_period_and_backoff\n");
	fpga_adv_wd_t wd = {0};
	CHECK(fpga_adv_wd_period_ms(&wd) == 10, "10 ms while alive");
	for (int i = 0; i < 3; i++) fpga_adv_wd_update(&wd, false, 0);
	CHECK(fpga_adv_wd_period_ms(&wd) == 100, "100 ms once the link is dead");
	fpga_adv_wd_update(&wd, true, 1);
	CHECK(fpga_adv_wd_period_ms(&wd) == 10, "10 ms again on the first good reply");
}

static void test_advert_skip_window(void)
{
	printf("test_advert_skip_window\n");
	CHECK(!fpga_adv_wd_skip(98), "98 ms after a frame: poll");
	CHECK(fpga_adv_wd_skip(99) && fpga_adv_wd_skip(101), "99..101 ms: skip");
	CHECK(!fpga_adv_wd_skip(102), "102 ms: poll");
	CHECK(!fpga_adv_wd_skip(0), "just after a frame: poll");
}

// Polls every 10 ms against a 100 ms frame, over every relative phase. With the
// skip no poll starts inside a frame; without it a phase-locked pair collides.
static int collisions(uint32_t phase_us, bool use_skip)
{
	int hits = 0;
	uint32_t next_ms = 0, last_end_ms = 0;
	for (uint32_t t = 0; t < 10000000UL; t += 100) {
		// Frame k ends at phase + 100 ms * k (>= 174 us after its start).
		uint32_t since = (t + 100000UL - phase_us % 100000UL) % 100000UL;
		if (t >= phase_us && since < 100) last_end_ms = t / 1000;
		uint32_t ms = t / 1000;
		if ((int32_t)(ms - next_ms) < 0) continue;
		if (use_skip && last_end_ms && fpga_adv_wd_skip(ms - last_end_ms))
			continue;
		next_ms = ms + 10;
		// Poll occupies [t, t + EXCHANGE); frame occupies [end - 174, end).
		uint32_t to_end = 100000UL - since;   // 100 ms at since == 0
		// Before the first frame is heard nothing can be predicted.
		if (last_end_ms && to_end < EXCHANGE_US + FRAME_US) hits++;
	}
	return hits;
}

static void test_no_poll_inside_advert_frame(void)
{
	printf("test_no_poll_inside_advert_frame\n");
	int noskip = 0, skip = 0;
	for (uint32_t phase = 0; phase < 10000; phase += 100) {
		noskip += collisions(phase, false);
		skip   += collisions(phase, true);
	}
	CHECK(noskip > 0, "control: without the skip polls do land in frames (%d)",
	      noskip);
	CHECK(skip == 0, "with the skip no poll lands in a frame (%d)", skip);
}

static void test_negative_control_poll_disabled(void)
{
	printf("test_negative_control_poll_disabled\n");
	scenario_t s = { 50000, 50000, false };   // everything dead, nobody polling
	uint8_t ever;
	uint32_t at = run(&s, 1000000UL, 0xFF, &ever);
	CHECK(at == 0 && ever == 0, "poll disabled: a dead SoC is never flagged, so "
	      "the detection checks above can fail");
}

int main(void)
{
	test_cpu_dead_detected_in_bound();
	test_link_dead_detected_in_bound();
	test_live_never_flagged();
	test_one_lost_reply_is_not_a_fault();
	test_cpu_dead_needs_three_equal_polls();
	test_period_and_backoff();
	test_advert_skip_window();
	test_no_poll_inside_advert_frame();
	test_negative_control_poll_disabled();
	printf("%s\n", failures ? "FAILED" : "all passed");
	return failures ? 1 : 0;
}
