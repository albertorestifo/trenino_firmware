#include "../../src/gauge_motion.h"
#include <math.h>
#include <stdio.h>
#include <unity.h>

using Gauge::Motion;
using Gauge::TICK_HZ;

void setUp() {}
void tearDown() {}

namespace {

struct Run {
    uint32_t ticks; // ticks until idle (or limit)
    uint32_t steps_up;
    uint32_t steps_down;
    uint32_t min_interval; // smallest tick gap between two steps
    bool consecutive; // two steps on adjacent ticks
    bool dir_changed_on_step; // direction() changed on a tick that stepped
    bool idle;
};

Run runUntilIdle(Motion& m, uint32_t limit = 2000000)
{
    Run r = {};
    r.min_interval = 0xffffffff;
    uint32_t last_step = 0;
    bool stepped_before = false;
    bool previous_tick_stepped = false;
    for (uint32_t t = 1; t <= limit; t++) {
        int8_t dir_before = m.direction();
        int8_t s = m.tick();
        if (s != 0) {
            if (s > 0) r.steps_up++;
            else r.steps_down++;
            if (stepped_before && t - last_step < r.min_interval) r.min_interval = t - last_step;
            if (previous_tick_stepped) r.consecutive = true;
            if (m.direction() != dir_before) r.dir_changed_on_step = true;
            last_step = t;
            stepped_before = true;
        }
        previous_tick_stepped = s != 0;
        if (m.isIdle()) {
            r.ticks = t;
            r.idle = true;
            return r;
        }
    }
    r.ticks = limit;
    return r;
}

// Ideal trapezoid (or triangle) time for distance d, in ticks.
double idealTicks(double d, double v, double a)
{
    double ramp_d = v * v / a; // accelerate plus brake
    double seconds = d >= ramp_d ? d / v + v / a : 2.0 * sqrt(d / a);
    return seconds * TICK_HZ;
}

// Unity is built without double support.
void assertNear(double expected, double actual, double fraction)
{
    char message[96];
    snprintf(message, sizeof(message), "expected %.0f ticks, got %.0f", expected, actual);
    TEST_ASSERT_TRUE_MESSAGE(fabs(actual - expected) <= expected * fraction, message);
}

const uint8_t VID_CYCLE = 24;
const uint8_t DIRECT_CYCLE = 6;

Motion zeroed(uint16_t travel, uint16_t vmax, uint16_t accel)
{
    Motion m;
    m.configure(travel, vmax, accel, 2400, travel, 240, VID_CYCLE);
    return m;
}

// Steps of the back-off and slow approach after the fast sweep, ending on
// phase 0 from `phase` (the phase the fast sweep left).
uint32_t settleUp(uint8_t cycle) { return (uint32_t)Motion::BACKOFF_CYCLES * cycle; }
uint32_t settleDown(uint8_t cycle, uint8_t phase)
{
    return (uint32_t)(Motion::BACKOFF_CYCLES + Motion::OVERDRIVE_CYCLES) * cycle + phase;
}

// Phase after n steps down from phase 0.
uint8_t phaseAfterDown(uint32_t n, uint8_t cycle) { return (uint8_t)((cycle - n % cycle) % cycle); }

} // namespace

void test_speed_conversion()
{
    TEST_ASSERT_EQUAL_UINT32(0x80000000UL, Gauge::speedToTick(5000)); // 0.5 steps/tick
    TEST_ASSERT_EQUAL_UINT32(0, Gauge::speedToTick(0));
}

void test_zero_sweep_length_rate_and_position()
{
    Motion m;
    m.configure(3780, 5000, 50000, 2400, 3780, 240, VID_CYCLE);
    m.setTarget(0);
    m.zero();
    TEST_ASSERT_TRUE(m.isZeroing());
    Run r = runUntilIdle(m);
    TEST_ASSERT_TRUE(r.idle);
    // 3780 mod 24 = 12: the fast sweep leaves phase 12, the approach adds 12.
    uint8_t phase = phaseAfterDown(3780, VID_CYCLE);
    TEST_ASSERT_EQUAL_UINT8(12, phase);
    TEST_ASSERT_EQUAL_UINT32(3780 + settleDown(VID_CYCLE, phase), r.steps_down);
    TEST_ASSERT_EQUAL_UINT32(settleUp(VID_CYCLE), r.steps_up);
    TEST_ASSERT_EQUAL_INT32(0, m.position());
    TEST_ASSERT_EQUAL_UINT8(0, m.phase());
    // 3780 steps at 2400 steps/s (15750 ticks), 48 + 84 at 240 steps/s
    // (5500 ticks), the dwell and two turns.
    TEST_ASSERT_UINT32_WITHIN(60, 15750 + 5500 + Motion::DWELL_TICKS, r.ticks);
    TEST_ASSERT_FALSE(r.consecutive);
    TEST_ASSERT_FALSE(r.dir_changed_on_step);
}

void test_zero_speed_capped_by_max_speed()
{
    Motion m;
    m.configure(945, 300, 1500, 600, 945, 900, DIRECT_CYCLE);
    m.zero();
    Run r = runUntilIdle(m);
    uint8_t phase = phaseAfterDown(945, DIRECT_CYCLE);
    TEST_ASSERT_EQUAL_UINT32(945 + settleDown(DIRECT_CYCLE, phase), r.steps_down);
    // Every leg at 300 steps/s, including the settle legs asked for at 900.
    uint32_t steps = r.steps_down + r.steps_up;
    TEST_ASSERT_UINT32_WITHIN(60, steps * 10000UL / 300 + Motion::DWELL_TICKS, r.ticks);
}

void test_zero_always_ends_on_phase_zero()
{
    // Wherever the needle starts, and whatever phase the driver held, 0 is
    // the same electrical phase, so it is the same needle angle.
    for (uint8_t seed = 0; seed < DIRECT_CYCLE; seed++) {
        for (uint16_t start = 0; start < 3 * DIRECT_CYCLE; start++) {
            Motion m;
            m.configure(945, 1800, 1500, 600, 975, 60, DIRECT_CYCLE);
            m.setPhase(seed);
            m.setTarget(start);
            runUntilIdle(m);
            TEST_ASSERT_EQUAL_INT32(start, m.position());
            m.zero();
            runUntilIdle(m);
            TEST_ASSERT_EQUAL_INT32(start, m.position()); // back to the target
            m.setTarget(0);
            runUntilIdle(m);
            TEST_ASSERT_EQUAL_INT32(0, m.position());
            TEST_ASSERT_EQUAL_UINT8(0, m.phase());
        }
    }
}

void test_rezero_matches_the_first_zero()
{
    Motion m = zeroed(3780, 5000, 50000);
    m.zero();
    runUntilIdle(m);
    TEST_ASSERT_EQUAL_UINT8(0, m.phase());
    const uint16_t starts[] = { 0, 1, 13, 1000, 3779 };
    for (uint8_t i = 0; i < 5; i++) {
        m.setTarget(starts[i]);
        runUntilIdle(m);
        TEST_ASSERT_EQUAL_UINT8(starts[i] % VID_CYCLE, m.phase());
        m.zero();
        m.setTarget(0);
        runUntilIdle(m);
        TEST_ASSERT_EQUAL_INT32(0, m.position());
        TEST_ASSERT_EQUAL_UINT8(0, m.phase());
    }
}

void test_rezero_during_any_zero_leg_still_ends_on_phase_zero()
{
    // Interrupt at points inside the fast sweep, the back-off, the slow
    // approach and the dwell: 15750 ticks of fast sweep at 2400 steps/s, then
    // 48 steps at 240 steps/s (2000 ticks), 84 more (3500), then 500 ticks.
    const uint32_t at[] = { 5000, 16500, 19000, 21300 };
    for (uint8_t i = 0; i < 4; i++) {
        Motion m = zeroed(3780, 5000, 50000);
        m.setTarget(37);
        m.zero();
        for (uint32_t t = 0; t < at[i]; t++) m.tick();
        TEST_ASSERT_TRUE(m.isZeroing());
        m.zero();
        Run r = runUntilIdle(m);
        TEST_ASSERT_TRUE(r.idle);
        TEST_ASSERT_FALSE(r.consecutive);
        TEST_ASSERT_FALSE(r.dir_changed_on_step);
        TEST_ASSERT_EQUAL_INT32(37, m.position());
        TEST_ASSERT_EQUAL_UINT8(37 % VID_CYCLE, m.phase());
    }
}

void test_zero_approaches_the_stop_slowly_then_dwells()
{
    Motion m;
    m.configure(945, 1800, 1500, 600, 945, 60, DIRECT_CYCLE);
    m.zero();
    // Skip the fast sweep.
    uint32_t down = 0;
    while (down < 945) {
        if (m.tick() < 0) down++;
    }
    // Back-off and approach: one step per 10000 / 60 = 166.7 ticks.
    uint32_t last_step = 0;
    uint32_t min_gap = 0xffffffff;
    uint32_t up = 0;
    uint32_t settle_down = 0;
    uint32_t t = 0;
    uint32_t expected_down = settleDown(DIRECT_CYCLE, phaseAfterDown(945, DIRECT_CYCLE));
    while (settle_down < expected_down) {
        t++;
        int8_t s = m.tick();
        if (s == 0) continue;
        if (s > 0) {
            TEST_ASSERT_EQUAL_UINT32(0, settle_down); // all back-off before the approach
            up++;
        } else {
            settle_down++;
        }
        if (last_step != 0 && t - last_step < min_gap) min_gap = t - last_step;
        last_step = t;
    }
    TEST_ASSERT_EQUAL_UINT32(settleUp(DIRECT_CYCLE), up);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(166, min_gap);
    // Holds the stop without stepping, still zeroing, for the dwell.
    for (uint32_t i = 0; i + 1 < Motion::DWELL_TICKS; i++) {
        TEST_ASSERT_EQUAL_INT8(0, m.tick());
        TEST_ASSERT_TRUE(m.isZeroing());
    }
    TEST_ASSERT_EQUAL_INT8(0, m.tick());
    TEST_ASSERT_FALSE(m.isZeroing());
    TEST_ASSERT_EQUAL_INT32(0, m.position());
}

void test_short_moves_land_exactly_without_overshoot()
{
    const uint16_t targets[] = { 1, 2, 3, 10, 57, 3780 };
    for (uint8_t i = 0; i < sizeof(targets) / sizeof(targets[0]); i++) {
        Motion m = zeroed(3780, 5000, 50000);
        m.setTarget(targets[i]);
        Run r = runUntilIdle(m);
        TEST_ASSERT_TRUE(r.idle);
        TEST_ASSERT_EQUAL_INT32(targets[i], m.position());
        TEST_ASSERT_EQUAL_UINT32(targets[i], r.steps_up);
        TEST_ASSERT_EQUAL_UINT32(0, r.steps_down);
    }
}

void test_speed_never_exceeds_max_and_no_consecutive_steps()
{
    Motion m = zeroed(3780, 5000, 60000);
    m.setTarget(3780);
    Run r = runUntilIdle(m);
    TEST_ASSERT_EQUAL_INT32(3780, m.position());
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(2, r.min_interval); // 10000 / 5000
    TEST_ASSERT_FALSE(r.consecutive);

    Motion d = zeroed(945, 1800, 60000);
    d.setTarget(945);
    r = runUntilIdle(d);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(5, r.min_interval); // 10000 / 1800 = 5.6
}

void test_long_move_matches_trapezoid_time()
{
    // A ramp count that grew during cruise would start braking early and
    // crawl, making the move much slower than the ideal profile.
    Motion m = zeroed(3780, 4000, 20000);
    m.setTarget(3780);
    Run r = runUntilIdle(m);
    double ideal = idealTicks(3780, 4000, 20000);
    assertNear(ideal, r.ticks, 0.05);
}

void test_low_accelerations_still_accelerate()
{
    const uint16_t accels[] = { 1, 100, 1000 };
    for (uint8_t i = 0; i < 3; i++) {
        Motion m = zeroed(4000, 1000, accels[i]);
        m.setTarget(200);
        Run r = runUntilIdle(m, 40000000UL);
        TEST_ASSERT_TRUE(r.idle);
        TEST_ASSERT_EQUAL_INT32(200, m.position());
        double ideal = idealTicks(200, 1000, accels[i]);
        assertNear(ideal, r.ticks, 0.15);
    }
}

void test_retarget_further_in_same_direction_keeps_moving()
{
    Motion m = zeroed(3780, 3000, 20000);
    m.setTarget(500);
    for (int i = 0; i < 2000; i++) m.tick();
    TEST_ASSERT_TRUE(m.position() > 0 && m.position() < 500);
    m.setTarget(2000);
    // Never idles between the two targets, never steps backwards.
    uint32_t down = 0;
    for (uint32_t t = 0; t < 400000 && !m.isIdle(); t++) {
        if (m.tick() < 0) down++;
        if (m.position() < 2000) TEST_ASSERT_FALSE(m.isIdle());
    }
    TEST_ASSERT_EQUAL_INT32(2000, m.position());
    TEST_ASSERT_EQUAL_UINT32(0, down);
}

void test_retarget_behind_overshoots_and_returns()
{
    Motion m = zeroed(3780, 4000, 20000);
    m.setTarget(3000);
    for (int i = 0; i < 6000; i++) m.tick(); // well into the move
    int32_t at = m.position();
    TEST_ASSERT_TRUE(at > 300 && at < 3000);
    m.setTarget(at - 200);
    Run r = runUntilIdle(m);
    TEST_ASSERT_TRUE(r.idle);
    TEST_ASSERT_EQUAL_INT32(at - 200, m.position());
    TEST_ASSERT_TRUE(r.steps_up > 0); // braked forward first
    TEST_ASSERT_EQUAL_UINT32(r.steps_up + 200, r.steps_down);
    TEST_ASSERT_FALSE(r.dir_changed_on_step);
}

void test_retarget_closer_than_braking_distance_overshoots_and_returns()
{
    Motion m = zeroed(3780, 4000, 5000);
    m.setTarget(3780);
    for (int i = 0; i < 9000; i++) m.tick();
    int32_t at = m.position();
    m.setTarget(at + 5); // far shorter than the braking distance
    Run r = runUntilIdle(m);
    TEST_ASSERT_EQUAL_INT32(at + 5, m.position());
    TEST_ASSERT_TRUE(r.steps_down > 0); // went past, came back
    TEST_ASSERT_FALSE(r.dir_changed_on_step);
}

void test_target_clamped_to_travel()
{
    Motion m = zeroed(945, 1800, 1500);
    m.setTarget(60000);
    TEST_ASSERT_EQUAL_UINT16(945, m.target());
    runUntilIdle(m);
    TEST_ASSERT_EQUAL_INT32(945, m.position());
}

void test_target_set_during_zero_applies_afterwards()
{
    Motion m;
    m.configure(945, 1800, 1500, 600, 945, 60, DIRECT_CYCLE);
    m.zero();
    for (int i = 0; i < 100; i++) m.tick();
    TEST_ASSERT_TRUE(m.isZeroing());
    m.setTarget(300);
    // Zero sweep runs to the end first.
    uint32_t down = 0;
    while (m.isZeroing()) {
        if (m.tick() < 0) down++;
    }
    TEST_ASSERT_EQUAL_INT32(0, m.position());
    runUntilIdle(m);
    TEST_ASSERT_EQUAL_INT32(300, m.position());
}

void test_rezero_interrupts_a_move()
{
    Motion m = zeroed(3780, 5000, 50000);
    m.setTarget(3000);
    for (int i = 0; i < 5000; i++) m.tick();
    m.zero();
    TEST_ASSERT_TRUE(m.isZeroing());
    int8_t first = m.tick();
    TEST_ASSERT_EQUAL_INT8(0, first); // DIR settles before the sweep
    TEST_ASSERT_EQUAL_INT8(-1, m.direction());
    Run r = runUntilIdle(m);
    TEST_ASSERT_EQUAL_INT32(3000, m.position()); // back to the target
    TEST_ASSERT_TRUE(r.steps_down >= 3780);
}

void test_direction_changes_only_on_ticks_without_a_step()
{
    Motion m = zeroed(3780, 5000, 50000);
    m.setTarget(100);
    Run r = runUntilIdle(m);
    TEST_ASSERT_FALSE(r.dir_changed_on_step);
    m.setTarget(20);
    r = runUntilIdle(m);
    TEST_ASSERT_FALSE(r.dir_changed_on_step);
    TEST_ASSERT_EQUAL_INT32(20, m.position());
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_speed_conversion);
    RUN_TEST(test_zero_sweep_length_rate_and_position);
    RUN_TEST(test_zero_speed_capped_by_max_speed);
    RUN_TEST(test_zero_always_ends_on_phase_zero);
    RUN_TEST(test_rezero_matches_the_first_zero);
    RUN_TEST(test_zero_approaches_the_stop_slowly_then_dwells);
    RUN_TEST(test_rezero_during_any_zero_leg_still_ends_on_phase_zero);
    RUN_TEST(test_short_moves_land_exactly_without_overshoot);
    RUN_TEST(test_speed_never_exceeds_max_and_no_consecutive_steps);
    RUN_TEST(test_long_move_matches_trapezoid_time);
    RUN_TEST(test_low_accelerations_still_accelerate);
    RUN_TEST(test_retarget_further_in_same_direction_keeps_moving);
    RUN_TEST(test_retarget_behind_overshoots_and_returns);
    RUN_TEST(test_retarget_closer_than_braking_distance_overshoots_and_returns);
    RUN_TEST(test_target_clamped_to_travel);
    RUN_TEST(test_target_set_during_zero_applies_afterwards);
    RUN_TEST(test_rezero_interrupts_a_move);
    RUN_TEST(test_direction_changes_only_on_ticks_without_a_step);
    return UNITY_END();
}
