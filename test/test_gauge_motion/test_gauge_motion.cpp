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

Motion zeroed(uint16_t travel, uint16_t vmax, uint16_t accel)
{
    Motion m;
    m.configure(travel, vmax, accel, 2400, travel);
    return m;
}

} // namespace

void test_speed_conversion()
{
    TEST_ASSERT_EQUAL_UINT32(0x80000000UL, Gauge::speedToTick(5000)); // 0.5 steps/tick
    TEST_ASSERT_EQUAL_UINT32(0, Gauge::speedToTick(0));
}

void test_zero_sweep_length_rate_and_position()
{
    Motion m;
    m.configure(3780, 5000, 50000, 2400, 3780);
    m.setTarget(0);
    m.zero();
    TEST_ASSERT_TRUE(m.isZeroing());
    Run r = runUntilIdle(m);
    TEST_ASSERT_TRUE(r.idle);
    TEST_ASSERT_EQUAL_UINT32(3780, r.steps_down);
    TEST_ASSERT_EQUAL_UINT32(0, r.steps_up);
    TEST_ASSERT_EQUAL_INT32(0, m.position());
    // 3780 steps at 2400 steps/s: 15750 ticks, unramped.
    TEST_ASSERT_UINT32_WITHIN(20, 15750, r.ticks);
    TEST_ASSERT_FALSE(r.consecutive);
}

void test_zero_speed_capped_by_max_speed()
{
    Motion m;
    m.configure(945, 300, 1500, 600, 945);
    m.zero();
    Run r = runUntilIdle(m);
    TEST_ASSERT_EQUAL_UINT32(945, r.steps_down);
    TEST_ASSERT_UINT32_WITHIN(20, 31500, r.ticks); // 945 steps at 300 steps/s
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
    m.configure(945, 1800, 1500, 600, 945);
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
