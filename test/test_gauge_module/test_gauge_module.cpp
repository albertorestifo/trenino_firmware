#include "../../src/gauge_module.h"
#include "../../src/gauge_scheduler.h"
#include <unity.h>

using namespace GaugeHal;
using Modules::GaugeModule;

namespace {

Protocol::GaugeConfig vid(uint8_t step, uint8_t dir, uint8_t reset = Protocol::GAUGE_NO_PIN)
{
    Protocol::GaugeConfig c;
    c.driver = Protocol::GAUGE_DRIVER_VID6606;
    c.pins[0] = step;
    c.pins[1] = dir;
    c.pins[2] = reset;
    c.pins[3] = Protocol::GAUGE_NO_PIN;
    c.travel = 3780;
    c.max_speed = 5000;
    c.acceleration = 50000;
    c.flags = 0;
    return c;
}

Protocol::GaugeConfig direct(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4)
{
    Protocol::GaugeConfig c;
    c.driver = Protocol::GAUGE_DRIVER_DIRECT;
    c.pins[0] = p1;
    c.pins[1] = p2;
    c.pins[2] = p3;
    c.pins[3] = p4;
    c.travel = 945;
    c.max_speed = 1800;
    c.acceleration = 1500;
    c.flags = 0;
    return c;
}

void runZero(GaugeModule& g)
{
    while (g.motion().isZeroing()) GaugeScheduler::tick();
}

uint8_t coils(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4)
{
    return g_mock_pin_level[p1] | (g_mock_pin_level[p2] << 1) | (g_mock_pin_level[p3] << 2) | (g_mock_pin_level[p4] << 3);
}

const uint8_t SIX_STATE[6] = { 0x9, 0x1, 0x7, 0x6, 0xE, 0x8 };

} // namespace

void setUp()
{
    mockReset();
}

void tearDown()
{
    TEST_ASSERT_EQUAL_UINT8(0, GaugeScheduler::count()); // every test deletes its gauges
}

void test_vid6606_begin_releases_reset_starts_timer_and_zeroes()
{
    GaugeModule g(vid(8, 7, 6));
    TEST_ASSERT_FALSE(g_mock_timer_running);
    TEST_ASSERT_TRUE(g.begin());
    TEST_ASSERT_TRUE(g_mock_timer_running);
    TEST_ASSERT_EQUAL_UINT8(1, GaugeScheduler::count());
    TEST_ASSERT_EQUAL_UINT32(GaugeModule::RESET_LOW_MS, g_mock_delay_ms);
    TEST_ASSERT_EQUAL_UINT8(1, g_mock_pin_level[6]); // RESET_N released high
    TEST_ASSERT_EQUAL_UINT8(0, g_mock_pin_level[8]); // STEP low
    TEST_ASSERT_TRUE(g.motion().isZeroing());
    TEST_ASSERT_EQUAL_UINT8(8, g.getPin());
    TEST_ASSERT_TRUE(g.getType() == Modules::ModuleType::Gauge);
}

void test_shared_reset_pin_is_pulsed_once()
{
    GaugeModule a(vid(8, 7, 6));
    GaugeModule b(vid(14, 16, 6));
    TEST_ASSERT_TRUE(a.begin());
    uint32_t writes = g_mock_pin_writes[6];
    TEST_ASSERT_TRUE(b.begin());
    TEST_ASSERT_EQUAL_UINT32(writes, g_mock_pin_writes[6]);
    TEST_ASSERT_EQUAL_UINT32(GaugeModule::RESET_LOW_MS, g_mock_delay_ms);
}

void test_reset_pulsed_again_after_scheduler_empties()
{
    {
        GaugeModule a(vid(8, 7, 6));
        a.begin();
    }
    TEST_ASSERT_FALSE(g_mock_timer_running);
    GaugeModule b(vid(8, 7, 6));
    b.begin();
    TEST_ASSERT_EQUAL_UINT32(2 * GaugeModule::RESET_LOW_MS, g_mock_delay_ms);
}

void test_vid6606_step_pulse_spans_one_tick_and_dir_leads_step()
{
    GaugeModule g(vid(8, 7));
    g.begin();
    TEST_ASSERT_EQUAL_UINT8(0, g_mock_pin_level[7]); // zero sweep: DIR low (toward the stop)
    runZero(g);
    g.setTarget(50);

    uint32_t rising = 0;
    uint8_t previous_step = 0;
    uint8_t previous_dir = g_mock_pin_level[7];
    bool dir_and_step_same_tick = false;
    for (int t = 0; t < 20000 && !g.motion().isIdle(); t++) {
        GaugeScheduler::tick();
        uint8_t step = g_mock_pin_level[8];
        if (step && previous_step) TEST_FAIL_MESSAGE("STEP high on two ticks in a row");
        if (step && !previous_step) {
            rising++;
            if (g_mock_pin_level[7] != previous_dir) dir_and_step_same_tick = true;
        }
        previous_step = step;
        previous_dir = g_mock_pin_level[7];
    }
    GaugeScheduler::tick(); // clears the last pulse
    TEST_ASSERT_EQUAL_UINT8(0, g_mock_pin_level[8]);
    TEST_ASSERT_EQUAL_UINT32(50, rising);
    TEST_ASSERT_EQUAL_UINT8(1, g_mock_pin_level[7]); // DIR high away from the stop
    TEST_ASSERT_FALSE(dir_and_step_same_tick);
}

void test_vid6606_reverse_flag_inverts_dir()
{
    Protocol::GaugeConfig c = vid(8, 7);
    c.flags = Protocol::GAUGE_FLAG_REVERSE;
    GaugeModule g(c);
    g.begin();
    TEST_ASSERT_EQUAL_UINT8(1, g_mock_pin_level[7]); // toward the stop is now high
    runZero(g);
    g.setTarget(10);
    for (int t = 0; t < 5000 && !g.motion().isIdle(); t++) GaugeScheduler::tick();
    TEST_ASSERT_EQUAL_UINT8(0, g_mock_pin_level[7]);
}

void test_direct_drive_walks_the_six_state_pattern()
{
    GaugeModule g(direct(2, 3, 4, 5));
    g.begin();
    TEST_ASSERT_EQUAL_HEX8(SIX_STATE[0], coils(2, 3, 4, 5));
    runZero(g);
    // The zero sequence always ends on the state it started from.
    TEST_ASSERT_EQUAL_HEX8(SIX_STATE[0], coils(2, 3, 4, 5));

    g.setTarget(7);
    uint8_t expected = 0;
    int32_t last = g.motion().position();
    for (int t = 0; t < 20000 && !g.motion().isIdle(); t++) {
        GaugeScheduler::tick();
        if (g.motion().position() != last) {
            expected = expected == 5 ? 0 : expected + 1;
            last = g.motion().position();
            TEST_ASSERT_EQUAL_HEX8(SIX_STATE[expected], coils(2, 3, 4, 5));
        }
    }
    TEST_ASSERT_EQUAL_INT32(7, g.motion().position());
}

void test_direct_drive_reverse_walks_backwards()
{
    Protocol::GaugeConfig c = direct(2, 3, 4, 5);
    c.flags = Protocol::GAUGE_FLAG_REVERSE;
    GaugeModule g(c);
    g.begin();
    runZero(g);
    TEST_ASSERT_EQUAL_HEX8(SIX_STATE[0], coils(2, 3, 4, 5));
    g.setTarget(1);
    for (int t = 0; t < 5000 && !g.motion().isIdle(); t++) GaugeScheduler::tick();
    // Reversed, a step away from the stop walks the table backwards.
    TEST_ASSERT_EQUAL_HEX8(SIX_STATE[5], coils(2, 3, 4, 5));
}

void test_destructor_unregisters_stops_timer_and_releases_coils()
{
    {
        GaugeModule g(direct(2, 3, 4, 5));
        g.begin();
        TEST_ASSERT_TRUE(GaugeScheduler::ownsPin(4));
        TEST_ASSERT_FALSE(GaugeScheduler::ownsPin(9));
    }
    TEST_ASSERT_EQUAL_UINT8(0, GaugeScheduler::count());
    TEST_ASSERT_FALSE(g_mock_timer_running);
    TEST_ASSERT_EQUAL_HEX8(0, coils(2, 3, 4, 5));
    TEST_ASSERT_FALSE(GaugeScheduler::ownsPin(4));
}

void test_unused_vid6606_pins_are_not_owned()
{
    GaugeModule g(vid(8, 7));
    g.begin();
    TEST_ASSERT_TRUE(GaugeScheduler::ownsPin(8));
    TEST_ASSERT_TRUE(GaugeScheduler::ownsPin(7));
    TEST_ASSERT_FALSE(GaugeScheduler::ownsPin(Protocol::GAUGE_NO_PIN));
}

void test_scheduler_refuses_a_fifth_gauge()
{
    GaugeModule a(vid(2, 3));
    GaugeModule b(vid(4, 5));
    GaugeModule c(vid(6, 7));
    GaugeModule d(vid(8, 9));
    GaugeModule e(vid(10, 11));
    TEST_ASSERT_TRUE(a.begin());
    TEST_ASSERT_TRUE(b.begin());
    TEST_ASSERT_TRUE(c.begin());
    TEST_ASSERT_TRUE(d.begin());
    TEST_ASSERT_FALSE(e.begin());
    TEST_ASSERT_EQUAL_UINT8(4, GaugeScheduler::count());
    TEST_ASSERT_EQUAL_UINT32(1, g_mock_timer_starts);
}

void test_begin_rejects_pins_the_board_lacks_before_touching_anything()
{
    GaugeModule g(vid(8, 200, 6)); // 200 is past the mock's 64 pins
    TEST_ASSERT_FALSE(g.begin());
    TEST_ASSERT_EQUAL_UINT32(0, g_mock_pin_writes[8]);
    TEST_ASSERT_EQUAL_UINT32(0, g_mock_pin_writes[6]); // RESET_N not claimed or pulsed
    TEST_ASSERT_EQUAL_UINT8(0, GaugeScheduler::count());
    GaugeModule ok(vid(8, 7, 6));
    TEST_ASSERT_TRUE(ok.begin());
    TEST_ASSERT_EQUAL_UINT32(GaugeModule::RESET_LOW_MS, g_mock_delay_ms); // claim still free
}

void test_suspend_and_resume_nest()
{
    GaugeModule g(vid(8, 7));
    g.begin();
    GaugeScheduler::suspend();
    GaugeScheduler::suspend();
    TEST_ASSERT_FALSE(g_mock_timer_running);
    GaugeScheduler::resume();
    TEST_ASSERT_FALSE(g_mock_timer_running);
    GaugeScheduler::resume();
    TEST_ASSERT_TRUE(g_mock_timer_running);
    GaugeScheduler::resume(); // unbalanced resume is ignored
    TEST_ASSERT_TRUE(g_mock_timer_running);
}

void test_suspend_without_gauges_keeps_timer_off()
{
    GaugeScheduler::suspend();
    GaugeScheduler::resume();
    TEST_ASSERT_FALSE(g_mock_timer_running);
}

void test_zero_command_resweeps_then_returns_to_target()
{
    GaugeModule g(direct(2, 3, 4, 5));
    g.begin();
    runZero(g);
    g.setTarget(100);
    for (int t = 0; t < 100000 && !g.motion().isIdle(); t++) GaugeScheduler::tick();
    TEST_ASSERT_EQUAL_INT32(100, g.motion().position());
    g.zero();
    TEST_ASSERT_TRUE(g.motion().isZeroing());
    runZero(g);
    for (int t = 0; t < 100000 && !g.motion().isIdle(); t++) GaugeScheduler::tick();
    TEST_ASSERT_EQUAL_INT32(100, g.motion().position());
}

void test_rezero_lands_on_the_same_coil_state_from_any_position()
{
    GaugeModule g(direct(2, 3, 4, 5));
    g.begin();
    runZero(g);
    const uint16_t starts[] = { 0, 1, 2, 100, 503, 945 };
    for (uint8_t i = 0; i < 6; i++) {
        g.setTarget(starts[i]);
        for (int t = 0; t < 200000 && !g.motion().isIdle(); t++) GaugeScheduler::tick();
        g.zero();
        runZero(g);
        TEST_ASSERT_EQUAL_HEX8(SIX_STATE[0], coils(2, 3, 4, 5));
        TEST_ASSERT_EQUAL_INT32(0, g.motion().position());
    }
}

void test_vid6606_without_reset_keeps_its_phase_across_reconfigure()
{
    {
        GaugeModule a(vid(20, 21));
        a.begin();
        runZero(a);
        a.setTarget(7);
        for (int t = 0; t < 20000 && !a.motion().isIdle(); t++) GaugeScheduler::tick();
        TEST_ASSERT_EQUAL_UINT8(7, a.motion().phase());
    }
    GaugeModule b(vid(20, 21));
    b.begin();
    TEST_ASSERT_EQUAL_UINT8(7, b.motion().phase()); // the chip did not move
    runZero(b);
    TEST_ASSERT_EQUAL_UINT8(0, b.motion().phase());
}

void test_vid6606_with_reset_starts_from_phase_zero()
{
    {
        GaugeModule a(vid(22, 23, 24));
        a.begin();
        runZero(a);
        a.setTarget(5);
        for (int t = 0; t < 20000 && !a.motion().isIdle(); t++) GaugeScheduler::tick();
    }
    GaugeModule b(vid(22, 23, 24));
    b.begin();
    TEST_ASSERT_EQUAL_UINT8(0, b.motion().phase()); // RESET_N homed the chip
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_vid6606_begin_releases_reset_starts_timer_and_zeroes);
    RUN_TEST(test_shared_reset_pin_is_pulsed_once);
    RUN_TEST(test_reset_pulsed_again_after_scheduler_empties);
    RUN_TEST(test_vid6606_step_pulse_spans_one_tick_and_dir_leads_step);
    RUN_TEST(test_vid6606_reverse_flag_inverts_dir);
    RUN_TEST(test_direct_drive_walks_the_six_state_pattern);
    RUN_TEST(test_direct_drive_reverse_walks_backwards);
    RUN_TEST(test_destructor_unregisters_stops_timer_and_releases_coils);
    RUN_TEST(test_unused_vid6606_pins_are_not_owned);
    RUN_TEST(test_scheduler_refuses_a_fifth_gauge);
    RUN_TEST(test_begin_rejects_pins_the_board_lacks_before_touching_anything);
    RUN_TEST(test_suspend_and_resume_nest);
    RUN_TEST(test_suspend_without_gauges_keeps_timer_off);
    RUN_TEST(test_zero_command_resweeps_then_returns_to_target);
    RUN_TEST(test_rezero_lands_on_the_same_coil_state_from_any_position);
    RUN_TEST(test_vid6606_without_reset_keeps_its_phase_across_reconfigure);
    RUN_TEST(test_vid6606_with_reset_starts_from_phase_zero);
    return UNITY_END();
}
