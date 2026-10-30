#define GET_STATUS 0 // Arduino native USB core uses this global macro.
#include "../../src/stepper.h"
#include <unity.h>

using namespace Stepper;
struct FakeDriver : Driver {
    int writes = 0, releases = 0, begins = 0;
    bool fail = false;
    uint8_t ports = 0;
    uint8_t last_phase = 0;
    bool begin(uint8_t) override { ++begins; return !fail; }
    uint8_t phaseCount(uint8_t mode) const override { return mode == 1 ? 6 : (mode == 2 ? 4 : (mode == 3 ? 8 : 0)); }
    bool write(uint8_t, uint8_t port, uint8_t, uint8_t phase) override { ++writes; ports |= 1 << port; last_phase = phase; return !fail; }
    bool release(uint8_t, uint8_t) override { ++releases; return !fail; }
};
FakeDriver driver;
Controller controller(driver);
Config config() {
    Config c = {};
    c.driver = 1; c.mode = 1; c.address = 0x60; c.port = 1;
    c.hold = 1; c.home_strategy = 1; c.home_direction = -1;
    c.minimum = 0; c.maximum = 100; c.max_speed = 100; c.acceleration = 100;
    c.home_steps = 110; c.home_speed = 20; c.home_timeout_ms = 10000;
    c.home_position = 0; c.watchdog_ms = 1000;
    return c;
}
void setUp() { controller.reset(); driver = FakeDriver(); }
void tearDown() {}
void run(uint32_t start, uint32_t count, uint32_t increment = 10000) {
    for (uint32_t i = 1; i <= count; ++i) {
        controller.keepAlive(start + i * increment);
        controller.update(start + i * increment);
    }
}
void test_unknown_reference_and_bounded_home() {
    TEST_ASSERT_EQUAL(OK, controller.configure(0, config(), 0));
    TEST_ASSERT_EQUAL(UNREFERENCED, controller.moveTo(0, 50, 0));
    TEST_ASSERT_EQUAL(OK, controller.home(0, 0));
    run(0, 550);
    TEST_ASSERT_TRUE(controller.axis(0).known);
    TEST_ASSERT_EQUAL(IDLE, controller.axis(0).state);
    TEST_ASSERT_EQUAL(110, driver.writes - 1); // initial energize + bounded phase transitions
    TEST_ASSERT_EQUAL(0, controller.axis(0).position);
}
void test_release_and_fault_forget_reference() {
    Config c = config(); c.home_strategy = 0;
    TEST_ASSERT_EQUAL(OK, controller.configure(0, c, 0));
    TEST_ASSERT_EQUAL(OK, controller.reference(0, 20, 0));
    TEST_ASSERT_EQUAL(OK, controller.release(0));
    TEST_ASSERT_FALSE(controller.axis(0).known);
    TEST_ASSERT_EQUAL(OK, controller.reference(0, 20, 0));
    controller.moveTo(0, 50, 0); driver.fail = true; run(0, 20);
    TEST_ASSERT_EQUAL(FAULT, controller.axis(0).state);
    TEST_ASSERT_FALSE(controller.axis(0).known);
}
void test_latest_target_limits_and_two_ports() {
    Config c = config(); c.home_strategy = 0;
    controller.configure(0, c, 0); c.port = 2;
    TEST_ASSERT_EQUAL(OK, controller.configure(1, c, 0));
    controller.reference(0, 0, 0); controller.reference(1, 0, 0);
    TEST_ASSERT_EQUAL(LIMIT, controller.moveTo(0, 101, 0));
    controller.moveTo(0, 90, 0); controller.moveTo(1, 20, 0); run(0, 30);
    controller.moveTo(0, 10, 300000); run(300000, 600);
    TEST_ASSERT_EQUAL(10, controller.axis(0).position);
    TEST_ASSERT_EQUAL(20, controller.axis(1).position);
    TEST_ASSERT_EQUAL(6, driver.ports);
}
void test_wrap_watchdog_homing_timeout_and_reconfigure() {
    Config c = config(); uint32_t start = 0xffff0000UL;
    controller.configure(0, c, start); controller.home(0, start);
    controller.update(start + 50000); TEST_ASSERT_EQUAL(1, driver.writes - 1);
    controller.update(start + 1000001); TEST_ASSERT_EQUAL(HOST_LOST, controller.axis(0).fault);
    TEST_ASSERT_FALSE(controller.axis(0).known);
    c.home_timeout_ms = 10;
    controller.configure(0, c, 0); controller.home(0, 0); controller.update(10000);
    TEST_ASSERT_EQUAL(HOME_TIMEOUT, controller.axis(0).fault);
    TEST_ASSERT_EQUAL(OK, controller.configure(0, config(), 0));
    TEST_ASSERT_FALSE(controller.axis(0).known);
    TEST_ASSERT_GREATER_THAN(0, driver.releases);
}
void test_reject_bad_configuration_and_resource_collision() {
    Config c = config(); c.driver = 2;
    TEST_ASSERT_EQUAL(UNSUPPORTED, controller.configure(0, c, 0));
    c = config(); c.maximum = c.minimum;
    TEST_ASSERT_EQUAL(INVALID, controller.configure(0, c, 0));
    c = config(); controller.configure(0, c, 0);
    TEST_ASSERT_EQUAL(CONFLICT, controller.configure(1, c, 0));
    c.port = 2; c.address = 0x61;
    TEST_ASSERT_EQUAL(CONFLICT, controller.configure(1, c, 0));
    TEST_ASSERT_EQUAL(BAD_AXIS, controller.configure(2, c, 0));
}
void test_wire_strict_sizes_and_endianness() {
    uint8_t bytes[] = {16,1,1,0,0x34,0x12,0x78,0x56,0x34,0x12};
    Command cmd;
    TEST_ASSERT_TRUE(cmd.decode(bytes, sizeof(bytes)));
    TEST_ASSERT_EQUAL(0x1234, cmd.request_id);
    TEST_ASSERT_EQUAL(0x12345678, cmd.position);
    TEST_ASSERT_FALSE(cmd.decode(bytes, sizeof(bytes)-1));
    bytes[1] = 2; TEST_ASSERT_FALSE(cmd.decode(bytes, sizeof(bytes)));
    bytes[1] = 1; bytes[2] = 99; TEST_ASSERT_FALSE(cmd.decode(bytes, sizeof(bytes)));
}
void test_backoff_reference_kind_stop_and_release_idle() {
    Config c = config(); c.home_steps = 2; c.home_backoff = 1;
    controller.configure(0,c,0); controller.home(0,0);
    run(0,10); TEST_ASSERT_FALSE(controller.axis(0).known);
    TEST_ASSERT_EQUAL(4, driver.last_phase); // two negative transitions
    run(100000,5); TEST_ASSERT_TRUE(controller.axis(0).known);
    TEST_ASSERT_EQUAL(5, driver.last_phase); // one positive backoff
    TEST_ASSERT_EQUAL(1, controller.axis(0).reference_kind);
    controller.moveTo(0,50,150000); controller.stop(0);
    TEST_ASSERT_FALSE(controller.axis(0).known);
    c.hold = 0; c.home_steps = 1; c.home_backoff = 0;
    controller.configure(0,c,0); controller.home(0,0); run(0,5);
    TEST_ASSERT_FALSE(controller.axis(0).known);
    TEST_ASSERT_EQUAL(DISABLED, controller.axis(0).state);
}
void test_reversal_brakes_and_stays_inside_limits() {
    Config c = config(); c.home_strategy = 0; c.maximum = 1000;
    controller.configure(0,c,0); controller.reference(0,100,0); controller.moveTo(0,900,0);
    run(0,100); int32_t at_retarget = controller.axis(0).position;
    TEST_ASSERT_GREATER_THAN(100,at_retarget);
    uint32_t fast = controller.axis(0).speed_squared;
    controller.moveTo(0,0,1000000); run(1000000,1);
    TEST_ASSERT_GREATER_THAN(at_retarget,controller.axis(0).position); // brakes before reversing
    TEST_ASSERT_LESS_THAN(fast,controller.axis(0).speed_squared);
    for (uint32_t i = 102; i < 1200; ++i) {
        uint32_t now = i * 10000; controller.keepAlive(now); controller.update(now);
        TEST_ASSERT_GREATER_OR_EQUAL(0,controller.axis(0).position);
        TEST_ASSERT_LESS_OR_EQUAL(1000,controller.axis(0).position);
    }
    TEST_ASSERT_EQUAL(0,controller.axis(0).position);
    TEST_ASSERT_EQUAL(IDLE,controller.axis(0).state);
}
void test_extreme_configuration_shared_fault_and_mode_phasecount() {
    Config c = config(); c.minimum = INT32_MIN;
    TEST_ASSERT_EQUAL(INVALID,controller.configure(0,c,0));
    c = config(); c.maximum = INT32_MAX;
    TEST_ASSERT_EQUAL(INVALID,controller.configure(0,c,0));
    c = config(); c.home_strategy = 0; c.minimum = -1000000; c.maximum = 1000000;
    c.acceleration = 10000; c.max_speed = 200; c.mode = 2;
    TEST_ASSERT_EQUAL(OK,controller.configure(0,c,0));
    controller.reference(0,0,0); controller.moveTo(0,1000000,0); run(0,10);
    TEST_ASSERT_LESS_THAN(4,controller.axis(0).phase);
    c.port = 2; c.mode = 3; controller.configure(1,c,100000); controller.reference(1,0,100000);
    driver.fail = true; run(100000,10);
    TEST_ASSERT_FALSE(controller.axis(0).known); TEST_ASSERT_FALSE(controller.axis(1).known);
    TEST_ASSERT_EQUAL(FAULT,controller.axis(1).state);
}
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_unknown_reference_and_bounded_home);
    RUN_TEST(test_release_and_fault_forget_reference);
    RUN_TEST(test_latest_target_limits_and_two_ports);
    RUN_TEST(test_wrap_watchdog_homing_timeout_and_reconfigure);
    RUN_TEST(test_reject_bad_configuration_and_resource_collision);
    RUN_TEST(test_wire_strict_sizes_and_endianness);
    RUN_TEST(test_backoff_reference_kind_stop_and_release_idle);
    RUN_TEST(test_reversal_brakes_and_stays_inside_limits);
    RUN_TEST(test_extreme_configuration_shared_fault_and_mode_phasecount);
    return UNITY_END();
}
