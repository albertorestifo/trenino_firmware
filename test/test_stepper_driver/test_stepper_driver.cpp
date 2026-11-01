#define ARDUINO_ARCH_AVR
#ifndef UNIT_TEST
#define UNIT_TEST
#endif
#include "../Wire.h"
#include "../../src/i2c_bus.cpp"
#include "../../src/stepper_driver.cpp"
#include <unity.h>
using namespace Stepper;
void setUp() { MockWire::reset(); I2CBus::resetForTest(); }
void tearDown() {}
void test_init_disables_allcall_and_shares_device() {
    ShieldDriver d;
    TEST_ASSERT_TRUE(d.begin(0x60));
    TEST_ASSERT_EQUAL(0x00, MockWire::transactions[0].data[0]);
    TEST_ASSERT_EQUAL(0x20, MockWire::transactions[0].data[1]);
    uint8_t count = MockWire::transaction_count;
    TEST_ASSERT_TRUE(d.begin(0x60));
    TEST_ASSERT_EQUAL(count, MockWire::transaction_count);
}
void test_six_states_port_mapping_and_burst_size() {
    ShieldDriver d; d.begin(0x60); MockWire::reset();
    TEST_ASSERT_TRUE(d.write(0x60, 1, 1, 0));
    TEST_ASSERT_EQUAL(25, MockWire::transactions[0].length);
    TEST_ASSERT_EQUAL(0x26, MockWire::transactions[0].data[0]);
    // Phase0 mask9: AIN1 high, AIN2 low, BIN1 low, BIN2 high.
    TEST_ASSERT_EQUAL(0x10, MockWire::transactions[0].data[10]);
    TEST_ASSERT_EQUAL(0x10, MockWire::transactions[0].data[18]);
    TEST_ASSERT_TRUE(d.write(0x60, 2, 1, 5));
    TEST_ASSERT_EQUAL(0x0e, MockWire::transactions[1].data[0]);
    TEST_ASSERT_TRUE(d.release(0x60, 2));
    TEST_ASSERT_EQUAL(0x10, MockWire::transactions[2].data[4]); // PWMA full off
    TEST_ASSERT_EQUAL(0x10, MockWire::transactions[2].data[24]); // PWMB full off
}
void test_nack_and_invalid_phase_report_failure() {
    ShieldDriver d; d.begin(0x60);
    MockWire::scheduleNacks(0x60, 1);
    TEST_ASSERT_FALSE(d.write(0x60, 1, 1, 0));
    TEST_ASSERT_FALSE(d.write(0x60, 1, 1, 6));
    TEST_ASSERT_FALSE(d.write(0x60, 0, 1, 0));
}
void test_every_commutation_pattern_both_motor_ports() {
    const uint8_t expected[3][8] = {{9,1,7,6,14,8,0,0},{5,6,10,9,0,0,0,0},{1,5,4,6,2,10,8,9}};
    const uint8_t count[3] = {6,4,8};
    ShieldDriver d; TEST_ASSERT_TRUE(d.begin(0x60));
    for (uint8_t port = 1; port <= 2; ++port) {
        MockWire::reset();
        for (uint8_t mode = 1; mode <= 3; ++mode) {
            TEST_ASSERT_EQUAL(count[mode-1],d.phaseCount(mode));
            for (uint8_t phase = 0; phase < count[mode-1]; ++phase) {
                TEST_ASSERT_TRUE(d.write(0x60,port,mode,phase));
                const MockWire::Transaction& t = MockWire::transactions[MockWire::transaction_count-1];
                uint8_t bits = (t.data[10] ? 1 : 0) | (t.data[6] ? 2 : 0) |
                    (t.data[14] ? 4 : 0) | (t.data[18] ? 8 : 0);
                TEST_ASSERT_EQUAL(expected[mode-1][phase],bits);
                TEST_ASSERT_EQUAL(25,t.length);
            }
        }
    }
}
int main() {
    UNITY_BEGIN(); RUN_TEST(test_init_disables_allcall_and_shares_device);
    RUN_TEST(test_six_states_port_mapping_and_burst_size);
    RUN_TEST(test_nack_and_invalid_phase_report_failure);
    RUN_TEST(test_every_commutation_pattern_both_motor_ports); return UNITY_END();
}
