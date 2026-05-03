#define UNIT_TEST
#include "../Wire.h"
#include "../../src/ht16k33_module.cpp"
#include <unity.h>

using namespace Modules;

void delay(unsigned long /*ms*/) {}

void setUp()
{
    MockWire::reset();
    ht16k33_reset_wire_state();
}

void tearDown() {}

// Test that begin() runs the four init transactions in order, on success
void test_begin_emits_init_sequence_on_success()
{
    HT16K33Module mod(0x70, 8, 4);

    bool ok = mod.begin();

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(4, MockWire::transaction_count);

    TEST_ASSERT_EQUAL_UINT8(0x70, MockWire::transactions[0].address);
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transactions[0].length);
    TEST_ASSERT_EQUAL_UINT8(0x21, MockWire::transactions[0].data[0]);

    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transactions[1].length);
    TEST_ASSERT_EQUAL_UINT8(0x81, MockWire::transactions[1].data[0]);

    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transactions[2].length);
    TEST_ASSERT_EQUAL_UINT8(0xE8, MockWire::transactions[2].data[0]); // 0xE0 | 8

    TEST_ASSERT_EQUAL_UINT8(17, MockWire::transactions[3].length); // 1 reg + 16 data
    TEST_ASSERT_EQUAL_UINT8(0x00, MockWire::transactions[3].data[0]);
    for (uint8_t i = 1; i < 17; i++) {
        TEST_ASSERT_EQUAL_UINT8(0x00, MockWire::transactions[3].data[i]);
    }
}

void test_begin_initializes_wire()
{
    HT16K33Module mod(0x70, 0, 4);

    TEST_ASSERT_FALSE(MockWire::wireBeginCalled());
    mod.begin();
    TEST_ASSERT_TRUE(MockWire::wireBeginCalled());
}

void test_constructor_clamps_brightness()
{
    HT16K33Module mod(0x70, 99, 4);
    TEST_ASSERT_EQUAL_UINT8(15, mod.getCachedBrightness());
}

// D3
void test_begin_fails_on_first_step_nack()
{
    MockWire::scheduleNacks(0x70, 1);
    HT16K33Module mod(0x70, 0, 4);

    bool ok = mod.begin();

    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_TRUE(mod.needsReinit());
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transaction_count);
}

// D4
void test_writeSegments_writes_data_at_register_zero()
{
    HT16K33Module mod(0x70, 0, 4);
    mod.begin();
    MockWire::reset();

    uint8_t data[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };
    bool ok = mod.writeSegments(data, 8);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(0x70, MockWire::transactions[0].address);
    TEST_ASSERT_EQUAL_UINT8(9, MockWire::transactions[0].length);
    TEST_ASSERT_EQUAL_UINT8(0x00, MockWire::transactions[0].data[0]);
    for (uint8_t i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL_UINT8(data[i], MockWire::transactions[0].data[1 + i]);
    }
}

void test_writeSegments_caps_at_num_digits_times_two()
{
    HT16K33Module mod(0x70, 0, 4);
    mod.begin();
    MockWire::reset();

    uint8_t data[16];
    for (uint8_t i = 0; i < 16; i++) data[i] = i + 1;

    bool ok = mod.writeSegments(data, 16);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(9, MockWire::transactions[0].length);
}

void test_writeSegments_8_digit_module()
{
    HT16K33Module mod(0x70, 0, 8);
    mod.begin();
    MockWire::reset();

    uint8_t data[16];
    for (uint8_t i = 0; i < 16; i++) data[i] = 0xA0 + i;

    bool ok = mod.writeSegments(data, 16);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(17, MockWire::transactions[0].length);
    for (uint8_t i = 0; i < 16; i++) {
        TEST_ASSERT_EQUAL_UINT8(0xA0 + i, MockWire::transactions[0].data[1 + i]);
    }
}

void test_setBrightness_writes_command()
{
    HT16K33Module mod(0x70, 0, 4);
    mod.begin();
    MockWire::reset();

    bool ok = mod.setBrightness(7);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(0x70, MockWire::transactions[0].address);
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transactions[0].length);
    TEST_ASSERT_EQUAL_UINT8(0xE7, MockWire::transactions[0].data[0]);
}

void test_setBrightness_clamps()
{
    HT16K33Module mod(0x70, 0, 4);
    mod.begin();
    MockWire::reset();

    mod.setBrightness(99);

    TEST_ASSERT_EQUAL_UINT8(0xEF, MockWire::transactions[0].data[0]);
    TEST_ASSERT_EQUAL_UINT8(15, mod.getCachedBrightness());
}

// D5
void test_writeSegments_retries_once_on_transient_nack()
{
    HT16K33Module mod(0x70, 0, 4);
    mod.begin();
    MockWire::reset();
    MockWire::scheduleNacks(0x70, 1);

    uint8_t data[2] = { 0xAB, 0xCD };
    bool ok = mod.writeSegments(data, 2);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(2, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(0, mod.getFailureCount());
}

void test_writeSegments_two_nacks_increment_failure_count()
{
    HT16K33Module mod(0x70, 0, 4);
    mod.begin();
    MockWire::reset();
    MockWire::scheduleNacks(0x70, 2);

    uint8_t data[2] = { 0xAB, 0xCD };
    bool ok = mod.writeSegments(data, 2);

    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_UINT8(2, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(1, mod.getFailureCount());
    TEST_ASSERT_FALSE(mod.needsReinit());
}

// D6
void test_three_failures_set_needs_reinit()
{
    HT16K33Module mod(0x70, 0, 4);
    mod.begin();

    uint8_t data[2] = { 0xAB, 0xCD };

    MockWire::reset();
    MockWire::scheduleNacks(0x70, 2);
    mod.writeSegments(data, 2);
    TEST_ASSERT_EQUAL_UINT8(1, mod.getFailureCount());
    TEST_ASSERT_FALSE(mod.needsReinit());

    MockWire::reset();
    MockWire::scheduleNacks(0x70, 2);
    mod.writeSegments(data, 2);
    TEST_ASSERT_EQUAL_UINT8(2, mod.getFailureCount());
    TEST_ASSERT_FALSE(mod.needsReinit());

    MockWire::reset();
    MockWire::scheduleNacks(0x70, 2);
    mod.writeSegments(data, 2);
    TEST_ASSERT_EQUAL_UINT8(0, mod.getFailureCount());
    TEST_ASSERT_TRUE(mod.needsReinit());
}

void test_next_write_after_needs_reinit_runs_init_and_restores_cache()
{
    HT16K33Module mod(0x70, 5, 4);
    mod.begin();

    uint8_t initial[4] = { 0x11, 0x22, 0x33, 0x44 };
    mod.writeSegments(initial, 4);

    uint8_t data[2] = { 0xAA, 0xBB };
    for (uint8_t i = 0; i < 3; i++) {
        MockWire::scheduleNacks(0x70, 2);
        mod.writeSegments(data, 2);
    }
    TEST_ASSERT_TRUE(mod.needsReinit());

    MockWire::reset();
    uint8_t newdata[2] = { 0xCC, 0xDD };
    bool ok = mod.writeSegments(newdata, 2);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(6, MockWire::transaction_count); // 4 init + 1 cache restore + 1 new write

    TEST_ASSERT_EQUAL_UINT8(0x21, MockWire::transactions[0].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x81, MockWire::transactions[1].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0xE5, MockWire::transactions[2].data[0]); // brightness 5
    TEST_ASSERT_EQUAL_UINT8(0x00, MockWire::transactions[3].data[0]); // zero RAM

    // Cache restore: 1 register + 2 cached bytes from the failed-write attempts
    TEST_ASSERT_EQUAL_UINT8(3, MockWire::transactions[4].length);
    TEST_ASSERT_EQUAL_UINT8(0x00, MockWire::transactions[4].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0xAA, MockWire::transactions[4].data[1]);
    TEST_ASSERT_EQUAL_UINT8(0xBB, MockWire::transactions[4].data[2]);

    // New write
    TEST_ASSERT_EQUAL_UINT8(3, MockWire::transactions[5].length);
    TEST_ASSERT_EQUAL_UINT8(0x00, MockWire::transactions[5].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0xCC, MockWire::transactions[5].data[1]);
    TEST_ASSERT_EQUAL_UINT8(0xDD, MockWire::transactions[5].data[2]);

    TEST_ASSERT_FALSE(mod.needsReinit());
    TEST_ASSERT_EQUAL_UINT8(0, mod.getFailureCount());
}

void test_brightness_change_persists_through_reinit()
{
    HT16K33Module mod(0x70, 0, 4);
    mod.begin();

    mod.setBrightness(12);
    TEST_ASSERT_EQUAL_UINT8(12, mod.getCachedBrightness());

    uint8_t data[2] = { 0x11, 0x22 };
    for (uint8_t i = 0; i < 3; i++) {
        MockWire::scheduleNacks(0x70, 2);
        mod.writeSegments(data, 2);
    }
    TEST_ASSERT_TRUE(mod.needsReinit());

    MockWire::reset();
    mod.writeSegments(data, 2);

    TEST_ASSERT_EQUAL_UINT8(0xEC, MockWire::transactions[2].data[0]); // 0xE0 | 12
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_begin_emits_init_sequence_on_success);
    RUN_TEST(test_begin_initializes_wire);
    RUN_TEST(test_constructor_clamps_brightness);
    RUN_TEST(test_begin_fails_on_first_step_nack);
    RUN_TEST(test_writeSegments_writes_data_at_register_zero);
    RUN_TEST(test_writeSegments_caps_at_num_digits_times_two);
    RUN_TEST(test_writeSegments_8_digit_module);
    RUN_TEST(test_setBrightness_writes_command);
    RUN_TEST(test_setBrightness_clamps);
    RUN_TEST(test_writeSegments_retries_once_on_transient_nack);
    RUN_TEST(test_writeSegments_two_nacks_increment_failure_count);
    RUN_TEST(test_three_failures_set_needs_reinit);
    RUN_TEST(test_next_write_after_needs_reinit_runs_init_and_restores_cache);
    RUN_TEST(test_brightness_change_persists_through_reinit);
    return UNITY_END();
}
