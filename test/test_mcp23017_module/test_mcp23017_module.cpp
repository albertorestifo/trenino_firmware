#define ARDUINO_ARCH_AVR
#ifndef UNIT_TEST
#define UNIT_TEST
#endif
#include "../Wire.h"
#include "../../src/i2c_bus.cpp"
#include "../../src/mcp23017_module.cpp"
#include <unity.h>

using namespace Modules;

static unsigned long g_now = 0;
unsigned long millis() { return g_now; }
void delay(unsigned long) {}

static const uint8_t ADDR = 0x20;
static const uint8_t BASE = 200;

void setUp()
{
    MockWire::reset();
    I2CBus::resetForTest();
    g_now = 0;
}

void tearDown() {}

// One scripted sample: GPIOA, GPIOB, then the OLAT bytes a healthy chip returns.
static void queueSample(uint8_t gpioA, uint8_t gpioB)
{
    const uint8_t bytes[4] = { gpioA, gpioB, 0x01, 0x00 };
    MockWire::queueRead(ADDR, bytes, 4);
}

static void queueShort()
{
    const uint8_t bytes[3] = { 0xFF, 0xFF, 0x01 };
    MockWire::queueRead(ADDR, bytes, 3);
}

// Advances 5 ms and scans once.
static void tick(Mcp23017Module& m)
{
    g_now += 5;
    m.scan();
}

static Mcp23017Module make(uint16_t direct_mask = 0x0003, uint8_t debounce = 3, uint16_t row_mask = 0)
{
    return Mcp23017Module(ADDR, BASE, 0, debounce, direct_mask, row_mask, 0);
}

void test_begin_writes_expected_registers()
{
    Mcp23017Module m = make(0x0003);
    TEST_ASSERT_TRUE(m.begin());
    TEST_ASSERT_EQUAL_UINT8(4, MockWire::transaction_count);

    const uint8_t expect[4][3] = {
        { 0x0A, 0x00, 0x00 }, // IOCON
        { 0x14, 0x01, 0x00 }, // OLAT: sentinel bit 0
        { 0x0C, 0x7F, 0x7F }, // GPPU
        { 0x00, 0x7F, 0x7F }, // IODIR
    };
    const uint8_t len[4] = { 2, 3, 3, 3 };
    for (uint8_t i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_UINT8(ADDR, MockWire::transactions[i].address);
        TEST_ASSERT_EQUAL_UINT8(len[i], MockWire::transactions[i].length);
        for (uint8_t j = 0; j < len[i]; j++) {
            TEST_ASSERT_EQUAL_UINT8(expect[i][j], MockWire::transactions[i].data[j]);
        }
    }
    // GPA7 and GPB7 are never inputs.
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::transactions[3].data[1] & 0x80);
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::transactions[3].data[2] & 0x80);
}

void test_begin_nack_returns_false()
{
    MockWire::scheduleNacks(ADDR, 1);
    Mcp23017Module m = make();
    TEST_ASSERT_FALSE(m.begin());
}

void test_no_read_before_5ms()
{
    Mcp23017Module m = make();
    m.begin();
    g_now = 4;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::read_request_count);
    g_now = 5;
    queueSample(0xFF, 0xFF);
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::read_request_count);
}

void test_reads_four_bytes_from_gpioa()
{
    Mcp23017Module m = make();
    m.begin();
    MockWire::transaction_count = 0;
    queueSample(0xFF, 0xFF);
    tick(m);
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transactions[0].length);
    TEST_ASSERT_EQUAL_UINT8(0x12, MockWire::transactions[0].data[0]);
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::read_request_count);
}

void test_press_reported_after_debounce_samples()
{
    Mcp23017Module m = make(0x0003, 3);
    m.begin();
    for (uint8_t i = 0; i < 2; i++) {
        queueSample(0xFE, 0xFF);
        tick(m);
        TEST_ASSERT_FALSE(m.getReading().has_value);
    }
    queueSample(0xFE, 0xFF);
    tick(m);
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(1, r.value);
    TEST_ASSERT_TRUE(r.type == ModuleType::MCP23017);
    TEST_ASSERT_EQUAL_UINT8(BASE + 0, r.pin);
    TEST_ASSERT_FALSE(m.getReading().has_value);
}

void test_release_reported()
{
    Mcp23017Module m = make(0x0003, 2);
    m.begin();
    for (uint8_t i = 0; i < 2; i++) { queueSample(0xFD, 0xFF); tick(m); }
    TEST_ASSERT_TRUE(m.getReading().has_value);
    for (uint8_t i = 0; i < 2; i++) { queueSample(0xFF, 0xFF); tick(m); }
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(0, r.value);
    TEST_ASSERT_EQUAL_UINT8(BASE + 1, r.pin);
}

void test_two_pins_change_two_readings()
{
    Mcp23017Module m = make(0x0103, 1);
    m.begin();
    queueSample(0xFE, 0xFE); // GPA0 and GPB0 low
    tick(m);
    Reading a = m.getReading();
    Reading b = m.getReading();
    TEST_ASSERT_TRUE(a.has_value);
    TEST_ASSERT_TRUE(b.has_value);
    TEST_ASSERT_EQUAL_UINT8(BASE + 0, a.pin);
    TEST_ASSERT_EQUAL_UINT8(BASE + 8, b.pin);
    TEST_ASSERT_FALSE(m.getReading().has_value);
}

void test_held_at_boot_reports_press()
{
    g_now = 1000;
    Mcp23017Module m = make(0x0001, 3);
    m.begin();
    for (uint8_t i = 0; i < 3; i++) { queueSample(0xFE, 0xFF); tick(m); }
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(1, r.value);
    TEST_ASSERT_EQUAL_UINT8(BASE, r.pin);
}

void test_pins_outside_mask_ignored()
{
    Mcp23017Module m = make(0x0001, 1);
    m.begin();
    queueSample(0xFE & 0x7F, 0x00); // bit 0 low, everything else low including GPA7/GPB7
    tick(m);
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_UINT8(BASE, r.pin);
    TEST_ASSERT_FALSE(m.getReading().has_value);

    // GPA7 in direct_mask is stripped as well.
    Mcp23017Module m2 = make(0x8080, 1);
    m2.begin();
    queueSample(0x00, 0x00);
    tick(m2);
    TEST_ASSERT_FALSE(m2.getReading().has_value);
}

void test_short_read_counts_as_failure()
{
    Mcp23017Module m = make(0x0001, 3);
    m.begin();
    for (uint8_t i = 0; i < 2; i++) { queueSample(0xFE, 0xFF); tick(m); }
    queueShort();
    tick(m);
    TEST_ASSERT_EQUAL_UINT8(1, m.health().failures());
    TEST_ASSERT_FALSE(m.getReading().has_value);
    // The failed sample left the debounce counter alone.
    queueSample(0xFE, 0xFF);
    tick(m);
    TEST_ASSERT_TRUE(m.getReading().has_value);
    TEST_ASSERT_EQUAL_UINT8(0, m.health().failures());
}

void test_pointer_write_nack_counts_as_failure()
{
    Mcp23017Module m = make(0x0001, 1);
    m.begin();
    MockWire::scheduleNacks(ADDR, 1);
    queueSample(0xFE, 0xFF);
    tick(m);
    TEST_ASSERT_EQUAL_UINT8(1, m.health().failures());
    TEST_ASSERT_FALSE(m.getReading().has_value);
}

void test_sentinel_exists_for_direct_only_chip()
{
    Mcp23017Module m = make(0x0003, 3, 0);
    TEST_ASSERT_EQUAL_UINT16(0x0001, m.sentinelBit());
    m.begin();
    TEST_ASSERT_EQUAL_UINT8(0x01, MockWire::transactions[1].data[1]);

    // With rows on bits 0-2 the sentinel is the next free bit and rows stay high.
    MockWire::reset();
    I2CBus::resetForTest();
    Mcp23017Module m2 = make(0x0000, 2, 0x0007);
    TEST_ASSERT_EQUAL_UINT16(0x0008, m2.sentinelBit());
    m2.begin();
    TEST_ASSERT_EQUAL_UINT8(0x0F, MockWire::transactions[1].data[1]);
}

void test_timer_survives_millis_wrap()
{
    g_now = ~0UL - 2;
    Mcp23017Module m = make(0x0001, 1);
    m.begin();
    g_now += 4; // 4 ms later, counter has wrapped
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::read_request_count);
    g_now += 1;
    queueSample(0xFE, 0xFF);
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::read_request_count);
    TEST_ASSERT_TRUE(m.getReading().has_value);
}

// --- I2CHealth ---

void test_health_threshold_and_recovery()
{
    I2CHealth h;
    TEST_ASSERT_FALSE(h.noteFailure(0));
    TEST_ASSERT_FALSE(h.noteFailure(1));
    TEST_ASSERT_FALSE(h.failed());
    TEST_ASSERT_TRUE(h.noteFailure(2));
    TEST_ASSERT_TRUE(h.failed());
    TEST_ASSERT_FALSE(h.noteFailure(3)); // already failed
    TEST_ASSERT_TRUE(h.noteSuccess());
    TEST_ASSERT_FALSE(h.failed());
    TEST_ASSERT_FALSE(h.noteSuccess());
}

void test_health_success_resets_count()
{
    I2CHealth h;
    h.noteFailure(0);
    h.noteFailure(0);
    h.noteSuccess();
    TEST_ASSERT_FALSE(h.noteFailure(0));
    TEST_ASSERT_FALSE(h.noteFailure(0));
    TEST_ASSERT_FALSE(h.failed());
}

void test_health_retry_every_1000ms_across_wrap()
{
    I2CHealth h;
    unsigned long t = ~0UL - 500;
    h.noteFailure(t);
    h.noteFailure(t);
    h.noteFailure(t);
    TEST_ASSERT_FALSE(h.retryDue(t + 999));
    TEST_ASSERT_TRUE(h.retryDue(t + 1000));
    h.noteFailure(t + 1000); // failed retry restarts the timer
    TEST_ASSERT_FALSE(h.retryDue(t + 1999));
    TEST_ASSERT_TRUE(h.retryDue(t + 2000));
    h.noteSuccess();
    TEST_ASSERT_FALSE(h.retryDue(t + 5000));
}

void test_health_error_queue()
{
    I2CHealth h;
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, h.take());
    h.push(1);
    h.push(2);
    TEST_ASSERT_EQUAL_UINT8(1, h.take());
    TEST_ASSERT_EQUAL_UINT8(2, h.take());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, h.take());
}

int main(int, char**)
{
    UNITY_BEGIN();
    RUN_TEST(test_begin_writes_expected_registers);
    RUN_TEST(test_begin_nack_returns_false);
    RUN_TEST(test_no_read_before_5ms);
    RUN_TEST(test_reads_four_bytes_from_gpioa);
    RUN_TEST(test_press_reported_after_debounce_samples);
    RUN_TEST(test_release_reported);
    RUN_TEST(test_two_pins_change_two_readings);
    RUN_TEST(test_held_at_boot_reports_press);
    RUN_TEST(test_pins_outside_mask_ignored);
    RUN_TEST(test_short_read_counts_as_failure);
    RUN_TEST(test_pointer_write_nack_counts_as_failure);
    RUN_TEST(test_sentinel_exists_for_direct_only_chip);
    RUN_TEST(test_timer_survives_millis_wrap);
    RUN_TEST(test_health_threshold_and_recovery);
    RUN_TEST(test_health_success_resets_count);
    RUN_TEST(test_health_retry_every_1000ms_across_wrap);
    RUN_TEST(test_health_error_queue);
    return UNITY_END();
}
