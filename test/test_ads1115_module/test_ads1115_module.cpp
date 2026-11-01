#define ARDUINO_ARCH_AVR
#ifndef UNIT_TEST
#define UNIT_TEST
#endif
#include "../Wire.h"
#include "../../src/i2c_bus.cpp"
#include "../../src/ads1115_module.cpp"
#include <unity.h>

using namespace Modules;

static unsigned long g_now = 0;
unsigned long millis() { return g_now; }
void delay(unsigned long) {}

static const uint8_t ADDR = 0x48;
static const uint8_t BASE = 10;
static const uint8_t NO_REF = Protocol::ADS_NO_REFERENCE;
static const uint8_t SENS10[4] = { 10, 10, 10, 10 };
static const uint8_t SENS1[4] = { 1, 1, 1, 1 };

void setUp()
{
    MockWire::reset();
    I2CBus::resetForTest();
    g_now = 0;
}

void tearDown() {}

static uint16_t cfgWord(uint8_t channel, uint8_t gain)
{
    return (uint16_t)(0xC1A3 | (channel << 12) | (gain << 9));
}

// Starts a conversion (asserting the channel and gain written), waits 5 ms and
// reads it back with `raw` as the result. The chip reports the config with OS set.
static void cycle(Ads1115Module& m, uint8_t channel, int16_t raw, uint8_t gain = 0)
{
    MockWire::transaction_count = 0;
    m.scan();
    uint16_t w = cfgWord(channel, gain);
    TEST_ASSERT_TRUE(MockWire::transaction_count >= 1);
    TEST_ASSERT_EQUAL_UINT8(3, MockWire::transactions[0].length);
    TEST_ASSERT_EQUAL_UINT8(0x01, MockWire::transactions[0].data[0]);
    TEST_ASSERT_EQUAL_UINT8(w >> 8, MockWire::transactions[0].data[1]);
    TEST_ASSERT_EQUAL_UINT8(w & 0xFF, MockWire::transactions[0].data[2]);
    g_now += 5;
    const uint8_t cfg[2] = { (uint8_t)(w >> 8), (uint8_t)(w & 0xFF) };
    const uint8_t res[2] = { (uint8_t)((uint16_t)raw >> 8), (uint8_t)((uint16_t)raw & 0xFF) };
    MockWire::queueRead(ADDR, cfg, 2);
    MockWire::queueRead(ADDR, res, 2);
    m.scan();
}

static Ads1115Module plain(uint8_t mask, const uint8_t* sens = SENS10)
{
    return Ads1115Module(ADDR, BASE, mask, 0, NO_REF, sens);
}

static Ads1115Module withRef(uint8_t mask, uint8_t ref = 3)
{
    return Ads1115Module(ADDR, BASE, mask, 0, ref, SENS10);
}

// Seeds the reference with 26000 counts.
static void seedRef(Ads1115Module& m, uint8_t ref = 3)
{
    cycle(m, ref, 26000);
}

void test_scaled_without_ref()
{
    TEST_ASSERT_EQUAL_UINT16(833, Ads1115Module::scaled(26666, 0));
    TEST_ASSERT_EQUAL_UINT16(1023, Ads1115Module::scaled(32767, 0));
}

void test_scaled_with_ref()
{
    TEST_ASSERT_EQUAL_UINT16(1023, Ads1115Module::scaled(26000, 26000));
    TEST_ASSERT_EQUAL_UINT16(511, Ads1115Module::scaled(13000, 26000));
}

void test_scaled_clamps()
{
    TEST_ASSERT_EQUAL_UINT16(1023, Ads1115Module::scaled(27000, 26000));
    TEST_ASSERT_EQUAL_UINT16(0, Ads1115Module::scaled(-5, 26000));
    TEST_ASSERT_EQUAL_UINT16(0, Ads1115Module::scaled(-5, 0));
}

void test_begin_probes_with_idle_config_and_no_conversion()
{
    Ads1115Module m = plain(0x01);
    TEST_ASSERT_TRUE(m.begin());
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(0x01, MockWire::transactions[0].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::transactions[0].data[1] & 0x80); // OS clear
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
}

void test_config_word_channel_2_gain_1()
{
    Ads1115Module m = Ads1115Module(ADDR, BASE, 0x04, 1, NO_REF, SENS10);
    m.begin();
    MockWire::transaction_count = 0;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(3, MockWire::transactions[0].length);
    TEST_ASSERT_EQUAL_HEX8(0x01, MockWire::transactions[0].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0xE3, MockWire::transactions[0].data[1]);
    TEST_ASSERT_EQUAL_HEX8(0xA3, MockWire::transactions[0].data[2]);
}

void test_no_read_before_5ms()
{
    Ads1115Module m = plain(0x01);
    m.begin();
    m.scan();
    g_now += 4;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::read_request_count);
    g_now += 1;
    const uint8_t cfg[2] = { 0xC1, 0xA3 };
    const uint8_t res[2] = { 0x06, 0x40 };
    MockWire::queueRead(ADDR, cfg, 2);
    MockWire::queueRead(ADDR, res, 2);
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(2, MockWire::read_request_count);
    TEST_ASSERT_EQUAL_UINT8(2, MockWire::last_request_quantity);
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(0x0640 >> 5, r.value); // 1600 -> 50, MSB first
    TEST_ASSERT_EQUAL_UINT8(BASE, r.pin);
    TEST_ASSERT_EQUAL(ModuleType::ADS1115, r.type);
}

void test_value_is_signed_16_bit()
{
    Ads1115Module m = plain(0x01);
    m.begin();
    cycle(m, 0, (int16_t)0xFFF0); // negative -> 0, equals last sent: no change
    TEST_ASSERT_FALSE(m.getReading().has_value);
}

void test_rotation_skips_disabled_channels()
{
    Ads1115Module m = plain(0x05);
    m.begin();
    cycle(m, 0, 0);
    cycle(m, 2, 0);
    cycle(m, 0, 0);
    cycle(m, 2, 0);
}

void test_sends_on_change_after_rate_limit()
{
    Ads1115Module hi = plain(0x01, SENS10);
    hi.begin();
    cycle(hi, 0, 1600);
    TEST_ASSERT_EQUAL_INT16(50, hi.getReading().value);
    cycle(hi, 0, 3200);
    Reading r = hi.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(100, r.value);

    setUp();
    Ads1115Module lo = plain(0x01, SENS1);
    lo.begin();
    for (int i = 0; i < 9; i++) {
        cycle(lo, 0, 3200);
        TEST_ASSERT_FALSE(lo.getReading().has_value);
    }
    cycle(lo, 0, 3200);
    r = lo.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(100, r.value);
}

void test_change_within_dead_zone_not_sent()
{
    Ads1115Module m = plain(0x01);
    m.begin();
    cycle(m, 0, 1600); // 50
    TEST_ASSERT_TRUE(m.getReading().has_value);
    cycle(m, 0, 52 << 5); // delta 2: ignored
    TEST_ASSERT_FALSE(m.getReading().has_value);
    cycle(m, 0, 53 << 5); // delta 3: sent
    TEST_ASSERT_EQUAL_INT16(53, m.getReading().value);
}

void test_still_channel_sends_every_100ms()
{
    Ads1115Module m = plain(0x01);
    m.begin();
    cycle(m, 0, 1600);
    TEST_ASSERT_TRUE(m.getReading().has_value);
    unsigned long sent_at = g_now;
    int sends = 0;
    for (int i = 0; i < 60; i++) {
        cycle(m, 0, 1600);
        if (m.getReading().has_value) {
            TEST_ASSERT_TRUE(g_now - sent_at >= 100);
            TEST_ASSERT_TRUE(g_now - sent_at < 110);
            sent_at = g_now;
            sends++;
        }
    }
    TEST_ASSERT_EQUAL_INT(3, sends); // 300 ms
}

void test_reference_channel_never_reported()
{
    Ads1115Module m = withRef(0x01);
    m.begin();
    seedRef(m);
    TEST_ASSERT_FALSE(m.getReading().has_value);
    cycle(m, 0, 13000);
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_UINT8(BASE, r.pin);
    TEST_ASSERT_EQUAL_INT16(511, r.value);
    cycle(m, 3, 26000);
    TEST_ASSERT_FALSE(m.getReading().has_value);
}

void test_reference_smoothed()
{
    Ads1115Module m = withRef(0x01);
    m.begin();
    seedRef(m); // 26000
    cycle(m, 0, 0);
    m.getReading();
    cycle(m, 3, 25200); // (26000*3 + 25200)/4 = 25800
    cycle(m, 0, 12900);
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(511, r.value);
}

void test_values_held_until_first_reference()
{
    Ads1115Module m = withRef(0x03, 3);
    m.begin();
    // Only the reference is converted until it has been read.
    cycle(m, 3, 3000); // bad first reading: still no reference
    TEST_ASSERT_FALSE(m.getReading().has_value);
    cycle(m, 3, 26000);
    TEST_ASSERT_FALSE(m.getReading().has_value);
    cycle(m, 0, 13000);
    TEST_ASSERT_EQUAL_INT16(511, m.getReading().value);
}

void test_reference_below_4000_stops_values_reports_code_3_once()
{
    Ads1115Module m = withRef(0x01);
    m.begin();
    seedRef(m);
    cycle(m, 0, 13000);
    TEST_ASSERT_TRUE(m.getReading().has_value);
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());

    cycle(m, 3, 3999);
    TEST_ASSERT_EQUAL_UINT8(3, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(3, m.activeError());
    TEST_ASSERT_FALSE(m.getReading().has_value);

    // Only the reference is converted, and nothing is repeated.
    cycle(m, 3, 100);
    cycle(m, 3, 0);
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
    TEST_ASSERT_FALSE(m.getReading().has_value);
    TEST_ASSERT_EQUAL_UINT8(3, m.activeError());
}

void test_reference_returning_reports_code_2()
{
    Ads1115Module m = withRef(0x01);
    m.begin();
    seedRef(m);
    cycle(m, 0, 13000);
    m.getReading();
    cycle(m, 3, 100);
    TEST_ASSERT_EQUAL_UINT8(3, m.takeRuntimeError());

    cycle(m, 3, 20000); // replaces the reference; not smoothed from the bad one
    TEST_ASSERT_EQUAL_UINT8(2, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.activeError());
    cycle(m, 0, 15000); // 15000 * 1023 / 20000
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(767, r.value);
}

void test_nack_produces_no_value()
{
    Ads1115Module m = plain(0x01);
    m.begin();
    m.scan(); // start
    g_now += 5;
    MockWire::scheduleNacks(ADDR, 1);
    m.scan(); // the config readback NACKs
    TEST_ASSERT_FALSE(m.getReading().has_value);
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(1, m.health().failures());
}

void test_short_read_produces_no_value()
{
    Ads1115Module m = plain(0x01);
    m.begin();
    m.scan();
    g_now += 5;
    const uint8_t cfg[2] = { 0xC1, 0xA3 };
    const uint8_t one[1] = { 0x12 };
    MockWire::queueRead(ADDR, cfg, 2);
    MockWire::queueRead(ADDR, one, 1);
    m.scan();
    TEST_ASSERT_FALSE(m.getReading().has_value);
    TEST_ASSERT_EQUAL_UINT8(1, m.health().failures());
}

void test_three_failures_report_code_1()
{
    Ads1115Module m = plain(0x01);
    m.begin();
    MockWire::scheduleNacks(ADDR, 100);
    for (int i = 0; i < 2; i++) {
        g_now += 5;
        m.scan();
        TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
    }
    g_now += 5;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(1, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(1, m.activeError());
    TEST_ASSERT_FALSE(m.getReading().has_value);

    // No polling while failed, retries once per second.
    uint8_t before = MockWire::transaction_count;
    g_now += 500;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(before, MockWire::transaction_count);
    g_now += 500;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(before + 1, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
}

void test_recovery_reports_code_2()
{
    Ads1115Module m = plain(0x01);
    m.begin();
    cycle(m, 0, 1600);
    TEST_ASSERT_EQUAL_INT16(50, m.getReading().value);
    MockWire::scheduleNacks(ADDR, 3);
    for (int i = 0; i < 3; i++) {
        g_now += 5;
        m.scan();
    }
    TEST_ASSERT_EQUAL_UINT8(1, m.takeRuntimeError());
    g_now += 1000;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(2, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.activeError());

    // Channel state survives: a still lever reports again once 100 ms passed.
    // (last sent 50, same value, last send long ago)
    cycle(m, 0, 1600);
    TEST_ASSERT_EQUAL_INT16(50, m.getReading().value);
}

void test_absent_at_boot_then_connected_recovers()
{
    MockWire::scheduleNacks(ADDR, 1);
    Ads1115Module m = plain(0x01);
    TEST_ASSERT_FALSE(m.begin());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError()); // reported as code 0 by the manager
    TEST_ASSERT_EQUAL_UINT8(1, m.activeError());
    uint8_t before = MockWire::transaction_count;
    g_now = 999;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(before, MockWire::transaction_count);
    g_now = 1000;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(2, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.activeError());
    cycle(m, 0, 1600);
    TEST_ASSERT_EQUAL_INT16(50, m.getReading().value);
}

void test_config_mismatch_discards_conversion()
{
    Ads1115Module m = plain(0x03);
    m.begin();
    m.scan(); // channel 0
    g_now += 5;
    MockWire::transaction_count = 0;
    const uint8_t reset_cfg[2] = { 0x85, 0x83 }; // power-on default
    const uint8_t res[2] = { 0x7F, 0xFF };
    MockWire::queueRead(ADDR, reset_cfg, 2);
    MockWire::queueRead(ADDR, res, 2);
    m.scan();
    TEST_ASSERT_FALSE(m.getReading().has_value);
    TEST_ASSERT_EQUAL_UINT8(1, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(2, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
    // The conversion register was not read; channel 0 was started again.
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::read_request_count);
    MockWire::read_queue_count = 0; // drop the unread result
    const MockWire::Transaction& t = MockWire::transactions[MockWire::transaction_count - 1];
    TEST_ASSERT_EQUAL_UINT8(3, t.length);
    TEST_ASSERT_EQUAL_HEX8(0xC1, t.data[1]);
    TEST_ASSERT_EQUAL_HEX8(0xA3, t.data[2]);
    // It continues normally on the same channel.
    g_now += 5;
    const uint8_t cfg[2] = { 0xC1, 0xA3 };
    const uint8_t ok[2] = { 0x06, 0x40 };
    MockWire::queueRead(ADDR, cfg, 2);
    MockWire::queueRead(ADDR, ok, 2);
    m.scan();
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_UINT8(BASE, r.pin);
    TEST_ASSERT_EQUAL_INT16(50, r.value);
}

void test_identity_accessors()
{
    Ads1115Module m = plain(0x01);
    TEST_ASSERT_EQUAL_UINT8(0, m.getPin());
    TEST_ASSERT_EQUAL_UINT8(ADDR, m.getI2CAddress());
    TEST_ASSERT_EQUAL(ModuleType::ADS1115, m.getType());
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_scaled_without_ref);
    RUN_TEST(test_scaled_with_ref);
    RUN_TEST(test_scaled_clamps);
    RUN_TEST(test_begin_probes_with_idle_config_and_no_conversion);
    RUN_TEST(test_config_word_channel_2_gain_1);
    RUN_TEST(test_no_read_before_5ms);
    RUN_TEST(test_value_is_signed_16_bit);
    RUN_TEST(test_rotation_skips_disabled_channels);
    RUN_TEST(test_sends_on_change_after_rate_limit);
    RUN_TEST(test_change_within_dead_zone_not_sent);
    RUN_TEST(test_still_channel_sends_every_100ms);
    RUN_TEST(test_reference_channel_never_reported);
    RUN_TEST(test_reference_smoothed);
    RUN_TEST(test_values_held_until_first_reference);
    RUN_TEST(test_reference_below_4000_stops_values_reports_code_3_once);
    RUN_TEST(test_reference_returning_reports_code_2);
    RUN_TEST(test_nack_produces_no_value);
    RUN_TEST(test_short_read_produces_no_value);
    RUN_TEST(test_three_failures_report_code_1);
    RUN_TEST(test_recovery_reports_code_2);
    RUN_TEST(test_absent_at_boot_then_connected_recovers);
    RUN_TEST(test_config_mismatch_discards_conversion);
    RUN_TEST(test_identity_accessors);
    return UNITY_END();
}
