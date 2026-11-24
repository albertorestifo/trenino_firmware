#define ARDUINO_ARCH_AVR
#define UNIT_TEST
#include "../Wire.h"
#include "../../src/mt6701_module.cpp"
#include <unity.h>

using namespace Modules;

static unsigned long g_now = 0;
unsigned long millis() { return g_now; }

void setUp()
{
    g_now = 1000;
    MockWire::reset();
    mt6701_reset_wire_state();
}

void tearDown() {}

// Queue one angle read: the two register bytes for a 14-bit angle.
static void queueAngle(uint16_t angle)
{
    MockWire::queueRead((uint8_t)(angle >> 6));
    MockWire::queueRead((uint8_t)((angle & 0x3F) << 2));
}

static Reading scanUntilReading(MT6701Module& mod, uint16_t angle, uint8_t max_scans)
{
    for (uint8_t i = 0; i < max_scans; i++) {
        queueAngle(angle);
        mod.scan();
        Reading r = mod.getReading();
        if (r.has_value) return r;
    }
    return Reading();
}

void test_decode_angle()
{
    TEST_ASSERT_EQUAL_UINT16(0, MT6701Module::decodeAngle(0x00, 0x00));
    TEST_ASSERT_EQUAL_UINT16(16383, MT6701Module::decodeAngle(0xFF, 0xFC));
    // Low two bits of register 0x04 are not angle bits.
    TEST_ASSERT_EQUAL_UINT16(16383, MT6701Module::decodeAngle(0xFF, 0xFF));
    TEST_ASSERT_EQUAL_UINT16(8192, MT6701Module::decodeAngle(0x80, 0x00));
    TEST_ASSERT_EQUAL_UINT16(0x40 | 0x15, MT6701Module::decodeAngle(0x01, 0x15 << 2));
}

void test_begin_reads_angle_register_at_address()
{
    MT6701Module mod(0x46, 240, 5);
    queueAngle(1234);

    TEST_ASSERT_TRUE(mod.begin());
    TEST_ASSERT_TRUE(MockWire::wireBeginCalled());
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(0x46, MockWire::transactions[0].address);
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transactions[0].length);
    TEST_ASSERT_EQUAL_UINT8(0x03, MockWire::transactions[0].data[0]);
    TEST_ASSERT_EQUAL_UINT16(1234, mod.getCurrentValue());
    TEST_ASSERT_EQUAL(ModuleType::MT6701, mod.getType());
    TEST_ASSERT_EQUAL_UINT8(240, mod.getPin());
    TEST_ASSERT_EQUAL_UINT8(0x46, mod.getI2CAddress());
}

void test_begin_fails_when_sensor_nacks()
{
    MT6701Module mod(0x06, 240, 5);
    MockWire::scheduleNacks(0x06, 1);

    TEST_ASSERT_FALSE(mod.begin());
    TEST_ASSERT_FALSE(mod.hasValue());
}

void test_begin_fails_on_short_read()
{
    MT6701Module mod(0x06, 240, 5);
    MockWire::queueRead(0x12); // only one byte

    TEST_ASSERT_FALSE(mod.begin());
}

void test_reports_value_under_report_pin()
{
    MT6701Module mod(0x06, 241, 10);
    queueAngle(5000);
    mod.begin();

    Reading r = scanUntilReading(mod, 5000, 3);
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(5000, r.value);
    TEST_ASSERT_EQUAL_UINT8(241, r.pin);
    TEST_ASSERT_EQUAL(ModuleType::MT6701, r.type);
}

void test_rate_limit_follows_sensitivity()
{
    MT6701Module mod(0x06, 240, 5); // 6 scans minimum
    queueAngle(100);
    mod.begin();
    TEST_ASSERT_TRUE(scanUntilReading(mod, 100, 10).has_value);

    for (uint8_t i = 0; i < 5; i++) {
        queueAngle(9000);
        mod.scan();
        TEST_ASSERT_FALSE(mod.getReading().has_value);
    }
    queueAngle(9000);
    mod.scan();
    Reading r = mod.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(9000, r.value);
}

void test_dead_zone_and_forced_send()
{
    MT6701Module mod(0x06, 240, 10);
    queueAngle(100);
    mod.begin();
    TEST_ASSERT_TRUE(scanUntilReading(mod, 100, 3).has_value);

    // +3 counts is inside the dead zone: nothing until the keep-alive is due.
    for (uint16_t i = 0; i < 20; i++) {
        g_now += 50;
        queueAngle(103);
        mod.scan();
        TEST_ASSERT_FALSE(mod.getReading().has_value);
    }
    g_now += MT6701Module::KEEPALIVE_MS;
    queueAngle(103);
    mod.scan();
    TEST_ASSERT_TRUE(mod.getReading().has_value);

    queueAngle(107);
    mod.scan();
    TEST_ASSERT_TRUE(mod.getReading().has_value);
}

void test_wrap_is_a_small_change()
{
    MT6701Module mod(0x06, 240, 10);
    queueAngle(16382);
    mod.begin();
    TEST_ASSERT_TRUE(scanUntilReading(mod, 16382, 3).has_value);

    // 16382 -> 1 is 3 counts across the wrap: inside the dead zone.
    queueAngle(1);
    mod.scan();
    TEST_ASSERT_FALSE(mod.getReading().has_value);

    queueAngle(5);
    mod.scan();
    Reading r = mod.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(5, r.value);
}

void test_failed_read_keeps_last_value()
{
    MT6701Module mod(0x06, 240, 10);
    queueAngle(2000);
    mod.begin();
    TEST_ASSERT_TRUE(scanUntilReading(mod, 2000, 3).has_value);

    MockWire::scheduleNacks(0x06, 1);
    mod.scan();
    TEST_ASSERT_EQUAL_UINT16(2000, mod.getCurrentValue());
    TEST_ASSERT_FALSE(mod.getReading().has_value);
}

void test_recovers_after_failed_begin()
{
    MT6701Module mod(0x06, 240, 10);
    MockWire::scheduleNacks(0x06, 1);
    TEST_ASSERT_FALSE(mod.begin());
    TEST_ASSERT_TRUE(mod.isLost());

    // While lost, the bus is only tried every RETRY_MS, however fast the loop runs.
    uint8_t requests_before = MockWire::request_count;
    for (uint8_t i = 0; i < 200; i++) {
        mod.scan();
        TEST_ASSERT_FALSE(mod.getReading().has_value);
    }
    TEST_ASSERT_EQUAL_UINT8(requests_before, MockWire::request_count);

    g_now += MT6701Module::RETRY_MS;
    queueAngle(777);
    mod.scan();
    TEST_ASSERT_FALSE(mod.isLost());
    Reading r = mod.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(777, r.value);
}

void test_lost_sensor_goes_quiet()
{
    MT6701Module mod(0x06, 240, 10);
    queueAngle(2000);
    mod.begin();
    TEST_ASSERT_TRUE(scanUntilReading(mod, 2000, 3).has_value);

    // Two failures are a glitch: still not lost.
    MockWire::scheduleNacks(0x06, MT6701Module::LOST_AFTER - 1);
    for (uint8_t i = 0; i < MT6701Module::LOST_AFTER - 1; i++) mod.scan();
    TEST_ASSERT_FALSE(mod.isLost());

    MockWire::reset();
    MockWire::scheduleNacks(0x06, 1);
    mod.scan();
    TEST_ASSERT_TRUE(mod.isLost());

    // No keep-alive with the stale value, however long it stays lost.
    for (uint16_t i = 0; i < 20; i++) {
        g_now += 250;
        MockWire::scheduleNacks(0x06, 1);
        mod.scan();
        TEST_ASSERT_FALSE(mod.getReading().has_value);
        MockWire::reset();
    }
}

void test_lost_sensor_retries_slowly_and_resends_at_once()
{
    MT6701Module mod(0x06, 240, 0); // slowest rate: 11 scans between sends
    queueAngle(3000);
    mod.begin();
    TEST_ASSERT_TRUE(scanUntilReading(mod, 3000, 20).has_value);

    MockWire::scheduleNacks(0x06, MT6701Module::LOST_AFTER);
    for (uint8_t i = 0; i < MT6701Module::LOST_AFTER; i++) mod.scan();
    TEST_ASSERT_TRUE(mod.isLost());

    uint8_t requests = MockWire::request_count;
    for (uint8_t i = 0; i < 100; i++) mod.scan();
    TEST_ASSERT_EQUAL_UINT8(requests, MockWire::request_count);

    g_now += MT6701Module::RETRY_MS;
    queueAngle(3001); // inside the dead zone, but it is sent: the host must see it is back
    mod.scan();
    Reading r = mod.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(3001, r.value);
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_decode_angle);
    RUN_TEST(test_begin_reads_angle_register_at_address);
    RUN_TEST(test_begin_fails_when_sensor_nacks);
    RUN_TEST(test_begin_fails_on_short_read);
    RUN_TEST(test_reports_value_under_report_pin);
    RUN_TEST(test_rate_limit_follows_sensitivity);
    RUN_TEST(test_dead_zone_and_forced_send);
    RUN_TEST(test_wrap_is_a_small_change);
    RUN_TEST(test_failed_read_keeps_last_value);
    RUN_TEST(test_recovers_after_failed_begin);
    RUN_TEST(test_lost_sensor_goes_quiet);
    RUN_TEST(test_lost_sensor_retries_slowly_and_resends_at_once);
    return UNITY_END();
}
