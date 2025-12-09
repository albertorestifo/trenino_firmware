// Mock Arduino environment for native testing
#include <stdint.h>

// Arduino pin definitions
#define INPUT 0
#define OUTPUT 1
#define A0 14

// Mock Arduino functions
static uint16_t g_mock_analog_value = 512;

void pinMode(uint8_t pin, uint8_t mode)
{
    (void)pin;
    (void)mode;
}
int analogRead(uint8_t pin)
{
    (void)pin;
    return g_mock_analog_value;
}

// Now include the sensor code (include .cpp directly since we provide mocks above)
#include "../../src/sensor.h"
#include "../../src/analog_sensor.cpp"
#include <unity.h>

using namespace Sensor;

// Helper to set mock analog value
void setMockAnalogValue(uint16_t value)
{
    g_mock_analog_value = value;
}

// Test initialization
void test_analog_sensor_init()
{
    AnalogSensor sensor(A0, 5);

    TEST_ASSERT_EQUAL(InputType::Analog, sensor.getType());
    TEST_ASSERT_EQUAL(A0, sensor.getPin());
}

// Test that first scan initializes EMA
void test_analog_sensor_first_scan_initializes()
{
    AnalogSensor sensor(A0, 5);
    sensor.begin();

    setMockAnalogValue(512);
    sensor.scan();

    // First scan should not produce a reading (just initializes)
    Reading r = sensor.getReading();
    TEST_ASSERT_FALSE(r.has_value);
}

// Test that low sensitivity enforces longer minimum send interval
void test_analog_sensor_low_sensitivity_rate_limit()
{
    AnalogSensor sensor(A0, 0); // sensitivity 0 -> min_send_interval = 11 scans
    sensor.begin();

    // Initialize and consume first reading
    setMockAnalogValue(500);
    for (int i = 0; i < 11; i++) {
        sensor.scan();
    }
    sensor.getReading(); // Consume initial reading

    // Change value
    setMockAnalogValue(600);

    // Scan less than min_send_interval times
    for (int i = 0; i < 10; i++) {
        sensor.scan();
    }

    // Should NOT have a reading yet (rate limit not passed)
    Reading r = sensor.getReading();
    TEST_ASSERT_FALSE(r.has_value);
}

// Test that high sensitivity sends more frequently
void test_analog_sensor_high_sensitivity_rate_limit()
{
    AnalogSensor sensor(A0, 10); // sensitivity 10 -> min_send_interval = 1 scan
    sensor.begin();

    // Initialize with value 500
    setMockAnalogValue(500);
    sensor.scan();

    // Change to 600
    setMockAnalogValue(600);

    // Scan just 1 time (min_send_interval = 1)
    sensor.scan();

    // Should have a reading (rate limit passed and value changed)
    Reading r = sensor.getReading();
    TEST_ASSERT_TRUE(r.has_value);
}

// Test that value is sent when it changes significantly
void test_analog_sensor_send_on_significant_change()
{
    AnalogSensor sensor(A0, 5); // sensitivity 5 -> min_send_interval = 6 scans
    sensor.begin();

    // Initialize with value 500
    setMockAnalogValue(500);
    sensor.scan();

    // Change to a higher value (600)
    setMockAnalogValue(600);

    // Scan min_send_interval times (6 scans)
    for (int i = 0; i < 6; i++) {
        sensor.scan();
    }

    // Should have a reading now (rate limit passed && value changed)
    Reading r = sensor.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL(InputType::Analog, r.type);
}

// Test that value is NOT sent when no change
void test_analog_sensor_no_send_without_change_or_time()
{
    AnalogSensor sensor(A0, 5); // sensitivity 5 -> min_send_interval = 6 scans
    sensor.begin();

    // Initialize and consume first reading
    setMockAnalogValue(500);
    for (int i = 0; i < 6; i++) {
        sensor.scan();
    }
    sensor.getReading(); // Consume initial reading

    // Keep value at 500 (no change)
    // Scan many times (but less than MAX_SEND_INTERVAL)
    for (int i = 0; i < 20; i++) {
        sensor.scan();
    }

    // Should NOT have a reading (no change, even though rate limit passed)
    Reading r = sensor.getReading();
    TEST_ASSERT_FALSE(r.has_value);
}

// Test that small changes within dead zone are filtered (jitter suppression)
void test_analog_sensor_dead_zone_filtering()
{
    AnalogSensor sensor(A0, 5); // sensitivity 5 -> min_send_interval = 6 scans
    sensor.begin();

    // Initialize and consume first reading
    setMockAnalogValue(500);
    for (int i = 0; i < 6; i++) {
        sensor.scan();
    }
    sensor.getReading(); // Consume initial reading (last_sent = 500)

    // Small jitter: 501 (delta = 1, within DEAD_ZONE of 2)
    setMockAnalogValue(501);

    // Scan past rate limit
    for (int i = 0; i < 10; i++) {
        sensor.scan();
    }

    // Should NOT have a reading (change within dead zone)
    Reading r = sensor.getReading();
    TEST_ASSERT_FALSE(r.has_value);

    // Now change to 503 (delta from last_sent=500 is 3, beyond DEAD_ZONE)
    setMockAnalogValue(503);

    // Scan past rate limit again
    for (int i = 0; i < 6; i++) {
        sensor.scan();
    }

    // Should have a reading now (change beyond dead zone)
    Reading r2 = sensor.getReading();
    TEST_ASSERT_TRUE(r2.has_value);
}

// Test that value is sent after MAX_SEND_INTERVAL regardless of change (periodic heartbeat)
void test_analog_sensor_forced_send_after_min_gap()
{
    AnalogSensor sensor(A0, 5); // sensitivity 5 -> min_send_interval = 6 scans
    sensor.begin();

    // Initialize with value 500
    setMockAnalogValue(500);
    sensor.scan();

    // Keep value at 500 (no change)
    setMockAnalogValue(500);

    // Scan MAX_SEND_INTERVAL times (200 scans = ~2 seconds)
    for (int i = 0; i < 200; i++) {
        sensor.scan();
    }

    // Should have a reading now (forced periodic update even with no change)
    Reading r = sensor.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL(InputType::Analog, r.type);
}

// Test that reading resets scan counter
void test_analog_sensor_reading_resets_counter()
{
    AnalogSensor sensor(A0, 5); // sensitivity 5 -> k=6, send_every = 10
    sensor.begin();

    // Initialize with value 500
    setMockAnalogValue(500);
    sensor.scan();

    // Change to 600 (large change)
    setMockAnalogValue(600);

    // Scan enough times for EMA to change significantly
    for (int i = 0; i < 15; i++) {
        sensor.scan();
    }

    // Get first reading
    Reading r1 = sensor.getReading();
    TEST_ASSERT_TRUE(r1.has_value);

    // Immediately try to get another reading - should not have one
    Reading r2 = sensor.getReading();
    TEST_ASSERT_FALSE(r2.has_value);

    // Scan a few more times (less than send_every)
    for (int i = 0; i < 5; i++) {
        sensor.scan();
    }

    // Still should not have a reading (not enough scans and value hasn't changed much)
    Reading r3 = sensor.getReading();
    TEST_ASSERT_FALSE(r3.has_value);
}

// Test consecutive readings with changing values
void test_analog_sensor_consecutive_readings()
{
    AnalogSensor sensor(A0, 5);
    sensor.begin();

    setMockAnalogValue(500);
    sensor.scan();

    // First reading
    setMockAnalogValue(600);
    for (int i = 0; i < 6; i++) {
        sensor.scan();
    }
    TEST_ASSERT_TRUE(sensor.getReading().has_value);

    // Second reading
    setMockAnalogValue(700);
    for (int i = 0; i < 6; i++) {
        sensor.scan();
    }
    TEST_ASSERT_TRUE(sensor.getReading().has_value);
}

// Test edge cases: boundary values (0 and 1023)
void test_analog_sensor_boundary_values()
{
    AnalogSensor sensor(A0, 5);
    sensor.begin();

    // Test minimum value
    setMockAnalogValue(0);
    sensor.scan();
    setMockAnalogValue(100);
    for (int i = 0; i < 6; i++) {
        sensor.scan();
    }
    TEST_ASSERT_TRUE(sensor.getReading().has_value);

    // Test maximum value
    setMockAnalogValue(1023);
    for (int i = 0; i < 6; i++) {
        sensor.scan();
    }
    TEST_ASSERT_TRUE(sensor.getReading().has_value);
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_analog_sensor_init);
    RUN_TEST(test_analog_sensor_first_scan_initializes);
    RUN_TEST(test_analog_sensor_low_sensitivity_rate_limit);
    RUN_TEST(test_analog_sensor_high_sensitivity_rate_limit);
    RUN_TEST(test_analog_sensor_send_on_significant_change);
    RUN_TEST(test_analog_sensor_no_send_without_change_or_time);
    RUN_TEST(test_analog_sensor_dead_zone_filtering);
    RUN_TEST(test_analog_sensor_forced_send_after_min_gap);
    RUN_TEST(test_analog_sensor_reading_resets_counter);
    RUN_TEST(test_analog_sensor_consecutive_readings);
    RUN_TEST(test_analog_sensor_boundary_values);

    return UNITY_END();
}
