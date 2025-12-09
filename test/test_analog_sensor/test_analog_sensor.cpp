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

// Now include the sensor code
#include "../../src/analog_sensor.h"
#include "../../src/sensor.h"
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

// Test that sensitivity affects send interval
void test_analog_sensor_ema_smoothing_low_sensitivity()
{
    AnalogSensor sensor(A0, 0); // sensitivity 0 -> min_send_interval = 11 scans
    sensor.begin();

    // Initialize with value 500
    setMockAnalogValue(500);
    sensor.scan();

    // Change to 600
    setMockAnalogValue(600);

    // Scan less than min_send_interval times (10 scans, less than 11)
    for (int i = 0; i < 10; i++) {
        sensor.scan();
    }

    // Should NOT have a reading yet (rate limit not passed)
    Reading r = sensor.getReading();
    TEST_ASSERT_FALSE(r.has_value);
}

// Test that high sensitivity sends more frequently
void test_analog_sensor_ema_smoothing_high_sensitivity()
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

// Test that sensors can be created with different sensitivities
void test_analog_sensor_send_every_calculation()
{
    // Sensitivity affects EMA smoothing (k = 1 + sensitivity)
    // but no longer affects send timing
    AnalogSensor sensor0(A0, 0);
    AnalogSensor sensor5(A0, 5);
    AnalogSensor sensor10(A0, 10);

    // Verify the sensors were created successfully
    TEST_ASSERT_EQUAL(InputType::Analog, sensor0.getType());
    TEST_ASSERT_EQUAL(InputType::Analog, sensor5.getType());
    TEST_ASSERT_EQUAL(InputType::Analog, sensor10.getType());
}

// Test that value is sent when it changes (after rate limit period)
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

// Test that changes beyond dead zone are detected
void test_analog_sensor_send_on_time_gate_with_change()
{
    AnalogSensor sensor(A0, 5); // sensitivity 5 -> min_send_interval = 6 scans
    sensor.begin();

    // Initialize with value 500
    setMockAnalogValue(500);
    sensor.scan();

    // Change to 503 (3 units, beyond DEAD_ZONE of 2)
    setMockAnalogValue(503);

    // Scan min_send_interval times (6 scans)
    for (int i = 0; i < 6; i++) {
        sensor.scan();
    }

    // Should have a reading now (rate limit passed && value changed beyond dead zone)
    Reading r = sensor.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL(InputType::Analog, r.type);
}

// Test that value is NOT sent when no change
void test_analog_sensor_no_send_without_change_or_time()
{
    AnalogSensor sensor(A0, 5); // sensitivity 5 -> min_send_interval = 6 scans
    sensor.begin();

    // Initialize with value 500
    setMockAnalogValue(500);
    sensor.scan();

    // Keep value at 500 (no change)
    setMockAnalogValue(500);

    // Scan many times (even past rate limit)
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

    // Initialize with value 500
    setMockAnalogValue(500);
    sensor.scan();

    // Small jitter: 501 (delta = 1, within DEAD_ZONE of 2)
    setMockAnalogValue(501);

    // Scan past rate limit
    for (int i = 0; i < 10; i++) {
        sensor.scan();
    }

    // Should NOT have a reading (change within dead zone)
    Reading r = sensor.getReading();
    TEST_ASSERT_FALSE(r.has_value);

    // Now change to 503 (delta = 3, beyond DEAD_ZONE)
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

// Test multiple readings over time
void test_analog_sensor_multiple_readings()
{
    AnalogSensor sensor(A0, 5); // sensitivity 5 -> min_send_interval = 6 scans
    sensor.begin();

    // Initialize with value 500
    setMockAnalogValue(500);
    sensor.scan();

    // First change to 600
    setMockAnalogValue(600);
    for (int i = 0; i < 6; i++) {
        sensor.scan();
    }
    Reading r1 = sensor.getReading();
    TEST_ASSERT_TRUE(r1.has_value);

    // Second change to 700
    setMockAnalogValue(700);
    for (int i = 0; i < 6; i++) {
        sensor.scan();
    }
    Reading r2 = sensor.getReading();
    TEST_ASSERT_TRUE(r2.has_value);

    // Third change to 800
    setMockAnalogValue(800);
    for (int i = 0; i < 6; i++) {
        sensor.scan();
    }
    Reading r3 = sensor.getReading();
    TEST_ASSERT_TRUE(r3.has_value);
}

// Test edge case: value at boundary (0)
void test_analog_sensor_boundary_zero()
{
    AnalogSensor sensor(A0, 5); // sensitivity 5 -> min_send_interval = 6 scans
    sensor.begin();

    setMockAnalogValue(0);
    sensor.scan();

    // Change to 100
    setMockAnalogValue(100);
    for (int i = 0; i < 6; i++) {
        sensor.scan();
    }

    Reading r = sensor.getReading();
    TEST_ASSERT_TRUE(r.has_value);
}

// Test edge case: value at boundary (1023)
void test_analog_sensor_boundary_max()
{
    AnalogSensor sensor(A0, 5); // sensitivity 5 -> min_send_interval = 6 scans
    sensor.begin();

    setMockAnalogValue(1023);
    sensor.scan();

    // Change to 1013
    setMockAnalogValue(1013);
    for (int i = 0; i < 6; i++) {
        sensor.scan();
    }

    Reading r = sensor.getReading();
    TEST_ASSERT_TRUE(r.has_value);
}

// Main test runner
int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_analog_sensor_init);
    RUN_TEST(test_analog_sensor_first_scan_initializes);
    RUN_TEST(test_analog_sensor_ema_smoothing_low_sensitivity);
    RUN_TEST(test_analog_sensor_ema_smoothing_high_sensitivity);
    RUN_TEST(test_analog_sensor_send_every_calculation);
    RUN_TEST(test_analog_sensor_send_on_significant_change);
    RUN_TEST(test_analog_sensor_send_on_time_gate_with_change);
    RUN_TEST(test_analog_sensor_no_send_without_change_or_time);
    RUN_TEST(test_analog_sensor_dead_zone_filtering);
    RUN_TEST(test_analog_sensor_forced_send_after_min_gap);
    RUN_TEST(test_analog_sensor_reading_resets_counter);
    RUN_TEST(test_analog_sensor_multiple_readings);
    RUN_TEST(test_analog_sensor_boundary_zero);
    RUN_TEST(test_analog_sensor_boundary_max);

    return UNITY_END();
}
