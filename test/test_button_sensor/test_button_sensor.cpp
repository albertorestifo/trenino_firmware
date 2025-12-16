// Mock Arduino environment for native testing
#include <stdint.h>

// Arduino pin definitions
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LOW 0
#define HIGH 1

// Mock Arduino functions
static int g_mock_digital_value = HIGH; // Default to HIGH (not pressed, due to INPUT_PULLUP)
static uint8_t g_last_pin_mode = 0;

void pinMode(uint8_t pin, uint8_t mode)
{
    (void)pin;
    g_last_pin_mode = mode;
}

int digitalRead(uint8_t pin)
{
    (void)pin;
    return g_mock_digital_value;
}

void digitalWrite(uint8_t pin, uint8_t val)
{
    (void)pin;
    (void)val;
}

// Now include the sensor code (include .cpp directly since we provide mocks above)
#include "../../src/sensor.h"
#include "../../src/button_sensor.cpp"
#include <unity.h>

using namespace Sensor;

// Helper to set mock digital value
void setMockDigitalValue(int value)
{
    g_mock_digital_value = value;
}

// Test initialization
void test_button_sensor_init()
{
    ButtonSensor sensor(7, 3);

    TEST_ASSERT_EQUAL(InputType::Button, sensor.getType());
    TEST_ASSERT_EQUAL(7, sensor.getPin());
}

// Test that begin() configures pin as INPUT_PULLUP
void test_button_sensor_begin_configures_pullup()
{
    ButtonSensor sensor(7, 3);
    sensor.begin();

    TEST_ASSERT_EQUAL(INPUT_PULLUP, g_last_pin_mode);
}

// Test that no reading on first scan (no state change yet)
void test_button_sensor_no_reading_initially()
{
    ButtonSensor sensor(7, 3);
    sensor.begin();

    setMockDigitalValue(HIGH); // Not pressed
    sensor.scan();

    Reading r = sensor.getReading();
    TEST_ASSERT_FALSE(r.has_value);
}

// Test press detection with debounce
void test_button_sensor_press_detection()
{
    ButtonSensor sensor(7, 3); // 3 scans debounce
    sensor.begin();

    setMockDigitalValue(HIGH); // Not pressed
    sensor.scan();
    sensor.getReading(); // Clear any pending

    // Press button (LOW due to INPUT_PULLUP)
    setMockDigitalValue(LOW);

    // Scan 2 times (less than debounce threshold of 3)
    sensor.scan();
    sensor.scan();

    // Should NOT have reading yet (debounce not complete)
    Reading r1 = sensor.getReading();
    TEST_ASSERT_FALSE(r1.has_value);

    // Scan one more time (completes debounce)
    sensor.scan();

    // Should have reading now (press event)
    Reading r2 = sensor.getReading();
    TEST_ASSERT_TRUE(r2.has_value);
    TEST_ASSERT_EQUAL(1, r2.value); // 1 = pressed
    TEST_ASSERT_EQUAL(InputType::Button, r2.type);
    TEST_ASSERT_EQUAL(7, r2.pin);
}

// Test release detection with debounce
void test_button_sensor_release_detection()
{
    ButtonSensor sensor(7, 3); // 3 scans debounce
    sensor.begin();

    // Start with button pressed
    setMockDigitalValue(LOW);
    for (int i = 0; i < 3; i++) {
        sensor.scan();
    }
    sensor.getReading(); // Consume press event

    // Release button
    setMockDigitalValue(HIGH);

    // Scan 2 times (less than debounce threshold)
    sensor.scan();
    sensor.scan();

    // Should NOT have reading yet
    Reading r1 = sensor.getReading();
    TEST_ASSERT_FALSE(r1.has_value);

    // Scan one more time (completes debounce)
    sensor.scan();

    // Should have reading now (release event)
    Reading r2 = sensor.getReading();
    TEST_ASSERT_TRUE(r2.has_value);
    TEST_ASSERT_EQUAL(0, r2.value); // 0 = released
}

// Test that holding button doesn't produce repeated events
void test_button_sensor_no_repeat_while_held()
{
    ButtonSensor sensor(7, 3);
    sensor.begin();

    // Press button
    setMockDigitalValue(LOW);
    for (int i = 0; i < 3; i++) {
        sensor.scan();
    }

    // Get press event
    Reading r1 = sensor.getReading();
    TEST_ASSERT_TRUE(r1.has_value);
    TEST_ASSERT_EQUAL(1, r1.value);

    // Keep button held and scan many more times
    for (int i = 0; i < 20; i++) {
        sensor.scan();
    }

    // Should NOT have another reading (button still held, no edge)
    Reading r2 = sensor.getReading();
    TEST_ASSERT_FALSE(r2.has_value);
}

// Test debounce filters glitches
void test_button_sensor_debounce_filters_glitches()
{
    ButtonSensor sensor(7, 3);
    sensor.begin();

    setMockDigitalValue(HIGH);
    sensor.scan();

    // Glitchy signal: LOW for 2 scans, then back to HIGH
    setMockDigitalValue(LOW);
    sensor.scan();
    sensor.scan();

    setMockDigitalValue(HIGH);
    sensor.scan();

    // Should NOT have reading (glitch was filtered)
    Reading r = sensor.getReading();
    TEST_ASSERT_FALSE(r.has_value);
}

// Test zero debounce (immediate response)
void test_button_sensor_zero_debounce()
{
    ButtonSensor sensor(7, 0); // 0 scans debounce = immediate
    sensor.begin();

    setMockDigitalValue(HIGH);
    sensor.scan();
    sensor.getReading(); // Clear initial

    // Press button - should register immediately
    setMockDigitalValue(LOW);
    sensor.scan();

    Reading r = sensor.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL(1, r.value);
}

// Test full press/release cycle
void test_button_sensor_full_cycle()
{
    ButtonSensor sensor(7, 3);
    sensor.begin();

    // Start unpressed
    setMockDigitalValue(HIGH);
    sensor.scan();

    // Press
    setMockDigitalValue(LOW);
    for (int i = 0; i < 3; i++) {
        sensor.scan();
    }

    Reading press = sensor.getReading();
    TEST_ASSERT_TRUE(press.has_value);
    TEST_ASSERT_EQUAL(1, press.value);

    // Release
    setMockDigitalValue(HIGH);
    for (int i = 0; i < 3; i++) {
        sensor.scan();
    }

    Reading release = sensor.getReading();
    TEST_ASSERT_TRUE(release.has_value);
    TEST_ASSERT_EQUAL(0, release.value);
}

// Test multiple press/release cycles
void test_button_sensor_multiple_cycles()
{
    ButtonSensor sensor(7, 3);
    sensor.begin();

    setMockDigitalValue(HIGH);
    sensor.scan();

    for (int cycle = 0; cycle < 3; cycle++) {
        // Press
        setMockDigitalValue(LOW);
        for (int i = 0; i < 3; i++) {
            sensor.scan();
        }
        Reading press = sensor.getReading();
        TEST_ASSERT_TRUE(press.has_value);
        TEST_ASSERT_EQUAL(1, press.value);

        // Release
        setMockDigitalValue(HIGH);
        for (int i = 0; i < 3; i++) {
            sensor.scan();
        }
        Reading release = sensor.getReading();
        TEST_ASSERT_TRUE(release.has_value);
        TEST_ASSERT_EQUAL(0, release.value);
    }
}

// Test reading clears pending event
void test_button_sensor_reading_clears_event()
{
    ButtonSensor sensor(7, 3);
    sensor.begin();

    setMockDigitalValue(HIGH);
    sensor.scan();

    // Press
    setMockDigitalValue(LOW);
    for (int i = 0; i < 3; i++) {
        sensor.scan();
    }

    // First reading should have value
    Reading r1 = sensor.getReading();
    TEST_ASSERT_TRUE(r1.has_value);

    // Second reading should be empty
    Reading r2 = sensor.getReading();
    TEST_ASSERT_FALSE(r2.has_value);
}

void setUp(void) { g_mock_digital_value = HIGH; }
void tearDown(void) {}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_button_sensor_init);
    RUN_TEST(test_button_sensor_begin_configures_pullup);
    RUN_TEST(test_button_sensor_no_reading_initially);
    RUN_TEST(test_button_sensor_press_detection);
    RUN_TEST(test_button_sensor_release_detection);
    RUN_TEST(test_button_sensor_no_repeat_while_held);
    RUN_TEST(test_button_sensor_debounce_filters_glitches);
    RUN_TEST(test_button_sensor_zero_debounce);
    RUN_TEST(test_button_sensor_full_cycle);
    RUN_TEST(test_button_sensor_multiple_cycles);
    RUN_TEST(test_button_sensor_reading_clears_event);

    return UNITY_END();
}
