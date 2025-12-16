// Mock Arduino environment for native testing
#include <stdint.h>
#include <string.h>

// Arduino pin definitions
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LOW 0
#define HIGH 1

// Mock matrix state: which buttons are pressed
// For a 3x4 matrix, buttons[row][col] = true means button pressed
static bool g_button_pressed[8][8];
static uint8_t g_pin_state[32]; // Pin states (supports pins 0-31)
static uint8_t g_row_pin_start = 2;
static uint8_t g_col_pin_start = 5;

void pinMode(uint8_t pin, uint8_t mode)
{
    (void)pin;
    (void)mode;
}

void digitalWrite(uint8_t pin, uint8_t val)
{
    if (pin < 32) {
        g_pin_state[pin] = val;
    }
}

int digitalRead(uint8_t pin)
{
    // Simulate matrix scanning
    // When a row pin is LOW and a button is pressed, the column reads LOW
    for (uint8_t row = 0; row < 8; row++) {
        uint8_t row_pin = g_row_pin_start + row;
        // Check if this row is active (LOW)
        if (row_pin < 32 && g_pin_state[row_pin] == LOW) {
            // Check all columns for this row
            for (uint8_t col = 0; col < 8; col++) {
                uint8_t col_pin = g_col_pin_start + col;
                // If reading this column pin and button is pressed
                if (pin == col_pin && g_button_pressed[row][col]) {
                    return LOW;
                }
            }
        }
    }
    return HIGH; // No button pressed (pullup)
}

void delayMicroseconds(unsigned int us)
{
    (void)us; // No-op in tests
}

// Now include the sensor code
#include "../../src/sensor.h"
#include "../../src/matrix_sensor.cpp"
#include <unity.h>

using namespace Sensor;

// Helper to reset mock state
void resetMockState()
{
    memset(g_button_pressed, false, sizeof(g_button_pressed));
    memset(g_pin_state, HIGH, sizeof(g_pin_state));
    g_row_pin_start = 2;
    g_col_pin_start = 5;
}

// Helper to configure mock pin mapping
void setMockPinMapping(uint8_t row_start, uint8_t col_start)
{
    g_row_pin_start = row_start;
    g_col_pin_start = col_start;
}

// Helper to press a button
void pressButton(uint8_t row, uint8_t col)
{
    g_button_pressed[row][col] = true;
}

// Helper to release a button
void releaseButton(uint8_t row, uint8_t col)
{
    g_button_pressed[row][col] = false;
}

// Test initialization
void test_matrix_sensor_init()
{
    uint8_t rows[] = {2, 3, 4};
    uint8_t cols[] = {5, 6, 7, 8};
    MatrixSensor sensor(3, 4, rows, cols);

    TEST_ASSERT_EQUAL(InputType::Matrix, sensor.getType());
    TEST_ASSERT_EQUAL(128, sensor.getPin()); // Virtual pin base
}

// Test no reading on first scan (no buttons pressed)
void test_matrix_sensor_no_reading_initially()
{
    uint8_t rows[] = {2, 3, 4};
    uint8_t cols[] = {5, 6, 7, 8};
    MatrixSensor sensor(3, 4, rows, cols);
    sensor.begin();

    sensor.scan();

    Reading r = sensor.getReading();
    TEST_ASSERT_FALSE(r.has_value);
}

// Test single button press detection with debounce
void test_matrix_sensor_single_button_press()
{
    uint8_t rows[] = {2, 3, 4};
    uint8_t cols[] = {5, 6, 7, 8};
    MatrixSensor sensor(3, 4, rows, cols);
    sensor.begin();

    // Press button at row 1, col 2
    pressButton(1, 2);

    // Scan 2 times (less than debounce threshold of 3)
    sensor.scan();
    sensor.scan();

    // Should NOT have reading yet
    Reading r1 = sensor.getReading();
    TEST_ASSERT_FALSE(r1.has_value);

    // Scan one more time (completes debounce)
    sensor.scan();

    // Should have reading now
    Reading r2 = sensor.getReading();
    TEST_ASSERT_TRUE(r2.has_value);
    TEST_ASSERT_EQUAL(1, r2.value); // 1 = pressed
    TEST_ASSERT_EQUAL(InputType::Matrix, r2.type);
    // Virtual pin = 128 + (row * num_cols + col) = 128 + (1 * 4 + 2) = 134
    TEST_ASSERT_EQUAL(134, r2.pin);
}

// Test virtual pin calculation
void test_matrix_sensor_virtual_pin_calculation()
{
    uint8_t rows[] = {2, 3, 4};
    uint8_t cols[] = {5, 6, 7, 8};
    MatrixSensor sensor(3, 4, rows, cols);
    sensor.begin();

    // Press button at row 0, col 0
    pressButton(0, 0);
    for (int i = 0; i < 3; i++) sensor.scan();
    Reading r1 = sensor.getReading();
    TEST_ASSERT_EQUAL(128, r1.pin); // 128 + (0*4 + 0)
    releaseButton(0, 0);
    for (int i = 0; i < 3; i++) sensor.scan();
    sensor.getReading(); // Consume release

    // Press button at row 2, col 3
    pressButton(2, 3);
    for (int i = 0; i < 3; i++) sensor.scan();
    Reading r2 = sensor.getReading();
    TEST_ASSERT_EQUAL(139, r2.pin); // 128 + (2*4 + 3) = 139
}

// Test button release detection
void test_matrix_sensor_button_release()
{
    uint8_t rows[] = {2, 3, 4};
    uint8_t cols[] = {5, 6, 7, 8};
    MatrixSensor sensor(3, 4, rows, cols);
    sensor.begin();

    // Press button
    pressButton(0, 0);
    for (int i = 0; i < 3; i++) sensor.scan();
    sensor.getReading(); // Consume press

    // Release button
    releaseButton(0, 0);
    for (int i = 0; i < 3; i++) sensor.scan();

    Reading r = sensor.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL(0, r.value); // 0 = released
    TEST_ASSERT_EQUAL(128, r.pin);
}

// Test multiple simultaneous button presses (NKRO)
void test_matrix_sensor_nkro()
{
    uint8_t rows[] = {2, 3, 4};
    uint8_t cols[] = {5, 6, 7, 8};
    MatrixSensor sensor(3, 4, rows, cols);
    sensor.begin();

    // Press two buttons simultaneously
    pressButton(0, 0);
    pressButton(1, 1);

    // Scan past debounce
    for (int i = 0; i < 3; i++) sensor.scan();

    // Should get two separate press events
    Reading r1 = sensor.getReading();
    TEST_ASSERT_TRUE(r1.has_value);
    TEST_ASSERT_EQUAL(1, r1.value);

    Reading r2 = sensor.getReading();
    TEST_ASSERT_TRUE(r2.has_value);
    TEST_ASSERT_EQUAL(1, r2.value);

    // Check we got both buttons (pins 128 and 133)
    TEST_ASSERT_TRUE((r1.pin == 128 && r2.pin == 133) || (r1.pin == 133 && r2.pin == 128));

    // No more readings
    Reading r3 = sensor.getReading();
    TEST_ASSERT_FALSE(r3.has_value);
}

// Test held button doesn't repeat
void test_matrix_sensor_no_repeat_while_held()
{
    uint8_t rows[] = {2, 3, 4};
    uint8_t cols[] = {5, 6, 7, 8};
    MatrixSensor sensor(3, 4, rows, cols);
    sensor.begin();

    // Press button
    pressButton(0, 0);
    for (int i = 0; i < 3; i++) sensor.scan();

    // Get press event
    Reading r1 = sensor.getReading();
    TEST_ASSERT_TRUE(r1.has_value);

    // Keep scanning with button held
    for (int i = 0; i < 20; i++) sensor.scan();

    // Should NOT have another reading
    Reading r2 = sensor.getReading();
    TEST_ASSERT_FALSE(r2.has_value);
}

// Test debounce filters glitches
void test_matrix_sensor_debounce_filters_glitches()
{
    uint8_t rows[] = {2, 3, 4};
    uint8_t cols[] = {5, 6, 7, 8};
    MatrixSensor sensor(3, 4, rows, cols);
    sensor.begin();

    // Glitchy button: pressed for 2 scans, then released
    pressButton(0, 0);
    sensor.scan();
    sensor.scan();

    releaseButton(0, 0);
    sensor.scan();

    // Should NOT have reading (glitch filtered)
    Reading r = sensor.getReading();
    TEST_ASSERT_FALSE(r.has_value);
}

// Test full press/release cycle
void test_matrix_sensor_full_cycle()
{
    uint8_t rows[] = {2, 3, 4};
    uint8_t cols[] = {5, 6, 7, 8};
    MatrixSensor sensor(3, 4, rows, cols);
    sensor.begin();

    // Press
    pressButton(1, 2);
    for (int i = 0; i < 3; i++) sensor.scan();

    Reading press = sensor.getReading();
    TEST_ASSERT_TRUE(press.has_value);
    TEST_ASSERT_EQUAL(1, press.value);

    // Release
    releaseButton(1, 2);
    for (int i = 0; i < 3; i++) sensor.scan();

    Reading release = sensor.getReading();
    TEST_ASSERT_TRUE(release.has_value);
    TEST_ASSERT_EQUAL(0, release.value);
}

// Test 2x2 matrix (small)
void test_matrix_sensor_2x2()
{
    uint8_t rows[] = {2, 3};
    uint8_t cols[] = {5, 6};
    MatrixSensor sensor(2, 2, rows, cols);
    sensor.begin();

    // Press all 4 buttons
    pressButton(0, 0);
    pressButton(0, 1);
    pressButton(1, 0);
    pressButton(1, 1);

    for (int i = 0; i < 3; i++) sensor.scan();

    // Should get 4 events
    for (int i = 0; i < 4; i++) {
        Reading r = sensor.getReading();
        TEST_ASSERT_TRUE(r.has_value);
        TEST_ASSERT_EQUAL(1, r.value);
    }

    // No more events
    Reading r5 = sensor.getReading();
    TEST_ASSERT_FALSE(r5.has_value);
}

// Test event queue overflow handling
void test_matrix_sensor_event_queue_overflow()
{
    // Use a 4x4 matrix and press more than EVENT_QUEUE_SIZE (8) buttons
    uint8_t rows[] = {2, 3, 4, 5};
    uint8_t cols[] = {6, 7, 8, 9};

    // Configure mock to match these pin mappings
    setMockPinMapping(2, 6);

    MatrixSensor sensor(4, 4, rows, cols);
    sensor.begin();

    // Press all 16 buttons at once (more than queue size of 8)
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            pressButton(r, c);
        }
    }

    for (int i = 0; i < 3; i++) sensor.scan();

    // Should get at least some events (queue drops oldest on overflow)
    int event_count = 0;
    while (true) {
        Reading r = sensor.getReading();
        if (!r.has_value) break;
        event_count++;
        if (event_count > 20) break; // Safety limit
    }

    // We pressed 16 buttons, but queue size is 8
    // Due to circular buffer with drop-oldest, we should get close to 8-9 events
    TEST_ASSERT_GREATER_OR_EQUAL(7, event_count);
}

void setUp(void) { resetMockState(); }
void tearDown(void) {}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_matrix_sensor_init);
    RUN_TEST(test_matrix_sensor_no_reading_initially);
    RUN_TEST(test_matrix_sensor_single_button_press);
    RUN_TEST(test_matrix_sensor_virtual_pin_calculation);
    RUN_TEST(test_matrix_sensor_button_release);
    RUN_TEST(test_matrix_sensor_nkro);
    RUN_TEST(test_matrix_sensor_no_repeat_while_held);
    RUN_TEST(test_matrix_sensor_debounce_filters_glitches);
    RUN_TEST(test_matrix_sensor_full_cycle);
    RUN_TEST(test_matrix_sensor_2x2);
    RUN_TEST(test_matrix_sensor_event_queue_overflow);

    return UNITY_END();
}
