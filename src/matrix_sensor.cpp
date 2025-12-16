#include "matrix_sensor.h"
#include <string.h>

namespace Sensor {

MatrixSensor::MatrixSensor(uint8_t rows, uint8_t cols,
                           const uint8_t* row_pin_array, const uint8_t* col_pin_array)
    : num_rows(rows < MAX_ROWS ? rows : MAX_ROWS)
    , num_cols(cols < MAX_COLS ? cols : MAX_COLS)
    , queue_head(0)
    , queue_tail(0)
    , debounce_threshold(DEFAULT_DEBOUNCE)
{
    // Copy pin arrays
    for (uint8_t i = 0; i < num_rows; i++) {
        row_pins[i] = row_pin_array[i];
    }
    for (uint8_t i = 0; i < num_cols; i++) {
        col_pins[i] = col_pin_array[i];
    }

    // Initialize state arrays
    memset(current_state, 0, sizeof(current_state));
    memset(last_reported, 0, sizeof(last_reported));
    memset(debounce_count, 0, sizeof(debounce_count));
}

void MatrixSensor::begin()
{
    // Configure row pins as outputs (active LOW when scanning)
    for (uint8_t r = 0; r < num_rows; r++) {
        pinMode(row_pins[r], OUTPUT);
        digitalWrite(row_pins[r], HIGH); // Inactive state
    }

    // Configure column pins as inputs with pullup
    for (uint8_t c = 0; c < num_cols; c++) {
        pinMode(col_pins[c], INPUT_PULLUP);
    }

    // Reset state
    memset(current_state, 0, sizeof(current_state));
    memset(last_reported, 0, sizeof(last_reported));
    memset(debounce_count, 0, sizeof(debounce_count));
    queue_head = 0;
    queue_tail = 0;
}

void MatrixSensor::scan()
{
    // Scan each row
    for (uint8_t row = 0; row < num_rows; row++) {
        // Activate current row (drive LOW)
        digitalWrite(row_pins[row], LOW);

        // Small delay for signal to settle
        delayMicroseconds(10);

        // Read all columns
        for (uint8_t col = 0; col < num_cols; col++) {
            // Button is pressed if column reads LOW (pulled down by row)
            bool raw_pressed = (digitalRead(col_pins[col]) == LOW);
            scanButton(row, col, raw_pressed);
        }

        // Deactivate row (drive HIGH)
        digitalWrite(row_pins[row], HIGH);
    }
}

void MatrixSensor::scanButton(uint8_t row, uint8_t col, bool raw_pressed)
{
    uint8_t idx = buttonIndex(row, col);

    // Counter-based debounce algorithm
    if (raw_pressed == current_state[idx]) {
        // Reading matches current state, reset counter
        debounce_count[idx] = 0;
    } else {
        // Reading differs from current state
        debounce_count[idx]++;

        if (debounce_count[idx] >= debounce_threshold) {
            // Stable new state detected
            current_state[idx] = raw_pressed;
            debounce_count[idx] = 0;

            // Check if this is a new edge event
            if (current_state[idx] != last_reported[idx]) {
                enqueueEvent(idx, current_state[idx]);
            }
        }
    }
}

void MatrixSensor::enqueueEvent(uint8_t button_index, bool pressed)
{
    // Calculate next tail position
    uint8_t next_tail = (queue_tail + 1) % EVENT_QUEUE_SIZE;

    // Check if queue is full
    if (next_tail == queue_head) {
        // Queue full, drop oldest event
        queue_head = (queue_head + 1) % EVENT_QUEUE_SIZE;
    }

    // Add event to queue
    event_queue[queue_tail].button_index = button_index;
    event_queue[queue_tail].pressed = pressed;
    queue_tail = next_tail;
}

Reading MatrixSensor::getReading()
{
    if (isQueueEmpty()) {
        return Reading(); // No events to report
    }

    // Get event from queue head
    PendingEvent& event = event_queue[queue_head];
    queue_head = (queue_head + 1) % EVENT_QUEUE_SIZE;

    // Update last reported state
    last_reported[event.button_index] = event.pressed;

    // Create reading with virtual pin
    // value = 1 for press, 0 for release
    int16_t value = event.pressed ? 1 : 0;
    uint8_t pin = virtualPin(event.button_index);

    return Reading(value, InputType::Matrix, pin);
}

} // namespace Sensor
