#pragma once

#include "sensor.h"
#include <Arduino.h>

namespace Sensor {

// Matrix sensor implementation
// Uses row/column scanning with per-button debouncing
// Reports edge events for each button with virtual pin scheme
class MatrixSensor : public ISensor {
public:
    // Maximum matrix size (to avoid dynamic allocation)
    static constexpr uint8_t MAX_ROWS = 8;
    static constexpr uint8_t MAX_COLS = 8;
    static constexpr uint8_t MAX_BUTTONS = MAX_ROWS * MAX_COLS;

    // Virtual pin base (matrix buttons use pins 128+)
    static constexpr uint8_t VIRTUAL_PIN_BASE = 128;

    // Default debounce threshold
    static constexpr uint8_t DEFAULT_DEBOUNCE = 3;

    // Event queue size for NKRO
    static constexpr uint8_t EVENT_QUEUE_SIZE = 8;

private:
    uint8_t num_rows;
    uint8_t num_cols;
    uint8_t row_pins[MAX_ROWS];
    uint8_t col_pins[MAX_COLS];

    // Per-button state
    bool current_state[MAX_BUTTONS];    // Debounced state (true = pressed)
    bool last_reported[MAX_BUTTONS];    // Last reported state
    uint8_t debounce_count[MAX_BUTTONS]; // Debounce counter per button

    // Event queue for NKRO support
    struct PendingEvent {
        uint8_t button_index;
        bool pressed;
    };
    PendingEvent event_queue[EVENT_QUEUE_SIZE];
    uint8_t queue_head;
    uint8_t queue_tail;

    // Debounce threshold
    uint8_t debounce_threshold;

public:
    MatrixSensor(uint8_t rows, uint8_t cols,
                 const uint8_t* row_pin_array, const uint8_t* col_pin_array);

    // ISensor interface implementation
    void begin() override;
    void scan() override;
    Reading getReading() override;
    InputType getType() const override { return InputType::Matrix; }
    uint8_t getPin() const override { return VIRTUAL_PIN_BASE; } // Base pin identifier

private:
    // Scan a single button and handle debouncing
    void scanButton(uint8_t row, uint8_t col, bool raw_pressed);

    // Add event to queue
    void enqueueEvent(uint8_t button_index, bool pressed);

    // Check if queue is empty
    bool isQueueEmpty() const { return queue_head == queue_tail; }

    // Get button index from row/col
    uint8_t buttonIndex(uint8_t row, uint8_t col) const { return row * num_cols + col; }

    // Get virtual pin for a button
    uint8_t virtualPin(uint8_t button_index) const { return VIRTUAL_PIN_BASE + button_index; }
};

} // namespace Sensor
