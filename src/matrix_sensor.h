#pragma once

#include "matrix_core.h"
#include "module.h"
#include <Arduino.h>

namespace Modules {

// Matrix sensor implementation
// Uses row/column scanning with per-button debouncing
// Reports edge events for each button with virtual pin scheme
class MatrixSensor : public IModule {
public:
    // Maximum matrix size (to avoid dynamic allocation)
    static constexpr uint8_t MAX_ROWS = MatrixCore::MAX_ROWS;
    static constexpr uint8_t MAX_COLS = MatrixCore::MAX_COLS;
    static constexpr uint8_t MAX_BUTTONS = MAX_ROWS * MAX_COLS;

    // Virtual pin base (matrix buttons use pins 128+)
    static constexpr uint8_t VIRTUAL_PIN_BASE = 128;

    // Default debounce threshold
    static constexpr uint8_t DEFAULT_DEBOUNCE = 3;

    // Event queue size for NKRO
    static constexpr uint8_t EVENT_QUEUE_SIZE = MatrixCore::EVENT_QUEUE_SIZE;

private:
    uint8_t num_rows;
    uint8_t num_cols;
    uint8_t row_pins[MAX_ROWS];
    uint8_t col_pins[MAX_COLS];

    // Debounce and event queue
    MatrixCore core;

public:
    MatrixSensor(uint8_t rows, uint8_t cols,
                 const uint8_t* row_pin_array, const uint8_t* col_pin_array);

    // IModule interface implementation
    bool begin() override;
    void scan() override;
    Reading getReading() override;
    ModuleType getType() const override { return ModuleType::Matrix; }
    uint8_t getPin() const override { return VIRTUAL_PIN_BASE; } // Base pin identifier

private:
    // Get virtual pin for a button
    uint8_t virtualPin(uint8_t button_index) const { return VIRTUAL_PIN_BASE + button_index; }
};

} // namespace Modules
