#include "matrix_sensor.h"

namespace Modules {

MatrixSensor::MatrixSensor(uint8_t rows, uint8_t cols,
                           const uint8_t* row_pin_array, const uint8_t* col_pin_array)
    : num_rows(rows < MAX_ROWS ? rows : MAX_ROWS)
    , num_cols(cols < MAX_COLS ? cols : MAX_COLS)
    , core(num_rows, num_cols, DEFAULT_DEBOUNCE)
{
    // Copy pin arrays
    for (uint8_t i = 0; i < num_rows; i++) {
        row_pins[i] = row_pin_array[i];
    }
    for (uint8_t i = 0; i < num_cols; i++) {
        col_pins[i] = col_pin_array[i];
    }
}

bool MatrixSensor::begin()
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
    core.reset();
    return true;
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
            core.sample(row, col, raw_pressed);
        }

        // Deactivate row (drive HIGH)
        digitalWrite(row_pins[row], HIGH);
    }
}

Reading MatrixSensor::getReading()
{
    uint8_t button_index;
    bool pressed;
    if (!core.pop(button_index, pressed)) {
        return Reading(); // No events to report
    }

    // value = 1 for press, 0 for release
    return Reading(pressed ? 1 : 0, ModuleType::Matrix, virtualPin(button_index));
}

} // namespace Modules
