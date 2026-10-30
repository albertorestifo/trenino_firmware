#pragma once

#include <stdint.h>

namespace OutputManager {

// Initialize output manager (resets all pin tracking)
void init();

// Set an output pin to a value
// Automatically configures pin as OUTPUT on first use
// pin: Pin number (0-31 tracked, higher pins still work but not tracked)
// value: 0 = LOW, non-zero = HIGH
void setOutput(uint8_t pin, uint8_t value);
bool isOutputPin(uint8_t pin);

} // namespace OutputManager
