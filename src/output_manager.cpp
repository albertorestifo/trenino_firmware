#include "output_manager.h"
#include <Arduino.h>

namespace OutputManager {

// Bitmask tracking which pins have been configured as OUTPUT (pins 0-31)
static uint32_t g_output_pins = 0;
bool isOutputPin(uint8_t pin) { return pin < 32 && (g_output_pins & (1UL << pin)); }

void init()
{
    g_output_pins = 0;
}

void setOutput(uint8_t pin, uint8_t value)
{
    // Configure as output if not already (only track pins 0-31)
    if (pin < 32 && !(g_output_pins & (1UL << pin))) {
        pinMode(pin, OUTPUT);
        g_output_pins |= (1UL << pin);
    }

    digitalWrite(pin, value ? HIGH : LOW);
}

} // namespace OutputManager
