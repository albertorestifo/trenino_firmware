#pragma once

#include "module.h"
#include <Arduino.h>

namespace Modules {

// Button sensor implementation
// Uses counter-based debouncing and reports edge events (press/release)
class ButtonSensor : public IModule {
private:
    uint8_t pin;               // Arduino pin number
    uint8_t debounce_threshold; // Number of scans for debounce

    // State
    bool current_state;        // Current debounced state (true = pressed)
    bool last_reported;        // Last reported state
    bool raw_state;            // Raw reading from pin
    uint8_t debounce_count;    // Counter for debounce
    bool has_pending_event;    // True if there's an event to report

public:
    ButtonSensor(uint8_t pin_number, uint8_t debounce_scans);

    // IModule interface implementation
    void begin() override;
    void scan() override;
    Reading getReading() override;
    ModuleType getType() const override { return ModuleType::Button; }
    uint8_t getPin() const override { return pin; }
};

} // namespace Modules
