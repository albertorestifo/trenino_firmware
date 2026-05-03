#pragma once

#include "module.h"
#include <Arduino.h>

namespace Modules {

// Analog sensor implementation
// Uses EMA (Exponential Moving Average) for smoothing
// Reports based on sensitivity, change threshold, and time-based forcing
class AnalogSensor : public IModule {
private:
    uint8_t pin; // Arduino pin number
    uint8_t sensitivity; // Sensitivity level (0-10, where 10 = most sensitive/sends most frequently)

    // State
    uint16_t current_value; // Current raw analog value (0-1023)
    uint16_t last_sent; // Last sent value
    uint16_t scans_since_send; // Number of scans since last send
    uint16_t min_send_interval; // Minimum scans between sends (computed from sensitivity)

    // Algorithm constants
    static constexpr uint16_t MAX_SEND_INTERVAL = 200; // Maximum 200 scans (~2s) - force send even if no change
    static constexpr uint16_t DEAD_ZONE = 2; // Ignore changes smaller than this (filters analog noise/jitter)

public:
    AnalogSensor(uint8_t pin_number, uint8_t sensitivity_level);

    // IModule interface implementation
    bool begin() override;
    void scan() override;
    Reading getReading() override;
    ModuleType getType() const override { return ModuleType::Analog; }
    uint8_t getPin() const override { return pin; }

private:
    // Check if we should send a value (simple rate limiting + periodic updates)
    bool shouldSend();

    // Compute minimum send interval from sensitivity
    uint16_t computeMinSendInterval() const;
};

} // namespace Modules
