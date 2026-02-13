#pragma once

#include <stdint.h>

namespace BLDC {

// Configuration for a single detent
struct DetentConfig {
    uint8_t position_percent;      // 0-100% of calibrated range
    uint8_t engagement_strength;   // 0-255: torque to click in
    uint8_t hold_strength;         // 0-255: torque to stay in
    uint8_t exit_strength;         // 0-255: torque to click out
    uint8_t spring_back_target;    // Detent index to return to, or 255 = none

    DetentConfig()
        : position_percent(0)
        , engagement_strength(0)
        , hold_strength(0)
        , exit_strength(0)
        , spring_back_target(255)
    {
    }
};

// Configuration for a linear range between detents
struct LinearRangeConfig {
    uint8_t start_detent_index;
    uint8_t end_detent_index;
    uint8_t damping_strength;      // 0-255: resistance while moving

    LinearRangeConfig()
        : start_detent_index(0)
        , end_detent_index(0)
        , damping_strength(0)
    {
    }
};

// Calibration error codes
enum CalibrationError : uint8_t {
    TIMEOUT = 0,
    RANGE_TOO_SMALL = 1,
    ENCODER_ERROR = 2
};

// Calibration constants
constexpr uint16_t MIN_ENCODER_RANGE = 1000;  // Minimum encoder ticks between endstops
constexpr uint32_t CALIBRATION_TIMEOUT_MS = 30000;  // 30 second timeout
constexpr uint32_t CALIBRATION_STALL_TIMEOUT_MS = 5000;  // 5 second stall timeout

// Motor control constants
constexpr float CALIBRATION_SPEED = 0.1f;  // 10% speed during calibration
constexpr float CALIBRATION_TORQUE = 0.5f;  // Low torque for calibration

} // namespace BLDC
