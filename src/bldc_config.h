#pragma once

#include <stdint.h>

namespace BLDC {

// Configuration for a single detent
struct DetentConfig {
    uint8_t position_percent;      // 0-100% of calibrated range
    uint8_t detent_strength;       // 0-255: overall detent torque strength

    DetentConfig()
        : position_percent(0)
        , detent_strength(0)
    {
    }
};

// Profile-level haptic parameters
struct ProfileConfig {
    uint8_t snap_point;            // 50-150: percentage of detent width where snap occurs
    uint8_t endstop_strength;      // 0-255: virtual endstop torque

    ProfileConfig()
        : snap_point(70)
        , endstop_strength(200)
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

// PD controller constants
constexpr float P_SCALE_FACTOR = 4.0f;

constexpr float D_LOWER_FACTOR = 0.08f;
constexpr float D_UPPER_FACTOR = 0.02f;
constexpr float D_WIDTH_LOWER_DEG = 3.0f;
constexpr float D_WIDTH_UPPER_DEG = 8.0f;

constexpr float DEAD_ZONE_FRACTION = 0.2f;
constexpr float DEAD_ZONE_MAX_DEG = 1.0f;

constexpr float VELOCITY_LPF_ALPHA = 0.1f;
constexpr float MAX_SAFE_VELOCITY = 60.0f;

constexpr float IDLE_VELOCITY_EWMA_ALPHA = 0.001f;
constexpr float IDLE_VELOCITY_THRESHOLD = 0.05f;
constexpr uint32_t IDLE_CORRECTION_DELAY_MS = 500;
constexpr float IDLE_CORRECTION_MAX_DEG = 5.0f;
constexpr float IDLE_CORRECTION_RATE_ALPHA = 0.0005f;

constexpr float DAMPING_SCALE = 1.0f;

constexpr float LEVER_ARC_DEGREES = 90.0f;

} // namespace BLDC
