#pragma once

#include "sensor.h"
#include "bldc_config.h"
#include <stdint.h>

// Forward declarations for SimpleFOC (avoid including heavy headers in tests)
class BLDCMotor;
class MagneticSensorSPI;

namespace Sensor {

class BLDCLever : public ISensor {
public:
    // Constructor
    BLDCLever(uint8_t motor_pin_a, uint8_t motor_pin_b, uint8_t motor_pin_c,
              uint8_t motor_enable_a, uint8_t motor_enable_b,
              uint8_t encoder_cs, uint8_t pole_pairs,
              uint8_t voltage, uint8_t current_limit, uint8_t encoder_bits);

    // Destructor
    ~BLDCLever() override;

    // ISensor interface
    void begin() override;
    void scan() override;
    Reading getReading() override;
    InputType getType() const override { return InputType::BLDCLever; }
    uint8_t getPin() const override { return pin_; }

    // BLDC-specific methods

    // Run calibration to find endstops (blocking, ~10-30 seconds)
    // Returns true on success, false on failure
    bool runCalibration();

    // Load a detent profile (replaces any existing profile)
    // Returns true on success, false if validation fails
    bool loadProfile(
        const BLDC::DetentConfig* detents,
        uint8_t num_detents,
        const BLDC::LinearRangeConfig* ranges,
        uint8_t num_ranges
    );

    // Deactivate current profile and enter freewheel state
    void deactivateProfile();

    // Update motor control (called by BLDCManager at 1kHz)
    void updateMotor();

    // Get calibration status
    bool isCalibrated() const { return calibrated_; }

    // Get profile active status
    bool isProfileActive() const { return profile_active_; }

    // Get last calibration error (if calibration failed)
    BLDC::CalibrationError getLastCalibrationError() const { return last_calibration_error_; }

    // Check if encoder communication is working
    bool isEncoderHealthy() const;

private:
    // Hardware configuration
    uint8_t motor_pin_a_;
    uint8_t motor_pin_b_;
    uint8_t motor_pin_c_;
    uint8_t motor_enable_a_;
    uint8_t motor_enable_b_;
    uint8_t encoder_cs_;
    uint8_t pole_pairs_;
    uint8_t voltage_;        // 0.1V units
    uint8_t current_limit_;  // 0.1A units (0 = no limit)
    uint8_t encoder_bits_;
    uint8_t pin_;  // Virtual pin for reporting (= encoder_cs_)

    // SimpleFOC objects (heap-allocated to avoid header dependency)
    BLDCMotor* motor_;
    MagneticSensorSPI* encoder_;

    // Calibration data
    bool calibrated_;
    uint16_t min_encoder_position_;
    uint16_t max_encoder_position_;
    BLDC::CalibrationError last_calibration_error_;

    // Active profile
    bool profile_active_;
    BLDC::DetentConfig* detents_;
    uint8_t num_detents_;
    BLDC::LinearRangeConfig* linear_ranges_;
    uint8_t num_linear_ranges_;

    // State tracking
    uint8_t current_detent_index_;
    uint16_t current_encoder_position_;
    uint8_t last_reported_detent_;
    uint32_t last_report_time_;
    bool detent_changed_;

    // Encoder health
    uint32_t last_encoder_success_time_;

    // Private helper methods
    void updateDetentState();
    float calculateTargetTorque();
    uint16_t percentToEncoderPosition(uint8_t percent) const;
    uint8_t findClosestDetent() const;
    bool isInLinearRange(uint8_t& start_detent, uint8_t& end_detent) const;
    bool validateProfile() const;
};

} // namespace Sensor
