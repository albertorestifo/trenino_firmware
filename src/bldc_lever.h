#pragma once

#include "sensor.h"
#include "bldc_config.h"
#include <stdint.h>

// Forward declarations for SimpleFOC (avoid including heavy headers in tests)
class BLDCMotor;
class MagneticSensorSPI;
#ifndef UNIT_TEST
class BLDCDriver3PWM;
#endif

namespace Sensors {

class BLDCLever : public ISensor {
public:
    // Constructor
    BLDCLever(uint8_t motor_pin_a, uint8_t motor_pin_b, uint8_t motor_pin_c,
              uint8_t motor_enable,
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

    // Load a detent profile with position bounds from host calibration
    // position_start/end in milliradians define the lever travel range
    // Returns true on success, false if validation fails
    bool loadProfile(
        int16_t position_start_mrad, int16_t position_end_mrad,
        const BLDCConfig::DetentConfig* detents,
        uint8_t num_detents,
        const BLDCConfig::LinearRangeConfig* ranges,
        uint8_t num_ranges,
        const BLDCConfig::ProfileConfig& profile_config
    );

    // Deactivate current profile and enter freewheel state
    void deactivateProfile();

    // Update motor control (called by BLDCManager at 1kHz)
    void updateMotor();

    // Get profile active status
    bool isProfileActive() const { return profile_active_; }

    // Check if encoder communication is working
    bool isEncoderHealthy() const;

#ifdef UNIT_TEST
    MagneticSensorSPI* getEncoder() { return encoder_; }
    BLDCMotor* getMotor() { return motor_; }
#endif

private:
    // Hardware configuration
    uint8_t motor_pin_a_;
    uint8_t motor_pin_b_;
    uint8_t motor_pin_c_;
    uint8_t motor_enable_;
    uint8_t encoder_cs_;
    uint8_t pole_pairs_;
    uint8_t voltage_;        // 0.1V units
    uint8_t current_limit_;  // 0.1A units (0 = no limit)
    uint8_t encoder_bits_;
    uint8_t pin_;  // Virtual pin for reporting (= encoder_cs_)

    // SimpleFOC objects (heap-allocated to avoid header dependency)
    BLDCMotor* motor_;
    MagneticSensorSPI* encoder_;
#ifndef UNIT_TEST
    BLDCDriver3PWM* driver_;
#endif

    // Position bounds (from host calibration via loadProfile)
    bool position_bounds_set_;
    uint16_t min_encoder_position_;
    uint16_t max_encoder_position_;
    float position_start_rad_;     // Encoder angle (rad) at 0%
    float position_end_rad_;       // Encoder angle (rad) at 100%

    // Raw encoder reporting (before profile loaded)
    int16_t current_angle_mrad_;
    uint32_t last_raw_report_time_;

    // Active profile
    bool profile_active_;
    BLDCConfig::DetentConfig* detents_;
    uint8_t num_detents_;
    BLDCConfig::LinearRangeConfig* linear_ranges_;
    uint8_t num_linear_ranges_;
    BLDCConfig::ProfileConfig profile_config_;

    // State tracking
    uint8_t current_detent_index_;
    uint16_t current_encoder_position_;
    uint8_t last_reported_detent_;
    uint32_t last_report_time_;
    bool detent_changed_;

    // PD controller state - velocity tracking
    float prev_position_;
    float current_velocity_;
    uint32_t prev_update_time_;

    // PD controller state - idle correction
    float velocity_ewma_;
    uint32_t idle_start_time_;
    float detent_center_offset_;

    // PD controller state - computed gains
    float current_p_gain_;
    float current_d_gain_;
    float current_dead_zone_;

    // Derived from calibration
    float ticks_per_degree_;

    // Encoder health
    uint32_t last_encoder_success_time_;

    // Private helper methods
    void updateDetentState();
    float calculateTargetTorque();
    uint16_t percentToEncoderPosition(uint8_t percent) const;
    uint8_t findClosestDetent() const;
    bool isInLinearRange(uint8_t& range_index) const;
    bool validateProfile(const BLDCConfig::ProfileConfig& profile_config) const;

    // PD controller helpers
    void recalculatePDGains();
    void updateVelocity();
    void updateIdleCorrection();
    float getDetentWidth(uint8_t detent_index) const;
    float getDetentCenter() const;

};

} // namespace Sensors
