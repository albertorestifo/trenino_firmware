#include "bldc_lever.h"
#include "board_profiles.h"

// Only include real SimpleFOC on hardware targets
#ifndef UNIT_TEST
#include <SimpleFOC.h>
#else
// Use mock SimpleFOC for tests
#include "BLDCMotor.h"
#include "MagneticSensorSPI.h"
#endif

#include "Arduino.h"

namespace Sensor {

BLDCLever::BLDCLever(uint8_t board_profile)
    : board_profile_(board_profile)
    , pin_(0)  // Will be set based on board profile
    , motor_(nullptr)
    , encoder_(nullptr)
    , calibrated_(false)
    , min_encoder_position_(0)
    , max_encoder_position_(0)
    , last_calibration_error_(BLDC::CalibrationError::TIMEOUT)
    , profile_active_(false)
    , detents_(nullptr)
    , num_detents_(0)
    , linear_ranges_(nullptr)
    , num_linear_ranges_(0)
    , current_detent_index_(0)
    , current_encoder_position_(0)
    , last_reported_detent_(0)
    , last_report_time_(0)
    , detent_changed_(false)
    , last_encoder_success_time_(0)
{
    // Set virtual pin based on board profile
    // Use encoder CS pin as the identifying pin
    uint8_t cs_pin;
    if (BoardProfiles::getEncoderCS(board_profile_, cs_pin)) {
        pin_ = cs_pin;
    }
}

BLDCLever::~BLDCLever() {
    // Clean up dynamically allocated SimpleFOC objects
    if (motor_ != nullptr) {
        delete motor_;
        motor_ = nullptr;
    }
    if (encoder_ != nullptr) {
        delete encoder_;
        encoder_ = nullptr;
    }

    // Clean up profile data
    if (detents_ != nullptr) {
        delete[] detents_;
        detents_ = nullptr;
    }
    if (linear_ranges_ != nullptr) {
        delete[] linear_ranges_;
        linear_ranges_ = nullptr;
    }
}

void BLDCLever::begin() {
    // Get hardware pin configuration
    uint8_t pin_a, pin_b, pin_c;
    if (!BoardProfiles::getMotorPins(board_profile_, pin_a, pin_b, pin_c)) {
        return;  // Invalid board profile
    }

    uint8_t enable_a, enable_b;
    if (!BoardProfiles::getEnablePins(board_profile_, enable_a, enable_b)) {
        return;  // Invalid board profile
    }

    uint8_t cs_pin;
    if (!BoardProfiles::getEncoderCS(board_profile_, cs_pin)) {
        return;  // Invalid board profile
    }

    // Initialize encoder (AS5047D: 14-bit = 16384 CPR, SPI mode 1)
    encoder_ = new MagneticSensorSPI(cs_pin, 14, 0x3FFF);
    encoder_->init();

    // Initialize motor (11 pole pairs for typical gimbal motor)
    motor_ = new BLDCMotor(11, pin_a, pin_b, pin_c, enable_a);
    motor_->linkSensor(encoder_);

    // Configure motor
    motor_->voltage_power_supply = 12.0f;
    motor_->controller = Type_torque;  // Torque control mode
    motor_->sensor_direction = 1;

    // Initialize motor
    motor_->init();
    motor_->initFOC();

    last_encoder_success_time_ = millis();
}

void BLDCLever::scan() {
    // BLDC lever doesn't need scanning - it's initialized in begin()
    // This is a no-op for compatibility with ISensor interface
}

Reading BLDCLever::getReading() {
    Reading reading;
    reading.has_value = detent_changed_;
    reading.type = InputType::BLDCLever;
    reading.pin = pin_;
    reading.value = last_reported_detent_;

    // Reset the changed flag after reporting
    if (detent_changed_) {
        detent_changed_ = false;
    }

    return reading;
}

void BLDCLever::deactivateProfile() {
    profile_active_ = false;

    // Disable motor (freewheel mode)
    if (motor_ != nullptr) {
        motor_->disable();
    }
}

void BLDCLever::updateMotor() {
    if (motor_ == nullptr || encoder_ == nullptr) {
        return;
    }

    // Update encoder reading
    encoder_->update();

    // Update FOC control loop
    motor_->loopFOC();

    if (profile_active_) {
        // Update detent state based on current position
        updateDetentState();

        // Calculate and apply target torque
        float target_torque = calculateTargetTorque();
        motor_->move(target_torque);
    } else {
        // No profile active - apply zero torque (freewheel)
        motor_->move(0.0f);
    }
}

bool BLDCLever::isEncoderHealthy() const {
    if (encoder_ == nullptr) {
        return false;
    }

    // Check if we've received valid encoder data recently (within last 100ms)
    uint32_t now = millis();
    return (now - last_encoder_success_time_) < 100;
}

bool BLDCLever::runCalibration() {
    if (motor_ == nullptr || encoder_ == nullptr) {
        last_calibration_error_ = BLDC::CalibrationError::ENCODER_ERROR;
        return false;
    }

    // TODO: Proper endstop detection will be added during hardware testing
    // For now, mock calibration by setting reasonable min/max encoder positions
    // Real implementation will:
    // 1. Move lever in one direction until stall/endstop detected
    // 2. Record min position
    // 3. Move in opposite direction until other endstop
    // 4. Record max position
    // 5. Validate range is large enough (> MIN_ENCODER_RANGE)
    // 6. Return to center position

    // Mock values for testing (full 14-bit encoder range)
    min_encoder_position_ = 0;
    max_encoder_position_ = 16383;

    calibrated_ = true;
    last_calibration_error_ = BLDC::CalibrationError::TIMEOUT;  // No error (using enum member as success state)

    return true;
}

// Private helper methods (stubs for now - will be implemented in later tasks)

void BLDCLever::updateDetentState() {
    // Stub: Will be implemented in Task 13
    // This will:
    // - Read current encoder position
    // - Find closest detent
    // - Check if in linear range
    // - Update current_detent_index_
    // - Set detent_changed_ flag if needed
}

float BLDCLever::calculateTargetTorque() {
    // Stub: Will be implemented in Task 14
    // This will calculate torque based on:
    // - Current position relative to detents
    // - Detent strengths (engagement, hold, exit)
    // - Linear range damping
    return 0.0f;
}

uint16_t BLDCLever::percentToEncoderPosition(uint8_t percent) const {
    if (!calibrated_) {
        return 0;
    }

    // Convert 0-100% to encoder position within calibrated range
    uint32_t range = max_encoder_position_ - min_encoder_position_;
    uint32_t offset = (range * percent) / 100;
    return min_encoder_position_ + offset;
}

uint8_t BLDCLever::findClosestDetent() const {
    // Stub: Will be implemented in Task 13
    // This will find the detent closest to current_encoder_position_
    return 0;
}

bool BLDCLever::isInLinearRange(uint8_t& start_detent, uint8_t& end_detent) const {
    // Stub: Will be implemented in Task 13
    // This will check if current position is in a linear range
    return false;
}

bool BLDCLever::validateProfile() const {
    // Stub: Will be implemented in Task 12
    // This will validate detents and ranges configuration
    return true;
}

} // namespace Sensor
