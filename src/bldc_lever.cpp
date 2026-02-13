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

bool BLDCLever::loadProfile(
    const BLDC::DetentConfig* detents,
    uint8_t num_detents,
    const BLDC::LinearRangeConfig* ranges,
    uint8_t num_ranges
) {
    if (!calibrated_) {
        return false;  // Must calibrate before loading profile
    }

    if (detents == nullptr || num_detents == 0) {
        return false;  // Need at least one detent
    }

    // Deactivate current profile if any
    deactivateProfile();

    // Clean up old profile data
    if (detents_ != nullptr) {
        delete[] detents_;
        detents_ = nullptr;
    }
    if (linear_ranges_ != nullptr) {
        delete[] linear_ranges_;
        linear_ranges_ = nullptr;
    }

    // Allocate and copy detents
    detents_ = new BLDC::DetentConfig[num_detents];
    for (uint8_t i = 0; i < num_detents; i++) {
        detents_[i] = detents[i];
    }
    num_detents_ = num_detents;

    // Allocate and copy ranges if provided
    if (ranges != nullptr && num_ranges > 0) {
        linear_ranges_ = new BLDC::LinearRangeConfig[num_ranges];
        for (uint8_t i = 0; i < num_ranges; i++) {
            linear_ranges_[i] = ranges[i];
        }
        num_linear_ranges_ = num_ranges;
    } else {
        linear_ranges_ = nullptr;
        num_linear_ranges_ = 0;
    }

    // Validate profile
    if (!validateProfile()) {
        // Clean up on validation failure
        delete[] detents_;
        detents_ = nullptr;
        num_detents_ = 0;
        if (linear_ranges_ != nullptr) {
            delete[] linear_ranges_;
            linear_ranges_ = nullptr;
        }
        num_linear_ranges_ = 0;
        return false;
    }

    // Initialize state
    current_detent_index_ = 255;  // Invalid index to force initial detection
    last_reported_detent_ = 0;
    detent_changed_ = false;

    // Enable profile and motor
    profile_active_ = true;
    if (motor_ != nullptr) {
        motor_->enable();
    }

    return true;
}

// Private helper methods (stubs for now - will be implemented in later tasks)

void BLDCLever::updateDetentState() {
    if (encoder_ == nullptr || !profile_active_) {
        return;
    }

    // Read current encoder position
    // In real SimpleFOC, this would be encoder_->getAngle() * counts_per_rev
    // For our mock, we need to read the raw position
    #ifdef UNIT_TEST
    current_encoder_position_ = static_cast<uint16_t>(encoder_->getPosition());
    #else
    // Real hardware: convert angle to position within calibrated range
    float angle = encoder_->getAngle();
    uint32_t range = max_encoder_position_ - min_encoder_position_;
    current_encoder_position_ = min_encoder_position_ +
        static_cast<uint16_t>((angle / 6.28318f) * range);
    #endif

    // Find closest detent
    uint8_t closest_detent = findClosestDetent();

    // Check if detent changed
    if (closest_detent != current_detent_index_) {
        current_detent_index_ = closest_detent;
        last_reported_detent_ = closest_detent;
        detent_changed_ = true;
    }
}

float BLDCLever::calculateTargetTorque() {
    if (detents_ == nullptr || num_detents_ == 0 || current_detent_index_ >= num_detents_) {
        return 0.0f;
    }

    // TODO: Proper engagement/exit/spring-back logic will be added during hardware tuning
    // For now, use simplified proportional spring model:
    // - Apply torque proportional to distance from current detent
    // - Torque strength based on detent hold_strength

    // Get current detent position
    uint16_t detent_pos = percentToEncoderPosition(detents_[current_detent_index_].position_percent);

    // Calculate distance from detent (signed)
    int32_t distance = static_cast<int32_t>(current_encoder_position_) - static_cast<int32_t>(detent_pos);

    // Calculate proportional torque
    // Scale: hold_strength of 255 = max torque at max distance
    // Normalize to -1.0 to 1.0 range for SimpleFOC
    float max_distance = (max_encoder_position_ - min_encoder_position_) / 10.0f;  // 10% of range
    if (max_distance < 1.0f) max_distance = 1.0f;

    float normalized_distance = static_cast<float>(distance) / max_distance;
    // Clamp to reasonable range
    if (normalized_distance > 1.0f) normalized_distance = 1.0f;
    if (normalized_distance < -1.0f) normalized_distance = -1.0f;

    // Scale by hold strength (0-255 -> 0-1.0)
    float strength_scale = detents_[current_detent_index_].hold_strength / 255.0f;

    // Return torque (negative = pull back to detent)
    return -normalized_distance * strength_scale;
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
    if (detents_ == nullptr || num_detents_ == 0) {
        return 0;
    }

    // Linear search for closest detent
    uint8_t closest_index = 0;
    uint32_t min_distance = 0xFFFFFFFF;

    for (uint8_t i = 0; i < num_detents_; i++) {
        // Convert detent position percent to encoder position
        uint16_t detent_pos = percentToEncoderPosition(detents_[i].position_percent);

        // Calculate distance (handle wraparound)
        uint32_t distance;
        if (current_encoder_position_ >= detent_pos) {
            distance = current_encoder_position_ - detent_pos;
        } else {
            distance = detent_pos - current_encoder_position_;
        }

        if (distance < min_distance) {
            min_distance = distance;
            closest_index = i;
        }
    }

    return closest_index;
}

bool BLDCLever::isInLinearRange(uint8_t& start_detent, uint8_t& end_detent) const {
    // Stub: Will be implemented in Task 13
    // This will check if current position is in a linear range
    return false;
}

bool BLDCLever::validateProfile() const {
    if (detents_ == nullptr || num_detents_ == 0) {
        return false;
    }

    // Validate each detent
    for (uint8_t i = 0; i < num_detents_; i++) {
        // Position must be 0-100%
        if (detents_[i].position_percent > 100) {
            return false;
        }

        // If spring-back is specified, target must be valid detent index
        if (detents_[i].spring_back_target != 255 &&
            detents_[i].spring_back_target >= num_detents_) {
            return false;
        }
    }

    // Validate linear ranges if present
    if (linear_ranges_ != nullptr && num_linear_ranges_ > 0) {
        for (uint8_t i = 0; i < num_linear_ranges_; i++) {
            // Range indices must be valid
            if (linear_ranges_[i].start_detent_index >= num_detents_ ||
                linear_ranges_[i].end_detent_index >= num_detents_) {
                return false;
            }

            // Start must be different from end
            if (linear_ranges_[i].start_detent_index == linear_ranges_[i].end_detent_index) {
                return false;
            }
        }
    }

    return true;
}

} // namespace Sensor
