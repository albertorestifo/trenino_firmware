#include "bldc_lever.h"

// Only include real SimpleFOC on hardware targets
#ifndef UNIT_TEST
#include <SimpleFOC.h>
#else
// Use mock SimpleFOC for tests
#include "BLDCMotor.h"
#include "MagneticSensorSPI.h"
#endif

#include "Arduino.h"

namespace Sensors {

BLDCLever::BLDCLever(uint8_t motor_pin_a, uint8_t motor_pin_b, uint8_t motor_pin_c,
                     uint8_t motor_enable,
                     uint8_t encoder_cs, uint8_t pole_pairs,
                     uint8_t voltage, uint8_t current_limit, uint8_t encoder_bits)
    : motor_pin_a_(motor_pin_a)
    , motor_pin_b_(motor_pin_b)
    , motor_pin_c_(motor_pin_c)
    , motor_enable_(motor_enable)
    , encoder_cs_(encoder_cs)
    , pole_pairs_(pole_pairs)
    , voltage_(voltage)
    , current_limit_(current_limit)
    , encoder_bits_(encoder_bits)
    , pin_(encoder_cs)
    , motor_(nullptr)
    , encoder_(nullptr)
#ifndef UNIT_TEST
    , driver_(nullptr)
#endif
    , calibrated_(false)
    , min_encoder_position_(0)
    , max_encoder_position_(0)
    , last_calibration_error_(BLDCConfig::CalibrationError::TIMEOUT)
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
    , prev_position_(0.0f)
    , current_velocity_(0.0f)
    , prev_update_time_(0)
    , velocity_ewma_(0.0f)
    , idle_start_time_(0)
    , detent_center_offset_(0.0f)
    , current_p_gain_(0.0f)
    , current_d_gain_(0.0f)
    , current_dead_zone_(0.0f)
    , ticks_per_degree_(0.0f)
    , last_encoder_success_time_(0)
{
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
#ifndef UNIT_TEST
    if (driver_ != nullptr) {
        delete driver_;
        driver_ = nullptr;
    }
#endif

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
#ifndef UNIT_TEST
    // Hardware path: real SimpleFOC API
    encoder_ = new MagneticSensorSPI(encoder_cs_, encoder_bits_, 0x3FFF);
    encoder_->init();

    driver_ = new BLDCDriver3PWM(motor_pin_a_, motor_pin_b_, motor_pin_c_, motor_enable_);
    driver_->voltage_power_supply = voltage_ / 10.0f;
    driver_->init();

    motor_ = new BLDCMotor(pole_pairs_);
    motor_->linkSensor(encoder_);
    motor_->linkDriver(driver_);
    motor_->controller = MotionControlType::torque;
    motor_->voltage_limit = voltage_ / 10.0f;
    if (current_limit_ > 0) {
        motor_->current_limit = current_limit_ / 10.0f;
    }
    motor_->init();
    motor_->initFOC();
#else
    // Mock path for unit tests
    uint16_t encoder_mask = (1 << encoder_bits_) - 1;
    encoder_ = new MagneticSensorSPI(encoder_cs_, encoder_bits_, encoder_mask);
    encoder_->init();

    motor_ = new BLDCMotor(pole_pairs_, motor_pin_a_, motor_pin_b_, motor_pin_c_, motor_enable_);
    motor_->linkSensor(encoder_);
    motor_->controller = Type_torque;
    motor_->init();
    motor_->initFOC();
#endif

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

    // Update encoder reading and FOC control loop
#ifdef UNIT_TEST
    encoder_->update();
#endif
    motor_->loopFOC();

    // Read encoder position (centralised here, not in updateDetentState)
#ifndef UNIT_TEST
    float angle = encoder_->getAngle();
    uint32_t range = max_encoder_position_ - min_encoder_position_;
    current_encoder_position_ = min_encoder_position_ + static_cast<uint16_t>((angle / 6.28318f) * range);
#else
    current_encoder_position_ = static_cast<uint16_t>(encoder_->getPosition());
#endif

    updateVelocity();
    updateIdleCorrection();

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
        last_calibration_error_ = BLDCConfig::CalibrationError::ENCODER_ERROR;
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
    last_calibration_error_ = BLDCConfig::CalibrationError::TIMEOUT;  // No error (using enum member as success state)

    // Compute ticks per degree from calibrated range
    uint32_t range = max_encoder_position_ - min_encoder_position_;
    ticks_per_degree_ = static_cast<float>(range) / BLDCConfig::LEVER_ARC_DEGREES;

    return true;
}

bool BLDCLever::loadProfile(
    const BLDCConfig::DetentConfig* detents,
    uint8_t num_detents,
    const BLDCConfig::LinearRangeConfig* ranges,
    uint8_t num_ranges,
    const BLDCConfig::ProfileConfig& profile_config
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
    detents_ = new BLDCConfig::DetentConfig[num_detents];
    for (uint8_t i = 0; i < num_detents; i++) {
        detents_[i] = detents[i];
    }
    num_detents_ = num_detents;

    // Allocate and copy ranges if provided
    if (ranges != nullptr && num_ranges > 0) {
        linear_ranges_ = new BLDCConfig::LinearRangeConfig[num_ranges];
        for (uint8_t i = 0; i < num_ranges; i++) {
            linear_ranges_[i] = ranges[i];
        }
        num_linear_ranges_ = num_ranges;
    } else {
        linear_ranges_ = nullptr;
        num_linear_ranges_ = 0;
    }

    // Validate profile
    if (!validateProfile(profile_config)) {
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

    // Store profile config
    profile_config_ = profile_config;

    // Enable profile and motor
    profile_active_ = true;
    if (motor_ != nullptr) {
        motor_->enable();
    }

    // PD gains will be computed on first updateDetentState() call
    // (current_detent_index_ == 255 triggers initial detection)

    return true;
}

// Private helper methods

void BLDCLever::updateDetentState() {
    if (!profile_active_ || detents_ == nullptr || num_detents_ == 0) {
        return;
    }

    // First call or invalid index: find closest detent and initialize
    if (current_detent_index_ >= num_detents_) {
        current_detent_index_ = findClosestDetent();
        last_reported_detent_ = current_detent_index_;
        detent_changed_ = true;
        detent_center_offset_ = 0.0f;
        recalculatePDGains();
        return;
    }

    float snap_fraction = profile_config_.snap_point / 100.0f;
    float current_pos = static_cast<float>(percentToEncoderPosition(detents_[current_detent_index_].position_percent));
    float pos = static_cast<float>(current_encoder_position_);

    // Check snap to higher neighbor
    if (current_detent_index_ < num_detents_ - 1) {
        float next_pos = static_cast<float>(percentToEncoderPosition(detents_[current_detent_index_ + 1].position_percent));
        float threshold = current_pos + (next_pos - current_pos) * snap_fraction;
        if (pos > threshold) {
            current_detent_index_ = current_detent_index_ + 1;
            last_reported_detent_ = current_detent_index_;
            detent_changed_ = true;
            detent_center_offset_ = 0.0f;
            recalculatePDGains();
            return;
        }
    }

    // Check snap to lower neighbor
    if (current_detent_index_ > 0) {
        float prev_pos = static_cast<float>(percentToEncoderPosition(detents_[current_detent_index_ - 1].position_percent));
        float threshold = current_pos - (current_pos - prev_pos) * snap_fraction;
        if (pos < threshold) {
            current_detent_index_ = current_detent_index_ - 1;
            last_reported_detent_ = current_detent_index_;
            detent_changed_ = true;
            detent_center_offset_ = 0.0f;
            recalculatePDGains();
            return;
        }
    }
}

float BLDCLever::calculateTargetTorque() {
    if (detents_ == nullptr || num_detents_ == 0) return 0.0f;

    // Safety cutoff
    float abs_vel = current_velocity_ > 0 ? current_velocity_ : -current_velocity_;
    if (abs_vel > BLDCConfig::MAX_SAFE_VELOCITY) return 0.0f;

    float position = static_cast<float>(current_encoder_position_);
    float detent_center = getDetentCenter();
    float angle_error = position - detent_center;

    // Out of bounds check
    float first_pos = static_cast<float>(percentToEncoderPosition(detents_[0].position_percent));
    float last_pos = static_cast<float>(percentToEncoderPosition(detents_[num_detents_ - 1].position_percent));
    bool out_of_bounds = position < first_pos || position > last_pos;

    float p_gain = current_p_gain_;
    if (out_of_bounds) {
        p_gain = (profile_config_.endstop_strength / 255.0f) * BLDCConfig::P_SCALE_FACTOR;
        if (position < first_pos) angle_error = position - first_pos;
        else angle_error = position - last_pos;
    }

    // Linear range check
    uint8_t range_index;
    bool in_range = isInLinearRange(range_index);
    if (in_range && !out_of_bounds) {
        float damping = linear_ranges_[range_index].damping_strength / 255.0f;
        return -damping * current_velocity_ * BLDCConfig::DAMPING_SCALE;
    }

    // Dead zone
    float dz_adj = 0.0f;
    if (!out_of_bounds) {
        if (angle_error > current_dead_zone_) dz_adj = current_dead_zone_;
        else if (angle_error < -current_dead_zone_) dz_adj = -current_dead_zone_;
        else dz_adj = angle_error;
    }

    float pid_input = -(angle_error - dz_adj);
    return p_gain * pid_input + current_d_gain_ * (-current_velocity_);
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

bool BLDCLever::isInLinearRange(uint8_t& range_index) const {
    if (linear_ranges_ == nullptr || num_linear_ranges_ == 0 || detents_ == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < num_linear_ranges_; i++) {
        uint8_t start_idx = linear_ranges_[i].start_detent_index;
        uint8_t end_idx = linear_ranges_[i].end_detent_index;

        if (start_idx >= num_detents_ || end_idx >= num_detents_) continue;

        float start_pos = static_cast<float>(percentToEncoderPosition(detents_[start_idx].position_percent));
        float end_pos = static_cast<float>(percentToEncoderPosition(detents_[end_idx].position_percent));

        // Ensure start < end
        if (start_pos > end_pos) {
            float tmp = start_pos;
            start_pos = end_pos;
            end_pos = tmp;
        }

        float pos = static_cast<float>(current_encoder_position_);

        // Position must be between the range endpoints AND current detent must be one of the endpoints
        if (pos > start_pos && pos < end_pos &&
            (current_detent_index_ == start_idx || current_detent_index_ == end_idx)) {
            range_index = i;
            return true;
        }
    }

    return false;
}

bool BLDCLever::validateProfile(const BLDCConfig::ProfileConfig& profile_config) const {
    if (detents_ == nullptr || num_detents_ == 0) {
        return false;
    }

    // Validate each detent
    for (uint8_t i = 0; i < num_detents_; i++) {
        // Position must be 0-100%
        if (detents_[i].position_percent > 100) {
            return false;
        }
    }

    // Validate monotonic ordering of detent positions
    for (uint8_t i = 1; i < num_detents_; i++) {
        if (detents_[i].position_percent <= detents_[i - 1].position_percent) {
            return false;
        }
    }

    // Validate snap_point range (50-150, maps to 0.50-1.50)
    if (profile_config.snap_point < 50 || profile_config.snap_point > 150) {
        return false;
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

// PD controller helper stubs (will be implemented in later tasks)

void BLDCLever::recalculatePDGains() {
    if (detents_ == nullptr || num_detents_ == 0 || current_detent_index_ >= num_detents_) {
        current_p_gain_ = 0.0f;
        current_d_gain_ = 0.0f;
        current_dead_zone_ = 0.0f;
        return;
    }

    float strength = detents_[current_detent_index_].detent_strength / 255.0f;

    // P gain
    current_p_gain_ = strength * BLDCConfig::P_SCALE_FACTOR;

    // D gain: piecewise interpolation based on detent width in degrees
    float detent_width_ticks = getDetentWidth(current_detent_index_);
    float detent_width_deg = (ticks_per_degree_ > 0.0f) ? (detent_width_ticks / ticks_per_degree_) : 0.0f;

    if (detent_width_deg <= BLDCConfig::D_WIDTH_LOWER_DEG) {
        current_d_gain_ = strength * BLDCConfig::D_LOWER_FACTOR;
    } else if (detent_width_deg >= BLDCConfig::D_WIDTH_UPPER_DEG) {
        current_d_gain_ = strength * BLDCConfig::D_UPPER_FACTOR;
    } else {
        // Linear interpolation
        float t = (detent_width_deg - BLDCConfig::D_WIDTH_LOWER_DEG) / (BLDCConfig::D_WIDTH_UPPER_DEG - BLDCConfig::D_WIDTH_LOWER_DEG);
        float d_factor = BLDCConfig::D_LOWER_FACTOR + t * (BLDCConfig::D_UPPER_FACTOR - BLDCConfig::D_LOWER_FACTOR);
        current_d_gain_ = strength * d_factor;
    }

    // Dead zone
    float dz_from_fraction = detent_width_ticks * BLDCConfig::DEAD_ZONE_FRACTION;
    float dz_max = ticks_per_degree_ * BLDCConfig::DEAD_ZONE_MAX_DEG;
    current_dead_zone_ = (dz_from_fraction < dz_max) ? dz_from_fraction : dz_max;
}

void BLDCLever::updateVelocity() {
    uint32_t now = millis();
    uint32_t dt_ms = now - prev_update_time_;
    if (dt_ms > 0 && prev_update_time_ > 0) {
        float dt = static_cast<float>(dt_ms);
        float raw_velocity = (static_cast<float>(current_encoder_position_) - prev_position_) / dt;
        current_velocity_ = current_velocity_ * (1.0f - BLDCConfig::VELOCITY_LPF_ALPHA)
                          + raw_velocity * BLDCConfig::VELOCITY_LPF_ALPHA;
    }
    prev_position_ = static_cast<float>(current_encoder_position_);
    prev_update_time_ = now;
}

void BLDCLever::updateIdleCorrection() {
    if (!profile_active_ || detents_ == nullptr || current_detent_index_ >= num_detents_) return;

    float abs_vel = current_velocity_ > 0 ? current_velocity_ : -current_velocity_;
    velocity_ewma_ = velocity_ewma_ * (1.0f - BLDCConfig::IDLE_VELOCITY_EWMA_ALPHA) + abs_vel * BLDCConfig::IDLE_VELOCITY_EWMA_ALPHA;

    uint32_t now = millis();
    if (velocity_ewma_ > BLDCConfig::IDLE_VELOCITY_THRESHOLD) { idle_start_time_ = now; return; }
    if ((now - idle_start_time_) < BLDCConfig::IDLE_CORRECTION_DELAY_MS) return;

    float nominal = static_cast<float>(percentToEncoderPosition(detents_[current_detent_index_].position_percent));
    float angle = static_cast<float>(current_encoder_position_) - nominal;
    float max_angle = ticks_per_degree_ * BLDCConfig::IDLE_CORRECTION_MAX_DEG;
    if (angle > max_angle || angle < -max_angle) return;

    float error = static_cast<float>(current_encoder_position_) - nominal - detent_center_offset_;
    detent_center_offset_ += error * BLDCConfig::IDLE_CORRECTION_RATE_ALPHA;
}

float BLDCLever::getDetentWidth(uint8_t detent_index) const {
    if (detents_ == nullptr || num_detents_ == 0 || detent_index >= num_detents_) {
        return 0.0f;
    }

    // For single-detent profiles, return the full calibrated range
    if (num_detents_ == 1) {
        return static_cast<float>(max_encoder_position_ - min_encoder_position_);
    }

    float current_pos = static_cast<float>(percentToEncoderPosition(detents_[detent_index].position_percent));
    float min_distance = static_cast<float>(max_encoder_position_ - min_encoder_position_);

    // Check distance to left neighbor
    if (detent_index > 0) {
        float neighbor_pos = static_cast<float>(percentToEncoderPosition(detents_[detent_index - 1].position_percent));
        float dist = current_pos - neighbor_pos;
        if (dist < 0.0f) dist = -dist;
        if (dist < min_distance) min_distance = dist;
    }

    // Check distance to right neighbor
    if (detent_index < num_detents_ - 1) {
        float neighbor_pos = static_cast<float>(percentToEncoderPosition(detents_[detent_index + 1].position_percent));
        float dist = neighbor_pos - current_pos;
        if (dist < 0.0f) dist = -dist;
        if (dist < min_distance) min_distance = dist;
    }

    return min_distance;
}

float BLDCLever::getDetentCenter() const {
    if (detents_ == nullptr || num_detents_ == 0 || current_detent_index_ >= num_detents_) {
        return static_cast<float>(current_encoder_position_);
    }
    float nominal = static_cast<float>(percentToEncoderPosition(detents_[current_detent_index_].position_percent));
    return nominal + detent_center_offset_;
}

} // namespace Sensors
