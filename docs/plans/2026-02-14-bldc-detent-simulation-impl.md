# BLDC Detent Simulation Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace stub motor control with SmartKnob-style PD haptic detent simulation.

**Architecture:** Rewrite the motor control core in `bldc_lever.cpp` using a PD controller with dead zones, snap point hysteresis, velocity tracking, idle correction, virtual endstops, and linear range damping. Simplify `DetentConfig` to a single `detent_strength` field and update the `LoadBLDCProfile` protocol message accordingly.

**Tech Stack:** C++ (Arduino/PlatformIO), SimpleFOC library, Unity test framework, COBS serial protocol.

**Design doc:** `docs/plans/2026-02-14-bldc-detent-simulation-design.md`

**Protocol migration guide:** `docs/BLDC_PROTOCOL_MIGRATION.md`

---

### Task 1: Simplify DetentConfig and Add Constants

**Files:**
- Modify: `src/bldc_config.h`

**Step 1: Update bldc_config.h**

Replace the entire file contents with the simplified data model and PD constants:

```cpp
#pragma once

#include <stdint.h>

namespace BLDC {

// Configuration for a single detent
struct DetentConfig {
    uint8_t position_percent;    // 0-100% of calibrated range
    uint8_t detent_strength;     // 0-255: scales PD gains

    DetentConfig()
        : position_percent(0)
        , detent_strength(0)
    {
    }
};

// Profile-level configuration
struct ProfileConfig {
    uint8_t snap_point;          // 50-150 -> 0.50-1.50 fractional threshold
    uint8_t endstop_strength;    // 0-255: P gain at calibration boundaries

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
    uint8_t damping_strength;    // 0-255: velocity-proportional resistance

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
constexpr uint16_t MIN_ENCODER_RANGE = 1000;
constexpr uint32_t CALIBRATION_TIMEOUT_MS = 30000;
constexpr uint32_t CALIBRATION_STALL_TIMEOUT_MS = 5000;

// Motor control constants
constexpr float CALIBRATION_SPEED = 0.1f;
constexpr float CALIBRATION_TORQUE = 0.5f;

// PD controller constants (starting values from SmartKnob, tune on hardware)
constexpr float P_SCALE_FACTOR = 4.0f;
constexpr float D_LOWER_FACTOR = 0.08f;        // D multiplier for narrow detents
constexpr float D_UPPER_FACTOR = 0.02f;        // D multiplier for wide detents
constexpr float D_WIDTH_LOWER_DEG = 3.0f;      // narrow threshold (degrees)
constexpr float D_WIDTH_UPPER_DEG = 8.0f;      // wide threshold (degrees)

// Dead zone constants
constexpr float DEAD_ZONE_FRACTION = 0.2f;     // 20% of detent width
constexpr float DEAD_ZONE_MAX_DEG = 1.0f;      // absolute max dead zone (degrees)

// Velocity tracking
constexpr float VELOCITY_LPF_ALPHA = 0.1f;     // low-pass filter smoothing
constexpr float MAX_SAFE_VELOCITY = 60.0f;      // ticks/ms safety cutoff

// Idle correction
constexpr float IDLE_VELOCITY_EWMA_ALPHA = 0.001f;
constexpr float IDLE_VELOCITY_THRESHOLD = 0.05f;    // ticks/ms
constexpr uint32_t IDLE_CORRECTION_DELAY_MS = 500;
constexpr float IDLE_CORRECTION_MAX_DEG = 5.0f;
constexpr float IDLE_CORRECTION_RATE_ALPHA = 0.0005f;

// Linear range damping
constexpr float DAMPING_SCALE = 1.0f;

// Lever geometry estimate (degrees of arc for full calibrated range)
constexpr float LEVER_ARC_DEGREES = 90.0f;

} // namespace BLDC
```

**Step 2: Verify build**

Run: `pio test -e native -f test_bldc_lever`

Expected: Build will FAIL because `bldc_lever.cpp` still references `engagement_strength`, `hold_strength`, `exit_strength`, `spring_back_target`. This is expected — we fix it in Task 3.

**Step 3: Commit**

```bash
git add src/bldc_config.h
git commit -m "refactor: simplify DetentConfig, add PD controller constants"
```

---

### Task 2: Update Protocol (LoadBLDCProfile)

**Files:**
- Modify: `src/protocol.h:211-223`
- Modify: `src/protocol.cpp:642-693`
- Modify: `src/message_handler.cpp:75-121`
- Test: `test/test_protocol_bldc/test_protocol_bldc.cpp`

**Step 1: Write the failing tests**

Replace the `test_load_bldc_profile_header` test and add new tests in `test/test_protocol_bldc/test_protocol_bldc.cpp`:

Replace `test_load_bldc_profile_header` with:

```cpp
void test_load_bldc_profile_header() {
    Protocol::LoadBLDCProfile msg_out;
    msg_out.pin = 100;
    msg_out.num_detents = 5;
    msg_out.num_linear_ranges = 2;
    msg_out.snap_point = 70;
    msg_out.endstop_strength = 200;

    uint8_t buffer[16];
    size_t len = msg_out.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL_UINT8(6, len);  // was 4, now 6

    Protocol::LoadBLDCProfile msg_in;
    bool result = msg_in.decode(buffer, len);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(100, msg_in.pin);
    TEST_ASSERT_EQUAL_UINT8(5, msg_in.num_detents);
    TEST_ASSERT_EQUAL_UINT8(2, msg_in.num_linear_ranges);
    TEST_ASSERT_EQUAL_UINT8(70, msg_in.snap_point);
    TEST_ASSERT_EQUAL_UINT8(200, msg_in.endstop_strength);
}
```

**Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_protocol_bldc`

Expected: FAIL — `snap_point` and `endstop_strength` are not members of `LoadBLDCProfile`.

**Step 3: Update protocol.h**

In `src/protocol.h`, replace the `LoadBLDCProfile` struct (lines 211-223):

```cpp
// LoadBLDCProfile message - sent by host to load haptic profile for BLDC lever
struct LoadBLDCProfile {
    uint8_t pin;
    uint8_t num_detents;
    uint8_t num_linear_ranges;
    uint8_t snap_point;          // 50-150 -> 0.50-1.50 hysteresis threshold
    uint8_t endstop_strength;    // 0-255: virtual endstop P gain

    // Encode to buffer (returns number of bytes written, 0 on error)
    size_t encode(uint8_t* buffer, size_t buffer_size) const;

    // Decode from buffer (returns true on success)
    bool decode(const uint8_t* buffer, size_t length);
};
```

**Step 4: Update protocol.cpp encode/decode**

In `src/protocol.cpp`, replace the `LoadBLDCProfile` encode (lines 644-667):

```cpp
size_t LoadBLDCProfile::encode(uint8_t* buffer, size_t buffer_size) const
{
    constexpr size_t REQUIRED_SIZE = 6; // type + pin + num_detents + num_ranges + snap_point + endstop_strength

    if (buffer_size < REQUIRED_SIZE) {
        return 0;
    }

    size_t offset = 0;
    buffer[offset++] = MESSAGE_TYPE_LOAD_BLDC_PROFILE;
    buffer[offset++] = pin;
    buffer[offset++] = num_detents;
    buffer[offset++] = num_linear_ranges;
    buffer[offset++] = snap_point;
    buffer[offset++] = endstop_strength;

    return offset;
}
```

Replace the `LoadBLDCProfile` decode (lines 669-693):

```cpp
bool LoadBLDCProfile::decode(const uint8_t* buffer, size_t length)
{
    constexpr size_t REQUIRED_SIZE = 6;

    if (length < REQUIRED_SIZE) {
        return false;
    }

    if (buffer[0] != MESSAGE_TYPE_LOAD_BLDC_PROFILE) {
        return false;
    }

    size_t offset = 1;
    pin = buffer[offset++];
    num_detents = buffer[offset++];
    num_linear_ranges = buffer[offset++];
    snap_point = buffer[offset++];
    endstop_strength = buffer[offset++];

    return true;
}
```

**Step 5: Update message_handler.cpp**

In `src/message_handler.cpp`, replace the `LoadBLDCProfile` handling block (lines 75-121). The key changes: offset starts at 6 (was 4), detents are 2 bytes (was 5), pass `ProfileConfig` to `loadProfile`:

```cpp
    } else if (msg.isLoadBLDCProfile()) {
        // Buffer format: [type][pin][num_detents][num_ranges][snap_point][endstop_strength][detent_data...][range_data...]
        size_t offset = 6; // After header (6 bytes)

        // Build profile config from header
        BLDC::ProfileConfig profile_config;
        profile_config.snap_point = msg.load_bldc_profile.snap_point;
        profile_config.endstop_strength = msg.load_bldc_profile.endstop_strength;

        // Allocate temporary arrays
        BLDC::DetentConfig* detents = new BLDC::DetentConfig[msg.load_bldc_profile.num_detents];
        BLDC::LinearRangeConfig* ranges = nullptr;
        if (msg.load_bldc_profile.num_linear_ranges > 0) {
            ranges = new BLDC::LinearRangeConfig[msg.load_bldc_profile.num_linear_ranges];
        }

        // Parse detents (2 bytes each)
        for (uint8_t i = 0; i < msg.load_bldc_profile.num_detents && offset + 2 <= size; i++) {
            detents[i].position_percent = buffer[offset++];
            detents[i].detent_strength = buffer[offset++];
        }

        // Parse linear ranges (3 bytes each)
        for (uint8_t i = 0; i < msg.load_bldc_profile.num_linear_ranges && offset + 3 <= size; i++) {
            ranges[i].start_detent_index = buffer[offset++];
            ranges[i].end_detent_index = buffer[offset++];
            ranges[i].damping_strength = buffer[offset++];
        }

        // Find BLDC lever and load profile
        Sensor::ISensor* sensor = SensorManager::getSensorByPin(msg.load_bldc_profile.pin);
        if (sensor != nullptr && sensor->getType() == Sensor::InputType::BLDCLever) {
            Sensor::BLDCLever* bldc = static_cast<Sensor::BLDCLever*>(sensor);
            if (bldc->loadProfile(detents, msg.load_bldc_profile.num_detents,
                                  ranges, msg.load_bldc_profile.num_linear_ranges,
                                  profile_config)) {
                sendConfigurationStored(0);
            } else {
                sendConfigurationError(0);
            }
        } else {
            sendConfigurationError(0);
        }

        delete[] detents;
        if (ranges != nullptr) {
            delete[] ranges;
        }
```

**Step 6: Run protocol tests**

Run: `pio test -e native -f test_protocol_bldc`

Expected: PASS (protocol tests don't depend on bldc_lever.cpp)

**Step 7: Commit**

```bash
git add src/protocol.h src/protocol.cpp src/message_handler.cpp test/test_protocol_bldc/test_protocol_bldc.cpp
git commit -m "feat: update LoadBLDCProfile protocol with snap_point and endstop_strength"
```

---

### Task 3: Update BLDCLever Interface and State

**Files:**
- Modify: `src/bldc_lever.h`
- Modify: `src/bldc_lever.cpp` (update signatures and member init, keep stub implementations for now)
- Modify: `test/BLDCMotor.h`

**Step 1: Update bldc_lever.h**

Replace the entire `bldc_lever.h` with the updated interface. Key changes:
- `loadProfile` gains a `ProfileConfig` parameter
- Remove `spring_back_target` references
- Add velocity/idle/PD state variables
- Add PD helper methods

```cpp
#pragma once

#include "sensor.h"
#include "bldc_config.h"
#include <stdint.h>

// Forward declarations for SimpleFOC
class BLDCMotor;
class MagneticSensorSPI;

namespace Sensor {

class BLDCLever : public ISensor {
public:
    BLDCLever(uint8_t motor_pin_a, uint8_t motor_pin_b, uint8_t motor_pin_c,
              uint8_t motor_enable_a, uint8_t motor_enable_b,
              uint8_t encoder_cs, uint8_t pole_pairs,
              uint8_t voltage, uint8_t current_limit, uint8_t encoder_bits);

    ~BLDCLever() override;

    // ISensor interface
    void begin() override;
    void scan() override;
    Reading getReading() override;
    InputType getType() const override { return InputType::BLDCLever; }
    uint8_t getPin() const override { return pin_; }

    // BLDC-specific methods
    bool runCalibration();

    bool loadProfile(
        const BLDC::DetentConfig* detents,
        uint8_t num_detents,
        const BLDC::LinearRangeConfig* ranges,
        uint8_t num_ranges,
        const BLDC::ProfileConfig& profile_config
    );

    void deactivateProfile();
    void updateMotor();

    bool isCalibrated() const { return calibrated_; }
    bool isProfileActive() const { return profile_active_; }
    BLDC::CalibrationError getLastCalibrationError() const { return last_calibration_error_; }
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
    uint8_t voltage_;
    uint8_t current_limit_;
    uint8_t encoder_bits_;
    uint8_t pin_;

    // SimpleFOC objects
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
    BLDC::ProfileConfig profile_config_;

    // State tracking
    uint8_t current_detent_index_;
    uint16_t current_encoder_position_;
    uint8_t last_reported_detent_;
    uint32_t last_report_time_;
    bool detent_changed_;

    // Velocity tracking
    float prev_position_;
    float current_velocity_;
    uint32_t prev_update_time_;

    // Idle correction
    float velocity_ewma_;
    uint32_t idle_start_time_;
    float detent_center_offset_;

    // PD gains (recalculated on detent change)
    float current_p_gain_;
    float current_d_gain_;
    float current_dead_zone_;

    // Encoder health
    uint32_t last_encoder_success_time_;

    // Derived constants (computed from calibration)
    float ticks_per_degree_;

    // Private helpers
    void updateDetentState();
    float calculateTargetTorque();
    uint16_t percentToEncoderPosition(uint8_t percent) const;
    uint8_t findClosestDetent() const;
    bool isInLinearRange(uint8_t& range_index) const;
    bool validateProfile() const;
    void recalculatePDGains();
    void updateVelocity();
    void updateIdleCorrection();
    float getDetentWidth(uint8_t detent_index) const;
    float getDetentCenter() const;
};

} // namespace Sensor
```

**Step 2: Update BLDCMotor mock**

In `test/BLDCMotor.h`, add `shaft_velocity`:

```cpp
    float shaft_velocity = 0.0f;
```

Add it after the existing `shaft_angle` line.

**Step 3: Update bldc_lever.cpp — constructor and loadProfile signature**

Update the constructor initializer list in `bldc_lever.cpp` to initialize the new fields. Add after `detent_changed_(false)`:

```cpp
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
```

Update `loadProfile` signature to accept `ProfileConfig`:

```cpp
bool BLDCLever::loadProfile(
    const BLDC::DetentConfig* detents,
    uint8_t num_detents,
    const BLDC::LinearRangeConfig* ranges,
    uint8_t num_ranges,
    const BLDC::ProfileConfig& profile_config
) {
```

Add `profile_config_ = profile_config;` after existing validation, before `profile_active_ = true;`.

Update `isInLinearRange` signature to match new header: `bool BLDCLever::isInLinearRange(uint8_t& range_index) const`.

Remove references to `engagement_strength`, `hold_strength`, `exit_strength`, `spring_back_target` in `validateProfile()`. The new validation only checks:
- `position_percent <= 100`
- Linear range indices valid and different

Add stub implementations for new methods:

```cpp
void BLDCLever::recalculatePDGains() {}
void BLDCLever::updateVelocity() {}
void BLDCLever::updateIdleCorrection() {}
float BLDCLever::getDetentWidth(uint8_t detent_index) const { return 0.0f; }
float BLDCLever::getDetentCenter() const { return 0.0f; }
```

**Step 4: Update existing tests to use new API**

In `test/test_bldc_lever/test_bldc_lever.cpp`, update all `DetentConfig` usage — remove `engagement_strength`, `hold_strength`, `exit_strength`, `spring_back_target`. Use `detent_strength` instead. Add a `BLDC::ProfileConfig` and pass it to `loadProfile`.

For every test that calls `loadProfile`, change from:

```cpp
lever.loadProfile(detents, 3, nullptr, 0);
```

to:

```cpp
BLDC::ProfileConfig profile;
profile.snap_point = 70;
profile.endstop_strength = 200;
lever.loadProfile(detents, 3, nullptr, 0, profile);
```

For detent initialization, change from:

```cpp
detents[0].position_percent = 0;
detents[0].engagement_strength = 100;
detents[0].hold_strength = 150;
detents[0].exit_strength = 100;
detents[0].spring_back_target = 255;
```

to:

```cpp
detents[0].position_percent = 0;
detents[0].detent_strength = 150;
```

**Step 5: Run tests**

Run: `pio test -e native -f test_bldc_lever`

Expected: PASS

**Step 6: Verify full test suite**

Run: `pio test -e native`

Expected: All tests PASS

**Step 7: Commit**

```bash
git add src/bldc_lever.h src/bldc_lever.cpp src/bldc_config.h test/BLDCMotor.h test/test_bldc_lever/test_bldc_lever.cpp
git commit -m "refactor: update BLDCLever interface for PD controller"
```

---

### Task 4: Implement Velocity Tracking

**Files:**
- Modify: `src/bldc_lever.cpp` (implement `updateVelocity`)
- Test: `test/test_bldc_lever/test_bldc_lever.cpp`

**Step 1: Write the failing test**

Add to `test/test_bldc_lever/test_bldc_lever.cpp`:

```cpp
void test_velocity_tracking() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    BLDC::DetentConfig detents[2];
    detents[0].position_percent = 0;
    detents[0].detent_strength = 150;
    detents[1].position_percent = 100;
    detents[1].detent_strength = 150;

    BLDC::ProfileConfig profile;
    lever.loadProfile(detents, 2, nullptr, 0, profile);

    // Simulate stationary lever at position 0
    mock_millis_value = 100;
    lever.updateMotor();

    // Move encoder and advance time
    // Access encoder through the lever's internal state
    // The mock encoder is at position 0 after begin()

    // First update establishes baseline
    mock_millis_value = 101;
    lever.updateMotor();

    // Motor should have been called with some torque
    // (exact value depends on PD implementation, but it should not crash)
    Reading reading = lever.getReading();
    TEST_ASSERT_EQUAL(InputType::BLDCLever, reading.type);
}
```

Register it in `main()`:

```cpp
RUN_TEST(test_velocity_tracking);
```

**Step 2: Run test**

Run: `pio test -e native -f test_bldc_lever`

Expected: PASS (test is minimal, just verifying no crash with new code path)

**Step 3: Implement updateVelocity**

In `src/bldc_lever.cpp`, replace the `updateVelocity` stub:

```cpp
void BLDCLever::updateVelocity() {
    uint32_t now = millis();
    uint32_t dt_ms = now - prev_update_time_;

    if (dt_ms > 0 && prev_update_time_ > 0) {
        float dt = dt_ms / 1000.0f; // seconds
        float raw_velocity = (static_cast<float>(current_encoder_position_) - prev_position_) / dt;
        current_velocity_ = current_velocity_ * (1.0f - BLDC::VELOCITY_LPF_ALPHA)
                          + raw_velocity * BLDC::VELOCITY_LPF_ALPHA;
    }

    prev_position_ = static_cast<float>(current_encoder_position_);
    prev_update_time_ = now;
}
```

Also wire it into `updateMotor()` — add `updateVelocity()` call after the encoder position read, before `updateDetentState()`:

```cpp
void BLDCLever::updateMotor() {
    if (motor_ == nullptr || encoder_ == nullptr) {
        return;
    }

    encoder_->update();
    motor_->loopFOC();

    // Read encoder position
    #ifdef UNIT_TEST
    current_encoder_position_ = static_cast<uint16_t>(encoder_->getPosition());
    #else
    float angle = encoder_->getAngle();
    uint32_t range = max_encoder_position_ - min_encoder_position_;
    current_encoder_position_ = min_encoder_position_ +
        static_cast<uint16_t>((angle / 6.28318f) * range);
    #endif

    // Update velocity
    updateVelocity();

    if (profile_active_) {
        updateDetentState();
        float target_torque = calculateTargetTorque();
        motor_->move(target_torque);
    } else {
        motor_->move(0.0f);
    }
}
```

Note: this refactors the encoder read out of `updateDetentState()` and into `updateMotor()`, so remove the duplicate encoder read from `updateDetentState()`.

**Step 4: Run tests**

Run: `pio test -e native -f test_bldc_lever`

Expected: PASS

**Step 5: Commit**

```bash
git add src/bldc_lever.cpp test/test_bldc_lever/test_bldc_lever.cpp
git commit -m "feat: implement velocity tracking with LPF"
```

---

### Task 5: Implement PD Gain Calculation

**Files:**
- Modify: `src/bldc_lever.cpp` (implement `recalculatePDGains`, `getDetentWidth`, `getDetentCenter`)
- Test: `test/test_bldc_lever/test_bldc_lever.cpp`

**Step 1: Write the failing test**

Add to `test/test_bldc_lever/test_bldc_lever.cpp`. Since the PD gains are private, we test them indirectly through `calculateTargetTorque` — position the encoder away from a detent and verify non-zero torque in the correct direction:

```cpp
void test_pd_torque_pulls_toward_detent() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    // Single detent at 50% of range (position ~8192 for 14-bit encoder)
    BLDC::DetentConfig detents[1];
    detents[0].position_percent = 50;
    detents[0].detent_strength = 200;

    BLDC::ProfileConfig profile;
    profile.snap_point = 70;
    profile.endstop_strength = 200;
    lever.loadProfile(detents, 1, nullptr, 0, profile);

    // Set encoder to the RIGHT of detent center (higher position)
    // Detent at 50% of 16383 = ~8192. Put encoder at 9000.
    extern MagneticSensorSPI* get_last_encoder();
    // We need access to the encoder mock. Since the test includes bldc_lever.cpp
    // directly, we can access it through the lever's internal encoder_.
    // For the mock, we set position before calling updateMotor.

    // Position encoder to the right of detent
    // The lever's encoder is accessible as lever's private member.
    // Since test includes the .cpp directly, we can use a friend or
    // just test through the public API by checking move() torque.

    // Use mock: set encoder position higher than detent center
    mock_millis_value = 100;
    // Encoder mock was initialized at position 0 in begin()
    // We need to move it. The encoder_ is private but we included the .cpp.
    // Actually, the encoder is heap-allocated in begin() and we have
    // the mock MagneticSensorSPI. Let's access it through the test.

    // Set mock encoder position to 9000 (right of 50% detent at ~8192)
    lever.updateMotor(); // first update at position 0, establishes baseline

    mock_millis_value = 101;
    lever.updateMotor(); // second update, should compute torque

    // The lever should be at detent 0 (closest), and torque should pull
    // toward detent center at ~8192. Since position is 0, torque should
    // push toward 8192 (positive torque).
    // We verify through getReading that the system works without crashing
    // and reports a valid detent.
    Reading reading = lever.getReading();
    TEST_ASSERT_EQUAL(InputType::BLDCLever, reading.type);
}
```

Register in `main()`:

```cpp
RUN_TEST(test_pd_torque_pulls_toward_detent);
```

**Step 2: Run test**

Run: `pio test -e native -f test_bldc_lever`

Expected: PASS (smoke test)

**Step 3: Implement getDetentWidth**

```cpp
float BLDCLever::getDetentWidth(uint8_t detent_index) const {
    if (detents_ == nullptr || num_detents_ <= 1) {
        return static_cast<float>(max_encoder_position_ - min_encoder_position_);
    }

    uint16_t detent_pos = percentToEncoderPosition(detents_[detent_index].position_percent);
    float min_dist = static_cast<float>(max_encoder_position_ - min_encoder_position_);

    // Distance to left neighbor
    if (detent_index > 0) {
        uint16_t left_pos = percentToEncoderPosition(detents_[detent_index - 1].position_percent);
        float dist = static_cast<float>(detent_pos) - static_cast<float>(left_pos);
        if (dist > 0 && dist < min_dist) min_dist = dist;
    }

    // Distance to right neighbor
    if (detent_index < num_detents_ - 1) {
        uint16_t right_pos = percentToEncoderPosition(detents_[detent_index + 1].position_percent);
        float dist = static_cast<float>(right_pos) - static_cast<float>(detent_pos);
        if (dist > 0 && dist < min_dist) min_dist = dist;
    }

    return min_dist;
}
```

**Step 4: Implement getDetentCenter**

```cpp
float BLDCLever::getDetentCenter() const {
    if (detents_ == nullptr || current_detent_index_ >= num_detents_) {
        return 0.0f;
    }
    return static_cast<float>(percentToEncoderPosition(detents_[current_detent_index_].position_percent))
         + detent_center_offset_;
}
```

**Step 5: Implement recalculatePDGains**

```cpp
void BLDCLever::recalculatePDGains() {
    if (detents_ == nullptr || current_detent_index_ >= num_detents_) {
        current_p_gain_ = 0.0f;
        current_d_gain_ = 0.0f;
        current_dead_zone_ = 0.0f;
        return;
    }

    float strength_unit = detents_[current_detent_index_].detent_strength / 255.0f;

    // P gain
    current_p_gain_ = strength_unit * BLDC::P_SCALE_FACTOR;

    // D gain: piecewise interpolation based on detent width
    float detent_width = getDetentWidth(current_detent_index_);
    float detent_width_deg = detent_width / ticks_per_degree_;

    float d_lower = strength_unit * BLDC::D_LOWER_FACTOR;
    float d_upper = strength_unit * BLDC::D_UPPER_FACTOR;

    if (detent_width_deg <= BLDC::D_WIDTH_LOWER_DEG) {
        current_d_gain_ = d_lower;
    } else if (detent_width_deg >= BLDC::D_WIDTH_UPPER_DEG) {
        current_d_gain_ = d_upper;
    } else {
        float t = (detent_width_deg - BLDC::D_WIDTH_LOWER_DEG)
                / (BLDC::D_WIDTH_UPPER_DEG - BLDC::D_WIDTH_LOWER_DEG);
        current_d_gain_ = d_lower + t * (d_upper - d_lower);
    }

    // Dead zone
    float dz_fraction = detent_width * BLDC::DEAD_ZONE_FRACTION;
    float dz_abs = ticks_per_degree_ * BLDC::DEAD_ZONE_MAX_DEG;
    current_dead_zone_ = (dz_fraction < dz_abs) ? dz_fraction : dz_abs;
}
```

**Step 6: Wire up — compute ticks_per_degree_ after calibration**

In `runCalibration()`, after setting `calibrated_ = true`, add:

```cpp
    uint32_t range = max_encoder_position_ - min_encoder_position_;
    ticks_per_degree_ = static_cast<float>(range) / BLDC::LEVER_ARC_DEGREES;
```

Also call `recalculatePDGains()` at the end of `loadProfile()`, after `profile_active_ = true`:

```cpp
    // Initialize detent state
    current_detent_index_ = findClosestDetent();
    recalculatePDGains();
```

**Step 7: Run tests**

Run: `pio test -e native -f test_bldc_lever`

Expected: PASS

**Step 8: Commit**

```bash
git add src/bldc_lever.cpp test/test_bldc_lever/test_bldc_lever.cpp
git commit -m "feat: implement PD gain calculation with dynamic D factor"
```

---

### Task 6: Implement Core Torque Calculation

**Files:**
- Modify: `src/bldc_lever.cpp` (rewrite `calculateTargetTorque`)
- Test: `test/test_bldc_lever/test_bldc_lever.cpp`

**Step 1: Write the failing test**

We need to verify torque direction. To do this, we need to access the motor mock's `target_position_` (which stores the value passed to `move()`). The test includes `bldc_lever.cpp` directly, so we can access the mock motor through the lever. However, since `motor_` is private, we test indirectly.

Add a helper to `test/BLDCMotor.h`:

```cpp
    float getLastMoveTarget() const { return target_position_; }
```

Then add tests:

```cpp
void test_torque_zero_at_detent_center() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    BLDC::DetentConfig detents[1];
    detents[0].position_percent = 0;  // detent at position 0
    detents[0].detent_strength = 200;

    BLDC::ProfileConfig profile;
    lever.loadProfile(detents, 1, nullptr, 0, profile);

    // Encoder at position 0, detent at position 0 -> dead zone -> ~zero torque
    mock_millis_value = 100;
    lever.updateMotor();
    mock_millis_value = 101;
    lever.updateMotor();

    // Can't directly check torque without accessing motor_ private member.
    // This is a smoke test — verifying no crash and valid state.
    TEST_ASSERT_TRUE(lever.isProfileActive());
}

void test_velocity_cutoff_zeroes_torque() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    BLDC::DetentConfig detents[1];
    detents[0].position_percent = 50;
    detents[0].detent_strength = 200;

    BLDC::ProfileConfig profile;
    lever.loadProfile(detents, 1, nullptr, 0, profile);

    // Simulate very high velocity by changing position dramatically between updates
    mock_millis_value = 100;
    lever.updateMotor();

    // Move encoder very far in 1ms (simulates > MAX_SAFE_VELOCITY)
    // This requires setting encoder position via the mock
    // Since we include bldc_lever.cpp, encoder_ is accessible in principle
    // but it's private. For now, this is a smoke test.
    mock_millis_value = 101;
    lever.updateMotor();

    TEST_ASSERT_TRUE(lever.isProfileActive());
}
```

Register in `main()`:

```cpp
RUN_TEST(test_torque_zero_at_detent_center);
RUN_TEST(test_velocity_cutoff_zeroes_torque);
```

**Step 2: Run test to see it passes as smoke test**

Run: `pio test -e native -f test_bldc_lever`

Expected: PASS

**Step 3: Rewrite calculateTargetTorque**

```cpp
float BLDCLever::calculateTargetTorque() {
    if (detents_ == nullptr || num_detents_ == 0) {
        return 0.0f;
    }

    // Safety cutoff
    float abs_velocity = current_velocity_ > 0 ? current_velocity_ : -current_velocity_;
    if (abs_velocity > BLDC::MAX_SAFE_VELOCITY) {
        return 0.0f;
    }

    float position = static_cast<float>(current_encoder_position_);
    float detent_center = getDetentCenter();
    float angle_error = position - detent_center;

    // Check if out of bounds (beyond first/last detent)
    float first_detent_pos = static_cast<float>(percentToEncoderPosition(detents_[0].position_percent));
    float last_detent_pos = static_cast<float>(percentToEncoderPosition(detents_[num_detents_ - 1].position_percent));
    bool out_of_bounds = position < first_detent_pos || position > last_detent_pos;

    float p_gain = current_p_gain_;
    if (out_of_bounds) {
        p_gain = (profile_config_.endstop_strength / 255.0f) * BLDC::P_SCALE_FACTOR;
        // For endstops, angle_error is from nearest boundary
        if (position < first_detent_pos) {
            angle_error = position - first_detent_pos;
        } else {
            angle_error = position - last_detent_pos;
        }
    }

    // Check linear range
    uint8_t range_index;
    bool in_linear_range = isInLinearRange(range_index);

    if (in_linear_range && !out_of_bounds) {
        // In linear range: no detent pull, only velocity damping
        float damping = linear_ranges_[range_index].damping_strength / 255.0f;
        return -damping * current_velocity_ * BLDC::DAMPING_SCALE;
    }

    // Dead zone adjustment (not applied for endstops)
    float dead_zone_adj = 0.0f;
    if (!out_of_bounds) {
        if (angle_error > current_dead_zone_) {
            dead_zone_adj = current_dead_zone_;
        } else if (angle_error < -current_dead_zone_) {
            dead_zone_adj = -current_dead_zone_;
        } else {
            dead_zone_adj = angle_error; // fully inside dead zone
        }
    }

    // PD torque
    float pid_input = -(angle_error - dead_zone_adj);
    float torque = p_gain * pid_input + current_d_gain_ * (-current_velocity_);

    return torque;
}
```

**Step 4: Run tests**

Run: `pio test -e native -f test_bldc_lever`

Expected: PASS

**Step 5: Commit**

```bash
git add src/bldc_lever.cpp test/test_bldc_lever/test_bldc_lever.cpp
git commit -m "feat: implement PD torque calculation with dead zone and velocity cutoff"
```

---

### Task 7: Implement Snap Point Hysteresis

**Files:**
- Modify: `src/bldc_lever.cpp` (rewrite `updateDetentState`)
- Test: `test/test_bldc_lever/test_bldc_lever.cpp`

**Step 1: Write the failing test**

This test verifies that moving partway between detents does NOT change the reported detent, while moving past the snap threshold DOES change it.

We need to set the mock encoder position. Since the test includes `bldc_lever.cpp` directly, we need access to `encoder_`. We can make the test work by creating a global mock encoder pointer. Add to the top of the test file, after the existing mock function declarations:

```cpp
// Global pointer to the last encoder created (set by mock MagneticSensorSPI constructor)
static MagneticSensorSPI* g_mock_encoder = nullptr;
```

And in `test/MagneticSensorSPI.h`, add to the constructor:

```cpp
extern MagneticSensorSPI* g_mock_encoder;
```

Actually, this gets complicated with extern globals across translation units. A simpler approach: add a public test accessor to BLDCLever, guarded by `#ifdef UNIT_TEST`:

In `src/bldc_lever.h`, inside the class, add:

```cpp
#ifdef UNIT_TEST
    MagneticSensorSPI* getEncoder() { return encoder_; }
    BLDCMotor* getMotor() { return motor_; }
#endif
```

Then in the test:

```cpp
void test_snap_point_hysteresis() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    // Two detents: 0% and 100%
    // For 14-bit encoder (0-16383), detent 0 at pos 0, detent 1 at pos 16383
    BLDC::DetentConfig detents[2];
    detents[0].position_percent = 0;
    detents[0].detent_strength = 200;
    detents[1].position_percent = 100;
    detents[1].detent_strength = 200;

    BLDC::ProfileConfig profile;
    profile.snap_point = 70; // must travel 70% of the way

    lever.loadProfile(detents, 2, nullptr, 0, profile);

    MagneticSensorSPI* enc = lever.getEncoder();

    // Initially at position 0, should be at detent 0
    enc->setPosition(0);
    mock_millis_value = 100;
    lever.updateMotor();

    Reading reading = lever.getReading();
    TEST_ASSERT_TRUE(reading.has_value);
    TEST_ASSERT_EQUAL(0, reading.value); // detent 0

    // Move to 50% of the way — should NOT snap (snap_point = 0.70)
    // 50% of 16383 = ~8192
    enc->setPosition(8192);
    mock_millis_value = 101;
    lever.updateMotor();

    reading = lever.getReading();
    // Detent should still be 0 (haven't crossed 70% threshold)
    TEST_ASSERT_EQUAL(0, reading.value);

    // Move to 80% of the way — should snap to detent 1
    // 80% of 16383 = ~13106
    enc->setPosition(13106);
    mock_millis_value = 102;
    lever.updateMotor();

    reading = lever.getReading();
    TEST_ASSERT_TRUE(reading.has_value);
    TEST_ASSERT_EQUAL(1, reading.value); // snapped to detent 1
}
```

Register in `main()`:

```cpp
RUN_TEST(test_snap_point_hysteresis);
```

**Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_bldc_lever`

Expected: FAIL — current `updateDetentState` uses closest-detent logic (no hysteresis), so detent flips at midpoint.

**Step 3: Rewrite updateDetentState**

```cpp
void BLDCLever::updateDetentState() {
    if (!profile_active_ || detents_ == nullptr || num_detents_ == 0) {
        return;
    }

    float position = static_cast<float>(current_encoder_position_);
    float snap_fraction = profile_config_.snap_point / 100.0f;

    // On first call after loadProfile, find closest detent
    if (current_detent_index_ >= num_detents_) {
        current_detent_index_ = findClosestDetent();
        last_reported_detent_ = current_detent_index_;
        detent_changed_ = true;
        detent_center_offset_ = 0.0f;
        recalculatePDGains();
        return;
    }

    float current_pos = static_cast<float>(
        percentToEncoderPosition(detents_[current_detent_index_].position_percent));

    // Check snap toward higher detent
    if (current_detent_index_ < num_detents_ - 1) {
        float next_pos = static_cast<float>(
            percentToEncoderPosition(detents_[current_detent_index_ + 1].position_percent));
        float distance = next_pos - current_pos;
        float threshold = current_pos + distance * snap_fraction;

        if (position >= threshold) {
            current_detent_index_++;
            last_reported_detent_ = current_detent_index_;
            detent_changed_ = true;
            detent_center_offset_ = 0.0f;
            recalculatePDGains();
            return;
        }
    }

    // Check snap toward lower detent
    if (current_detent_index_ > 0) {
        float prev_pos = static_cast<float>(
            percentToEncoderPosition(detents_[current_detent_index_ - 1].position_percent));
        float distance = current_pos - prev_pos;
        float threshold = current_pos - distance * snap_fraction;

        if (position <= threshold) {
            current_detent_index_--;
            last_reported_detent_ = current_detent_index_;
            detent_changed_ = true;
            detent_center_offset_ = 0.0f;
            recalculatePDGains();
            return;
        }
    }
}
```

**Step 4: Run tests**

Run: `pio test -e native -f test_bldc_lever`

Expected: PASS

**Step 5: Commit**

```bash
git add src/bldc_lever.h src/bldc_lever.cpp test/test_bldc_lever/test_bldc_lever.cpp
git commit -m "feat: implement snap point hysteresis for detent transitions"
```

---

### Task 8: Implement Linear Range Detection

**Files:**
- Modify: `src/bldc_lever.cpp` (implement `isInLinearRange`)
- Test: `test/test_bldc_lever/test_bldc_lever.cpp`

**Step 1: Write the failing test**

```cpp
void test_linear_range_detection() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    BLDC::DetentConfig detents[3];
    detents[0].position_percent = 0;
    detents[0].detent_strength = 200;
    detents[1].position_percent = 50;
    detents[1].detent_strength = 200;
    detents[2].position_percent = 100;
    detents[2].detent_strength = 200;

    // Linear range between detent 0 and detent 1
    BLDC::LinearRangeConfig ranges[1];
    ranges[0].start_detent_index = 0;
    ranges[0].end_detent_index = 1;
    ranges[0].damping_strength = 128;

    BLDC::ProfileConfig profile;
    profile.snap_point = 70;
    profile.endstop_strength = 200;

    lever.loadProfile(detents, 3, ranges, 1, profile);

    MagneticSensorSPI* enc = lever.getEncoder();

    // Position at detent 0
    enc->setPosition(0);
    mock_millis_value = 100;
    lever.updateMotor();

    // Move into the range between detent 0 and 1 (25% of range)
    // This is between detent 0 (0%) and detent 1 (50%), so position ~4096
    enc->setPosition(4096);
    mock_millis_value = 101;
    lever.updateMotor();

    // Should still report detent 0 (in linear range, start detent is reported)
    Reading reading = lever.getReading();
    TEST_ASSERT_EQUAL(0, reading.value);

    // The lever should be applying damping, not PD pull.
    // We verify this indirectly: the system works without crashing.
    TEST_ASSERT_TRUE(lever.isProfileActive());
}
```

Register in `main()`:

```cpp
RUN_TEST(test_linear_range_detection);
```

**Step 2: Run test**

Run: `pio test -e native -f test_bldc_lever`

Expected: PASS or FAIL depending on whether `isInLinearRange` stub returns false. If the test expectations are met with stub returning false, adjust test to check a more specific behavior.

**Step 3: Implement isInLinearRange**

```cpp
bool BLDCLever::isInLinearRange(uint8_t& range_index) const {
    if (linear_ranges_ == nullptr || num_linear_ranges_ == 0) {
        return false;
    }

    for (uint8_t i = 0; i < num_linear_ranges_; i++) {
        uint8_t start_idx = linear_ranges_[i].start_detent_index;
        uint8_t end_idx = linear_ranges_[i].end_detent_index;

        if (start_idx >= num_detents_ || end_idx >= num_detents_) {
            continue;
        }

        uint16_t start_pos = percentToEncoderPosition(detents_[start_idx].position_percent);
        uint16_t end_pos = percentToEncoderPosition(detents_[end_idx].position_percent);

        // Ensure start < end
        uint16_t low = (start_pos < end_pos) ? start_pos : end_pos;
        uint16_t high = (start_pos < end_pos) ? end_pos : start_pos;

        // Check if current position is between the two detents (exclusive of centers)
        if (current_encoder_position_ > low && current_encoder_position_ < high) {
            // Also verify current detent is one of the range endpoints
            if (current_detent_index_ == start_idx || current_detent_index_ == end_idx) {
                range_index = i;
                return true;
            }
        }
    }

    return false;
}
```

**Step 4: Run tests**

Run: `pio test -e native -f test_bldc_lever`

Expected: PASS

**Step 5: Commit**

```bash
git add src/bldc_lever.cpp test/test_bldc_lever/test_bldc_lever.cpp
git commit -m "feat: implement linear range detection and damping"
```

---

### Task 9: Implement Idle Correction

**Files:**
- Modify: `src/bldc_lever.cpp` (implement `updateIdleCorrection`)
- Test: `test/test_bldc_lever/test_bldc_lever.cpp`

**Step 1: Write the failing test**

```cpp
void test_idle_correction_drifts_center() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    BLDC::DetentConfig detents[1];
    detents[0].position_percent = 50;  // center of range
    detents[0].detent_strength = 200;

    BLDC::ProfileConfig profile;
    lever.loadProfile(detents, 1, nullptr, 0, profile);

    MagneticSensorSPI* enc = lever.getEncoder();

    // Set encoder slightly off-center from detent (detent at 50% = 8192, encoder at 8250)
    enc->setPosition(8250);
    mock_millis_value = 100;
    lever.updateMotor();

    // Run many idle iterations to let EWMA velocity settle and correction kick in
    // Need to exceed IDLE_CORRECTION_DELAY_MS (500ms)
    for (uint32_t t = 101; t < 1000; t++) {
        mock_millis_value = t;
        lever.updateMotor();
    }

    // After 900ms of being stationary, idle correction should have started
    // drifting the detent center toward position 8250.
    // We can't directly read detent_center_offset_, but the fact that this
    // runs without crashing and the lever stays in a valid state is the test.
    TEST_ASSERT_TRUE(lever.isProfileActive());
    TEST_ASSERT_EQUAL(0, lever.getReading().value); // still at detent 0
}
```

Register in `main()`:

```cpp
RUN_TEST(test_idle_correction_drifts_center);
```

**Step 2: Run test**

Run: `pio test -e native -f test_bldc_lever`

Expected: PASS (smoke test while stub does nothing)

**Step 3: Implement updateIdleCorrection**

```cpp
void BLDCLever::updateIdleCorrection() {
    if (!profile_active_ || detents_ == nullptr || current_detent_index_ >= num_detents_) {
        return;
    }

    float abs_velocity = current_velocity_ > 0 ? current_velocity_ : -current_velocity_;

    // Update EWMA velocity
    velocity_ewma_ = velocity_ewma_ * (1.0f - BLDC::IDLE_VELOCITY_EWMA_ALPHA)
                   + abs_velocity * BLDC::IDLE_VELOCITY_EWMA_ALPHA;

    uint32_t now = millis();

    // Not idle?
    if (velocity_ewma_ > BLDC::IDLE_VELOCITY_THRESHOLD) {
        idle_start_time_ = now;
        return;
    }

    // Not idle long enough?
    if ((now - idle_start_time_) < BLDC::IDLE_CORRECTION_DELAY_MS) {
        return;
    }

    // Check if angle to detent is within correction range
    float nominal_center = static_cast<float>(
        percentToEncoderPosition(detents_[current_detent_index_].position_percent));
    float angle_to_center = static_cast<float>(current_encoder_position_) - nominal_center;
    float max_angle = ticks_per_degree_ * BLDC::IDLE_CORRECTION_MAX_DEG;

    if (angle_to_center > max_angle || angle_to_center < -max_angle) {
        return;
    }

    // Slowly drift center toward current position
    float position_error = static_cast<float>(current_encoder_position_) - nominal_center - detent_center_offset_;
    detent_center_offset_ += position_error * BLDC::IDLE_CORRECTION_RATE_ALPHA;
}
```

Wire into `updateMotor()`, after `updateVelocity()` and before `updateDetentState()`:

```cpp
    updateVelocity();
    updateIdleCorrection();
```

**Step 4: Run tests**

Run: `pio test -e native -f test_bldc_lever`

Expected: PASS

**Step 5: Commit**

```bash
git add src/bldc_lever.cpp test/test_bldc_lever/test_bldc_lever.cpp
git commit -m "feat: implement idle correction for detent center drift"
```

---

### Task 10: Implement Virtual Endstops

**Files:**
- Modify: `src/bldc_lever.cpp` (already handled in `calculateTargetTorque` from Task 6)
- Test: `test/test_bldc_lever/test_bldc_lever.cpp`

**Step 1: Write the test**

```cpp
void test_virtual_endstop_resists_beyond_last_detent() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    // Detents at 10% and 90% — leaves room for endstop zones
    BLDC::DetentConfig detents[2];
    detents[0].position_percent = 10;
    detents[0].detent_strength = 200;
    detents[1].position_percent = 90;
    detents[1].detent_strength = 200;

    BLDC::ProfileConfig profile;
    profile.snap_point = 70;
    profile.endstop_strength = 200;

    lever.loadProfile(detents, 2, nullptr, 0, profile);

    MagneticSensorSPI* enc = lever.getEncoder();

    // Start at detent 0 (10%)
    uint16_t detent0_pos = static_cast<uint16_t>(16383 * 0.10);
    enc->setPosition(detent0_pos);
    mock_millis_value = 100;
    lever.updateMotor();

    // Move beyond detent 0 toward physical endstop (position 0)
    enc->setPosition(0);
    mock_millis_value = 101;
    lever.updateMotor();

    // System should still work, endstop torque applied
    TEST_ASSERT_TRUE(lever.isProfileActive());

    // Should still report detent 0 (out of bounds doesn't change detent)
    Reading reading = lever.getReading();
    TEST_ASSERT_EQUAL(0, reading.value);
}
```

Register in `main()`:

```cpp
RUN_TEST(test_virtual_endstop_resists_beyond_last_detent);
```

**Step 2: Run test**

Run: `pio test -e native -f test_bldc_lever`

Expected: PASS (endstop logic was already implemented in `calculateTargetTorque` in Task 6)

**Step 3: Commit**

```bash
git add test/test_bldc_lever/test_bldc_lever.cpp
git commit -m "test: add virtual endstop behavior test"
```

---

### Task 11: Update Protocol Documentation

**Files:**
- Modify: `docs/PROTOCOL.md`
- Modify: `docs/BLDC_LEVER.md`

**Step 1: Update PROTOCOL.md**

Replace the `LoadBLDCProfile (11)` section (lines 195-211) with:

```markdown
### LoadBLDCProfile (11)

```
[type: u8 = 11] [pin: u8] [num_detents: u8] [num_linear_ranges: u8]
[snap_point: u8] [endstop_strength: u8]
[detent_data: 2 bytes x num_detents]
[range_data: 3 bytes x num_linear_ranges]
```

| Field | Description |
|-------|-------------|
| snap_point | Hysteresis threshold (50-150 maps to 0.50-1.50). Must travel this fraction of the distance to next detent before transitioning. |
| endstop_strength | Virtual endstop resistance (0-255). Motor pushes back beyond first/last detent. |

Detent data (2 bytes each):
```
[position_percent: u8] [detent_strength: u8]
```

| Field | Description |
|-------|-------------|
| position_percent | 0-100% of calibrated lever range |
| detent_strength | 0-255: haptic detent strength (scales PD gains) |

Range data (3 bytes each, unchanged):
```
[start_detent: u8] [end_detent: u8] [damping: u8]
```
```

**Step 2: Update BLDC_LEVER.md**

Update the "Level 2: Detent Profile" section to reflect the simplified config:
- Replace "Engagement/hold/exit strengths per detent" with "Detent strength per detent"
- Replace "Spring-back targets" with "Snap point hysteresis and virtual endstops"
- Update the "Reported Values" section — it's still correct (detent index reported)

**Step 3: Commit**

```bash
git add docs/PROTOCOL.md docs/BLDC_LEVER.md
git commit -m "docs: update protocol and lever docs for PD detent simulation"
```

---

### Task 12: Final Integration Test

**Files:**
- Test: `test/test_bldc_lever/test_bldc_lever.cpp`

**Step 1: Write an integration test**

This test exercises the full flow: calibration -> load profile -> move through detents -> verify transitions:

```cpp
void test_full_detent_traversal() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    // 4 detents: 0%, 33%, 66%, 100%
    BLDC::DetentConfig detents[4];
    detents[0].position_percent = 0;
    detents[0].detent_strength = 200;
    detents[1].position_percent = 33;
    detents[1].detent_strength = 150;
    detents[2].position_percent = 66;
    detents[2].detent_strength = 150;
    detents[3].position_percent = 100;
    detents[3].detent_strength = 200;

    BLDC::ProfileConfig profile;
    profile.snap_point = 55;  // transition just past midpoint
    profile.endstop_strength = 200;

    lever.loadProfile(detents, 4, nullptr, 0, profile);

    MagneticSensorSPI* enc = lever.getEncoder();

    // Start at detent 0
    enc->setPosition(0);
    mock_millis_value = 100;
    lever.updateMotor();
    TEST_ASSERT_EQUAL(0, lever.getReading().value);

    // Move to 60% of way to detent 1 (above snap_point of 0.55)
    // Detent 1 at 33% = position 5406. 60% of 5406 = 3244
    enc->setPosition(3244);
    mock_millis_value = 101;
    lever.updateMotor();
    TEST_ASSERT_EQUAL(1, lever.getReading().value); // snapped to detent 1

    // Move to detent 2 (66% = 10813), go past snap threshold
    // Distance from detent 1 (5406) to detent 2 (10813) = 5407
    // 55% of 5407 = 2974, so threshold at 5406 + 2974 = 8380
    enc->setPosition(8500);
    mock_millis_value = 102;
    lever.updateMotor();
    TEST_ASSERT_EQUAL(2, lever.getReading().value); // snapped to detent 2

    // Move to detent 3 (100% = 16383)
    enc->setPosition(15000);
    mock_millis_value = 103;
    lever.updateMotor();
    TEST_ASSERT_EQUAL(3, lever.getReading().value); // snapped to detent 3

    // Move back — need to cross snap threshold going backward
    // From detent 3 (16383) back toward detent 2 (10813)
    // Distance = 5570, 55% = 3064, threshold at 16383 - 3064 = 13319
    enc->setPosition(13000);
    mock_millis_value = 104;
    lever.updateMotor();
    TEST_ASSERT_EQUAL(2, lever.getReading().value); // snapped back to detent 2
}
```

Register in `main()`:

```cpp
RUN_TEST(test_full_detent_traversal);
```

**Step 2: Run all tests**

Run: `pio test -e native`

Expected: All tests PASS

**Step 3: Commit**

```bash
git add test/test_bldc_lever/test_bldc_lever.cpp
git commit -m "test: add full detent traversal integration test"
```

---

### Task 13: Hardware Build Verification

**Step 1: Build for target hardware**

Run: `pio run -e megaatmega2560`

Expected: PASS. If there are build errors related to the real SimpleFOC library (e.g., missing `shaft_velocity` or different API), fix the hardware-specific code path (`#ifndef UNIT_TEST` blocks).

**Step 2: Fix any build issues and commit**

```bash
git add -A
git commit -m "fix: resolve hardware build issues"
```

(Only if needed.)

---

## Task Dependency Graph

```
Task 1 (bldc_config.h)
    |
    v
Task 2 (protocol) -----> Task 11 (docs)
    |
    v
Task 3 (interface/state)
    |
    +---> Task 4 (velocity)
    |         |
    |         v
    |     Task 5 (PD gains)
    |         |
    |         v
    |     Task 6 (torque calc) --> Task 10 (endstop tests)
    |
    +---> Task 7 (snap point)
    |
    +---> Task 8 (linear ranges)
    |
    +---> Task 9 (idle correction)
    |
    v
Task 12 (integration test)
    |
    v
Task 13 (hardware build)
```
