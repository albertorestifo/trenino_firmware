# BLDC Haptic Lever Design

**Date:** 2026-02-12
**Status:** Approved
**Target Hardware:** Arduino Mega 2560 + SimpleFOCShield v2 + AS5047D encoder

## Overview

Add support for BLDC motor-based haptic levers with virtual detents using the SimpleFOC library. The system provides configurable detent profiles with variable engagement strengths, spring-back behavior, and linear ranges between detents.

## Use Cases

- Train throttle (smooth acceleration with detents at idle, cruise speeds, max)
- Brake lever (soft detents for normal braking, hard detent for emergency, spring-back to idle)
- Gear/mode selector (discrete detents for different modes)
- Multi-function lever (combined throttle/brake with profile switching)

## Key Design Decisions

### Two-Level Configuration

**1. Hardware Configuration (EEPROM-persisted)**
- Defines that a BLDC lever exists and which board profile to use
- Triggers calibration to find physical endstops
- Minimal EEPROM footprint (1 byte: board profile)
- Persists across reboots

**2. Detent Profile Configuration (Runtime-only)**
- Actual detent positions, strengths, linear ranges
- Sent by host when train/scenario loads
- NOT persisted to EEPROM
- Can be changed without recalibration
- Allows instant switching between different train controls

### Reporting Strategy

Firmware reports **detent index** (not raw encoder position) because:
- Firmware already tracks detent state for motor control
- Host cares about semantic events ("entered detent 4") not raw positions
- Cleaner handling of spring-back behavior
- More intuitive for application layer

**Rule:** Report start detent while in linear range. Only report new detent when fully engaged (like real train gears - you're in gear 1 until gear 2 clicks in).

---

## Architecture

### Core Components

**New Files:**
- `src/bldc_lever.h/cpp` - BLDCLever class implementing ISensor
- `src/bldc_manager.h/cpp` - Static manager for motor control updates
- `src/board_profiles.h` - Hardware pin mappings for different board configs

**Modified Files:**
- `src/sensor.h` - Add `InputType::BLDCLever = 3`
- `src/sensor_manager.cpp` - Create BLDCLever instances in `applyConfiguration()`
- `src/main.cpp` - Add `BLDCManager::updateMotorControl()` call, reduce delay to 1ms
- `src/protocol.h` - Add BLDC-specific message types and Configure payload
- `src/message_handler.cpp` - Handle new message types

### Class Structure

```cpp
class BLDCLever : public ISensor {
private:
    // SimpleFOC instances
    BLDCMotor motor;
    MagneticSensorSPI encoder;

    // Hardware config (EEPROM-persisted)
    uint8_t board_profile;

    // Calibration data (runtime, recalculated on boot)
    uint16_t min_encoder_position;
    uint16_t max_encoder_position;
    bool calibrated;

    // Active profile (runtime, loaded by host)
    bool profile_active;
    DetentConfig* detents;
    uint8_t num_detents;
    LinearRangeConfig* linear_ranges;
    uint8_t num_linear_ranges;

    // State tracking
    uint8_t current_detent_index;
    uint16_t current_encoder_position;

public:
    // ISensor interface
    void begin() override;
    void scan() override;
    Reading getReading() override;
    InputType getType() const override { return InputType::BLDCLever; }
    uint8_t getPin() const override;

    // BLDC-specific
    void updateMotor();           // Called by BLDCManager at 1kHz
    bool runCalibration();        // Auto-run on config, or manual retry
    bool loadProfile(/*...*/);    // Load runtime detent profile
    void deactivateProfile();     // Return to freewheel
};
```

```cpp
namespace BLDCManager {
    void init();
    void registerLever(BLDCLever* lever);
    void updateMotorControl();  // Called from main loop at 1kHz
}
```

### Main Loop Changes

```cpp
loop() {
    PacketSerial.update();
    BLDCManager::updateMotorControl();  // New: Run motor PID loops at 1kHz
    MessageHandler.update();             // Scans sensors at ~100Hz
    delay(1);                            // Changed from 10ms to 1ms
}
```

---

## Data Structures

### Detent Configuration

```cpp
struct DetentConfig {
    uint8_t position_percent;      // 0-100% of calibrated range
    uint8_t engagement_strength;   // 0-255: torque to click INTO this detent
    uint8_t hold_strength;         // 0-255: torque to STAY in this detent
    uint8_t exit_strength;         // 0-255: torque to click OUT of this detent
    uint8_t spring_back_target;    // Detent index to return to, or 255 = none
};
```

### Linear Range Configuration

```cpp
struct LinearRangeConfig {
    uint8_t start_detent_index;
    uint8_t end_detent_index;
    uint8_t damping_strength;      // 0-255: resistance while moving through range
};
```

### Board Profile

```cpp
enum BoardProfile : uint8_t {
    SIMPLEFOC_SHIELD_V2_MEGA = 0,
    // Future profiles...
};

// Profile 0: SimpleFOCShield v2 on Arduino Mega 2560
// Motor pins: 5, 6, 9 (phases A, B, C)
// Enable pins: 7, 8
// Encoder CS: 10
// SPI: MOSI=51, MISO=50, SCK=52
```

---

## Protocol Design

### Hardware Configuration (EEPROM-persisted)

Uses standard `Configure` message with new input type:

```
MESSAGE_TYPE_CONFIGURE (2)
  Common header:
    [type: u8 = 2]
    [config_id: u32]
    [total_parts: u8]
    [part_number: u8]
    [input_type: u8 = 3]  // INPUT_TYPE_BLDC_LEVER

  BLDC payload:
    [board_profile: u8]
```

**Behavior:**
- Triggers immediate calibration
- On success: enters freewheel state, sends ConfigurationStored
- On failure: sends CalibrationError, waits for RetryCalibration

### Runtime Detent Profile (NOT persisted)

New message type for loading detent configurations:

```
MESSAGE_TYPE_LOAD_BLDC_PROFILE (11)
  Multi-part message:

  Part 0 (header):
    [type: u8 = 11]
    [pin: u8]
    [num_detents: u8]
    [num_linear_ranges: u8]

  Parts 1+N (detent data, 5 bytes each):
    [position: u8]
    [engagement: u8]
    [hold: u8]
    [exit: u8]
    [spring_back: u8]

  Final parts (linear range data, 3 bytes each):
    [start_detent: u8]
    [end_detent: u8]
    [damping: u8]
```

**Behavior:**
- Validates profile (positions 0-100, no circular spring-backs, etc.)
- Replaces any existing active profile
- Activates motor control with new haptic behavior
- On error: sends ConfigurationError, keeps previous profile (or freewheel)

### Deactivate Profile

```
MESSAGE_TYPE_DEACTIVATE_BLDC_PROFILE (12)
  [type: u8 = 12]
  [pin: u8]
```

**Behavior:**
- Immediately enters freewheel state (no motor torque)
- Clears active profile from memory
- Used when train/scenario unloads

### Retry Calibration

```
MESSAGE_TYPE_RETRY_CALIBRATION (8)
  [type: u8 = 8]
  [pin: u8]
```

**Behavior:**
- Re-runs calibration sequence
- Used after CalibrationError (user fixed obstruction)

### Error Messages

```
MESSAGE_TYPE_CALIBRATION_ERROR (9)
  [type: u8 = 9]
  [pin: u8]
  [error_code: u8]
    0 = timeout (endstops not found in 30s)
    1 = range_too_small (endstops too close, mechanical issue)
    2 = encoder_error (SPI communication failed)

MESSAGE_TYPE_ENCODER_ERROR (10)
  [type: u8 = 10]
  [pin: u8]
```

### Position Reporting

Uses standard `InputValue` message:

```
MESSAGE_TYPE_INPUT_VALUE (5)
  [type: u8 = 5]
  [pin: u8]
  [value: i16]  // Current detent index (0-based)
```

Reported when:
- Detent transition occurs (user clicks into new detent)
- Periodic updates (every ~2s like analog sensors)

---

## Behavior & Data Flow

### Boot Sequence

1. Load hardware config from EEPROM (board_profile)
2. Initialize SimpleFOC motor and encoder based on profile
3. Run calibration:
   - Sweep slowly to positive endstop (low torque, ~10% speed)
   - Record `max_encoder_position`
   - Sweep slowly to negative endstop
   - Record `min_encoder_position`
   - Validate range (must be > 1000 ticks minimum)
4. If calibration succeeds:
   - Enter freewheel state (no motor torque)
   - Send ConfigurationStored
   - Wait for host to load detent profile
5. If calibration fails:
   - Send CalibrationError with error code
   - Enter safe freewheel
   - Wait for RetryCalibration command

### Profile Loading Lifecycle

```
Boot → Calibrate → Freewheel
  ↓
Train loads → LoadBLDCProfile → Active haptics
  ↓
Train unloads → DeactivateBLDCProfile → Freewheel
  ↓
Different train loads → LoadBLDCProfile → Active haptics (replaces)
```

### Motor Control Loop (1kHz)

For each active BLDCLever in `BLDCManager::updateMotorControl()`:

1. **Read encoder position** via SimpleFOC
2. **Determine current zone:**
   - Which detent are we closest to?
   - Are we in a linear range between detents?
   - Is spring-back active?
3. **Calculate target position and torque:**
   - **Engaged in detent:** Hold at detent center, apply `hold_strength`
   - **Approaching detent:** Apply attraction (`engagement_strength`)
   - **Leaving detent:** Apply resistance (`exit_strength`)
   - **In linear range:** Light damping only (`damping_strength`)
   - **Spring-back:** Pull toward `spring_back_target` detent
4. **Apply via SimpleFOC:** `motor.move(target)` or `motor.setTorque(force)`

### Position Scanning (100Hz)

In `BLDCLever::scan()`:

1. Update detent state tracking
2. Detect detent transitions:
   - User clicks into new detent → mark for reporting
   - Spring-back completes → mark for reporting
3. Set reading flag for `getReading()`

In `BLDCLever::getReading()`:

1. If detent changed or periodic update time:
   - Return `Reading(current_detent_index, InputType::BLDCLever, pin)`
   - Reset reporting flag

### Spring-Back Example

User at detent 3, pushes to detent 4 (springs back to 3):

1. **Moving 3→4:** In linear range, report detent 3 (start detent)
2. **Engage detent 4:** Click in, report detent 4, motor begins pulling back
3. **User releases:** Motor pulls lever back toward detent 3
4. **Re-engage detent 3:** Click in, report detent 3

---

## Error Handling & Safety

### Calibration Errors

**On failure:**
- Send `CalibrationError` with error code
- Enter safe freewheel (no torque)
- Wait for `RetryCalibration` command

**Error codes:**
- `0` Timeout: Endstops not found within 30 seconds
- `1` Range too small: Endstops < 1000 encoder ticks (mechanical issue)
- `2` Encoder error: SPI communication failed

**Retry behavior:**
- Host sends `RetryCalibration` after user fixes issue
- Firmware re-runs full calibration sequence
- On success: enters freewheel, sends ConfigurationStored

### Runtime Encoder Errors

**During operation, if encoder SPI fails:**
- Send `EncoderError`
- Motor enters safe freewheel immediately
- Stop sending InputValue messages
- Auto-retry encoder communication every 100ms
- If encoder recovers:
  - Send ConfigurationStored (indicates recovery)
  - Stay in freewheel until new profile loaded

### Profile Loading Errors

**If LoadBLDCProfile is invalid:**
- Send `ConfigurationError`
- Keep current profile active (or freewheel if no profile)

**Validation checks:**
- Detent positions in 0-100 range
- No circular spring-back references
- Linear range start/end indices valid
- No overlapping linear ranges

### Safety Features

**Calibration timeout:** 30 seconds maximum
**Motor stall detection:** Abort if encoder stationary for 5s during calibration
**Emergency deactivation:** `DeactivateBLDCProfile` immediately freewheels, even during calibration or errors
**Always freewheel on error:** Never apply torque in error states

**No watchdog:** As long as serial connection is active, lever stays in configured state. Host is in control.

---

## Testing Strategy

### Unit Tests (Native Platform)

**Test files:**

1. **`test/test_bldc_detent_logic/`**
   - Detent engagement/disengagement state machine
   - Spring-back state transitions
   - Linear range detection and reporting
   - Edge cases: boundary positions, circular references

2. **`test/test_bldc_protocol/`**
   - LoadBLDCProfile message encoding/decoding
   - Multi-part message assembly
   - Configuration validation
   - Board profile mapping

3. **`test/test_bldc_calibration/`**
   - Mock encoder with simulated endstops
   - Timeout detection
   - Range validation
   - Retry behavior

**Mocking:**
- Mock SimpleFOC classes in `test/SimpleFOC.h`
- Mock SPI for encoder
- Simulate encoder positions and motor torque

### Integration Tests (Manual on Hardware)

1. **Basic calibration:**
   - Configure BLDC lever → verify calibration → check freewheel

2. **Profile loading:**
   - Load 3-detent profile → manually move lever → verify clicks

3. **Spring-back:**
   - Load profile with spring-back → push to detent → release → verify return

4. **Profile switching:**
   - Load profile A → verify → load profile B → verify change

5. **Error recovery:**
   - Block lever during calibration → error → clear → retry → success

6. **Encoder failure:**
   - Disconnect SPI → verify error/freewheel → reconnect → verify recovery

**Success criteria:**
- Reliable calibration
- Distinct, consistent detent feel
- Smooth spring-back
- No unexpected motor movement
- Safe error states (always freewheel)

---

## Implementation Notes

### SimpleFOC Integration

**Required library:** SimpleFOC 2.3+ via PlatformIO
**Sensor:** AS5047D (14-bit magnetic encoder, SPI mode)
**Motor control mode:** Position or torque control depending on zone
**Update frequency:** 1kHz minimum for smooth haptics

### Memory Considerations

**EEPROM (per lever):**
- Board profile: 1 byte
- **Total: 1 byte per BLDC lever**

**Runtime RAM (per lever):**
- Calibration data: ~4 bytes (min/max positions)
- Active profile: ~10 bytes × num_detents + ~5 bytes × num_ranges
- SimpleFOC objects: ~200 bytes
- **Total: ~250 bytes + profile size**

**Example:** 8-detent, 3-range profile = ~330 bytes RAM

### Performance Impact

**Main loop frequency:** 10ms → 1ms (10× faster)
**Impact on other sensors:**
- Analog/Button/Matrix sensors now scanned 10× more frequently (beneficial for responsiveness)
- Minimal CPU overhead - motor control is the heavy work

### Future Enhancements

- Multiple board profiles (different shields, custom pinouts)
- Configurable calibration parameters (speed, torque limits)
- Advanced haptic effects (vibration, ratcheting)
- Encoder-less mode using back-EMF sensing
- Profile presets stored on device

---

## References

- **SimpleFOC Library:** https://simplefoc.com/
- **SmartKnob Project:** https://github.com/scottbez1/smartknob (inspiration for virtual detents)
- **AS5047D Datasheet:** AMS magnetic encoder
- **SimpleFOCShield v2:** https://simplefoc.com/simplefocshield
