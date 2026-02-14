# Runtime-Configurable BLDC Parameters Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Make all BLDC motor/sensor parameters runtime-configurable via the Configure protocol message, removing hardcoded board profiles.

**Architecture:** Replace the single `board_profile` byte in the Configure message with 10 explicit parameter bytes (pins, pole pairs, voltage, current limit, encoder bits). Remove `board_profiles.h` entirely. Bump EEPROM format version to auto-invalidate old configs.

**Tech Stack:** C++11, PlatformIO, Unity test framework, SimpleFOC library

---

### Task 1: Protocol — Update Configure struct for BLDC lever

**Files:**
- Modify: `src/protocol.h:90-93`
- Modify: `src/protocol.cpp:142-143,194-196,268-273`

**Step 1: Write the failing test**

Add to `test/test_protocol_bldc/test_protocol_bldc.cpp`:

```cpp
void test_configure_bldc_lever_encode() {
    Protocol::Configure cfg;
    cfg.config_id = 0x00000001;
    cfg.total_parts = 1;
    cfg.part_number = 0;
    cfg.input_type = Protocol::INPUT_TYPE_BLDC_LEVER;
    cfg.bldc_lever.motor_pin_a = 5;
    cfg.bldc_lever.motor_pin_b = 6;
    cfg.bldc_lever.motor_pin_c = 9;
    cfg.bldc_lever.motor_enable_a = 7;
    cfg.bldc_lever.motor_enable_b = 8;
    cfg.bldc_lever.encoder_cs = 10;
    cfg.bldc_lever.pole_pairs = 11;
    cfg.bldc_lever.voltage = 120;       // 12.0V
    cfg.bldc_lever.current_limit = 0;   // no limit
    cfg.bldc_lever.encoder_bits = 14;

    uint8_t buffer[64];
    size_t size = cfg.encode(buffer, sizeof(buffer));

    // header(8) + 10 payload bytes = 18
    TEST_ASSERT_EQUAL(18, size);
    TEST_ASSERT_EQUAL_UINT8(Protocol::MESSAGE_TYPE_CONFIGURE, buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(Protocol::INPUT_TYPE_BLDC_LEVER, buffer[7]);
    TEST_ASSERT_EQUAL_UINT8(5, buffer[8]);    // motor_pin_a
    TEST_ASSERT_EQUAL_UINT8(6, buffer[9]);    // motor_pin_b
    TEST_ASSERT_EQUAL_UINT8(9, buffer[10]);   // motor_pin_c
    TEST_ASSERT_EQUAL_UINT8(7, buffer[11]);   // motor_enable_a
    TEST_ASSERT_EQUAL_UINT8(8, buffer[12]);   // motor_enable_b
    TEST_ASSERT_EQUAL_UINT8(10, buffer[13]);  // encoder_cs
    TEST_ASSERT_EQUAL_UINT8(11, buffer[14]);  // pole_pairs
    TEST_ASSERT_EQUAL_UINT8(120, buffer[15]); // voltage
    TEST_ASSERT_EQUAL_UINT8(0, buffer[16]);   // current_limit
    TEST_ASSERT_EQUAL_UINT8(14, buffer[17]);  // encoder_bits
}

void test_configure_bldc_lever_decode() {
    uint8_t buffer[] = {
        Protocol::MESSAGE_TYPE_CONFIGURE,
        0x01, 0x00, 0x00, 0x00,  // config_id
        0x01,                     // total_parts
        0x00,                     // part_number
        Protocol::INPUT_TYPE_BLDC_LEVER,
        5, 6, 9,                  // motor pins A/B/C
        7, 8,                     // enable pins A/B
        10,                       // encoder_cs
        11,                       // pole_pairs
        120,                      // voltage (12.0V)
        15,                       // current_limit (1.5A)
        14                        // encoder_bits
    };

    Protocol::Configure cfg;
    bool result = cfg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(Protocol::INPUT_TYPE_BLDC_LEVER, cfg.input_type);
    TEST_ASSERT_EQUAL_UINT8(5, cfg.bldc_lever.motor_pin_a);
    TEST_ASSERT_EQUAL_UINT8(6, cfg.bldc_lever.motor_pin_b);
    TEST_ASSERT_EQUAL_UINT8(9, cfg.bldc_lever.motor_pin_c);
    TEST_ASSERT_EQUAL_UINT8(7, cfg.bldc_lever.motor_enable_a);
    TEST_ASSERT_EQUAL_UINT8(8, cfg.bldc_lever.motor_enable_b);
    TEST_ASSERT_EQUAL_UINT8(10, cfg.bldc_lever.encoder_cs);
    TEST_ASSERT_EQUAL_UINT8(11, cfg.bldc_lever.pole_pairs);
    TEST_ASSERT_EQUAL_UINT8(120, cfg.bldc_lever.voltage);
    TEST_ASSERT_EQUAL_UINT8(15, cfg.bldc_lever.current_limit);
    TEST_ASSERT_EQUAL_UINT8(14, cfg.bldc_lever.encoder_bits);
}

void test_configure_bldc_lever_roundtrip() {
    Protocol::Configure original;
    original.config_id = 0xDEADBEEF;
    original.total_parts = 2;
    original.part_number = 1;
    original.input_type = Protocol::INPUT_TYPE_BLDC_LEVER;
    original.bldc_lever.motor_pin_a = 3;
    original.bldc_lever.motor_pin_b = 5;
    original.bldc_lever.motor_pin_c = 6;
    original.bldc_lever.motor_enable_a = 7;
    original.bldc_lever.motor_enable_b = 8;
    original.bldc_lever.encoder_cs = 10;
    original.bldc_lever.pole_pairs = 7;
    original.bldc_lever.voltage = 240;       // 24.0V
    original.bldc_lever.current_limit = 30;  // 3.0A
    original.bldc_lever.encoder_bits = 12;

    uint8_t buffer[64];
    size_t size = original.encode(buffer, sizeof(buffer));

    Protocol::Configure decoded;
    bool result = decoded.decode(buffer, size);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.motor_pin_a, decoded.bldc_lever.motor_pin_a);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.motor_pin_b, decoded.bldc_lever.motor_pin_b);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.motor_pin_c, decoded.bldc_lever.motor_pin_c);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.motor_enable_a, decoded.bldc_lever.motor_enable_a);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.motor_enable_b, decoded.bldc_lever.motor_enable_b);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.encoder_cs, decoded.bldc_lever.encoder_cs);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.pole_pairs, decoded.bldc_lever.pole_pairs);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.voltage, decoded.bldc_lever.voltage);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.current_limit, decoded.bldc_lever.current_limit);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.encoder_bits, decoded.bldc_lever.encoder_bits);
}

void test_configure_bldc_lever_decode_insufficient_data() {
    uint8_t buffer[] = {
        Protocol::MESSAGE_TYPE_CONFIGURE,
        0x01, 0x00, 0x00, 0x00,
        0x01, 0x00,
        Protocol::INPUT_TYPE_BLDC_LEVER,
        5, 6, 9  // Only 3 bytes, need 10
    };

    Protocol::Configure cfg;
    bool result = cfg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_FALSE(result);
}
```

Register all 4 tests in `main()`.

**Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_protocol_bldc -v`
Expected: Compilation error — `bldc_lever` struct has no member `motor_pin_a`

**Step 3: Update protocol.h struct**

In `src/protocol.h`, replace the `bldc_lever` union member (lines 90-93):

```cpp
// INPUT_TYPE_BLDC_LEVER
struct {
    uint8_t motor_pin_a;
    uint8_t motor_pin_b;
    uint8_t motor_pin_c;
    uint8_t motor_enable_a;
    uint8_t motor_enable_b;
    uint8_t encoder_cs;
    uint8_t pole_pairs;
    uint8_t voltage;        // 0.1V units (e.g. 120 = 12.0V)
    uint8_t current_limit;  // 0.1A units (0 = no limit)
    uint8_t encoder_bits;   // Encoder resolution (e.g. 14 for AS5047D)
} bldc_lever;
```

**Step 4: Update protocol.cpp encode/decode**

In `src/protocol.cpp`, update the `Configure::encode` BLDC case:

```cpp
case INPUT_TYPE_BLDC_LEVER:
    payload_size = 10;
    break;
```

And the encode switch body:

```cpp
case INPUT_TYPE_BLDC_LEVER:
    buffer[offset++] = bldc_lever.motor_pin_a;
    buffer[offset++] = bldc_lever.motor_pin_b;
    buffer[offset++] = bldc_lever.motor_pin_c;
    buffer[offset++] = bldc_lever.motor_enable_a;
    buffer[offset++] = bldc_lever.motor_enable_b;
    buffer[offset++] = bldc_lever.encoder_cs;
    buffer[offset++] = bldc_lever.pole_pairs;
    buffer[offset++] = bldc_lever.voltage;
    buffer[offset++] = bldc_lever.current_limit;
    buffer[offset++] = bldc_lever.encoder_bits;
    break;
```

And the decode switch body:

```cpp
case INPUT_TYPE_BLDC_LEVER:
    if (length < HEADER_SIZE + 10) {
        return false;
    }
    bldc_lever.motor_pin_a = buffer[offset++];
    bldc_lever.motor_pin_b = buffer[offset++];
    bldc_lever.motor_pin_c = buffer[offset++];
    bldc_lever.motor_enable_a = buffer[offset++];
    bldc_lever.motor_enable_b = buffer[offset++];
    bldc_lever.encoder_cs = buffer[offset++];
    bldc_lever.pole_pairs = buffer[offset++];
    bldc_lever.voltage = buffer[offset++];
    bldc_lever.current_limit = buffer[offset++];
    bldc_lever.encoder_bits = buffer[offset++];
    break;
```

**Step 5: Run test to verify it passes**

Run: `pio test -e native -f test_protocol_bldc -v`
Expected: All tests PASS

**Step 6: Run full protocol test suite**

Run: `pio test -e native -f test_protocol -v`
Expected: All existing tests still PASS (no regressions in analog/button/matrix)

**Step 7: Commit**

```bash
git add src/protocol.h src/protocol.cpp test/test_protocol_bldc/test_protocol_bldc.cpp
git commit -m "feat: expand BLDC lever Configure payload with explicit hardware params"
```

---

### Task 2: ConfigManager — Update InputConfig and EEPROM serialization

**Files:**
- Modify: `src/config_manager.h:65-68`
- Modify: `src/config_manager.cpp:147,190-193,270-272`
- Modify: `src/device_info.h:12`

**Step 1: Write the failing test**

Add to `test/test_config_manager/test_config_manager.cpp`:

```cpp
void test_store_and_load_bldc_lever() {
    ConfigManager::InputConfig inputs[1];
    inputs[0].input_type = Protocol::INPUT_TYPE_BLDC_LEVER;
    inputs[0].bldc.motor_pin_a = 5;
    inputs[0].bldc.motor_pin_b = 6;
    inputs[0].bldc.motor_pin_c = 9;
    inputs[0].bldc.motor_enable_a = 7;
    inputs[0].bldc.motor_enable_b = 8;
    inputs[0].bldc.encoder_cs = 10;
    inputs[0].bldc.pole_pairs = 11;
    inputs[0].bldc.voltage = 120;
    inputs[0].bldc.current_limit = 15;
    inputs[0].bldc.encoder_bits = 14;

    ConfigManager::storeToEEPROM(42, inputs, 1);
    bool result = ConfigManager::loadFromEEPROM();
    TEST_ASSERT_TRUE(result);

    uint8_t num_inputs = 0;
    const ConfigManager::InputConfig* loaded = ConfigManager::getCurrentConfig(num_inputs);
    TEST_ASSERT_EQUAL_UINT8(1, num_inputs);
    TEST_ASSERT_EQUAL_UINT8(Protocol::INPUT_TYPE_BLDC_LEVER, loaded[0].input_type);
    TEST_ASSERT_EQUAL_UINT8(5, loaded[0].bldc.motor_pin_a);
    TEST_ASSERT_EQUAL_UINT8(6, loaded[0].bldc.motor_pin_b);
    TEST_ASSERT_EQUAL_UINT8(9, loaded[0].bldc.motor_pin_c);
    TEST_ASSERT_EQUAL_UINT8(7, loaded[0].bldc.motor_enable_a);
    TEST_ASSERT_EQUAL_UINT8(8, loaded[0].bldc.motor_enable_b);
    TEST_ASSERT_EQUAL_UINT8(10, loaded[0].bldc.encoder_cs);
    TEST_ASSERT_EQUAL_UINT8(11, loaded[0].bldc.pole_pairs);
    TEST_ASSERT_EQUAL_UINT8(120, loaded[0].bldc.voltage);
    TEST_ASSERT_EQUAL_UINT8(15, loaded[0].bldc.current_limit);
    TEST_ASSERT_EQUAL_UINT8(14, loaded[0].bldc.encoder_bits);
}
```

Register in `main()`.

**Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_config_manager -v`
Expected: Compilation error — `bldc` struct has no member `motor_pin_a`

**Step 3: Update config_manager.h InputConfig struct**

In `src/config_manager.h`, replace the `bldc` union member (lines 65-68):

```cpp
// INPUT_TYPE_BLDC_LEVER
struct {
    uint8_t motor_pin_a;
    uint8_t motor_pin_b;
    uint8_t motor_pin_c;
    uint8_t motor_enable_a;
    uint8_t motor_enable_b;
    uint8_t encoder_cs;
    uint8_t pole_pairs;
    uint8_t voltage;
    uint8_t current_limit;
    uint8_t encoder_bits;
} bldc;
```

**Step 4: Update config_manager.h addPart()**

In `ConfigState::addPart()`, update the `INPUT_TYPE_BLDC_LEVER` case (line 148):

```cpp
case Protocol::INPUT_TYPE_BLDC_LEVER:
    inputs[cfg.part_number].bldc.motor_pin_a = cfg.bldc_lever.motor_pin_a;
    inputs[cfg.part_number].bldc.motor_pin_b = cfg.bldc_lever.motor_pin_b;
    inputs[cfg.part_number].bldc.motor_pin_c = cfg.bldc_lever.motor_pin_c;
    inputs[cfg.part_number].bldc.motor_enable_a = cfg.bldc_lever.motor_enable_a;
    inputs[cfg.part_number].bldc.motor_enable_b = cfg.bldc_lever.motor_enable_b;
    inputs[cfg.part_number].bldc.encoder_cs = cfg.bldc_lever.encoder_cs;
    inputs[cfg.part_number].bldc.pole_pairs = cfg.bldc_lever.pole_pairs;
    inputs[cfg.part_number].bldc.voltage = cfg.bldc_lever.voltage;
    inputs[cfg.part_number].bldc.current_limit = cfg.bldc_lever.current_limit;
    inputs[cfg.part_number].bldc.encoder_bits = cfg.bldc_lever.encoder_bits;
    break;
```

**Step 5: Update config_manager.cpp EEPROM store**

In `storeToEEPROM()`, update the `INPUT_TYPE_BLDC_LEVER` case (lines 190-193):

```cpp
case Protocol::INPUT_TYPE_BLDC_LEVER:
    eeprom_put(addr, inputs[i].bldc.motor_pin_a);
    addr += sizeof(uint8_t);
    eeprom_put(addr, inputs[i].bldc.motor_pin_b);
    addr += sizeof(uint8_t);
    eeprom_put(addr, inputs[i].bldc.motor_pin_c);
    addr += sizeof(uint8_t);
    eeprom_put(addr, inputs[i].bldc.motor_enable_a);
    addr += sizeof(uint8_t);
    eeprom_put(addr, inputs[i].bldc.motor_enable_b);
    addr += sizeof(uint8_t);
    eeprom_put(addr, inputs[i].bldc.encoder_cs);
    addr += sizeof(uint8_t);
    eeprom_put(addr, inputs[i].bldc.pole_pairs);
    addr += sizeof(uint8_t);
    eeprom_put(addr, inputs[i].bldc.voltage);
    addr += sizeof(uint8_t);
    eeprom_put(addr, inputs[i].bldc.current_limit);
    addr += sizeof(uint8_t);
    eeprom_put(addr, inputs[i].bldc.encoder_bits);
    addr += sizeof(uint8_t);
    break;
```

**Step 6: Update config_manager.cpp EEPROM load**

In `loadFromEEPROM()`, update the `INPUT_TYPE_BLDC_LEVER` case (lines 270-272):

```cpp
case Protocol::INPUT_TYPE_BLDC_LEVER:
    eeprom_get(addr, g_current_inputs[i].bldc.motor_pin_a);
    addr += sizeof(uint8_t);
    eeprom_get(addr, g_current_inputs[i].bldc.motor_pin_b);
    addr += sizeof(uint8_t);
    eeprom_get(addr, g_current_inputs[i].bldc.motor_pin_c);
    addr += sizeof(uint8_t);
    eeprom_get(addr, g_current_inputs[i].bldc.motor_enable_a);
    addr += sizeof(uint8_t);
    eeprom_get(addr, g_current_inputs[i].bldc.motor_enable_b);
    addr += sizeof(uint8_t);
    eeprom_get(addr, g_current_inputs[i].bldc.encoder_cs);
    addr += sizeof(uint8_t);
    eeprom_get(addr, g_current_inputs[i].bldc.pole_pairs);
    addr += sizeof(uint8_t);
    eeprom_get(addr, g_current_inputs[i].bldc.voltage);
    addr += sizeof(uint8_t);
    eeprom_get(addr, g_current_inputs[i].bldc.current_limit);
    addr += sizeof(uint8_t);
    eeprom_get(addr, g_current_inputs[i].bldc.encoder_bits);
    addr += sizeof(uint8_t);
    break;
```

**Step 7: Bump EEPROM format version**

In `src/device_info.h`, change line 12:

```cpp
// EEPROM format version - increment when EEPROM layout changes
// Version 3: BLDC lever uses explicit hardware params instead of board_profile
constexpr uint8_t EEPROM_FORMAT_VERSION = 3;
```

**Step 8: Run test to verify it passes**

Run: `pio test -e native -f test_config_manager -v`
Expected: All tests PASS

**Step 9: Commit**

```bash
git add src/config_manager.h src/config_manager.cpp src/device_info.h test/test_config_manager/test_config_manager.cpp
git commit -m "feat: BLDC config EEPROM uses explicit hardware params

Bump EEPROM_FORMAT_VERSION to 3."
```

---

### Task 3: BLDCLever — Accept explicit params instead of board profile

**Files:**
- Modify: `src/bldc_lever.h:16,62-64`
- Modify: `src/bldc_lever.cpp:17-44,68-96`
- Delete: `src/board_profiles.h`

**Step 1: Write the failing test**

Replace the constructor calls in `test/test_bldc_lever/test_bldc_lever.cpp`. First, remove the `#include "board_profiles.h"` line and update all tests to use the new constructor signature.

Replace the full file with:

```cpp
#include "bldc_lever.h"

using namespace Sensor;

// Mock Arduino functions
static unsigned long mock_millis_value = 0;
unsigned long millis() {
    return mock_millis_value;
}

void pinMode(uint8_t pin, uint8_t mode) {}
int analogRead(uint8_t pin) { return 0; }
int digitalRead(uint8_t pin) { return 0; }
void digitalWrite(uint8_t pin, uint8_t val) {}
void delayMicroseconds(unsigned int us) {}

// Include implementation after mocks are defined
#include "../../src/bldc_lever.cpp"
#include <unity.h>

// Helper: create a lever with typical test params
BLDCLever* createTestLever() {
    return new BLDCLever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
}

void test_bldc_lever_construction() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);

    TEST_ASSERT_EQUAL(InputType::BLDCLever, lever.getType());
    TEST_ASSERT_EQUAL_UINT8(10, lever.getPin());  // encoder_cs is the identifying pin
    TEST_ASSERT_FALSE(lever.isCalibrated());
    TEST_ASSERT_FALSE(lever.isProfileActive());
}

void test_bldc_lever_custom_params() {
    // Different hardware: 7 pole pairs, 24V, 3A limit, 12-bit encoder
    BLDCLever lever(3, 5, 6, 7, 8, 15, 7, 240, 30, 12);

    TEST_ASSERT_EQUAL(InputType::BLDCLever, lever.getType());
    TEST_ASSERT_EQUAL_UINT8(15, lever.getPin());  // encoder_cs pin
}

void test_calibration_success() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();

    bool result = lever.runCalibration();

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(lever.isCalibrated());
}

void test_load_profile() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    BLDC::DetentConfig detents[3];
    detents[0].position_percent = 0;
    detents[0].engagement_strength = 100;
    detents[0].hold_strength = 150;
    detents[0].exit_strength = 100;
    detents[0].spring_back_target = 255;

    detents[1].position_percent = 50;
    detents[1].engagement_strength = 100;
    detents[1].hold_strength = 150;
    detents[1].exit_strength = 100;
    detents[1].spring_back_target = 255;

    detents[2].position_percent = 100;
    detents[2].engagement_strength = 100;
    detents[2].hold_strength = 150;
    detents[2].exit_strength = 100;
    detents[2].spring_back_target = 255;

    bool result = lever.loadProfile(detents, 3, nullptr, 0);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(lever.isProfileActive());
}

void test_detent_state_tracking() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    BLDC::DetentConfig detents[3];
    detents[0].position_percent = 0;
    detents[0].engagement_strength = 100;
    detents[0].hold_strength = 150;
    detents[0].exit_strength = 100;
    detents[0].spring_back_target = 255;

    detents[1].position_percent = 50;
    detents[1].engagement_strength = 100;
    detents[1].hold_strength = 150;
    detents[1].exit_strength = 100;
    detents[1].spring_back_target = 255;

    detents[2].position_percent = 100;
    detents[2].engagement_strength = 100;
    detents[2].hold_strength = 150;
    detents[2].exit_strength = 100;
    detents[2].spring_back_target = 255;

    lever.loadProfile(detents, 3, nullptr, 0);

    Reading reading = lever.getReading();
    TEST_ASSERT_FALSE(reading.has_value);

    lever.updateMotor();

    reading = lever.getReading();
    TEST_ASSERT_TRUE(reading.has_value);
    TEST_ASSERT_EQUAL(InputType::BLDCLever, reading.type);
    TEST_ASSERT_EQUAL(0, reading.value);
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_bldc_lever_construction);
    RUN_TEST(test_bldc_lever_custom_params);
    RUN_TEST(test_calibration_success);
    RUN_TEST(test_load_profile);
    RUN_TEST(test_detent_state_tracking);
    return UNITY_END();
}
```

**Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_bldc_lever -v`
Expected: Compilation error — `BLDCLever` constructor takes wrong number of arguments

**Step 3: Update bldc_lever.h**

Replace the constructor and private members in `src/bldc_lever.h`:

```cpp
// Constructor — takes explicit hardware parameters
BLDCLever(uint8_t motor_pin_a, uint8_t motor_pin_b, uint8_t motor_pin_c,
          uint8_t motor_enable_a, uint8_t motor_enable_b,
          uint8_t encoder_cs, uint8_t pole_pairs,
          uint8_t voltage, uint8_t current_limit, uint8_t encoder_bits);
```

Replace private hardware config members:

```cpp
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
```

Remove the `board_profile_` member entirely.

**Step 4: Update bldc_lever.cpp**

Replace the constructor in `src/bldc_lever.cpp`:

```cpp
BLDCLever::BLDCLever(uint8_t motor_pin_a, uint8_t motor_pin_b, uint8_t motor_pin_c,
                     uint8_t motor_enable_a, uint8_t motor_enable_b,
                     uint8_t encoder_cs, uint8_t pole_pairs,
                     uint8_t voltage, uint8_t current_limit, uint8_t encoder_bits)
    : motor_pin_a_(motor_pin_a)
    , motor_pin_b_(motor_pin_b)
    , motor_pin_c_(motor_pin_c)
    , motor_enable_a_(motor_enable_a)
    , motor_enable_b_(motor_enable_b)
    , encoder_cs_(encoder_cs)
    , pole_pairs_(pole_pairs)
    , voltage_(voltage)
    , current_limit_(current_limit)
    , encoder_bits_(encoder_bits)
    , pin_(encoder_cs)
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
}
```

Replace `begin()` to use stored members instead of board profile lookups:

```cpp
void BLDCLever::begin() {
    // Initialize encoder using configured params
    uint16_t encoder_mask = (1 << encoder_bits_) - 1;
    encoder_ = new MagneticSensorSPI(encoder_cs_, encoder_bits_, encoder_mask);
    encoder_->init();

    // Initialize motor with configured params
    motor_ = new BLDCMotor(pole_pairs_, motor_pin_a_, motor_pin_b_, motor_pin_c_, motor_enable_a_);
    motor_->linkSensor(encoder_);

    // Configure motor
    motor_->voltage_power_supply = voltage_ / 10.0f;
    motor_->controller = Type_torque;
    motor_->sensor_direction = 1;

    // Set current limit if specified
    if (current_limit_ > 0) {
        motor_->current_limit = current_limit_ / 10.0f;
    }

    // Initialize motor
    motor_->init();
    motor_->initFOC();

    last_encoder_success_time_ = millis();
}
```

Remove the `#include "board_profiles.h"` from `bldc_lever.cpp`.

**Step 5: Add `current_limit` field to mock BLDCMotor**

In `test/BLDCMotor.h`, add to the public section:

```cpp
float current_limit = 0.0f;
```

**Step 6: Run test to verify it passes**

Run: `pio test -e native -f test_bldc_lever -v`
Expected: All tests PASS

**Step 7: Commit**

```bash
git add src/bldc_lever.h src/bldc_lever.cpp test/test_bldc_lever/test_bldc_lever.cpp test/BLDCMotor.h
git commit -m "refactor: BLDCLever takes explicit hardware params instead of board profile"
```

---

### Task 4: SensorManager — Wire new params through, delete board_profiles.h

**Files:**
- Modify: `src/sensor_manager.cpp:69-70`
- Delete: `src/board_profiles.h`

**Step 1: Update sensor_manager.cpp**

In `src/sensor_manager.cpp`, update the `INPUT_TYPE_BLDC_LEVER` case (line 69-70):

```cpp
case Protocol::INPUT_TYPE_BLDC_LEVER:
    sensor = new Sensor::BLDCLever(
        config.bldc.motor_pin_a,
        config.bldc.motor_pin_b,
        config.bldc.motor_pin_c,
        config.bldc.motor_enable_a,
        config.bldc.motor_enable_b,
        config.bldc.encoder_cs,
        config.bldc.pole_pairs,
        config.bldc.voltage,
        config.bldc.current_limit,
        config.bldc.encoder_bits);
    break;
```

**Step 2: Delete board_profiles.h**

```bash
git rm src/board_profiles.h
```

**Step 3: Run all tests**

Run: `pio test -e native -v`
Expected: All tests PASS across all test suites

**Step 4: Build for hardware target**

Run: `pio run -e megaatmega2560`
Expected: Build succeeds (verifies no remaining references to board_profiles.h)

**Step 5: Commit**

```bash
git add src/sensor_manager.cpp
git rm src/board_profiles.h
git commit -m "feat: wire explicit BLDC params through SensorManager, remove board_profiles.h"
```

---

### Task 5: Update protocol documentation

**Files:**
- Modify: `docs/PROTOCOL.md` (if it documents the Configure message BLDC payload)

**Step 1: Check if PROTOCOL.md needs updating**

Read `docs/PROTOCOL.md` and update any documentation of the BLDC lever Configure payload to reflect the new 10-byte format.

**Step 2: Commit**

```bash
git add docs/PROTOCOL.md
git commit -m "docs: update BLDC lever Configure payload in protocol spec"
```
