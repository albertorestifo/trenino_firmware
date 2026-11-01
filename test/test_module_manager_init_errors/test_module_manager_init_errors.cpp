// Regression test for ModuleManager init-error storage and drain semantics.
//
// sensor_manager.cpp is excluded from the native build's src filter, so we
// inline-include it here as a single translation unit.  applyConfiguration()
// references concrete module constructors (AnalogSensor, ButtonSensor,
// MatrixSensor, HT16K33Module), so we provide minimal stub definitions for
// all of them below so the linker is satisfied — even though no test in this
// file ever calls applyConfiguration with real modules.
//
// The tests only exercise:
//   ModuleManager::init()
//   ModuleManager::testInjectInitError()   (UNIT_TEST guard)
//   ModuleManager::getInitErrors()
//
// Wire.h must be included before sensor_manager.cpp so that the HT16K33Module
// stub compiled inside sensor_manager.cpp can see the TwoWire mock type.

#define UNIT_TEST
#define ARDUINO_ARCH_AVR

#include "../Wire.h"
#include "../Arduino.h"
#include "../EEPROM.h"

// ---- Arduino stub implementations ----
// sensor_manager.cpp's headers declare/use these; the test never calls them
// but they must be defined to satisfy the linker.
void pinMode(uint8_t, uint8_t) {}
int analogRead(uint8_t) { return 0; }
int digitalRead(uint8_t) { return 0; }
void digitalWrite(uint8_t, uint8_t) {}
void delay(unsigned long) {}
void delayMicroseconds(unsigned int) {}
unsigned long millis() { return 0; }

// ---- EEPROM mock storage ----
uint8_t mock_eeprom_storage[1024] = {};
EEPROMClass EEPROM;

// ---- Stub constructors for concrete module types ----
// applyConfiguration() references these via `new`, but the loop body never
// executes when module_count == 0.  We still need definitions at link time.

#include "../../src/analog_sensor.h"
#include "../../src/button_sensor.h"
#include "../../src/matrix_sensor.h"
#include "../../src/matrix_core.cpp"
#include "../../src/ht16k33_module.h"
#include "../../src/mcp23017_module.h"
#include "../../src/ads1115_module.h"

namespace Modules {

AnalogSensor::AnalogSensor(uint8_t pin_number, uint8_t sensitivity_level)
    : pin(pin_number)
    , sensitivity(sensitivity_level)
    , current_value(0)
    , last_sent(0)
    , scans_since_send(0)
    , min_send_interval(0)
{}
bool AnalogSensor::begin() { return true; }
void AnalogSensor::scan() {}
Reading AnalogSensor::getReading() { return Reading(); }
bool AnalogSensor::shouldSend() { return false; }
uint16_t AnalogSensor::computeMinSendInterval() const { return 0; }

ButtonSensor::ButtonSensor(uint8_t pin_number, uint8_t debounce_scans)
    : pin(pin_number)
    , debounce_threshold(debounce_scans)
    , current_state(false)
    , last_reported(false)
    , raw_state(false)
    , debounce_count(0)
    , has_pending_event(false)
{}
bool ButtonSensor::begin() { return true; }
void ButtonSensor::scan() {}
Reading ButtonSensor::getReading() { return Reading(); }

MatrixSensor::MatrixSensor(uint8_t rows, uint8_t cols,
                            const uint8_t* row_pin_array, const uint8_t* col_pin_array)
    : num_rows(rows)
    , num_cols(cols)
    , core(rows, cols, DEFAULT_DEBOUNCE)
{
    for (uint8_t i = 0; i < rows && i < MAX_ROWS; i++) row_pins[i] = row_pin_array[i];
    for (uint8_t i = 0; i < cols && i < MAX_COLS; i++) col_pins[i] = col_pin_array[i];
}
bool MatrixSensor::begin() { return true; }
void MatrixSensor::scan() {}
Reading MatrixSensor::getReading() { return Reading(); }

// HT16K33Module stubs — applyConfiguration() calls `new HT16K33Module(...)` so we
// need a definition, but no test here exercises the real I2C logic.
HT16K33Module::HT16K33Module(uint8_t i2c_address, uint8_t brightness, uint8_t num_digits)
    : i2c_address_(i2c_address)
    , cached_brightness_(brightness > 15 ? 15 : brightness)
    , num_digits_(num_digits)
    , cached_segment_bytes_(0)
    , failure_count_(0)
    , needs_reinit_(false)
{
    for (uint8_t i = 0; i < MAX_DISPLAY_BYTES; i++) cached_segments_[i] = 0;
}
bool HT16K33Module::begin() { return true; }
bool HT16K33Module::writeSegments(const uint8_t*, uint8_t) { return true; }
bool HT16K33Module::setBrightness(uint8_t) { return true; }
bool HT16K33Module::runInitSequence() { return true; }
bool HT16K33Module::writeWithRetry(const uint8_t*, uint8_t) { return true; }

// MCP23017 / ADS1115 stubs — applyConfiguration() references their constructors
// and vtables; no test here runs their real logic.
Mcp23017Module::Mcp23017Module(uint8_t address, uint8_t, uint8_t, uint8_t, uint16_t, uint16_t, uint16_t)
    : address_(address), matrix_(nullptr) {}
Mcp23017Module::Mcp23017Module(const Mcp23017Module& other)
    : address_(other.address_), matrix_(nullptr) {}
Mcp23017Module::~Mcp23017Module() {}
bool Mcp23017Module::begin() { return true; }
void Mcp23017Module::scan() {}
Reading Mcp23017Module::getReading() { return Reading(); }
uint8_t Mcp23017Module::takeRuntimeError() { return MODULE_ERROR_NONE; }
uint8_t Mcp23017Module::activeError() const { return MODULE_ERROR_NONE; }

Ads1115Module::Ads1115Module(uint8_t address, uint8_t, uint8_t, uint8_t, uint8_t, const uint8_t*)
    : address_(address) {}
bool Ads1115Module::begin() { return true; }
void Ads1115Module::scan() {}
Reading Ads1115Module::getReading() { return Reading(); }
uint8_t Ads1115Module::takeRuntimeError() { return MODULE_ERROR_NONE; }
uint8_t Ads1115Module::activeError() const { return MODULE_ERROR_NONE; }

} // namespace Modules

// ---- Now inline-include the unit under test ----
#include "../../src/sensor_manager.cpp"

// ---- Tests ----
#include <unity.h>

void setUp() {}
void tearDown() {}

void test_drain_returns_injected_errors()
{
    ModuleManager::init();

    ModuleManager::InitError e1{ Modules::ModuleType::HT16K33, 0x70, 0, 0 };
    ModuleManager::InitError e2{ Modules::ModuleType::HT16K33, 0x71, 0, 0 };
    ModuleManager::testInjectInitError(e1);
    ModuleManager::testInjectInitError(e2);

    ModuleManager::InitError out[ModuleManager::MAX_MODULES];
    uint8_t n = ModuleManager::getInitErrors(out, ModuleManager::MAX_MODULES);

    TEST_ASSERT_EQUAL_UINT8(2, n);
    TEST_ASSERT_EQUAL_UINT8(0x70, out[0].i2c_address);
    TEST_ASSERT_EQUAL_UINT8(0x71, out[1].i2c_address);
    TEST_ASSERT_EQUAL_UINT8(0, out[0].error_code);
    TEST_ASSERT_EQUAL_UINT8(0, out[1].error_code);

    // Second drain must return nothing (list was cleared)
    n = ModuleManager::getInitErrors(out, ModuleManager::MAX_MODULES);
    TEST_ASSERT_EQUAL_UINT8(0, n);
}

void test_drain_caps_at_max_count()
{
    ModuleManager::init();

    for (uint8_t i = 0; i < 4; i++) {
        ModuleManager::InitError e{ Modules::ModuleType::HT16K33, (uint8_t)(0x70 + i), 0, 0 };
        ModuleManager::testInjectInitError(e);
    }

    ModuleManager::InitError out[2];
    uint8_t n = ModuleManager::getInitErrors(out, 2);

    TEST_ASSERT_EQUAL_UINT8(2, n);
    TEST_ASSERT_EQUAL_UINT8(0x70, out[0].i2c_address);
    TEST_ASSERT_EQUAL_UINT8(0x71, out[1].i2c_address);
}

// Fake I2C module: queued runtime errors plus a fixed active error.
class FakeModule : public Modules::IModule {
public:
    FakeModule(uint8_t address, uint8_t active)
        : address_(address), active_(active), pending_count_(0), next_(0) {}
    void push(uint8_t code) { pending_[pending_count_++] = code; }
    bool begin() override { return true; }
    Modules::ModuleType getType() const override { return Modules::ModuleType::MCP23017; }
    uint8_t getPin() const override { return 0; }
    uint8_t getI2CAddress() const override { return address_; }
    uint8_t takeRuntimeError() override
    {
        return next_ < pending_count_ ? pending_[next_++] : Modules::MODULE_ERROR_NONE;
    }
    uint8_t activeError() const override { return active_; }

private:
    uint8_t address_;
    uint8_t active_;
    uint8_t pending_[4];
    uint8_t pending_count_;
    uint8_t next_;
};

void test_runtime_errors_drain_with_address()
{
    ModuleManager::init();
    FakeModule* a = new FakeModule(0x20, Modules::MODULE_ERROR_NONE);
    FakeModule* b = new FakeModule(0x48, Modules::MODULE_ERROR_NONE);
    a->push(1);
    a->push(2);
    b->push(3);
    ModuleManager::testAddModule(a);
    ModuleManager::testAddModule(b);

    uint8_t address = 0, code = 0;
    TEST_ASSERT_TRUE(ModuleManager::getNextRuntimeError(address, code));
    TEST_ASSERT_EQUAL_UINT8(0x20, address);
    TEST_ASSERT_EQUAL_UINT8(1, code);
    TEST_ASSERT_TRUE(ModuleManager::getNextRuntimeError(address, code));
    TEST_ASSERT_EQUAL_UINT8(0x20, address);
    TEST_ASSERT_EQUAL_UINT8(2, code);
    TEST_ASSERT_TRUE(ModuleManager::getNextRuntimeError(address, code));
    TEST_ASSERT_EQUAL_UINT8(0x48, address);
    TEST_ASSERT_EQUAL_UINT8(3, code);
    TEST_ASSERT_FALSE(ModuleManager::getNextRuntimeError(address, code));
    ModuleManager::init();
}

void test_active_errors_list_failed_modules()
{
    ModuleManager::init();
    ModuleManager::testAddModule(new FakeModule(0x20, 1));
    ModuleManager::testAddModule(new FakeModule(0x21, Modules::MODULE_ERROR_NONE));
    ModuleManager::testAddModule(new FakeModule(0x48, 3));

    ModuleManager::InitError out[ModuleManager::MAX_MODULES];
    uint8_t n = ModuleManager::getActiveErrors(out, ModuleManager::MAX_MODULES);
    TEST_ASSERT_EQUAL_UINT8(2, n);
    TEST_ASSERT_EQUAL_UINT8(0x20, out[0].i2c_address);
    TEST_ASSERT_EQUAL_UINT8(1, out[0].error_code);
    TEST_ASSERT_EQUAL_UINT8(0x48, out[1].i2c_address);
    TEST_ASSERT_EQUAL_UINT8(3, out[1].error_code);

    // Not drained: identity requests can repeat the same errors.
    TEST_ASSERT_EQUAL_UINT8(2, ModuleManager::getActiveErrors(out, ModuleManager::MAX_MODULES));
    // Capped at max_count.
    TEST_ASSERT_EQUAL_UINT8(1, ModuleManager::getActiveErrors(out, 1));
    ModuleManager::init();
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_drain_returns_injected_errors);
    RUN_TEST(test_drain_caps_at_max_count);
    RUN_TEST(test_runtime_errors_drain_with_address);
    RUN_TEST(test_active_errors_list_failed_modules);
    return UNITY_END();
}
