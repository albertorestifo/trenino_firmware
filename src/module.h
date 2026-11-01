#pragma once

#include <stdint.h>

namespace Modules {

// Module types (matches protocol MODULE_TYPE_* constants)
enum class ModuleType : uint8_t {
    Analog = 0,
    Button = 1,
    Matrix = 2,
    // 3 = reserved (formerly BLDCLever)
    HT16K33 = 4,
    MCP23017 = 5,
    ADS1115 = 6
};

// Returned by takeRuntimeError() / activeError() when there is no error.
// Error codes: 0 init failed, 1 communication lost, 2 recovered,
// 3 supply reference lost.
constexpr uint8_t MODULE_ERROR_NONE = 0xFF;

// Module reading result
struct Reading {
    bool has_value; // True if module has a value to report
    int16_t value; // Normalized integer value
    ModuleType type; // Type of module
    uint8_t pin; // Pin number

    Reading()
        : has_value(false)
        , value(0)
        , type(ModuleType::Analog)
        , pin(0)
    {
    }

    Reading(int16_t val, ModuleType t, uint8_t p)
        : has_value(true)
        , value(val)
        , type(t)
        , pin(p)
    {
    }
};

// Base module interface
class IModule {
public:
    virtual ~IModule() { }

    // Initialize the module. Returns true on success, false if init failed.
    // Modules that cannot fail at init (analog, button, matrix) return true
    // unconditionally. I2C modules return false if the chip NACKed.
    virtual bool begin() = 0;

    // Scan the module (read current value, update state).
    // Default: no-op for output-only modules.
    virtual void scan() { }

    // Delays the first sample by ms after begin(), so that chips created together
    // do not share a bus phase. Default: no-op for modules with no timed sampling.
    virtual void setFirstSampleOffset(uint8_t /*ms*/) { }

    // Check if module has a value to report. Default: never has a value
    // (suitable for output-only modules).
    virtual Reading getReading() { return Reading(); }

    // Get the module type
    virtual ModuleType getType() const = 0;

    // Get the pin number (0 for I2C-only modules)
    virtual uint8_t getPin() const = 0;

    // Get the I2C address (0 for hardware-pin modules)
    virtual uint8_t getI2CAddress() const { return 0; }

    // Pops the oldest pending runtime error code, or MODULE_ERROR_NONE.
    virtual uint8_t takeRuntimeError() { return MODULE_ERROR_NONE; }

    // Error the module is in right now (repeated on IdentityRequest), or
    // MODULE_ERROR_NONE.
    virtual uint8_t activeError() const { return MODULE_ERROR_NONE; }
};

} // namespace Modules
