#pragma once

#include <stdint.h>

namespace Sensor {

// Input types (matches protocol INPUT_TYPE_* constants)
enum class InputType : uint8_t {
    Analog = 0,
    Button = 1,
    Matrix = 2
};

// Sensor reading result
struct Reading {
    bool has_value; // True if sensor has a value to report
    int16_t value; // Normalized integer value
    InputType type; // Type of input
    uint8_t pin; // Pin number

    Reading()
        : has_value(false)
        , value(0)
        , type(InputType::Analog)
        , pin(0)
    {
    }

    Reading(int16_t val, InputType t, uint8_t p)
        : has_value(true)
        , value(val)
        , type(t)
        , pin(p)
    {
    }
};

// Base sensor interface
class ISensor {
public:
    virtual ~ISensor() { }

    // Initialize the sensor
    virtual void begin() = 0;

    // Scan the sensor (read current value, update running average)
    virtual void scan() = 0;

    // Check if sensor has a value to report (based on sensitivity)
    // If true, returns the reading and resets the reporting state
    virtual Reading getReading() = 0;

    // Get the input type
    virtual InputType getType() const = 0;

    // Get the pin number
    virtual uint8_t getPin() const = 0;
};

} // namespace Sensor
