#include "button_sensor.h"

namespace Sensor {

ButtonSensor::ButtonSensor(uint8_t pin_number, uint8_t debounce_scans)
    : pin(pin_number)
    , debounce_threshold(debounce_scans)
    , current_state(false)
    , last_reported(false)
    , raw_state(false)
    , debounce_count(0)
    , has_pending_event(false)
{
}

void ButtonSensor::begin()
{
    // Configure pin as input with pullup
    // Button connects pin to GND when pressed (active LOW)
    pinMode(pin, INPUT_PULLUP);

    // Reset state
    current_state = false;
    last_reported = false;
    raw_state = false;
    debounce_count = 0;
    has_pending_event = false;
}

void ButtonSensor::scan()
{
    // Read raw state (LOW = pressed due to INPUT_PULLUP)
    bool new_raw = (digitalRead(pin) == LOW);

    // Counter-based debounce algorithm:
    // Only change state after seeing consistent readings for debounce_threshold scans
    if (new_raw == current_state) {
        // Reading matches current state, reset counter
        debounce_count = 0;
    } else {
        // Reading differs from current state
        debounce_count++;

        if (debounce_count >= debounce_threshold) {
            // Stable new state detected
            current_state = new_raw;
            debounce_count = 0;

            // Check if this is a new edge event
            if (current_state != last_reported) {
                has_pending_event = true;
            }
        }
    }

    raw_state = new_raw;
}

Reading ButtonSensor::getReading()
{
    if (!has_pending_event) {
        return Reading(); // No event to report
    }

    // Report the edge event
    // value = 1 for press, 0 for release
    int16_t value = current_state ? 1 : 0;

    // Update state
    last_reported = current_state;
    has_pending_event = false;

    return Reading(value, InputType::Button, pin);
}

} // namespace Sensor
