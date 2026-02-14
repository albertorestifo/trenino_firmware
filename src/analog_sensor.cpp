#include "analog_sensor.h"

namespace Sensors {

AnalogSensor::AnalogSensor(uint8_t pin_number, uint8_t sensitivity_level)
    : pin(pin_number)
    , sensitivity(sensitivity_level)
    , current_value(0)
    , last_sent(0)
    , scans_since_send(0)
    , min_send_interval(computeMinSendInterval())
{
}

void AnalogSensor::begin()
{
    // No pinMode needed for analog inputs - analogRead() handles pin configuration.
    // Calling pinMode(channel, INPUT) with a channel number (0, 1, etc.) would
    // incorrectly configure the wrong digital pin (e.g., RX/TX on Nano).

    // Reset state
    current_value = 0;
    last_sent = 0;
    scans_since_send = 0;
}

void AnalogSensor::scan()
{
    // Read raw analog value (0-1023)
    current_value = (uint16_t)analogRead(pin);

    // Increment scan counter
    scans_since_send++;
}

Reading AnalogSensor::getReading()
{
    // Check if we should send
    if (!shouldSend()) {
        return Reading(); // Not ready to send yet
    }

    // Send the current raw value
    int16_t value = (int16_t)current_value;

    // Update state
    last_sent = current_value;
    scans_since_send = 0;

    return Reading(value, InputType::Analog, pin);
}

uint16_t AnalogSensor::computeMinSendInterval() const
{
    // Sensitivity 0-10 maps to send interval
    // Higher sensitivity = lower interval = send more frequently
    // sensitivity 10: 1 scan (~10ms minimum)
    // sensitivity 5:  6 scans (~60ms minimum)
    // sensitivity 0:  11 scans (~110ms minimum)
    return (uint16_t)(11 - sensitivity);
}

bool AnalogSensor::shouldSend()
{
    // Simple algorithm:
    // 1. Force send every MAX_SEND_INTERVAL scans (~2 seconds) to ensure we don't go silent
    if (scans_since_send >= MAX_SEND_INTERVAL) {
        return true;
    }

    // 2. Rate limit: don't send faster than min_send_interval
    if (scans_since_send < min_send_interval) {
        return false;
    }

    // 3. Send if value changed beyond dead zone (filters analog noise/jitter)
    uint16_t delta = (current_value > last_sent) ? (current_value - last_sent) : (last_sent - current_value);
    return delta > DEAD_ZONE;
}

} // namespace Sensors
