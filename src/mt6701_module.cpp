#include "mt6701_module.h"
#include <Arduino.h>
#include <Wire.h>

namespace Modules {

namespace {
bool g_mt6701_wire_initialized = false;
}

#ifdef UNIT_TEST
void mt6701_reset_wire_state() { g_mt6701_wire_initialized = false; }
#endif

MT6701Module::MT6701Module(uint8_t i2c_address, uint8_t report_pin, uint8_t sensitivity)
    : i2c_address_(i2c_address)
    , report_pin_(report_pin)
    , min_send_interval_((uint8_t)(11 - (sensitivity > 10 ? 10 : sensitivity)))
    , current_value_(0)
    , last_sent_(0)
    , scans_since_send_(0)
    , has_value_(false)
    , sent_once_(false)
{
}

bool MT6701Module::begin()
{
    if (!g_mt6701_wire_initialized) {
        // Wire.begin() again after another I2C module has called it is harmless.
        Wire.begin();
#ifdef ARDUINO_ARCH_AVR
        // A stuck or unplugged bus must not hang the firmware.
        Wire.setWireTimeout(25000, true);
#endif
        g_mt6701_wire_initialized = true;
    }

    current_value_ = 0;
    last_sent_ = 0;
    scans_since_send_ = 0;
    has_value_ = false;
    sent_once_ = false;

    uint16_t angle;
    if (!readAngle(angle)) {
        return false; // reported to the host as a ModuleError; scan() keeps trying
    }
    current_value_ = angle;
    has_value_ = true;
    return true;
}

bool MT6701Module::readAngle(uint16_t& angle)
{
    // The datasheet asks for 0x03 to be read before 0x04; a two-byte read
    // starting at 0x03 does that.
    Wire.beginTransmission(i2c_address_);
    Wire.write(REG_ANGLE_H);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom(i2c_address_, (uint8_t)2) != 2) {
        return false;
    }
    int high = Wire.read();
    int low = Wire.read();
    if (high < 0 || low < 0) {
        return false;
    }
    angle = decodeAngle((uint8_t)high, (uint8_t)low);
    return true;
}

void MT6701Module::scan()
{
    uint16_t angle;
    if (readAngle(angle)) {
        current_value_ = angle;
        has_value_ = true;
    }
    if (scans_since_send_ < 0xFFFF) {
        scans_since_send_++;
    }
}

bool MT6701Module::shouldSend() const
{
    if (!has_value_) return false;
    if (!sent_once_) return scans_since_send_ >= min_send_interval_;
    if (scans_since_send_ >= MAX_SEND_INTERVAL) return true;
    if (scans_since_send_ < min_send_interval_) return false;
    // Circular distance, so 16383 -> 0 counts as a step of 1.
    uint16_t delta = (current_value_ > last_sent_) ? (current_value_ - last_sent_) : (last_sent_ - current_value_);
    if (delta > (MAX_VALUE + 1) / 2) delta = (uint16_t)(MAX_VALUE + 1 - delta);
    return delta > DEAD_ZONE;
}

Reading MT6701Module::getReading()
{
    if (!shouldSend()) {
        return Reading();
    }
    last_sent_ = current_value_;
    scans_since_send_ = 0;
    sent_once_ = true;
    return Reading((int16_t)current_value_, ModuleType::MT6701, report_pin_);
}

} // namespace Modules
