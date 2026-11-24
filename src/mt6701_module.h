#pragma once

#include "module.h"
#include <stdint.h>

namespace Modules {

// MT6701 magnetic angle sensor read over I2C (module type 8).
//
// Reads the 14-bit angle (0-16383 for one turn) from registers 0x03/0x04
// and reports it like an analog input, under the host-chosen report pin.
// The MT6701 answers at 0x06 by default, or at 0x46 once that is saved in
// its EEPROM, so one I2C bus holds two sensors.
//
// A failed read keeps the last good value and sends nothing.
class MT6701Module : public IModule {
public:
    static constexpr uint8_t REG_ANGLE_H = 0x03; // Angle<13:6>
    static constexpr uint8_t REG_ANGLE_L = 0x04; // Angle<5:0> in bits 7:2
    static constexpr uint16_t MAX_VALUE = 16383;
    static constexpr uint16_t MAX_SEND_INTERVAL = 200; // force a send every ~2 s
    static constexpr uint16_t DEAD_ZONE = 3; // counts, about 0.07 degrees

    MT6701Module(uint8_t i2c_address, uint8_t report_pin, uint8_t sensitivity);

    bool begin() override;
    void scan() override;
    Reading getReading() override;
    ModuleType getType() const override { return ModuleType::MT6701; }
    uint8_t getPin() const override { return report_pin_; }
    uint8_t getI2CAddress() const override { return i2c_address_; }

#ifdef UNIT_TEST
    uint16_t getCurrentValue() const { return current_value_; }
    bool hasValue() const { return has_value_; }
#endif

    // 14-bit angle from the two register bytes.
    static uint16_t decodeAngle(uint8_t high, uint8_t low)
    {
        return (uint16_t)(((uint16_t)high << 6) | (low >> 2));
    }

private:
    uint8_t i2c_address_;
    uint8_t report_pin_;
    uint8_t min_send_interval_;
    uint16_t current_value_;
    uint16_t last_sent_;
    uint16_t scans_since_send_;
    bool has_value_;
    bool sent_once_;

    bool readAngle(uint16_t& angle);
    bool shouldSend() const;
};

} // namespace Modules
