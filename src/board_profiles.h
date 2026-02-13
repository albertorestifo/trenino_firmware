#pragma once

#include <stdint.h>

namespace BoardProfiles {

// Board profile identifiers
enum Profile : uint8_t {
    SIMPLEFOC_SHIELD_V2_MEGA = 0
};

// Pin configuration for SimpleFOCShield v2 on Arduino Mega 2560
// Configuration: Custom solder pad setup for AS5047D encoder on SPI
// - PWM pads: A=5, B=6, C=9
// - Enable pads: EA=7, EB=8
// - Encoder: CS=10, using hardware SPI (MOSI=51, MISO=50, SCK=52)
// Reference: https://docs.simplefoc.com/pads_soldering_v2
struct SimpleFOCShieldV2Mega {
    // Motor driver pins (3-phase BLDC)
    static constexpr uint8_t MOTOR_PIN_A = 5;
    static constexpr uint8_t MOTOR_PIN_B = 6;
    static constexpr uint8_t MOTOR_PIN_C = 9;

    // Motor enable pins
    static constexpr uint8_t MOTOR_ENABLE_A = 7;
    static constexpr uint8_t MOTOR_ENABLE_B = 8;

    // Encoder pins (AS5047D on SPI)
    static constexpr uint8_t ENCODER_CS = 10;
    // Hardware SPI pins on Mega: MOSI=51, MISO=50, SCK=52
};

// Get motor pins for a given profile
inline bool getMotorPins(uint8_t profile, uint8_t& pin_a, uint8_t& pin_b, uint8_t& pin_c) {
    switch (profile) {
    case SIMPLEFOC_SHIELD_V2_MEGA:
        pin_a = SimpleFOCShieldV2Mega::MOTOR_PIN_A;
        pin_b = SimpleFOCShieldV2Mega::MOTOR_PIN_B;
        pin_c = SimpleFOCShieldV2Mega::MOTOR_PIN_C;
        return true;
    default:
        return false;
    }
}

// Get enable pins for a given profile
inline bool getEnablePins(uint8_t profile, uint8_t& enable_a, uint8_t& enable_b) {
    switch (profile) {
    case SIMPLEFOC_SHIELD_V2_MEGA:
        enable_a = SimpleFOCShieldV2Mega::MOTOR_ENABLE_A;
        enable_b = SimpleFOCShieldV2Mega::MOTOR_ENABLE_B;
        return true;
    default:
        return false;
    }
}

// Get encoder CS pin for a given profile
inline bool getEncoderCS(uint8_t profile, uint8_t& cs_pin) {
    switch (profile) {
    case SIMPLEFOC_SHIELD_V2_MEGA:
        cs_pin = SimpleFOCShieldV2Mega::ENCODER_CS;
        return true;
    default:
        return false;
    }
}

} // namespace BoardProfiles
