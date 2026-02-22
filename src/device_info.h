#pragma once

#include <stdint.h>

// Device version (semantic versioning)
constexpr uint8_t DEVICE_VERSION_MAJOR = 2;
constexpr uint8_t DEVICE_VERSION_MINOR = 2;
constexpr uint8_t DEVICE_VERSION_PATCH = 1;

// EEPROM format version - increment when EEPROM layout changes
// Version 4: BLDC lever simplified to single motor_enable pin
constexpr uint8_t EEPROM_FORMAT_VERSION = 4;
