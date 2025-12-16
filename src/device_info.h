#pragma once

#include <stdint.h>

// Device version (semantic versioning)
constexpr uint8_t DEVICE_VERSION_MAJOR = 2;
constexpr uint8_t DEVICE_VERSION_MINOR = 0;
constexpr uint8_t DEVICE_VERSION_PATCH = 0;

// EEPROM format version - increment when EEPROM layout changes
// Version 2: Added button and matrix input types with union-based storage
constexpr uint8_t EEPROM_FORMAT_VERSION = 2;
