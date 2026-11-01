#pragma once

#include <stdint.h>

// Shared I2C bus setup for every I2C driver. Bus speed stays at the Wire
// default (100 kHz).
namespace I2CBus {

// Starts Wire and, on AVR, bounds any single transaction with a timeout.
// Safe to call repeatedly; only the first call touches the hardware.
void begin();

// One complete write transaction. Returns true if the chip acknowledged it.
bool write(uint8_t address, const uint8_t* bytes, uint8_t count);

// Sets the register pointer, then reads count bytes. Returns false on a NACK,
// a short read or a failed byte; out is only meaningful on success.
bool readRegisters(uint8_t address, uint8_t reg, uint8_t* out, uint8_t count);

#ifdef UNIT_TEST
void resetForTest();
#endif

}
