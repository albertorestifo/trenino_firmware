#pragma once

// Shared I2C bus setup for every I2C driver. Bus speed stays at the Wire
// default (100 kHz).
namespace I2CBus {

// Starts Wire and, on AVR, bounds any single transaction with a timeout.
// Safe to call repeatedly; only the first call touches the hardware.
void begin();

#ifdef UNIT_TEST
void resetForTest();
#endif

}
