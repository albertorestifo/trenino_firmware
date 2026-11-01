#include "i2c_bus.h"
#include <Arduino.h>
#include <Wire.h>

namespace I2CBus {

namespace {
bool g_initialized = false;
}

void begin()
{
    if (g_initialized) return;
    Wire.begin();
#ifdef ARDUINO_ARCH_AVR
    // Bound any single I2C transaction so a stuck bus (no pull-ups, shorted
    // SDA, missing slave) does not hang the firmware. 25 ms is comfortably
    // longer than any transaction at 100 kHz.
    Wire.setWireTimeout(25000, true);
#endif
    g_initialized = true;
}

#ifdef UNIT_TEST
void resetForTest() { g_initialized = false; }
#endif

}
