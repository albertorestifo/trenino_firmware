#include "i2c_bus.h"
#include <Arduino.h>
#include <Wire.h>

namespace I2CBus {

namespace {
bool g_initialized = false;
uint8_t g_transactions = 0;
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

bool write(uint8_t address, const uint8_t* bytes, uint8_t count)
{
    g_transactions++;
    Wire.beginTransmission(address);
    Wire.write(bytes, count);
    return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t* out, uint8_t count)
{
    if (!write(address, &reg, 1)) return false;
    if (Wire.requestFrom(address, count) != count) return false;
    for (uint8_t i = 0; i < count; i++) {
        int v = Wire.read();
        if (v < 0) return false;
        out[i] = (uint8_t)v;
    }
    return true;
}

uint8_t transactionCount() { return g_transactions; }

#ifdef UNIT_TEST
void resetForTest() { g_initialized = false; }
#endif

}
