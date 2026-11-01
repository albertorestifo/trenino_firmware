#include "mcp23017_module.h"
#include "i2c_bus.h"
#include <Arduino.h>
#include <Wire.h>

namespace Modules {

namespace {
constexpr uint8_t REG_IODIR = 0x00;
constexpr uint8_t REG_IOCON = 0x0A;
constexpr uint8_t REG_GPPU = 0x0C;
constexpr uint8_t REG_GPIO = 0x12;
constexpr uint8_t REG_OLAT = 0x14;
constexpr uint8_t SAMPLE_BYTES = 4; // GPIOA, GPIOB, OLATA, OLATB
}

Mcp23017Module::Mcp23017Module(uint8_t address, uint8_t direct_base, uint8_t matrix_base,
    uint8_t debounce, uint16_t direct_mask, uint16_t row_mask, uint16_t col_mask)
    : address_(address)
    , direct_base_(direct_base)
    , matrix_base_(matrix_base)
    , debounce_(debounce == 0 ? 1 : debounce)
    , direct_mask_(direct_mask & ~(row_mask | OUTPUT_ONLY_MASK))
    , row_mask_(row_mask)
    , col_mask_(col_mask)
    , sentinel_(0)
    , olat_expected_(0)
    , last_sample_ms_(0)
    , state_(0)
    , reported_(0)
{
    // Lowest bit that is not a row. For a direct-only chip this is bit 0.
    uint16_t bit = 1;
    while (bit != 0 && (row_mask_ & bit)) bit <<= 1;
    sentinel_ = bit;
    olat_expected_ = row_mask_ | sentinel_;
    olat_read_[0] = olat_read_[1] = 0;
    for (uint8_t i = 0; i < 16; i++) counts_[i] = 0;
}

bool Mcp23017Module::writeRegisters(uint8_t reg, uint16_t value)
{
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.write((uint8_t)(value & 0xFF));
    Wire.write((uint8_t)(value >> 8));
    return Wire.endTransmission() == 0;
}

bool Mcp23017Module::writeIocon()
{
    Wire.beginTransmission(address_);
    Wire.write(REG_IOCON);
    Wire.write((uint8_t)0);
    return Wire.endTransmission() == 0;
}

bool Mcp23017Module::begin()
{
    I2CBus::begin();

    state_ = 0;
    reported_ = 0;
    for (uint8_t i = 0; i < 16; i++) counts_[i] = 0;
    last_sample_ms_ = millis();

    // Order matters: outputs get their idle level before they become outputs.
    uint16_t inputs = (uint16_t) ~(row_mask_ | OUTPUT_ONLY_MASK);
    if (!writeIocon()) return false;
    if (!writeRegisters(REG_OLAT, olat_expected_)) return false;
    if (!writeRegisters(REG_GPPU, inputs)) return false;
    if (!writeRegisters(REG_IODIR, inputs)) return false;
    return true;
}

bool Mcp23017Module::sample(uint16_t& gpio, uint8_t olat[2])
{
    Wire.beginTransmission(address_);
    Wire.write(REG_GPIO);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom(address_, SAMPLE_BYTES) != SAMPLE_BYTES) return false;
    uint8_t b[SAMPLE_BYTES];
    for (uint8_t i = 0; i < SAMPLE_BYTES; i++) {
        int v = Wire.read();
        if (v < 0) return false;
        b[i] = (uint8_t)v;
    }
    gpio = (uint16_t)(b[0] | (b[1] << 8));
    olat[0] = b[2];
    olat[1] = b[3];
    return true;
}

void Mcp23017Module::scan()
{
    unsigned long now = millis();
    if ((now - last_sample_ms_) < SAMPLE_INTERVAL_MS) return;
    last_sample_ms_ = now;

    uint16_t gpio;
    uint8_t olat[2];
    if (!sample(gpio, olat)) {
        // A failed transaction changes no input state. Threshold handling
        // and recovery are added with the failure-handling step.
        health_.noteFailure(now);
        return;
    }
    health_.noteSuccess();
    // Place for the OLAT-vs-olat_expected_ check (reset detection).
    olat_read_[0] = olat[0];
    olat_read_[1] = olat[1];

    debounceDirect(~gpio); // pull-ups: pressed = low
}

void Mcp23017Module::debounceDirect(uint16_t pressed)
{
    for (uint8_t bit = 0; bit < 16; bit++) {
        uint16_t m = (uint16_t)1 << bit;
        if (!(direct_mask_ & m)) continue;
        bool raw = (pressed & m) != 0;
        bool cur = (state_ & m) != 0;
        if (raw == cur) {
            counts_[bit] = 0;
        } else if (++counts_[bit] >= debounce_) {
            counts_[bit] = 0;
            state_ ^= m;
        }
    }
}

Reading Mcp23017Module::getReading()
{
    uint16_t pending = state_ ^ reported_;
    if (!pending) return Reading();
    for (uint8_t bit = 0; bit < 16; bit++) {
        uint16_t m = (uint16_t)1 << bit;
        if (!(pending & m)) continue;
        reported_ ^= m;
        return Reading((state_ & m) ? 1 : 0, ModuleType::MCP23017, (uint8_t)(direct_base_ + bit));
    }
    return Reading();
}

} // namespace Modules
