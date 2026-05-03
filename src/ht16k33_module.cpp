#include "ht16k33_module.h"
#include <Wire.h>

namespace Modules {

namespace {
bool g_wire_initialized = false;
}

#ifdef UNIT_TEST
void ht16k33_reset_wire_state() { g_wire_initialized = false; }
#endif

HT16K33Module::HT16K33Module(uint8_t i2c_address, uint8_t brightness, uint8_t num_digits)
    : i2c_address_(i2c_address)
    , cached_brightness_(brightness > 15 ? 15 : brightness)
    , num_digits_(num_digits)
    , cached_segment_bytes_(0)
    , failure_count_(0)
    , needs_reinit_(false)
{
    for (uint8_t i = 0; i < MAX_DISPLAY_BYTES; i++) {
        cached_segments_[i] = 0;
    }
}

bool HT16K33Module::runInitSequence()
{
    Wire.beginTransmission(i2c_address_);
    Wire.write((uint8_t)0x21);
    if (Wire.endTransmission() != 0) return false;

    Wire.beginTransmission(i2c_address_);
    Wire.write((uint8_t)0x81);
    if (Wire.endTransmission() != 0) return false;

    Wire.beginTransmission(i2c_address_);
    Wire.write((uint8_t)(0xE0 | (cached_brightness_ & 0x0F)));
    if (Wire.endTransmission() != 0) return false;

    Wire.beginTransmission(i2c_address_);
    Wire.write((uint8_t)0x00);
    for (uint8_t i = 0; i < 16; i++) {
        Wire.write((uint8_t)0x00);
    }
    if (Wire.endTransmission() != 0) return false;

    return true;
}

bool HT16K33Module::begin()
{
    if (!g_wire_initialized) {
        Wire.begin();
        g_wire_initialized = true;
    }

    if (runInitSequence()) {
        failure_count_ = 0;
        needs_reinit_ = false;
        return true;
    }
    needs_reinit_ = true;
    return false;
}

bool HT16K33Module::writeWithRetry(const uint8_t* /*data*/, uint8_t /*length*/)
{
    return false; // implemented in D4
}

bool HT16K33Module::writeSegments(const uint8_t* /*data*/, uint8_t /*num_bytes*/)
{
    return false; // implemented in D4
}

bool HT16K33Module::setBrightness(uint8_t /*level*/)
{
    return false; // implemented in D4
}

} // namespace Modules
