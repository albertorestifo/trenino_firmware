#include "ht16k33_module.h"
#include <Arduino.h>
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

bool HT16K33Module::writeWithRetry(const uint8_t* data, uint8_t length)
{
    if (needs_reinit_) {
        if (!runInitSequence()) {
            return false;
        }
        if (cached_segment_bytes_ > 0) {
            Wire.beginTransmission(i2c_address_);
            Wire.write((uint8_t)0x00);
            for (uint8_t i = 0; i < cached_segment_bytes_; i++) {
                Wire.write(cached_segments_[i]);
            }
            if (Wire.endTransmission() != 0) {
                needs_reinit_ = true;
                return false;
            }
        }
        failure_count_ = 0;
        needs_reinit_ = false;
    }

    Wire.beginTransmission(i2c_address_);
    Wire.write(data, length);
    if (Wire.endTransmission() == 0) {
        failure_count_ = 0;
        return true;
    }

    delay(1);
    Wire.beginTransmission(i2c_address_);
    Wire.write(data, length);
    if (Wire.endTransmission() == 0) {
        failure_count_ = 0;
        return true;
    }

    failure_count_++;
    if (failure_count_ >= REINIT_THRESHOLD) {
        needs_reinit_ = true;
        failure_count_ = 0;
    }
    return false;
}

bool HT16K33Module::writeSegments(const uint8_t* data, uint8_t num_bytes)
{
    uint8_t cap = num_digits_ * 2;
    if (cap > MAX_DISPLAY_BYTES) cap = MAX_DISPLAY_BYTES;
    if (num_bytes > cap) num_bytes = cap;

    // Build payload (register 0x00 + segment bytes) before updating the cache,
    // so the cache restore in writeWithRetry uses the previously cached content.
    uint8_t payload[1 + MAX_DISPLAY_BYTES];
    payload[0] = 0x00;
    for (uint8_t i = 0; i < num_bytes; i++) {
        payload[1 + i] = data[i];
    }

    bool ok = writeWithRetry(payload, 1 + num_bytes);

    // Update cache after the write attempt so failed retries keep the last
    // attempted content and the reinit restore uses what was most recently sent.
    cached_segment_bytes_ = num_bytes;
    for (uint8_t i = 0; i < num_bytes; i++) {
        cached_segments_[i] = data[i];
    }

    return ok;
}

bool HT16K33Module::setBrightness(uint8_t level)
{
    if (level > 15) level = 15;
    cached_brightness_ = level;

    uint8_t cmd = 0xE0 | level;
    return writeWithRetry(&cmd, 1);
}

} // namespace Modules
