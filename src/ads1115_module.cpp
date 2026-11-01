#include "ads1115_module.h"
#include "i2c_bus.h"
#include "protocol.h"
#include <Arduino.h>

namespace Modules {

namespace {
constexpr uint8_t REG_CONVERSION = 0x00;
constexpr uint8_t REG_CONFIG = 0x01;
// OS start, single-ended, single-shot, 250 SPS, comparator off; the channel
// goes in bits 14:12 (with bit 14 already set for single-ended) and the gain in
// bits 11:9.
constexpr uint16_t CONFIG_BASE = 0xC1A3;
constexpr uint16_t OS_BIT = 0x8000;
// MUX, PGA and MODE: what a reset would change. OS and the rest are ignored.
constexpr uint16_t CONFIG_CHECK_MASK = 0x7F00;
}

Ads1115Module::Ads1115Module(uint8_t address, uint8_t pin_base, uint8_t channel_mask,
    uint8_t gain, uint8_t ref_channel, const uint8_t* sensitivity)
    : address_(address)
    , pin_base_(pin_base)
    , channel_mask_(channel_mask & 0x0F)
    , gain_(gain)
    , ref_channel_(ref_channel)
    , state_(State::IDLE)
    , channel_(CHANNELS - 1)
    , start_ms_(0)
    , retry_wait_(false)
    , ref_seeded_(false)
    , ref_lost_(false)
    , ref_(0)
{
    if (ref_channel_ < CHANNELS) channel_mask_ &= (uint8_t) ~(1u << ref_channel_);
    for (uint8_t i = 0; i < CHANNELS; i++) {
        sensitivity_[i] = sensitivity ? sensitivity[i] : 0;
        value_[i] = 0;
        last_sent_[i] = 0;
        conversions_[i] = 0;
        last_send_ms_[i] = 0;
        fresh_[i] = false;
    }
}

uint16_t Ads1115Module::scaled(int16_t raw, uint16_t ref)
{
    if (raw < 0) raw = 0;
    if (ref == 0) return (uint16_t)(raw >> 5);
    uint32_t v = (uint32_t)raw * 1023UL / ref;
    return (uint16_t)(v > 1023 ? 1023 : v);
}

uint8_t Ads1115Module::activeMask() const
{
    if (!hasReference()) return channel_mask_;
    // Reference only until it is known good; values cannot be derived without it.
    if (!referenceUsable()) return (uint8_t)(1u << ref_channel_);
    return (uint8_t)(channel_mask_ | (1u << ref_channel_));
}

uint16_t Ads1115Module::configWord(uint8_t channel) const
{
    return (uint16_t)(CONFIG_BASE | ((uint16_t)channel << 12) | ((uint16_t)gain_ << 9));
}

bool Ads1115Module::writeConfig(uint16_t word)
{
    const uint8_t bytes[3] = { REG_CONFIG, (uint8_t)(word >> 8), (uint8_t)(word & 0xFF) };
    return I2CBus::write(address_, bytes, sizeof(bytes));
}

bool Ads1115Module::readRegister(uint8_t reg, uint16_t& value)
{
    uint8_t bytes[2];
    if (!I2CBus::readRegisters(address_, reg, bytes, sizeof(bytes))) return false;
    value = (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
    return true;
}

bool Ads1115Module::begin()
{
    I2CBus::begin();

    state_ = State::IDLE;
    channel_ = CHANNELS - 1;
    retry_wait_ = false;
    ref_seeded_ = false;
    ref_lost_ = false;
    ref_ = 0;
    unsigned long now = millis();
    start_ms_ = now;
    for (uint8_t i = 0; i < CHANNELS; i++) {
        value_[i] = 0;
        last_sent_[i] = 0;
        conversions_[i] = 0;
        last_send_ms_[i] = now;
        fresh_[i] = false;
    }

    // Probe with the config register at an idle value (OS clear: no conversion
    // starts), so a missing chip NACKs. No runtime code 1 here: ModuleManager
    // reports a failed begin() as code 0, and recovery reports code 2.
    if (!writeConfig((uint16_t)(configWord(0) & ~OS_BIT))) {
        markFailed(now);
        return false;
    }
    return true;
}

void Ads1115Module::markFailed(unsigned long now)
{
    health_.markFailed(now);
    state_ = State::IDLE;
    clearFresh();
}

void Ads1115Module::noteFailure(unsigned long now)
{
    if (health_.noteFailure(now)) {
        health_.push(1);
        clearFresh();
    }
    state_ = State::IDLE;
    retry_wait_ = true;
    start_ms_ = now;
}

// Nothing converted before an outage is sent after it; per-channel state is kept.
void Ads1115Module::clearFresh()
{
    for (uint8_t i = 0; i < CHANNELS; i++) fresh_[i] = false;
}

// Same probe as begin(); per-channel state and the reference are kept.
void Ads1115Module::recover(unsigned long now)
{
    if (writeConfig((uint16_t)(configWord(0) & ~OS_BIT))) {
        health_.noteSuccess();
        health_.push(2);
        if (ref_lost_) health_.push(3); // code 2 clears the host's error; the reference is still gone
        state_ = State::IDLE;
        retry_wait_ = false;
        // Values and last-sent state are kept; the silent outage must not make
        // the first sample after it count as 100 ms overdue.
        for (uint8_t i = 0; i < CHANNELS; i++) last_send_ms_[i] = now;
    } else {
        health_.noteFailure(now); // restarts the retry timer
    }
}

uint8_t Ads1115Module::takeRuntimeError()
{
    return health_.take();
}

uint8_t Ads1115Module::activeError() const
{
    if (health_.failed()) return 1;
    if (ref_lost_) return 3;
    return MODULE_ERROR_NONE;
}

void Ads1115Module::scan()
{
    unsigned long now = millis();
    if (health_.failed()) {
        if (health_.retryDue(now)) recover(now);
        return;
    }
    if (state_ == State::IDLE) {
        startConversion(now);
    } else if ((now - start_ms_) >= CONVERSION_MS) {
        finishConversion(now);
    }
}

void Ads1115Module::startConversion(unsigned long now)
{
    // After a failure, space the retries out so a glitch is not counted three times in microseconds.
    if (retry_wait_ && (now - start_ms_) < CONVERSION_MS) return;
    uint8_t mask = activeMask();
    if (mask == 0) return;
    uint8_t next = channel_;
    do {
        next = (uint8_t)((next + 1) % CHANNELS);
    } while (!(mask & (1u << next)));

    if (!writeConfig(configWord(next))) {
        noteFailure(now);
        return;
    }
    channel_ = next;
    state_ = State::CONVERTING;
    start_ms_ = now;
    retry_wait_ = false;
}

void Ads1115Module::finishConversion(unsigned long now)
{
    // A reset chip has default MUX/PGA/MODE; a result read now would belong to
    // the wrong channel and gain.
    uint16_t cfg;
    if (!readRegister(REG_CONFIG, cfg)) {
        noteFailure(now);
        return;
    }
    if ((cfg & CONFIG_CHECK_MASK) != (configWord(channel_) & CONFIG_CHECK_MASK)) {
        // A mismatch is a failure: repeated resets (or another device answering
        // at this address) must reach the threshold, not loop through 1/2 pairs.
        if (health_.noteFailure(now)) {
            health_.push(1);
            clearFresh();
            state_ = State::IDLE;
            return;
        }
        handleReset(now);
        return;
    }
    uint16_t word;
    if (!readRegister(REG_CONVERSION, word)) {
        noteFailure(now);
        return;
    }
    health_.noteSuccess();
    state_ = State::IDLE;
    retry_wait_ = false;

    int16_t raw = (int16_t)word;
    if (hasReference() && channel_ == ref_channel_) {
        handleReference(raw);
    } else {
        handleSensor(channel_, raw);
    }
}

// Discards this conversion and starts it again on the same channel.
void Ads1115Module::handleReset(unsigned long now)
{
    health_.push(1);
    clearFresh();
    if (writeConfig(configWord(channel_))) {
        health_.push(2);
        if (ref_lost_) health_.push(3);
        start_ms_ = now;
    } else {
        markFailed(now); // code 1 already queued; recovers through the retry
    }
}

void Ads1115Module::handleReference(int16_t raw)
{
    if (raw < REF_MIN_RAW) {
        if (!ref_lost_) {
            ref_lost_ = true;
            ref_seeded_ = false; // the next good reading replaces it
            health_.push(3);
            clearFresh();
        }
        return;
    }
    if (ref_lost_) {
        ref_lost_ = false;
        health_.push(2);
    }
    if (ref_seeded_) {
        ref_ = (uint16_t)(((uint32_t)ref_ * 3 + (uint16_t)raw) / 4);
    } else {
        ref_ = (uint16_t)raw;
        ref_seeded_ = true;
    }
}

void Ads1115Module::handleSensor(uint8_t channel, int16_t raw)
{
    if (!referenceUsable()) return;
    value_[channel] = scaled(raw, hasReference() ? ref_ : 0);
    if (conversions_[channel] < 0xFFFF) conversions_[channel]++;
    fresh_[channel] = true;
}

Reading Ads1115Module::getReading()
{
    if (health_.failed() || !referenceUsable()) return Reading();
    unsigned long now = millis();
    for (uint8_t ch = 0; ch < CHANNELS; ch++) {
        if (!(channel_mask_ & (1u << ch)) || !fresh_[ch]) continue;
        uint8_t sens = sensitivity_[ch] > 10 ? 10 : sensitivity_[ch];
        uint16_t min_conversions = (uint16_t)(11 - sens);
        uint16_t delta = value_[ch] > last_sent_[ch] ? value_[ch] - last_sent_[ch] : last_sent_[ch] - value_[ch];
        bool changed = conversions_[ch] >= min_conversions && delta > DEAD_ZONE;
        bool stale = (now - last_send_ms_[ch]) >= MAX_SEND_INTERVAL_MS;
        if (!changed && !stale) continue;
        last_sent_[ch] = value_[ch];
        conversions_[ch] = 0;
        last_send_ms_[ch] = now;
        fresh_[ch] = false;
        return Reading((int16_t)value_[ch], ModuleType::ADS1115, (uint8_t)(pin_base_ + ch));
    }
    return Reading();
}

} // namespace Modules
