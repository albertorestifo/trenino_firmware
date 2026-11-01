#pragma once

#include "i2c_health.h"
#include "module.h"
#include <stdint.h>

namespace Modules {

// ADS1115 4-channel 16-bit ADC: single-ended, single-shot, 250 SPS. A state
// machine starts one conversion (IDLE -> CONVERTING) and reads it on the first
// scan() at least CONVERSION_MS later. scan() never blocks.
//
// An optional reference channel is wired to the analog supply. Values are
// scaled against its smoothed reading so they are 0-1023 on any supply. It is
// converted in the same rotation but never reported. While it is below
// REF_MIN_RAW (or not yet seen) only it is converted and no values are sent.
class Ads1115Module : public IModule {
public:
    static constexpr uint8_t CHANNELS = 4;
    static constexpr unsigned long CONVERSION_MS = 5;
    static constexpr int16_t REF_MIN_RAW = 4000;
    static constexpr uint16_t DEAD_ZONE = 2; // 10-bit units
    static constexpr unsigned long MAX_SEND_INTERVAL_MS = 100;
    static constexpr uint8_t MAX_NOT_DONE_RETRIES = 3; // re-reads while OS is clear

    Ads1115Module(uint8_t address, uint8_t pin_base, uint8_t channel_mask, uint8_t gain,
        uint8_t ref_channel, const uint8_t* sensitivity);

    // 10-bit value for a raw reading; ref == 0 means no reference.
    static uint16_t scaled(int16_t raw, uint16_t ref);

    bool begin() override;
    void scan() override;
    Reading getReading() override;
    ModuleType getType() const override { return ModuleType::ADS1115; }
    uint8_t getPin() const override { return 0; }
    uint8_t getI2CAddress() const override { return address_; }
    uint8_t takeRuntimeError() override;
    uint8_t activeError() const override;

#ifdef UNIT_TEST
    const I2CHealth& health() const { return health_; }
#endif

private:
    enum class State : uint8_t { IDLE, CONVERTING };

    bool hasReference() const { return ref_channel_ < CHANNELS; }
    bool referenceUsable() const { return !hasReference() || (ref_seeded_ && !ref_lost_); }
    uint8_t activeMask() const;
    uint16_t configWord(uint8_t channel) const;
    bool writeConfig(uint16_t word);
    bool readRegister(uint8_t reg, uint16_t& value);
    void markFailed(unsigned long now);
    void noteFailure(unsigned long now);
    void recover(unsigned long now);
    void startConversion(unsigned long now);
    void finishConversion(unsigned long now);
    void handleReset(unsigned long now);
    void handleReference(int16_t raw);
    void handleSensor(uint8_t channel, int16_t raw);
    void clearFresh();

    uint8_t address_;
    uint8_t pin_base_;
    uint8_t channel_mask_; // reported channels (reference excluded)
    uint8_t gain_;
    uint8_t ref_channel_; // 0-3, or Protocol::ADS_NO_REFERENCE
    uint8_t sensitivity_[CHANNELS];

    I2CHealth health_;
    State state_;
    uint8_t channel_; // channel of the latest conversion; the next one follows it
    unsigned long start_ms_; // conversion start, or time of the last failure
    bool retry_wait_; // after a failed transaction, wait before starting again
    uint8_t not_done_reads_; // re-reads of this conversion while OS was clear

    bool ref_seeded_;
    bool ref_lost_;
    uint16_t ref_;

    uint16_t value_[CHANNELS];
    uint16_t last_sent_[CHANNELS];
    uint16_t conversions_[CHANNELS]; // since the last send
    unsigned long last_send_ms_[CHANNELS];
    bool fresh_[CHANNELS]; // converted since the last send
};

} // namespace Modules
