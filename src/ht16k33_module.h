#pragma once

#include "module.h"
#include "protocol.h"
#include <stdint.h>

namespace Modules {

class HT16K33Module : public IModule {
public:
    static constexpr uint8_t REINIT_THRESHOLD = 3;
    static constexpr uint8_t MAX_DISPLAY_BYTES = 16;

    HT16K33Module(uint8_t i2c_address, uint8_t brightness, uint8_t num_digits);

    bool begin() override;
    ModuleType getType() const override { return ModuleType::HT16K33; }
    uint8_t getPin() const override { return 0; }
    uint8_t getI2CAddress() const override { return i2c_address_; }

    bool writeSegments(const uint8_t* data, uint8_t num_bytes);
    bool setBrightness(uint8_t level);

#ifdef UNIT_TEST
    bool needsReinit() const { return needs_reinit_; }
    uint8_t getCachedBrightness() const { return cached_brightness_; }
    uint8_t getFailureCount() const { return failure_count_; }
#endif

private:
    uint8_t i2c_address_;
    uint8_t cached_brightness_;
    uint8_t num_digits_;
    uint8_t cached_segments_[MAX_DISPLAY_BYTES];
    uint8_t cached_segment_bytes_;
    uint8_t failure_count_;
    bool needs_reinit_;

    bool runInitSequence();
    bool writeWithRetry(const uint8_t* data, uint8_t length);
};

} // namespace Modules

static_assert(Modules::HT16K33Module::MAX_DISPLAY_BYTES <= Protocol::MAX_SEGMENT_BYTES,
    "HT16K33 display RAM must fit within the WriteSegments protocol payload cap");
