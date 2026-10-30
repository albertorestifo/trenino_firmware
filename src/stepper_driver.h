#pragma once
#include "stepper.h"

namespace Stepper {
// PCA9685 + TB6612 Adafruit Motor Shield V2/V3 compatible electrical layout.
// No Adafruit blocking step()/global reset; no PWM microstepping implemented.
class ShieldDriver : public Driver {
public:
    ShieldDriver() : address_(0), ready_(false) {}
    bool begin(uint8_t address) override;
    uint8_t phaseCount(uint8_t mode) const override;
    bool write(uint8_t address, uint8_t port, uint8_t mode, uint8_t phase) override;
    bool release(uint8_t address, uint8_t port) override;
    bool shutdown(uint8_t address);
private:
    uint8_t address_;
    bool ready_;
    bool burst(uint8_t address, uint8_t port, uint8_t mask, bool enabled);
};
}
