#include "stepper_driver.h"
// Six-state pattern derived from SwitecX25, Guy Carpenter/Clearwater Software,
// Copyright(c)2012. BSD license and disclaimer: docs/THIRD_PARTY_NOTICES.md.
#include <Arduino.h>
#include <Wire.h>
#include "i2c_bus.h"

namespace Stepper {
namespace {
bool reg(uint8_t address, uint8_t index, uint8_t value) {
    const uint8_t bytes[2] = {index, value};
    return I2CBus::write(address, bytes, sizeof(bytes));
}
}
bool ShieldDriver::shutdown(uint8_t address) {
    I2CBus::begin();
    // AI on, awake, ALLCALL and every SUBCALL disabled. Never address 0x70
    // as a broadcast. MODE2=OUTDRV, changes take effect at STOP.
    if (!reg(address, 0x00, 0x20) || !reg(address, 0x01, 0x04)) return false;
    return burst(address, 1, 0, false) && burst(address, 2, 0, false);
}
bool ShieldDriver::begin(uint8_t address) {
    if (ready_ && address == address_) return true;
    if (!shutdown(address)) { ready_ = false; return false; }
    address_ = address; ready_ = true; return true;
}
bool ShieldDriver::burst(uint8_t address, uint8_t port, uint8_t mask, bool enabled) {
    if (port != 1 && port != 2) return false;
    // Consecutive PCA channels: PWMA, AIN2, AIN1, BIN1, BIN2, PWMB.
    // Logical bits0..3 correspond to Switec physical contacts1..4 respectively.
    bool high[6] = {enabled, bool(mask & 2), bool(mask & 1), bool(mask & 4), bool(mask & 8), enabled};
    Wire.beginTransmission(address);
    Wire.write(uint8_t(0x06 + 4 * (port == 1 ? 8 : 2)));
    for (uint8_t i = 0; i < 6; ++i) {
        bool on = enabled && high[i];
        Wire.write(uint8_t(0)); Wire.write(uint8_t(on ? 0x10 : 0));
        Wire.write(uint8_t(0)); Wire.write(uint8_t(on ? 0 : 0x10));
    }
    return Wire.endTransmission() == 0;
}
uint8_t ShieldDriver::phaseCount(uint8_t mode) const {
    return mode == 1 ? 6 : (mode == 2 ? 4 : (mode == 3 ? 8 : 0));
}
bool ShieldDriver::write(uint8_t address, uint8_t port, uint8_t mode, uint8_t phase) {
    // SwitecX25's six contact states (Guy Carpenter/Clearwater, BSD-2).
    static const uint8_t states[6] = {0x9, 0x1, 0x7, 0x6, 0xe, 0x8};
    static const uint8_t full[4] = {0x5, 0x6, 0xa, 0x9};
    static const uint8_t half[8] = {0x1, 0x5, 0x4, 0x6, 0x2, 0xa, 0x8, 0x9};
    if (!ready_ || address != address_ || phase >= phaseCount(mode)) return false;
    bool ok = burst(address, port, mode == 1 ? states[phase] : (mode == 2 ? full[phase] : half[phase]), true);
    if (!ok) ready_ = false;
    return ok;
}
bool ShieldDriver::release(uint8_t address, uint8_t port) {
    bool ok = burst(address, port, 0, false); if (!ok) ready_ = false; return ok;
}
}
