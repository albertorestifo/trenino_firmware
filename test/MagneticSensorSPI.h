#pragma once

#include <stdint.h>

class MagneticSensorSPI {
public:
    MagneticSensorSPI(int cs, float bit_resolution, int spi_mode)
        : cs_pin_(cs), position_(0), angle_(0.0f), healthy_(true) {}

    void init() {}
    void update() { /* Mock: position/angle already set via setPosition() */ }

    float getAngle() { return angle_; }
    int32_t getFullRotations() { return position_ / 16384; }
    float getMechanicalAngle() { return angle_; }

    // Mock control methods
    void setPosition(int32_t pos) { position_ = pos; angle_ = (pos % 16384) / 16384.0f * 6.28318f; }
    void setHealthy(bool healthy) { healthy_ = healthy; }
    bool isHealthy() const { return healthy_; }

private:
    uint8_t cs_pin_;
    int32_t position_;
    float angle_;
    bool healthy_;
};
