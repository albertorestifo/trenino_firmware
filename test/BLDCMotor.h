#pragma once

#include <stdint.h>

enum MotionControlType {
    Type_torque,
    Type_velocity,
    Type_angle
};

class BLDCMotor {
public:
    BLDCMotor(uint8_t pp, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t enable = 255)
        : pole_pairs_(pp), applied_torque_(0.0f), target_position_(0.0f) {}

    void init() {}
    void linkSensor(void* sensor) {}
    void initFOC() {}

    void loopFOC() {}
    void move(float target = 0.0f) { target_position_ = target; }

    void enable() {}
    void disable() {}

    // Mock control
    float getAppliedTorque() const { return applied_torque_; }
    float getTargetPosition() const { return target_position_; }
    void setTorque(float torque) { applied_torque_ = torque; }

    MotionControlType controller = Type_angle;
    float voltage_power_supply = 12.0f;
    int sensor_direction = 1;
    float shaft_angle = 0.0f;

private:
    uint8_t pole_pairs_;
    float applied_torque_;
    float target_position_;
};
