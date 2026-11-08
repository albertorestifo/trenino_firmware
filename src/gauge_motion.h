#pragma once

#include "gauge_hal.h"
#include <stdint.h>

namespace Gauge {

// Rate of the shared gauge timer interrupt.
constexpr uint32_t TICK_HZ = 10000;

// Open-loop position, ramp and zero sweep of one gauge, in that gauge's steps.
// tick() runs inside the timer interrupt: integer adds and compares only.
// Velocity is in Q32 steps per tick, so a carry out of the 32-bit accumulator
// is one step. Every speed is below 0.5 steps per tick, so there is at most one
// step per tick and never two in consecutive ticks.
//
// Callers outside the interrupt must hold GaugeHal::Critical around zero()
// and setTarget().
class Motion {
public:
    Motion();

    // speeds in steps/s, acceleration in steps/s^2. Calls sqrt, so never
    // from the interrupt. Leaves the gauge idle at position 0 with no zero
    // pending.
    void configure(uint16_t travel, uint16_t max_speed, uint16_t acceleration,
        uint16_t zero_speed, uint16_t zero_steps);

    // Sweep zero_steps toward the end stop at the fixed zero speed, then call
    // that position 0. Starts on the next tick and interrupts any move.
    void zero();

    // Latest target wins. Clamped to travel. Applied after a zero sweep.
    void setTarget(uint16_t target);

    // One timer tick. Returns -1, 0 or +1: the step taken, in gauge direction
    // (+1 away from the end stop).
    GAUGE_IRAM int8_t tick();

    // Direction of the current or next step: -1 or +1. Changes only on a tick
    // that returns 0, so a driver can set its DIR pin a full tick before the
    // next step.
    int8_t direction() const { return dir_; }

    int32_t position() const { return position_; }
    uint16_t target() const { return target_; }
    bool isZeroing() const { return state_ == ZEROING || zero_pending_; }
    bool isIdle() const { return state_ == IDLE && !zero_pending_ && position_ == target_; }

private:
    enum State : uint8_t { IDLE, MOVING, ZEROING };

    uint32_t v_max_; // Q32 steps/tick
    uint32_t v_min_; // Q32 steps/tick, speed after one step from rest
    uint32_t v_zero_; // Q32 steps/tick
    uint32_t a_tick_; // Q32 steps/tick per tick
    uint32_t v_; // current speed, Q32 steps/tick
    uint32_t acc_; // step accumulator
    int32_t position_;
    uint16_t travel_;
    uint16_t zero_steps_;
    volatile uint16_t target_;
    uint16_t ramp_steps_; // steps taken while accelerating, not yet braked off
    uint16_t zero_remaining_;
    volatile bool zero_pending_;
    int8_t dir_;
    State state_;
};

// Converts steps/s to Q32 steps per tick (exposed for tests).
uint32_t speedToTick(uint32_t steps_per_second);

} // namespace Gauge
