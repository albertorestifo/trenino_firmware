#include "gauge_motion.h"
#include <math.h>

namespace Gauge {

uint32_t speedToTick(uint32_t steps_per_second)
{
    return (uint32_t)(((uint64_t)steps_per_second << 32) / TICK_HZ);
}

Motion::Motion()
    : v_max_(0)
    , v_min_(0)
    , v_zero_(0)
    , v_settle_(0)
    , a_tick_(0)
    , v_(0)
    , acc_(0)
    , position_(0)
    , travel_(0)
    , zero_steps_(0)
    , target_(0)
    , ramp_steps_(0)
    , zero_remaining_(0)
    , zero_pending_(false)
    , dir_(-1)
    , phase_(0)
    , cycle_(1)
    , state_(IDLE)
{
}

void Motion::configure(uint16_t travel, uint16_t max_speed, uint16_t acceleration,
    uint16_t zero_speed, uint16_t zero_steps, uint16_t settle_speed, uint8_t cycle_steps)
{
    travel_ = travel;
    zero_steps_ = zero_steps;
    cycle_ = cycle_steps > 0 ? cycle_steps : 1;
    v_max_ = speedToTick(max_speed);
    v_zero_ = speedToTick(zero_speed < max_speed ? zero_speed : max_speed);
    v_settle_ = speedToTick(settle_speed < max_speed ? settle_speed : max_speed);
    if (v_zero_ == 0) v_zero_ = 1;
    if (v_settle_ == 0) v_settle_ = 1;
    a_tick_ = (uint32_t)(((uint64_t)acceleration << 32) / ((uint64_t)TICK_HZ * TICK_HZ));
    if (a_tick_ == 0) a_tick_ = 1;

    // Speed reached after one step from rest under constant acceleration.
    uint32_t start = (uint32_t)sqrt(2.0 * acceleration);
    v_min_ = speedToTick(start);
    if (v_min_ == 0) v_min_ = 1;
    if (v_min_ > v_max_) v_min_ = v_max_;

    v_ = 0;
    acc_ = 0;
    position_ = 0;
    target_ = 0;
    ramp_steps_ = 0;
    zero_remaining_ = 0;
    zero_pending_ = false;
    dir_ = -1;
    phase_ = 0;
    state_ = IDLE;
}

void Motion::zero()
{
    zero_pending_ = true;
}

void Motion::setTarget(uint16_t target)
{
    target_ = target > travel_ ? travel_ : target;
}

GAUGE_IRAM int8_t Motion::tick()
{
    if (zero_pending_) {
        zero_pending_ = false;
        state_ = ZERO_FAST;
        zero_remaining_ = zero_steps_;
        v_ = v_zero_;
        acc_ = 0;
        ramp_steps_ = 0;
        if (dir_ != -1) {
            dir_ = -1;
            return 0; // DIR settles for one tick
        }
    }

    // An if-chain, not a switch: a jump table would live in flash, which the
    // ESP32 interrupt must not read.
    if (state_ == ZERO_FAST) {
        int8_t s = zeroLeg(-1);
        if (s != 0 && zero_remaining_ == 0) {
            state_ = ZERO_BACKOFF;
            zero_remaining_ = (uint16_t)BACKOFF_CYCLES * cycle_;
            v_ = v_settle_;
            acc_ = 0;
        }
        return s;
    }
    if (state_ == ZERO_BACKOFF) {
        int8_t s = zeroLeg(1);
        if (s != 0 && zero_remaining_ == 0) {
            // Whole cycles plus the current phase: the last step lands on phase 0.
            state_ = ZERO_SEEK;
            zero_remaining_ = (uint16_t)(BACKOFF_CYCLES + OVERDRIVE_CYCLES) * cycle_ + phase_;
            acc_ = 0;
        }
        return s;
    }
    if (state_ == ZERO_SEEK) {
        int8_t s = zeroLeg(-1);
        if (s != 0 && zero_remaining_ == 0) {
            state_ = ZERO_DWELL;
            zero_remaining_ = DWELL_TICKS;
        }
        return s;
    }
    if (state_ == ZERO_DWELL) {
        if (--zero_remaining_ == 0) {
            position_ = 0;
            v_ = 0;
            acc_ = 0;
            state_ = IDLE;
        }
        return 0;
    }

    int32_t target = target_;

    if (state_ == IDLE) {
        if (target == position_) return 0;
        dir_ = target > position_ ? 1 : -1;
        v_ = v_min_;
        acc_ = 0;
        ramp_steps_ = 0;
        state_ = MOVING;
        return 0; // DIR settles for one tick
    }

    // MOVING: brake once the remaining distance is no longer than the
    // distance it took to reach this speed. A target behind the needle
    // (remaining <= 0) also brakes.
    int32_t remaining = dir_ > 0 ? target - position_ : position_ - target; // no multiply on AVR
    bool braking = remaining <= (int32_t)ramp_steps_;
    bool accelerating = false;
    if (braking) {
        v_ = (v_ - v_min_ > a_tick_) ? v_ - a_tick_ : v_min_;
    } else if (v_ < v_max_) {
        v_ = (v_max_ - v_ > a_tick_) ? v_ + a_tick_ : v_max_;
        accelerating = true;
    }

    uint32_t before = acc_;
    acc_ += v_;
    if (acc_ >= before) return 0; // no carry, no step

    step(dir_);
    if (braking) {
        if (ramp_steps_ > 0) ramp_steps_--;
    } else if (accelerating) {
        ramp_steps_++;
    }

    // Stop once the ramp is spent and the needle is at or past the target.
    // Past it (a closer target arrived too late to brake) the next ticks
    // start a move back.
    if (ramp_steps_ <= 1 && remaining - 1 <= 0) {
        ramp_steps_ = 0;
        v_ = 0;
        acc_ = 0;
        state_ = IDLE;
    }
    return dir_;
}

// One tick of a constant-speed zero leg toward dir, at v_. A turn spends one
// tick without a step so DIR settles. Counts zero_remaining_ down per step.
GAUGE_IRAM int8_t Motion::zeroLeg(int8_t dir)
{
    if (dir_ != dir) {
        dir_ = dir;
        acc_ = 0;
        return 0;
    }
    uint32_t before = acc_;
    acc_ += v_;
    if (acc_ >= before) return 0; // no carry, no step
    step(dir);
    zero_remaining_--;
    return dir;
}

GAUGE_IRAM void Motion::step(int8_t dir)
{
    position_ += dir;
    if (dir > 0) {
        phase_ = phase_ + 1 >= cycle_ ? 0 : phase_ + 1;
    } else {
        phase_ = phase_ == 0 ? cycle_ - 1 : phase_ - 1;
    }
}

} // namespace Gauge
