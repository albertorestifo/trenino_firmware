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
    , state_(IDLE)
{
}

void Motion::configure(uint16_t travel, uint16_t max_speed, uint16_t acceleration,
    uint16_t zero_speed, uint16_t zero_steps)
{
    travel_ = travel;
    zero_steps_ = zero_steps;
    v_max_ = speedToTick(max_speed);
    v_zero_ = speedToTick(zero_speed < max_speed ? zero_speed : max_speed);
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
        state_ = ZEROING;
        zero_remaining_ = zero_steps_;
        v_ = v_zero_;
        acc_ = 0;
        ramp_steps_ = 0;
        dir_ = -1;
        return 0; // DIR settles for one tick
    }

    if (state_ == ZEROING) {
        uint32_t before = acc_;
        acc_ += v_;
        if (acc_ >= before) return 0; // no carry, no step
        position_--;
        if (--zero_remaining_ == 0) {
            position_ = 0;
            v_ = 0;
            acc_ = 0;
            state_ = IDLE;
        }
        return -1;
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

    position_ += dir_;
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

} // namespace Gauge
