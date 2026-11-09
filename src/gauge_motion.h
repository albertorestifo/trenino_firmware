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

    // Zero sequence, in electrical cycles of the motor (cycle_steps steps).
    static constexpr uint8_t BACKOFF_CYCLES = 2; // back off the stop after the fast sweep
    static constexpr uint8_t OVERDRIVE_CYCLES = 1; // slow approach past the backed-off point
    static constexpr uint16_t DWELL_TICKS = TICK_HZ / 20; // 50 ms holding the stop

    // speeds in steps/s, acceleration in steps/s^2. Calls sqrt, so never
    // from the interrupt. Leaves the gauge idle at position 0 and phase 0
    // with no zero pending. cycle_steps is one electrical cycle of the motor
    // (6 direct steps, 24 VID6606 microsteps).
    void configure(uint16_t travel, uint16_t max_speed, uint16_t acceleration,
        uint16_t zero_speed, uint16_t zero_steps, uint16_t settle_speed, uint8_t cycle_steps);

    // Find the end stop and call it position 0. Starts on the next tick and
    // interrupts any move:
    // 1. sweep zero_steps toward the stop at the zero speed (the needle may
    //    bounce off it here),
    // 2. back off BACKOFF_CYCLES at the settle speed,
    // 3. creep back into the stop at the settle speed, OVERDRIVE_CYCLES past
    //    the backed-off point plus however many steps it takes to land on
    //    phase 0,
    // 4. hold the stop for DWELL_TICKS so the needle settles.
    // Ending on the same electrical phase every time is what makes 0 land on
    // the same needle angle: against the stop, the rotor rests where the
    // energized phase pulls it, so a different phase is a different zero.
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
    // Electrical phase, 0..cycle_steps-1: the phase reached after configure()
    // is 0, and every step in gauge direction adds one.
    uint8_t phase() const { return phase_; }
    // Seeds the phase when the driver kept its own across a reconfigure (a
    // VID6606 whose RESET_N is not wired). Call before the first tick.
    void setPhase(uint8_t phase) { phase_ = phase < cycle_ ? phase : 0; }
    bool isZeroing() const { return state_ >= ZERO_FAST || zero_pending_; }
    bool isIdle() const { return state_ == IDLE && !zero_pending_ && position_ == target_; }

private:
    // Every zero state sorts after MOVING (see isZeroing()).
    enum State : uint8_t { IDLE, MOVING, ZERO_FAST, ZERO_BACKOFF, ZERO_SEEK, ZERO_DWELL };

    GAUGE_IRAM int8_t zeroLeg(int8_t dir);
    GAUGE_IRAM void step(int8_t dir);

    uint32_t v_max_; // Q32 steps/tick
    uint32_t v_min_; // Q32 steps/tick, speed after one step from rest
    uint32_t v_zero_; // Q32 steps/tick
    uint32_t v_settle_; // Q32 steps/tick
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
    uint8_t phase_;
    uint8_t cycle_;
    State state_;
};

// Converts steps/s to Q32 steps per tick (exposed for tests).
uint32_t speedToTick(uint32_t steps_per_second);

} // namespace Gauge
