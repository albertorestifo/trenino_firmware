#include "gauge_module.h"
#include "gauge_scheduler.h"

// Six-state contact pattern derived from SwitecX25, Guy Carpenter/Clearwater
// Software, Copyright (c) 2012. BSD license: docs/THIRD_PARTY_NOTICES.md.
// Bit n drives X27 contact n+1 (config pins[n]).
namespace {
GAUGE_DRAM const uint8_t SIX_STATE[6] = { 0x9, 0x1, 0x7, 0x6, 0xE, 0x8 };

// Microstep phase of VID6606 channels without RESET_N, kept from a deleted
// gauge for the next one on the same STEP pin (a Configure deletes every
// gauge, then creates the new ones). A full table overwrites its oldest entry,
// so the last MAX_GAUGES deleted gauges are always there.
struct KeptPhase {
    uint8_t step_pin;
    uint8_t phase;
};
KeptPhase g_kept[Protocol::MAX_GAUGES];
uint8_t g_kept_count = 0;
uint8_t g_kept_next = 0; // slot overwritten once the table is full

void keepPhase(uint8_t step_pin, uint8_t phase)
{
    for (uint8_t i = 0; i < g_kept_count; i++) {
        if (g_kept[i].step_pin == step_pin) {
            g_kept[i].phase = phase;
            return;
        }
    }
    uint8_t slot;
    if (g_kept_count < Protocol::MAX_GAUGES) {
        slot = g_kept_count++;
    } else {
        slot = g_kept_next;
        g_kept_next = g_kept_next + 1 >= Protocol::MAX_GAUGES ? 0 : g_kept_next + 1;
    }
    g_kept[slot].step_pin = step_pin;
    g_kept[slot].phase = phase;
}

// The kept phase for step_pin, or 0 (the chip's power-on phase).
uint8_t takePhase(uint8_t step_pin)
{
    for (uint8_t i = 0; i < g_kept_count; i++) {
        if (g_kept[i].step_pin != step_pin) continue;
        uint8_t phase = g_kept[i].phase;
        g_kept[i] = g_kept[--g_kept_count];
        if (g_kept_next >= g_kept_count) g_kept_next = 0;
        return phase;
    }
    return 0;
}
} // namespace

namespace Modules {

GaugeModule::GaugeModule(const Protocol::GaugeConfig& config)
    : config_(config)
    , phase_(0)
    , dir_written_(0)
    , step_high_(false)
    , registered_(false)
{
}

GaugeModule::~GaugeModule()
{
    if (registered_) {
        GaugeScheduler::remove(this);
        releaseOutputs();
        if (keepsChipPhase()) keepPhase(config_.pins[0], motion_.phase());
    }
}

bool GaugeModule::begin()
{
    if (registered_) return true;

    // Fail before touching any pin or claiming a RESET_N pin.
    if (!GaugeScheduler::hasRoom()) return false;
    for (uint8_t i = 0; i < 4; i++) {
        if (config_.pins[i] != Protocol::GAUGE_NO_PIN && !GaugeHal::isOutputPin(config_.pins[i])) return false;
    }

    uint16_t sweep = isVid6606() ? FULL_SWEEP_VID6606 : FULL_SWEEP_DIRECT;
    if (config_.travel > sweep) sweep = config_.travel;
    sweep += isVid6606() ? ZERO_MARGIN_VID6606 : ZERO_MARGIN_DIRECT;
    motion_.configure(config_.travel, config_.max_speed, config_.acceleration,
        isVid6606() ? ZERO_SPEED_VID6606 : ZERO_SPEED_DIRECT, sweep,
        isVid6606() ? SETTLE_SPEED_VID6606 : SETTLE_SPEED_DIRECT,
        isVid6606() ? CYCLE_VID6606 : CYCLE_DIRECT);
    if (keepsChipPhase()) motion_.setPhase(takePhase(config_.pins[0]));

    if (isVid6606()) {
        dir_written_ = motion_.direction();
        out_[0] = GaugeHal::makeOutput(config_.pins[0], false);
        out_[1] = GaugeHal::makeOutput(config_.pins[1], physical(dir_written_) > 0);
        uint8_t reset = config_.pins[2];
        if (reset != Protocol::GAUGE_NO_PIN && GaugeScheduler::claimReset(reset)) {
            // RESET_N low for over 1 ms, then high. The four motors on the
            // chip share it, so only the first gauge to claim it does this.
            GaugeHal::makeOutput(reset, false);
            GaugeHal::delayMs(RESET_LOW_MS);
            GaugeHal::makeOutput(reset, true);
        }
    } else {
        phase_ = 0;
        for (uint8_t i = 0; i < 4; i++) {
            out_[i] = GaugeHal::makeOutput(config_.pins[i], (SIX_STATE[0] >> i) & 1);
        }
    }

    motion_.zero();
    registered_ = GaugeScheduler::add(this);
    return registered_;
}

void GaugeModule::setTarget(uint16_t position)
{
    GaugeHal::Critical lock;
    motion_.setTarget(position);
}

void GaugeModule::zero()
{
    GaugeHal::Critical lock;
    motion_.zero();
}

bool GaugeModule::usesPin(uint8_t pin) const
{
    for (uint8_t i = 0; i < 4; i++) {
        if (config_.pins[i] != Protocol::GAUGE_NO_PIN && config_.pins[i] == pin) return true;
    }
    return false;
}

GAUGE_IRAM int8_t GaugeModule::physical(int8_t direction) const
{
    return (config_.flags & Protocol::GAUGE_FLAG_REVERSE) ? -direction : direction;
}

GAUGE_IRAM void GaugeModule::writeCoils()
{
    uint8_t state = SIX_STATE[phase_];
    for (uint8_t i = 0; i < 4; i++) {
        GaugeHal::write(out_[i], (state >> i) & 1);
    }
}

GAUGE_IRAM void GaugeModule::tick()
{
    // A STEP pulse spans one whole tick (100 us), far above the VID6606's
    // 450 ns minimum, and Motion never steps on two ticks in a row, so the low
    // time is at least one tick too.
    if (step_high_) {
        GaugeHal::write(out_[0], false);
        step_high_ = false;
    }

    int8_t step = motion_.tick();
    if (step != 0) {
        if (isVid6606()) {
            GaugeHal::write(out_[0], true);
            step_high_ = true;
        } else {
            if (physical(step) > 0) {
                phase_ = phase_ == 5 ? 0 : phase_ + 1;
            } else {
                phase_ = phase_ == 0 ? 5 : phase_ - 1;
            }
            writeCoils();
        }
    }

    // Motion changes direction only on a tick without a step, so DIR is
    // always set at least one tick before the next STEP edge.
    if (isVid6606() && motion_.direction() != dir_written_) {
        dir_written_ = motion_.direction();
        GaugeHal::write(out_[1], physical(dir_written_) > 0);
    }
}

void GaugeModule::releaseOutputs()
{
    // Called after remove(), so the interrupt no longer touches these pins.
    if (isVid6606()) {
        GaugeHal::Critical lock;
        GaugeHal::write(out_[0], false);
        step_high_ = false;
    } else {
        GaugeHal::Critical lock;
        for (uint8_t i = 0; i < 4; i++) {
            GaugeHal::write(out_[i], false);
        }
    }
}

} // namespace Modules
