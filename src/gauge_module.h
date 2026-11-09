#pragma once

#include "gauge_hal.h"
#include "gauge_motion.h"
#include "module.h"
#include "protocol.h"

namespace Modules {

// X27.168 gauge stepper, driven by a VID6606 channel (STEP/DIR) or straight
// from four MCU pins with the Switec six-state pattern. Output-only: it never
// reports a reading. The shared timer interrupt calls tick().
class GaugeModule : public IModule {
public:
    // 315 degree sweep into the end stop, in each driver's steps.
    static constexpr uint16_t FULL_SWEEP_VID6606 = 3780; // 1/12 degree per pulse
    static constexpr uint16_t FULL_SWEEP_DIRECT = 945; // 1/3 degree per step
    // Extra fast-sweep travel, 10 degrees, in case the needle lost steps
    // past where the firmware thinks it is.
    static constexpr uint16_t ZERO_MARGIN_VID6606 = 120;
    static constexpr uint16_t ZERO_MARGIN_DIRECT = 30;
    // Unramped zero sweep at 200 degrees/s, the X27 start-stop limit.
    static constexpr uint16_t ZERO_SPEED_VID6606 = 2400;
    static constexpr uint16_t ZERO_SPEED_DIRECT = 600;
    // Back-off and slow approach into the stop at 20 degrees/s, slow enough
    // that the needle does not bounce off it.
    static constexpr uint16_t SETTLE_SPEED_VID6606 = 240;
    static constexpr uint16_t SETTLE_SPEED_DIRECT = 60;
    // One electrical cycle (2 degrees): 24 VID6606 microsteps, 6 direct steps.
    static constexpr uint8_t CYCLE_VID6606 = 24;
    static constexpr uint8_t CYCLE_DIRECT = 6;
    // RESET_N low time when releasing a VID6606 (datasheet: at least 1 ms).
    static constexpr uint16_t RESET_LOW_MS = 2;

    explicit GaugeModule(const Protocol::GaugeConfig& config);
    ~GaugeModule() override;

    // Sets up the pins, releases the VID6606 reset, registers with the
    // scheduler and starts the zero sweep.
    bool begin() override;

    ModuleType getType() const override { return ModuleType::Gauge; }
    uint8_t getPin() const override { return config_.pins[0]; }

    // Called from the main loop.
    void setTarget(uint16_t position);
    void zero();
    bool usesPin(uint8_t pin) const;

    // Called from the timer interrupt only.
    GAUGE_IRAM void tick();

    const Gauge::Motion& motion() const { return motion_; }
    const Protocol::GaugeConfig& config() const { return config_; }

private:
    GaugeModule(const GaugeModule&);
    GaugeModule& operator=(const GaugeModule&);

    bool isVid6606() const { return config_.driver == Protocol::GAUGE_DRIVER_VID6606; }
    // A VID6606 without RESET_N keeps its microstep phase when the module is
    // recreated, so the firmware carries it over by STEP pin. The phase is
    // counted in gauge direction: flipping the reverse flag between two
    // Configures shifts the first zero after it (the host deletes the dial
    // calibration on a direction change anyway).
    bool keepsChipPhase() const { return isVid6606() && config_.pins[2] == Protocol::GAUGE_NO_PIN; }
    GAUGE_IRAM int8_t physical(int8_t direction) const;
    GAUGE_IRAM void writeCoils();
    void releaseOutputs();

    Protocol::GaugeConfig config_;
    Gauge::Motion motion_;
    GaugeHal::FastPin out_[4];
    uint8_t phase_; // direct drive: index into the six-state table
    int8_t dir_written_; // VID6606: motion direction last written to DIR
    bool step_high_; // VID6606: STEP raised on the previous tick
    bool registered_;
};

} // namespace Modules
