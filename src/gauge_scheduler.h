#pragma once

#include "gauge_hal.h"
#include <stdint.h>

namespace Modules {
class GaugeModule;
}

// Owns the shared gauge tick interrupt. The timer runs only while at least
// one gauge is registered, so boards without gauges are unaffected.
namespace GaugeScheduler {

// Called from GaugeModule::begin() and its destructor, outside the interrupt.
// add() returns false when Protocol::MAX_GAUGES are already registered.
bool add(Modules::GaugeModule* gauge);
void remove(Modules::GaugeModule* gauge);

// The timer interrupt body: ticks every registered gauge.
GAUGE_IRAM void tick();

// Pause and restart the tick around flash writes (ESP32 commit, Due IAP).
// Nests. Needles hold still while suspended.
void suspend();
void resume();

// True if a registered gauge drives this pin (SetOutput must leave it alone).
bool ownsPin(uint8_t pin);

// VID6606 RESET_N pins are shared by the four motors on one chip. Returns true
// for the first gauge to claim a pin since the scheduler was last empty: that
// gauge pulses RESET_N, the others leave it alone.
bool claimReset(uint8_t pin);

uint8_t count();

// True if add() would accept another gauge.
bool hasRoom();

} // namespace GaugeScheduler
