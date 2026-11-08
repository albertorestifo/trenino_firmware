#include "gauge_scheduler.h"
#include "gauge_module.h"
#include "protocol.h"

namespace GaugeScheduler {

namespace {
Modules::GaugeModule* g_gauges[Protocol::MAX_GAUGES];
volatile uint8_t g_count = 0;
uint8_t g_suspended = 0;
uint8_t g_reset_pins[Protocol::MAX_GAUGES];
uint8_t g_reset_count = 0;

void updateTimer()
{
    if (g_count > 0 && g_suspended == 0) {
        GaugeHal::startTimer();
    } else {
        GaugeHal::stopTimer();
    }
}
} // namespace

bool add(Modules::GaugeModule* gauge)
{
    {
        GaugeHal::Critical lock;
        for (uint8_t i = 0; i < g_count; i++) {
            if (g_gauges[i] == gauge) return true;
        }
        if (g_count >= Protocol::MAX_GAUGES) return false;
        g_gauges[g_count] = gauge;
        g_count = g_count + 1;
    }
    if (g_count == 1) updateTimer();
    return true;
}

void remove(Modules::GaugeModule* gauge)
{
    {
        GaugeHal::Critical lock;
        for (uint8_t i = 0; i < g_count; i++) {
            if (g_gauges[i] != gauge) continue;
            for (uint8_t j = i + 1; j < g_count; j++) {
                g_gauges[j - 1] = g_gauges[j];
            }
            g_count = g_count - 1;
            break;
        }
    }
    if (g_count == 0) {
        g_reset_count = 0;
        updateTimer();
    }
}

GAUGE_IRAM void tick()
{
    for (uint8_t i = 0; i < g_count; i++) {
        g_gauges[i]->tick();
    }
}

void suspend()
{
    g_suspended++;
    if (g_suspended == 1) updateTimer();
}

void resume()
{
    if (g_suspended == 0) return;
    g_suspended--;
    if (g_suspended == 0) updateTimer();
}

bool ownsPin(uint8_t pin)
{
    for (uint8_t i = 0; i < g_count; i++) {
        if (g_gauges[i]->usesPin(pin)) return true;
    }
    return false;
}

bool claimReset(uint8_t pin)
{
    for (uint8_t i = 0; i < g_reset_count; i++) {
        if (g_reset_pins[i] == pin) return false;
    }
    if (g_reset_count < Protocol::MAX_GAUGES) {
        g_reset_pins[g_reset_count++] = pin;
    }
    return true;
}

bool hasRoom()
{
    return g_count < Protocol::MAX_GAUGES;
}

uint8_t count()
{
    return g_count;
}

} // namespace GaugeScheduler
