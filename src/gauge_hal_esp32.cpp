#if defined(ARDUINO) && defined(ESP32)

#include "gauge_hal.h"
#include "gauge_motion.h"
#include "gauge_scheduler.h"
#include <driver/gpio.h>

namespace GaugeHal {

portMUX_TYPE g_gauge_mux = portMUX_INITIALIZER_UNLOCKED;

namespace {
hw_timer_t* g_timer = nullptr;

void IRAM_ATTR onTimer()
{
    portENTER_CRITICAL_ISR(&g_gauge_mux);
    GaugeScheduler::tick();
    portEXIT_CRITICAL_ISR(&g_gauge_mux);
}
} // namespace

bool isOutputPin(uint8_t pin)
{
    return GPIO_IS_VALID_OUTPUT_GPIO(pin);
}

FastPin makeOutput(uint8_t pin, bool high)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, high ? HIGH : LOW);
    FastPin p;
    p.pin = pin;
    return p;
}

// The timer counts at 1 MHz and fires every 1000000 / TICK_HZ counts.
// Core 2.x (espressif32 6.x) and core 3.x (espressif32 8.x) have different
// timer APIs.
#if ESP_ARDUINO_VERSION_MAJOR >= 3

void startTimer()
{
    if (g_timer == nullptr) {
        g_timer = timerBegin(1000000UL); // starts counting
        timerAttachInterrupt(g_timer, &onTimer);
        timerAlarm(g_timer, 1000000UL / Gauge::TICK_HZ, true, 0);
    }
    timerWrite(g_timer, 0);
    timerStart(g_timer);
}

void stopTimer()
{
    if (g_timer != nullptr) {
        timerStop(g_timer);
    }
}

#else

void startTimer()
{
    if (g_timer == nullptr) {
        g_timer = timerBegin(0, 80, true); // 80 MHz APB / 80 = 1 MHz
        timerAttachInterrupt(g_timer, &onTimer, true);
        timerAlarmWrite(g_timer, 1000000UL / Gauge::TICK_HZ, true);
    }
    // The counter kept running while stopped; restart it below the alarm.
    timerWrite(g_timer, 0);
    timerAlarmEnable(g_timer);
}

void stopTimer()
{
    if (g_timer != nullptr) {
        timerAlarmDisable(g_timer);
    }
}

#endif

void delayMs(uint16_t ms)
{
    delay(ms);
}

} // namespace GaugeHal

#endif
