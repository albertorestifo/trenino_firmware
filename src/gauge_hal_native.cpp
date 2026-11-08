#if !defined(ARDUINO)

#include "gauge_hal.h"

// Recording mock used by the native unit tests.
namespace GaugeHal {

uint8_t g_mock_pin_level[MOCK_PINS];
uint32_t g_mock_pin_writes[MOCK_PINS];
bool g_mock_timer_running = false;
uint32_t g_mock_timer_starts = 0;
uint32_t g_mock_delay_ms = 0;

void mockReset()
{
    for (uint8_t i = 0; i < MOCK_PINS; i++) {
        g_mock_pin_level[i] = 0;
        g_mock_pin_writes[i] = 0;
    }
    g_mock_timer_running = false;
    g_mock_timer_starts = 0;
    g_mock_delay_ms = 0;
}

bool isOutputPin(uint8_t pin)
{
    return pin < MOCK_PINS;
}

FastPin makeOutput(uint8_t pin, bool high)
{
    FastPin p;
    p.pin = pin;
    write(p, high);
    return p;
}

void startTimer()
{
    g_mock_timer_running = true;
    g_mock_timer_starts++;
}

void stopTimer()
{
    g_mock_timer_running = false;
}

void delayMs(uint16_t ms)
{
    g_mock_delay_ms += ms;
}

} // namespace GaugeHal

#endif
