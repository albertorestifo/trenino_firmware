#if defined(ARDUINO) && defined(ARDUINO_SAM_DUE)

#include "gauge_hal.h"
#include "gauge_motion.h"
#include "gauge_scheduler.h"

// TC1 channel 0 (TC3 interrupt). analogWrite uses TC0 and TC2.
void TC3_Handler()
{
    TC_GetStatus(TC1, 0); // reading the status clears the interrupt
    GaugeScheduler::tick();
}

namespace GaugeHal {

bool isOutputPin(uint8_t pin)
{
    return pin < NUM_DIGITAL_PINS && g_APinDescription[pin].ulPinType != PIO_NOT_A_PIN;
}

FastPin makeOutput(uint8_t pin, bool high)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, high ? HIGH : LOW);
    FastPin p;
    p.port = g_APinDescription[pin].pPort;
    p.mask = g_APinDescription[pin].ulPin;
    return p;
}

void startTimer()
{
    pmc_set_writeprotect(false);
    pmc_enable_periph_clk(ID_TC3);
    // MCK / 8 = 10.5 MHz, count up to RC and restart.
    TC_Configure(TC1, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2);
    TC_SetRC(TC1, 0, VARIANT_MCK / 8 / Gauge::TICK_HZ);
    TC1->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
    TC1->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;
    NVIC_ClearPendingIRQ(TC3_IRQn);
    NVIC_EnableIRQ(TC3_IRQn);
    TC_Start(TC1, 0);
}

void stopTimer()
{
    NVIC_DisableIRQ(TC3_IRQn);
    TC_Stop(TC1, 0);
}

void delayMs(uint16_t ms)
{
    delay(ms);
}

} // namespace GaugeHal

#endif
