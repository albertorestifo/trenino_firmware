#if defined(ARDUINO) && defined(ARDUINO_ARCH_AVR)

#include "gauge_hal.h"
#include "gauge_motion.h"
#include "gauge_scheduler.h"
#include <avr/interrupt.h>

// Timer1 is free on every supported AVR board: millis() runs on Timer0, and
// nothing in this firmware uses analogWrite, tone or Servo. Running it in CTC
// mode disables Timer1 PWM on its pins (328P: 9, 10; 32U4: 9, 10; 2560: 11,
// 12, 13) while any gauge is configured.
ISR(TIMER1_COMPA_vect)
{
    GaugeScheduler::tick();
}

namespace GaugeHal {

bool isOutputPin(uint8_t pin)
{
    return pin < NUM_DIGITAL_PINS && digitalPinToPort(pin) != NOT_A_PIN;
}

FastPin makeOutput(uint8_t pin, bool high)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, high ? HIGH : LOW);
    FastPin p;
    p.out = portOutputRegister(digitalPinToPort(pin));
    p.mask = digitalPinToBitMask(pin);
    return p;
}

void startTimer()
{
    Critical lock;
    TCCR1A = 0; // the core leaves Timer1 in phase-correct PWM mode
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = (uint16_t)(F_CPU / 8 / Gauge::TICK_HZ - 1);
    TIFR1 = _BV(OCF1A);
    TIMSK1 |= _BV(OCIE1A);
    TCCR1B = _BV(WGM12) | _BV(CS11); // CTC on OCR1A, clock / 8
}

void stopTimer()
{
    Critical lock;
    TIMSK1 &= (uint8_t)~_BV(OCIE1A);
    TCCR1B = 0;
}

void delayMs(uint16_t ms)
{
    delay(ms);
}

} // namespace GaugeHal

#endif
