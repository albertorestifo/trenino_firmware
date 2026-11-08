#pragma once

// Board-specific pieces of gauge driving: fast pin writes for the timer
// interrupt, a critical section, the tick timer, and a millisecond delay.
// One implementation per architecture in gauge_hal_*.cpp; the native build
// gets a recording mock for unit tests.

#include <stdint.h>

// Real boards define ARDUINO; native tests do not (some define
// ARDUINO_ARCH_AVR to imitate one, which must still get the mock).
#if defined(ARDUINO) && defined(ARDUINO_ARCH_AVR)
#define GAUGE_HAL_AVR
#include <Arduino.h>
#elif defined(ARDUINO) && defined(ARDUINO_SAM_DUE)
#define GAUGE_HAL_SAM
#include <Arduino.h>
#elif defined(ARDUINO) && defined(ESP32)
#define GAUGE_HAL_ESP32
#include <Arduino.h>
#include <esp_arduino_version.h>
#if ESP_ARDUINO_VERSION_MAJOR != 2
#error "Gauge timer code targets the ESP32 Arduino core 2.x timer API"
#endif
#include <soc/gpio_struct.h>
#elif defined(ARDUINO)
#error "Gauge support is not implemented for this architecture"
#endif

// Code reached from the timer interrupt must live in IRAM on ESP32, because
// flash writes disable the flash cache.
#if defined(GAUGE_HAL_ESP32)
#define GAUGE_IRAM IRAM_ATTR
#define GAUGE_DRAM DRAM_ATTR
#else
#define GAUGE_IRAM
#define GAUGE_DRAM
#endif

namespace GaugeHal {

#if defined(GAUGE_HAL_AVR)

struct FastPin {
    volatile uint8_t* out;
    uint8_t mask;
};

// Only call with interrupts disabled (the tick interrupt, or Critical).
inline void write(const FastPin& p, bool high)
{
    if (high) {
        *p.out |= p.mask;
    } else {
        *p.out &= (uint8_t)~p.mask;
    }
}

class Critical {
public:
    Critical() : sreg_(SREG) { cli(); }
    ~Critical() { SREG = sreg_; }

private:
    uint8_t sreg_;
};

#elif defined(GAUGE_HAL_SAM)

struct FastPin {
    Pio* port;
    uint32_t mask;
};

inline void write(const FastPin& p, bool high)
{
    if (high) {
        p.port->PIO_SODR = p.mask;
    } else {
        p.port->PIO_CODR = p.mask;
    }
}

class Critical {
public:
    Critical() : primask_(__get_PRIMASK()) { __disable_irq(); }
    ~Critical() { __set_PRIMASK(primask_); }

private:
    uint32_t primask_;
};

#elif defined(GAUGE_HAL_ESP32)

struct FastPin {
    uint8_t pin;
};

// Always inlined into its IRAM callers; an IRAM_ATTR inline in a header
// trips an l32r literal-placement link error.
inline __attribute__((always_inline)) void write(const FastPin& p, bool high)
{
    if (p.pin < 32) {
        if (high) {
            GPIO.out_w1ts = 1UL << p.pin;
        } else {
            GPIO.out_w1tc = 1UL << p.pin;
        }
    } else {
        if (high) {
            GPIO.out1_w1ts.val = 1UL << (p.pin - 32);
        } else {
            GPIO.out1_w1tc.val = 1UL << (p.pin - 32);
        }
    }
}

// Shared with the timer interrupt, which takes it with portENTER_CRITICAL_ISR.
extern portMUX_TYPE g_gauge_mux;

class Critical {
public:
    Critical() { portENTER_CRITICAL(&g_gauge_mux); }
    ~Critical() { portEXIT_CRITICAL(&g_gauge_mux); }
};

#else // native unit tests

struct FastPin {
    uint8_t pin;
};

constexpr uint8_t MOCK_PINS = 64;
extern uint8_t g_mock_pin_level[MOCK_PINS];
extern uint32_t g_mock_pin_writes[MOCK_PINS]; // rising and falling edges written
extern bool g_mock_timer_running;
extern uint32_t g_mock_timer_starts;
extern uint32_t g_mock_delay_ms;
void mockReset();

inline void write(const FastPin& p, bool high)
{
    if (p.pin < MOCK_PINS) {
        g_mock_pin_writes[p.pin]++;
        g_mock_pin_level[p.pin] = high ? 1 : 0;
    }
}

class Critical {
};

#endif

// True if the pin exists on this board and can drive an output. Checked
// before makeOutput, whose result the interrupt writes through as a raw
// register pointer.
bool isOutputPin(uint8_t pin);

// Sets the pin as an output at the given level (call outside the interrupt).
FastPin makeOutput(uint8_t pin, bool high);

// Start or stop calling GaugeScheduler::tick() at Gauge::TICK_HZ.
void startTimer();
void stopTimer();

void delayMs(uint16_t ms);

} // namespace GaugeHal
