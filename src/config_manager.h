#pragma once

#include "protocol.h"
#include <Arduino.h>
#include <stdint.h>

// Platform-specific EEPROM handling
#if defined(ESP32_PLATFORM) || defined(ESP32) || defined(ESP8266)
#include <EEPROM.h>
#define EEPROM_NEEDS_BEGIN
#define EEPROM_NEEDS_COMMIT
#define EEPROM_SIZE 512
#elif defined(EEPROM_EMULATION) || defined(ARDUINO_SAM_DUE)
#include <DueFlashStorage.h>
#define EEPROM_USE_DUE_FLASH
#else
#include <EEPROM.h>
#endif

namespace ConfigManager {

// EEPROM bytes 500..504 are reserved: they held the stepper shield address
// registration in firmware 3.1.0-3.2.1. Never read, write or reuse them.

// Maximum number of inputs that can be configured
constexpr uint8_t MAX_MODULES = 8;

// Timeout for configuration in milliseconds (5 seconds)
constexpr unsigned long CONFIG_TIMEOUT_MS = 5000;

// EEPROM addresses
constexpr int EEPROM_MAGIC_ADDR = 0; // 4 bytes - magic number to validate EEPROM
constexpr int EEPROM_VERSION_ADDR = 4; // 1 byte - device version that created this config
constexpr int EEPROM_CONFIG_ID_ADDR = 5; // 4 bytes - config_id
constexpr int EEPROM_NUM_INPUTS_ADDR = 9; // 1 byte - number of inputs
constexpr int EEPROM_INPUTS_ADDR = 10; // Start of input configurations

// Magic number to validate EEPROM data
constexpr uint32_t EEPROM_MAGIC = 0xC0FF1234;

// Maximum matrix pins (row + col pins)
constexpr uint8_t MAX_MATRIX_PINS = Protocol::MAX_MATRIX_PINS;

// Single input configuration - union-based to match protocol
struct ModuleConfig {
    uint8_t module_type;

    union {
        // MODULE_TYPE_ANALOG
        struct {
            uint8_t pin;
            uint8_t sensitivity;
        } analog;

        // MODULE_TYPE_BUTTON
        struct {
            uint8_t pin;
            uint8_t debounce;
        } button;

        // MODULE_TYPE_MATRIX
        struct {
            uint8_t num_row_pins;
            uint8_t num_col_pins;
            uint8_t pins[MAX_MATRIX_PINS]; // row_pins followed by col_pins
        } matrix;

        // MODULE_TYPE_HT16K33
        struct {
            uint8_t i2c_address;
            uint8_t brightness;
            uint8_t num_digits;
        } ht16k33;

        // MODULE_TYPE_GAUGE
        Protocol::GaugeConfig gauge;
    };

    ModuleConfig()
        : module_type(Protocol::MODULE_TYPE_ANALOG)
    {
        analog.pin = 0;
        analog.sensitivity = 0;
    }
};

// True if this board can run the gauge driver. Direct GPIO drive wants about
// 20 mA per coil at 5 V: Due pins are rated 3 mA, and ESP32's 3.3 V gives a
// 260 ohm coil only about 13 mA, too little torque.
inline bool isGaugeDriverSupported(uint8_t driver)
{
#if defined(ARDUINO_SAM_DUE) || defined(ESP32)
    return driver == Protocol::GAUGE_DRIVER_VID6606;
#else
    return driver == Protocol::GAUGE_DRIVER_VID6606 || driver == Protocol::GAUGE_DRIVER_DIRECT;
#endif
}

// Full check for one stored or received gauge record.
inline bool isUsableGauge(const Protocol::GaugeConfig& gauge)
{
    return Protocol::isValidGaugeConfig(gauge) && isGaugeDriverSupported(gauge.driver);
}

// Number of gauge modules in a module list.
inline uint8_t countGauges(const ModuleConfig* modules, uint8_t count)
{
    uint8_t gauges = 0;
    for (uint8_t i = 0; i < count; i++) {
        if (modules[i].module_type == Protocol::MODULE_TYPE_GAUGE) gauges++;
    }
    return gauges;
}

// Pins a gauge drives on its own (RESET_N is shared by design, so excluded).
inline uint8_t gaugeOwnPins(const Protocol::GaugeConfig& g, uint8_t out[4])
{
    if (g.driver == Protocol::GAUGE_DRIVER_VID6606) {
        out[0] = g.pins[0];
        out[1] = g.pins[1];
        return 2;
    }
    for (uint8_t i = 0; i < 4; i++) out[i] = g.pins[i];
    return 4;
}

// True if two gauges share a STEP, DIR or coil pin, or one gauge's RESET_N is
// another's STEP/DIR/coil. Gauges are addressed by pins[0], so a shared pin
// would also make one of them unreachable.
inline bool gaugePinsConflict(const ModuleConfig* modules, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        if (modules[i].module_type != Protocol::MODULE_TYPE_GAUGE) continue;
        uint8_t a[4];
        uint8_t na = gaugeOwnPins(modules[i].gauge, a);
        for (uint8_t j = 0; j < count; j++) {
            if (j == i || modules[j].module_type != Protocol::MODULE_TYPE_GAUGE) continue;
            const Protocol::GaugeConfig& other = modules[j].gauge;
            uint8_t b[4];
            uint8_t nb = gaugeOwnPins(other, b);
            for (uint8_t x = 0; x < na; x++) {
                if (other.driver == Protocol::GAUGE_DRIVER_VID6606 && a[x] == other.pins[2]) return true;
                for (uint8_t y = 0; y < nb; y++) {
                    if (j > i && a[x] == b[y]) return true;
                }
            }
        }
    }
    return false;
}

// Configuration state
class ConfigState {
private:
    uint32_t config_id;
    uint8_t total_parts;
    uint8_t received_parts;
    bool parts_received[MAX_MODULES]; // Track which parts we've received
    ModuleConfig modules[MAX_MODULES];
    unsigned long start_time; // When we started receiving this configuration
    bool active; // Is there an active configuration being received?

public:
    ConfigState()
        : config_id(0)
        , total_parts(0)
        , received_parts(0)
        , start_time(0)
        , active(false)
    {
        for (uint8_t i = 0; i < MAX_MODULES; i++) {
            parts_received[i] = false;
        }
    }

    // Start a new configuration
    void start(uint32_t cfg_id, uint8_t total)
    {
        config_id = cfg_id;
        total_parts = total;
        received_parts = 0;
        start_time = millis();
        active = true;

        for (uint8_t i = 0; i < MAX_MODULES; i++) {
            parts_received[i] = false;
            modules[i] = ModuleConfig();
        }
    }

    // Add a configuration part from a Configure message
    bool addPart(const Protocol::Configure& cfg)
    {
        if (!active || cfg.part_number >= total_parts || cfg.part_number >= MAX_MODULES) {
            return false;
        }

        // Store the input configuration based on type
        modules[cfg.part_number].module_type = cfg.module_type;

        switch (cfg.module_type) {
        case Protocol::MODULE_TYPE_ANALOG:
            modules[cfg.part_number].analog.pin = cfg.analog.pin;
            modules[cfg.part_number].analog.sensitivity = cfg.analog.sensitivity;
            break;

        case Protocol::MODULE_TYPE_BUTTON:
            modules[cfg.part_number].button.pin = cfg.button.pin;
            modules[cfg.part_number].button.debounce = cfg.button.debounce;
            break;

        case Protocol::MODULE_TYPE_MATRIX:
            modules[cfg.part_number].matrix.num_row_pins = cfg.matrix.num_row_pins;
            modules[cfg.part_number].matrix.num_col_pins = cfg.matrix.num_col_pins;
            for (uint8_t i = 0; i < cfg.matrix.num_row_pins + cfg.matrix.num_col_pins; i++) {
                modules[cfg.part_number].matrix.pins[i] = cfg.matrix.pins[i];
            }
            break;

        case Protocol::MODULE_TYPE_HT16K33:
            modules[cfg.part_number].ht16k33.i2c_address = cfg.ht16k33.i2c_address;
            modules[cfg.part_number].ht16k33.brightness = cfg.ht16k33.brightness;
            modules[cfg.part_number].ht16k33.num_digits = cfg.ht16k33.num_digits;
            break;

        case Protocol::MODULE_TYPE_GAUGE:
            if (!isUsableGauge(cfg.gauge)) {
                return false;
            }
            modules[cfg.part_number].gauge = cfg.gauge;
            break;

        default:
            return false; // Unknown input type
        }

        // Mark this part as received
        if (!parts_received[cfg.part_number]) {
            parts_received[cfg.part_number] = true;
            received_parts++;
        }

        return true;
    }

    // Check if configuration is complete
    bool isComplete() const
    {
        return active && received_parts == total_parts;
    }

    // Check if configuration has timed out
    bool hasTimedOut() const
    {
        return active && (millis() - start_time) > CONFIG_TIMEOUT_MS;
    }

    // Get the configuration ID
    uint32_t getConfigId() const
    {
        return config_id;
    }

    // Get the modules
    const ModuleConfig* getModules() const
    {
        return modules;
    }

    // Get the number of modules
    uint8_t getNumModules() const
    {
        return total_parts;
    }

    // Reset the state
    void reset()
    {
        active = false;
        config_id = 0;
        total_parts = 0;
        received_parts = 0;
    }

    // Check if there's an active configuration
    bool isActive() const
    {
        return active;
    }
};

// Global configuration state
extern ConfigState g_config_state;
extern uint32_t g_current_config_id;

// Initialize configuration manager
void init();

// Handle a Configure message
// Returns true if configuration is complete or error occurred
// Sets complete=true if configuration is complete
// Sets error=true if configuration failed
bool handleConfigure(const Protocol::Configure& cfg, bool& complete, bool& error);

// Check for configuration timeout
// Returns true if timeout occurred
bool checkTimeout();

// Store configuration to EEPROM
void storeToEEPROM(uint32_t config_id, const ModuleConfig* modules, uint8_t num_modules);

// Load configuration from EEPROM
// Returns true if valid configuration was loaded
bool loadFromEEPROM();

// Get current configuration ID
uint32_t getCurrentConfigId();

// Get current configuration
const ModuleConfig* getCurrentConfig(uint8_t& num_modules);

} // namespace ConfigManager
