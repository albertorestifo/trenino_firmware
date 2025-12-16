#pragma once

#include "protocol.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <stdint.h>

namespace ConfigManager {

// Maximum number of inputs that can be configured
constexpr uint8_t MAX_INPUTS = 8;

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
struct InputConfig {
    uint8_t input_type;

    union {
        // INPUT_TYPE_ANALOG
        struct {
            uint8_t pin;
            uint8_t sensitivity;
        } analog;

        // INPUT_TYPE_BUTTON
        struct {
            uint8_t pin;
            uint8_t debounce;
        } button;

        // INPUT_TYPE_MATRIX
        struct {
            uint8_t num_row_pins;
            uint8_t num_col_pins;
            uint8_t pins[MAX_MATRIX_PINS]; // row_pins followed by col_pins
        } matrix;
    };

    InputConfig()
        : input_type(Protocol::INPUT_TYPE_ANALOG)
    {
        analog.pin = 0;
        analog.sensitivity = 0;
    }
};

// Configuration state
class ConfigState {
private:
    uint32_t config_id;
    uint8_t total_parts;
    uint8_t received_parts;
    bool parts_received[MAX_INPUTS]; // Track which parts we've received
    InputConfig inputs[MAX_INPUTS];
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
        for (uint8_t i = 0; i < MAX_INPUTS; i++) {
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

        for (uint8_t i = 0; i < MAX_INPUTS; i++) {
            parts_received[i] = false;
            inputs[i] = InputConfig();
        }
    }

    // Add a configuration part from a Configure message
    bool addPart(const Protocol::Configure& cfg)
    {
        if (!active || cfg.part_number >= total_parts || cfg.part_number >= MAX_INPUTS) {
            return false;
        }

        // Store the input configuration based on type
        inputs[cfg.part_number].input_type = cfg.input_type;

        switch (cfg.input_type) {
        case Protocol::INPUT_TYPE_ANALOG:
            inputs[cfg.part_number].analog.pin = cfg.analog.pin;
            inputs[cfg.part_number].analog.sensitivity = cfg.analog.sensitivity;
            break;

        case Protocol::INPUT_TYPE_BUTTON:
            inputs[cfg.part_number].button.pin = cfg.button.pin;
            inputs[cfg.part_number].button.debounce = cfg.button.debounce;
            break;

        case Protocol::INPUT_TYPE_MATRIX:
            inputs[cfg.part_number].matrix.num_row_pins = cfg.matrix.num_row_pins;
            inputs[cfg.part_number].matrix.num_col_pins = cfg.matrix.num_col_pins;
            for (uint8_t i = 0; i < cfg.matrix.num_row_pins + cfg.matrix.num_col_pins; i++) {
                inputs[cfg.part_number].matrix.pins[i] = cfg.matrix.pins[i];
            }
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

    // Get the inputs
    const InputConfig* getInputs() const
    {
        return inputs;
    }

    // Get the number of inputs
    uint8_t getNumInputs() const
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
void storeToEEPROM(uint32_t config_id, const InputConfig* inputs, uint8_t num_inputs);

// Load configuration from EEPROM
// Returns true if valid configuration was loaded
bool loadFromEEPROM();

// Get current configuration ID
uint32_t getCurrentConfigId();

// Get current configuration
const InputConfig* getCurrentConfig(uint8_t& num_inputs);

} // namespace ConfigManager
