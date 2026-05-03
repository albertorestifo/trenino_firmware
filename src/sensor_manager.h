#pragma once

#include "analog_sensor.h"
#include "button_sensor.h"
#include "config_manager.h"
#include "ht16k33_module.h"
#include "matrix_sensor.h"
#include "module.h"
#include <stdint.h>

namespace ModuleManager {

// Maximum number of sensors (matches MAX_MODULES in config_manager)
constexpr uint8_t MAX_MODULES = 8;

// Information about a module that failed to initialize.
// Used by MessageHandler to emit ModuleError messages after applyConfiguration.
struct InitError {
    Modules::ModuleType type;
    uint8_t i2c_address; // populated for I2C modules; 0 for hardware-pin modules
    uint8_t pin;         // populated for hardware-pin modules; 0 for I2C-only modules
    uint8_t error_code;  // see Protocol::ModuleError error codes (when added)
};

// Initialize sensor manager with configuration from ConfigManager
void init();

// Apply configuration - creates sensors based on configuration
// Returns true if configuration was successfully applied
bool applyConfiguration(const ConfigManager::ModuleConfig* modules, uint8_t module_count);

// Scan all sensors (read values, update running averages)
void scan();

// Check if any sensor has a reading to report
// Returns true if a reading is available
// Populates the reading parameter with the sensor reading
bool getNextReading(Modules::Reading& reading);

// Get number of active sensors
uint8_t getSensorCount();

// Get sensor by pin number
// Returns nullptr if not found
Modules::IModule* getModuleByPin(uint8_t pin);

// Get module by I2C address. Returns nullptr if not found or if address is 0.
Modules::IModule* getModuleByI2CAddress(uint8_t address);

// Drain the init-error list collected during the most recent applyConfiguration.
// Copies up to max_count errors into out, returns the actual count copied,
// and clears the internal list.
uint8_t getInitErrors(InitError* out, uint8_t max_count);

} // namespace ModuleManager
