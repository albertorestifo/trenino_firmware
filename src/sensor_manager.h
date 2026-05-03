#pragma once

#include "analog_sensor.h"
#include "button_sensor.h"
#include "config_manager.h"
#include "matrix_sensor.h"
#include "module.h"
#include <stdint.h>

namespace ModuleManager {

// Maximum number of sensors (matches MAX_MODULES in config_manager)
constexpr uint8_t MAX_MODULES = 8;

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

} // namespace ModuleManager
