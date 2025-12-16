#pragma once

#include "analog_sensor.h"
#include "button_sensor.h"
#include "config_manager.h"
#include "matrix_sensor.h"
#include "sensor.h"
#include <stdint.h>

namespace SensorManager {

// Maximum number of sensors (matches MAX_INPUTS in config_manager)
constexpr uint8_t MAX_SENSORS = 8;

// Initialize sensor manager with configuration from ConfigManager
void init();

// Apply configuration - creates sensors based on configuration
// Returns true if configuration was successfully applied
bool applyConfiguration(const ConfigManager::InputConfig* inputs, uint8_t input_count);

// Scan all sensors (read values, update running averages)
void scan();

// Check if any sensor has a reading to report
// Returns true if a reading is available
// Populates the reading parameter with the sensor reading
bool getNextReading(Sensor::Reading& reading);

// Get number of active sensors
uint8_t getSensorCount();

} // namespace SensorManager
