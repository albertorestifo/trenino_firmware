#include "sensor_manager.h"

namespace SensorManager {

// Array of sensor pointers
static Sensor::ISensor* g_sensors[MAX_SENSORS];
static uint8_t g_sensor_count = 0;

// Index for round-robin reading retrieval
static uint8_t g_next_reading_index = 0;

void init()
{
    // Clear all sensors
    for (uint8_t i = 0; i < MAX_SENSORS; i++) {
        if (g_sensors[i] != nullptr) {
            delete g_sensors[i];
            g_sensors[i] = nullptr;
        }
    }
    g_sensor_count = 0;
    g_next_reading_index = 0;
}

bool applyConfiguration(const ConfigManager::InputConfig* inputs, uint8_t input_count)
{
    // Clear existing sensors
    for (uint8_t i = 0; i < MAX_SENSORS; i++) {
        if (g_sensors[i] != nullptr) {
            delete g_sensors[i];
            g_sensors[i] = nullptr;
        }
    }
    g_sensor_count = 0;
    g_next_reading_index = 0;

    // Validate input count
    if (input_count > MAX_SENSORS) {
        return false;
    }

    // Create sensors based on configuration
    for (uint8_t i = 0; i < input_count; i++) {
        const ConfigManager::InputConfig& config = inputs[i];

        Sensor::ISensor* sensor = nullptr;

        // Create sensor based on input type
        switch (config.input_type) {
        case Protocol::INPUT_TYPE_ANALOG:
            sensor = new Sensor::AnalogSensor(config.analog.pin, config.analog.sensitivity);
            break;

        case Protocol::INPUT_TYPE_BUTTON:
            sensor = new Sensor::ButtonSensor(config.button.pin, config.button.debounce);
            break;

        case Protocol::INPUT_TYPE_MATRIX:
            sensor = new Sensor::MatrixSensor(
                config.matrix.num_row_pins,
                config.matrix.num_col_pins,
                config.matrix.pins, // row pins
                config.matrix.pins + config.matrix.num_row_pins); // col pins
            break;

        default:
            // Unknown input type - skip
            continue;
        }

        if (sensor != nullptr) {
            sensor->begin();
            g_sensors[g_sensor_count++] = sensor;
        }
    }

    return true;
}

void scan()
{
    // Scan all active sensors
    for (uint8_t i = 0; i < g_sensor_count; i++) {
        if (g_sensors[i] != nullptr) {
            g_sensors[i]->scan();
        }
    }
}

bool getNextReading(Sensor::Reading& reading)
{
    // Check all sensors starting from the next index (round-robin)
    for (uint8_t i = 0; i < g_sensor_count; i++) {
        uint8_t index = (g_next_reading_index + i) % g_sensor_count;

        if (g_sensors[index] != nullptr) {
            Sensor::Reading r = g_sensors[index]->getReading();
            if (r.has_value) {
                reading = r;
                // Move to next sensor for next call
                g_next_reading_index = (index + 1) % g_sensor_count;
                return true;
            }
        }
    }

    return false; // No readings available
}

uint8_t getSensorCount()
{
    return g_sensor_count;
}

} // namespace SensorManager
