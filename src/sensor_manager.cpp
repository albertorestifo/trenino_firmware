#include "sensor_manager.h"
#include "bldc_lever.h"
#include "bldc_manager.h"
#include "message_handler.h"

namespace SensorManager {

// Array of sensor pointers
static Sensors::ISensor* g_sensors[MAX_SENSORS];
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
    // Unregister any BLDC levers from BLDCManager before deleting
    for (uint8_t i = 0; i < g_sensor_count; i++) {
        if (g_sensors[i] != nullptr && g_sensors[i]->getType() == Sensors::InputType::BLDCLever) {
            BLDCManager::unregisterLever(static_cast<Sensors::BLDCLever*>(g_sensors[i]));
        }
    }

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

        Sensors::ISensor* sensor = nullptr;

        // Create sensor based on input type
        switch (config.input_type) {
        case Protocol::INPUT_TYPE_ANALOG:
            sensor = new Sensors::AnalogSensor(config.analog.pin, config.analog.sensitivity);
            break;

        case Protocol::INPUT_TYPE_BUTTON:
            sensor = new Sensors::ButtonSensor(config.button.pin, config.button.debounce);
            break;

        case Protocol::INPUT_TYPE_MATRIX:
            sensor = new Sensors::MatrixSensor(
                config.matrix.num_row_pins,
                config.matrix.num_col_pins,
                config.matrix.pins, // row pins
                config.matrix.pins + config.matrix.num_row_pins); // col pins
            break;

        case Protocol::INPUT_TYPE_BLDC_LEVER:
            sensor = new Sensors::BLDCLever(
                config.bldc.motor_pin_a,
                config.bldc.motor_pin_b,
                config.bldc.motor_pin_c,
                config.bldc.motor_enable,
                config.bldc.encoder_cs,
                config.bldc.pole_pairs,
                config.bldc.voltage,
                config.bldc.current_limit,
                config.bldc.encoder_bits);
            break;

        default:
            // Unknown input type - skip
            continue;
        }

        if (sensor != nullptr) {
            sensor->begin();

            // Register BLDC levers with BLDCManager for motor updates
            if (config.input_type == Protocol::INPUT_TYPE_BLDC_LEVER) {
                Sensors::BLDCLever* bldc = static_cast<Sensors::BLDCLever*>(sensor);
                BLDCManager::registerLever(bldc);
            }

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

bool getNextReading(Sensors::Reading& reading)
{
    // Check all sensors starting from the next index (round-robin)
    for (uint8_t i = 0; i < g_sensor_count; i++) {
        uint8_t index = (g_next_reading_index + i) % g_sensor_count;

        if (g_sensors[index] != nullptr) {
            Sensors::Reading r = g_sensors[index]->getReading();
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

Sensors::ISensor* getSensorByPin(uint8_t pin)
{
    for (uint8_t i = 0; i < g_sensor_count; i++) {
        if (g_sensors[i] != nullptr && g_sensors[i]->getPin() == pin) {
            return g_sensors[i];
        }
    }
    return nullptr;
}

} // namespace SensorManager
