#include "sensor_manager.h"
#include "message_handler.h"

namespace ModuleManager {

// Array of sensor pointers
static Modules::IModule* g_sensors[MAX_MODULES];
static uint8_t g_sensor_count = 0;

// Index for round-robin reading retrieval
static uint8_t g_next_reading_index = 0;

// Init-error list, populated by applyConfiguration, drained by getInitErrors
static InitError g_init_errors[MAX_MODULES];
static uint8_t g_init_error_count = 0;

void init()
{
    // Clear all sensors
    for (uint8_t i = 0; i < MAX_MODULES; i++) {
        if (g_sensors[i] != nullptr) {
            delete g_sensors[i];
            g_sensors[i] = nullptr;
        }
    }
    g_sensor_count = 0;
    g_next_reading_index = 0;
}

bool applyConfiguration(const ConfigManager::ModuleConfig* modules, uint8_t module_count)
{
    // Clear existing sensors
    for (uint8_t i = 0; i < MAX_MODULES; i++) {
        if (g_sensors[i] != nullptr) {
            delete g_sensors[i];
            g_sensors[i] = nullptr;
        }
    }
    g_sensor_count = 0;
    g_next_reading_index = 0;
    g_init_error_count = 0;

    // Validate module count
    if (module_count > MAX_MODULES) {
        return false;
    }

    // Create sensors based on configuration
    for (uint8_t i = 0; i < module_count; i++) {
        const ConfigManager::ModuleConfig& config = modules[i];

        Modules::IModule* sensor = nullptr;

        // Create sensor based on input type
        switch (config.module_type) {
        case Protocol::MODULE_TYPE_ANALOG:
            sensor = new Modules::AnalogSensor(config.analog.pin, config.analog.sensitivity);
            break;

        case Protocol::MODULE_TYPE_BUTTON:
            sensor = new Modules::ButtonSensor(config.button.pin, config.button.debounce);
            break;

        case Protocol::MODULE_TYPE_MATRIX:
            sensor = new Modules::MatrixSensor(
                config.matrix.num_row_pins,
                config.matrix.num_col_pins,
                config.matrix.pins, // row pins
                config.matrix.pins + config.matrix.num_row_pins); // col pins
            break;

        case Protocol::MODULE_TYPE_HT16K33:
            sensor = new Modules::HT16K33Module(
                config.ht16k33.i2c_address,
                config.ht16k33.brightness,
                config.ht16k33.num_digits);
            break;

        default:
            // Unknown input type - skip
            continue;
        }

        if (sensor != nullptr) {
            bool ok = sensor->begin();

            if (!ok && g_init_error_count < MAX_MODULES) {
                InitError err;
                err.type = sensor->getType();
                err.i2c_address = sensor->getI2CAddress();
                err.pin = sensor->getPin();
                err.error_code = 0; // init_failed
                g_init_errors[g_init_error_count++] = err;
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

bool getNextReading(Modules::Reading& reading)
{
    // Check all sensors starting from the next index (round-robin)
    for (uint8_t i = 0; i < g_sensor_count; i++) {
        uint8_t index = (g_next_reading_index + i) % g_sensor_count;

        if (g_sensors[index] != nullptr) {
            Modules::Reading r = g_sensors[index]->getReading();
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

Modules::IModule* getModuleByPin(uint8_t pin)
{
    for (uint8_t i = 0; i < g_sensor_count; i++) {
        if (g_sensors[i] != nullptr && g_sensors[i]->getPin() == pin) {
            return g_sensors[i];
        }
    }
    return nullptr;
}

Modules::IModule* getModuleByI2CAddress(uint8_t address)
{
    if (address == 0) {
        return nullptr; // 0 means "no I2C address" — never match
    }
    for (uint8_t i = 0; i < g_sensor_count; i++) {
        if (g_sensors[i] != nullptr && g_sensors[i]->getI2CAddress() == address) {
            return g_sensors[i];
        }
    }
    return nullptr;
}

uint8_t getInitErrors(InitError* out, uint8_t max_count)
{
    uint8_t copied = (g_init_error_count < max_count) ? g_init_error_count : max_count;
    for (uint8_t i = 0; i < copied; i++) {
        out[i] = g_init_errors[i];
    }
    g_init_error_count = 0;
    return copied;
}

} // namespace ModuleManager
