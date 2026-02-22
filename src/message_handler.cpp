#include "message_handler.h"
#include "bldc_config.h"
#include "bldc_lever.h"
#include "bldc_manager.h"
#include "config_manager.h"
#include "heartbeat.h"
#include "output_manager.h"
#include "sensor_manager.h"

namespace MessageHandler {

// Global packet serial instance
static PacketSerial_<COBS>* g_packet_serial = nullptr;

// Heartbeat manager
static Heartbeat::HeartbeatManager* g_heartbeat_manager = nullptr;

// Template implementation - sends any protocol message and notifies heartbeat
template <typename T>
void sendMessage(const T& message)
{
    if (!g_packet_serial) {
        return;
    }

    uint8_t buffer[128];
    size_t encoded_size = message.encode(buffer, sizeof(buffer));

    if (encoded_size > 0) {
        g_packet_serial->send(buffer, encoded_size);

        // Notify heartbeat manager if initialized
        if (g_heartbeat_manager) {
            g_heartbeat_manager->notifyMessageSent(millis());
        }
    }
}

void init(PacketSerial_<COBS>* serial)
{
    g_packet_serial = serial;

    // Initialize heartbeat manager with 2 second interval and callback
    g_heartbeat_manager = new Heartbeat::HeartbeatManager(HEARTBEAT_INTERVAL_MS, sendHeartbeat);
}

void onPacketReceived(const uint8_t* buffer, size_t size)
{
    // Decode the protocol message
    Protocol::Message msg;
    if (!msg.decode(buffer, size)) {
        // Invalid message, ignore
        return;
    }

    // Handle different message types
    if (msg.isIdentityRequest()) {
        handleIdentityRequest(msg.identity_request.request_id);
    } else if (msg.isConfigure()) {
        handleConfigure(msg.configure);
    } else if (msg.isSetOutput()) {
        handleSetOutput(msg.set_output);
    } else if (msg.isLoadBLDCProfile()) {
        // Buffer format: [msg_type][pin][num_detents][num_linear_ranges][snap_point][endstop_strength][pos_start_lo][pos_start_hi][pos_end_lo][pos_end_hi][detent_data...][range_data...]
        size_t offset = 10; // After header (10 bytes with position bounds)

        // Build profile config from header fields
        BLDCConfig::ProfileConfig profile_config;
        profile_config.snap_point = msg.load_bldc_profile.snap_point;
        profile_config.endstop_strength = msg.load_bldc_profile.endstop_strength;

        // Allocate temporary arrays
        BLDCConfig::DetentConfig* detents = new BLDCConfig::DetentConfig[msg.load_bldc_profile.num_detents];
        BLDCConfig::LinearRangeConfig* ranges = nullptr;
        if (msg.load_bldc_profile.num_linear_ranges > 0) {
            ranges = new BLDCConfig::LinearRangeConfig[msg.load_bldc_profile.num_linear_ranges];
        }

        // Parse detents (2 bytes each: position_percent + detent_strength)
        for (uint8_t i = 0; i < msg.load_bldc_profile.num_detents && offset + 2 <= size; i++) {
            detents[i].position_percent = buffer[offset++];
            detents[i].detent_strength = buffer[offset++];
        }

        // Parse linear ranges (3 bytes each)
        for (uint8_t i = 0; i < msg.load_bldc_profile.num_linear_ranges && offset + 3 <= size; i++) {
            ranges[i].start_detent_index = buffer[offset++];
            ranges[i].end_detent_index = buffer[offset++];
            ranges[i].damping_strength = buffer[offset++];
        }

        // Find BLDC lever and load profile
        Sensors::ISensor* sensor = SensorManager::getSensorByPin(msg.load_bldc_profile.pin);
        if (sensor != nullptr && sensor->getType() == Sensors::InputType::BLDCLever) {
            Sensors::BLDCLever* bldc = static_cast<Sensors::BLDCLever*>(sensor);
            if (bldc->loadProfile(
                    msg.load_bldc_profile.position_start,
                    msg.load_bldc_profile.position_end,
                    detents, msg.load_bldc_profile.num_detents,
                    ranges, msg.load_bldc_profile.num_linear_ranges,
                    profile_config)) {
                sendConfigurationStored(0); // Success
            } else {
                sendConfigurationError(0); // Validation failed
            }
        } else {
            sendConfigurationError(0); // Sensor not found
        }

        // Clean up
        delete[] detents;
        if (ranges != nullptr) {
            delete[] ranges;
        }
    } else if (msg.isDeactivateBLDCProfile()) {
        Sensors::ISensor* sensor = SensorManager::getSensorByPin(msg.deactivate_bldc_profile.pin);
        if (sensor != nullptr && sensor->getType() == Sensors::InputType::BLDCLever) {
            Sensors::BLDCLever* bldc = static_cast<Sensors::BLDCLever*>(sensor);
            bldc->deactivateProfile();
            // No response - fire and forget
        }
    }
}

void update()
{
    // Update heartbeat manager (automatically sends heartbeat if needed)
    g_heartbeat_manager->update(millis());

    // Check for configuration timeout
    if (ConfigManager::checkTimeout()) {
        sendConfigurationError(ConfigManager::g_config_state.getConfigId());
    }

    // Scan all sensors
    SensorManager::scan();

    // Check for sensor readings and send them
    Sensors::Reading reading;
    while (SensorManager::getNextReading(reading)) {
        sendInputValue(reading);
    }
}

void handleIdentityRequest(uint32_t request_id)
{
    uint32_t config_id = ConfigManager::getCurrentConfigId();
    sendIdentityResponse(request_id, config_id);
}

void handleConfigure(const Protocol::Configure& cfg)
{
    bool complete = false;
    bool error = false;

    ConfigManager::handleConfigure(cfg, complete, error);

    if (complete) {
        // Apply configuration to sensors
        uint8_t num_inputs = 0;
        const ConfigManager::InputConfig* inputs = ConfigManager::getCurrentConfig(num_inputs);
        SensorManager::applyConfiguration(inputs, num_inputs);

        sendConfigurationStored(cfg.config_id);
    } else if (error) {
        sendConfigurationError(cfg.config_id);
    }
}

void handleSetOutput(const Protocol::SetOutput& cmd)
{
    OutputManager::setOutput(cmd.pin, cmd.value);
}

void sendIdentityResponse(uint32_t request_id, uint32_t config_id)
{
    Protocol::IdentityResponse response;
    response.request_id = request_id;
    response.version_major = DEVICE_VERSION_MAJOR;
    response.version_minor = DEVICE_VERSION_MINOR;
    response.version_patch = DEVICE_VERSION_PATCH;
    response.config_id = config_id;

    sendMessage(response);
}

void sendConfigurationStored(uint32_t config_id)
{
    Protocol::ConfigurationStored stored;
    stored.config_id = config_id;

    sendMessage(stored);
}

void sendConfigurationError(uint32_t config_id)
{
    Protocol::ConfigurationError error;
    error.config_id = config_id;

    sendMessage(error);
}

void sendInputValue(const Sensors::Reading& reading)
{
    Protocol::InputValue input_value;
    input_value.pin = reading.pin;
    input_value.value = reading.value;

    sendMessage(input_value);
}

void sendHeartbeat()
{
    Protocol::Heartbeat heartbeat;

    // Use sendMessage template, but DON'T notify heartbeat manager
    // (heartbeat sends are already tracked by HeartbeatManager)
    if (!g_packet_serial) {
        return;
    }

    uint8_t buffer[128];
    size_t encoded_size = heartbeat.encode(buffer, sizeof(buffer));

    if (encoded_size > 0) {
        g_packet_serial->send(buffer, encoded_size);
        // Note: We don't call notifyMessageSent here because the heartbeat
        // manager already knows it sent a heartbeat via its callback
    }
}

void sendEncoderError(uint8_t pin)
{
    Protocol::EncoderError msg;
    msg.pin = pin;

    sendMessage(msg);
}

} // namespace MessageHandler
