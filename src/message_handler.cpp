#include "message_handler.h"
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
    }
    // isLoadBLDCProfile() and isDeactivateBLDCProfile() are silently ignored
    // (wire-level messages kept for backwards compatibility with older hosts)
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
    ModuleManager::scan();

    // Check for sensor readings and send them
    Modules::Reading reading;
    while (ModuleManager::getNextReading(reading)) {
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
        ModuleManager::applyConfiguration(inputs, num_inputs);

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

void sendInputValue(const Modules::Reading& reading)
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

} // namespace MessageHandler
