#include "message_handler.h"
#include "config_manager.h"
#include "heartbeat.h"
#include "ht16k33_module.h"
#include "output_manager.h"
#include "sensor_manager.h"
#include "stepper_service.h"

namespace MessageHandler {

// Global packet serial instance
static PacketSerial_<COBS>* g_packet_serial = nullptr;

// Heartbeat manager
static Heartbeat::HeartbeatManager* g_heartbeat_manager = nullptr;

// Sends encoded bytes and notifies the heartbeat manager
static void sendBytes(const uint8_t* bytes, size_t size)
{
    if (g_packet_serial) g_packet_serial->send(bytes, size);
    if (g_heartbeat_manager) g_heartbeat_manager->notifyMessageSent(millis());
}

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
        sendBytes(buffer, encoded_size);
    }
}

void init(PacketSerial_<COBS>* serial)
{
    g_packet_serial = serial;

    // Initialize heartbeat manager with 2 second interval and callback
    g_heartbeat_manager = new Heartbeat::HeartbeatManager(HEARTBEAT_INTERVAL_MS, sendHeartbeat);
    StepperService::init(sendBytes);
    ModuleManager::setBusTurnCallback(StepperService::update);
}

void onPacketReceived(const uint8_t* buffer, size_t size)
{
    if (StepperService::onPacket(buffer, size)) return;
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
    } else if (msg.isWriteSegments()) {
        handleWriteSegments(msg.write_segments);
    } else if (msg.isSetModuleBrightness()) {
        handleSetModuleBrightness(msg.set_module_brightness);
    }
    // isLoadBLDCProfile() and isDeactivateBLDCProfile() are silently ignored
    // (wire-level messages kept for backwards compatibility with older hosts)
}

// Reports queued module init errors to the host; returns how many were sent.
static uint8_t sendInitErrors(ModuleManager::InitError* errors)
{
    uint8_t error_count = ModuleManager::getInitErrors(errors, ModuleManager::MAX_MODULES);
    for (uint8_t i = 0; i < error_count; i++) {
        sendModuleError(errors[i].i2c_address, errors[i].error_code);
    }
    return error_count;
}

void update()
{
    StepperService::update();
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
        StepperService::update();
    }

    // Report runtime errors raised by I2C modules (lost, recovered, ...)
    uint8_t error_address;
    uint8_t error_code;
    while (ModuleManager::getNextRuntimeError(error_address, error_code)) {
        sendModuleError(error_address, error_code);
    }
}

void handleIdentityRequest(uint32_t request_id)
{
    uint32_t config_id = ConfigManager::getCurrentConfigId();
    sendIdentityResponse(request_id, config_id);
    if (StepperService::bootFaultAddress()) sendModuleError(StepperService::bootFaultAddress(), 0);

    // Drain any pending module-init errors and report them to the host.
    // This catches errors from the boot-time applyConfiguration() in setup()
    // — at that point the host isn't connected yet, so errors stay in the
    // ModuleManager's queue until the host identifies and we flush them here.
    // The same flush happens after every host-driven Configure (in
    // handleConfigure), which is the other place applyConfiguration runs.
    ModuleManager::InitError errors[ModuleManager::MAX_MODULES];
    uint8_t error_count = sendInitErrors(errors);

    // Repeat errors modules are in right now (communication lost, supply
    // reference lost), skipping any address already reported above.
    ModuleManager::InitError active[ModuleManager::MAX_MODULES];
    uint8_t active_count = ModuleManager::getActiveErrors(active, ModuleManager::MAX_MODULES);
    for (uint8_t i = 0; i < active_count; i++) {
        bool already_sent = false;
        for (uint8_t j = 0; j < error_count; j++) {
            if (errors[j].i2c_address == active[i].i2c_address) already_sent = true;
        }
        if (!already_sent) sendModuleError(active[i].i2c_address, active[i].error_code);
    }
}

void handleConfigure(const Protocol::Configure& cfg)
{
    if (StepperService::conflicts(cfg)) {
        ConfigManager::g_config_state.reset();
        sendConfigurationError(cfg.config_id); return;
    }
    bool complete = false;
    bool error = false;

    ConfigManager::handleConfigure(cfg, complete, error);

    if (complete) {
        if (!StepperService::reset()) sendModuleError(ConfigManager::getRegisteredShieldAddress(), 0);
        // Apply configuration to sensors
        uint8_t num_modules = 0;
        const ConfigManager::ModuleConfig* modules = ConfigManager::getCurrentConfig(num_modules);
        // Removing every legacy module requires no I2C initialization and must
        // still work when a registered shield prevented boot-time module init.
        if (num_modules == 0 || StepperService::canApplyModules()) ModuleManager::applyConfiguration(modules, num_modules);

        sendConfigurationStored(cfg.config_id);

        // Drain any per-module init errors and report them to the host
        ModuleManager::InitError errors[ModuleManager::MAX_MODULES];
        sendInitErrors(errors);
    } else if (error) {
        sendConfigurationError(cfg.config_id);
    }
}

void handleSetOutput(const Protocol::SetOutput& cmd)
{
    if (StepperService::reservesPin(cmd.pin)) return;
    OutputManager::setOutput(cmd.pin, cmd.value);
}

void handleWriteSegments(const Protocol::WriteSegments& cmd)
{
    Modules::IModule* module = ModuleManager::getModuleByI2CAddress(cmd.i2c_address);
    if (module == nullptr) return;
    if (module->getType() != Modules::ModuleType::HT16K33) return;

    Modules::HT16K33Module* ht = static_cast<Modules::HT16K33Module*>(module);
    ht->writeSegments(cmd.data, cmd.num_bytes);
}

void handleSetModuleBrightness(const Protocol::SetModuleBrightness& cmd)
{
    Modules::IModule* module = ModuleManager::getModuleByI2CAddress(cmd.i2c_address);
    if (module == nullptr) return;
    if (module->getType() != Modules::ModuleType::HT16K33) return;

    Modules::HT16K33Module* ht = static_cast<Modules::HT16K33Module*>(module);
    ht->setBrightness(cmd.brightness);
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

void sendModuleError(uint8_t i2c_address, uint8_t error_code)
{
    Protocol::ModuleError msg;
    msg.i2c_address = i2c_address;
    msg.error_code = error_code;
    sendMessage(msg);
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
