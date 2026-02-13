#pragma once

#include "device_info.h"
#include "protocol.h"
#include "sensor.h"
#include <PacketSerial.h>
#include <stdint.h>

namespace MessageHandler {

// Heartbeat interval in milliseconds
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 2000;

// Initialize message handler
void init(PacketSerial_<COBS>* serial);

// Main packet received callback
void onPacketReceived(const uint8_t* buffer, size_t size);

// Update message handler (call in loop)
void update();

// Message handlers for specific message types
void handleIdentityRequest(uint32_t request_id);
void handleConfigure(const Protocol::Configure& cfg);
void handleSetOutput(const Protocol::SetOutput& cmd);

// Internal helper - sends a message and notifies heartbeat manager
// Template function to handle any protocol message type
template <typename T>
void sendMessage(const T& message);

// Message senders
void sendIdentityResponse(uint32_t request_id, uint32_t config_id);
void sendConfigurationStored(uint32_t config_id);
void sendConfigurationError(uint32_t config_id);
void sendInputValue(const Sensor::Reading& reading);
void sendHeartbeat();
void sendCalibrationError(uint8_t pin, uint8_t error_code);
void sendEncoderError(uint8_t pin);

} // namespace MessageHandler
