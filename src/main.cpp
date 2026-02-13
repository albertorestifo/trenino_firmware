#include "bldc_manager.h"
#include "config_manager.h"
#include "message_handler.h"
#include "output_manager.h"
#include "sensor_manager.h"
#include <Arduino.h>
#include <PacketSerial.h>

// Global packet serial instance
PacketSerial_<COBS> g_packet_serial;

// Forward declaration for packet callback
void onPacketReceived(const uint8_t* buffer, size_t size);

void setup()
{
    // Initialize serial communication
    g_packet_serial.begin(115200);
    g_packet_serial.setPacketHandler(&onPacketReceived);

    // Initialize subsystems
    ConfigManager::init();
    SensorManager::init();
    BLDCManager::init();
    OutputManager::init();
    MessageHandler::init(&g_packet_serial);

    // Apply loaded configuration to sensors
    uint8_t num_inputs = 0;
    const ConfigManager::InputConfig* inputs = ConfigManager::getCurrentConfig(num_inputs);
    SensorManager::applyConfiguration(inputs, num_inputs);
}

void loop()
{
    // Update packet serial (processes incoming packets)
    g_packet_serial.update();

    // Update motor control at 1kHz
    BLDCManager::updateMotorControl();

    // Update message handler (handles timeouts, etc.)
    MessageHandler::update();

    // Control loop rate: delay to achieve ~1 kHz loop rate for motor control
    delay(1);
}

// Packet received callback - delegates to message handler
void onPacketReceived(const uint8_t* buffer, size_t size)
{
    MessageHandler::onPacketReceived(buffer, size);
}
