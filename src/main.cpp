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
    ModuleManager::init();
    OutputManager::init();
    MessageHandler::init(&g_packet_serial);

    // Apply loaded configuration to sensors
    uint8_t num_modules = 0;
    const ConfigManager::ModuleConfig* modules = ConfigManager::getCurrentConfig(num_modules);
    ModuleManager::applyConfiguration(modules, num_modules);
}

void loop()
{
    // Update packet serial (processes incoming packets)
    g_packet_serial.update();

    // Update message handler (handles timeouts, etc.)
    MessageHandler::update();
}

// Packet received callback - delegates to message handler
void onPacketReceived(const uint8_t* buffer, size_t size)
{
    MessageHandler::onPacketReceived(buffer, size);
}
