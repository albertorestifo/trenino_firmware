#pragma once

#include <stdint.h>

class BLDCLever; // Forward declaration

namespace BLDCManager {

// Initialize the BLDC manager
void init();

// Register a BLDC lever for motor control updates
void registerLever(BLDCLever* lever);

// Unregister a BLDC lever
void unregisterLever(BLDCLever* lever);

// Update all registered levers' motor control (called at 1kHz from main loop)
void updateMotorControl();

// Get number of registered levers
uint8_t getLeverCount();

} // namespace BLDCManager
