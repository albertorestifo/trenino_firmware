// Stub implementation of BLDCLever for testing BLDCManager
// This avoids SimpleFOC dependencies in native tests

#include "bldc_lever.h"

namespace Sensor {

// Minimal stub - only updateMotor() is called by BLDCManager
void BLDCLever::updateMotor() {
    // Stub - do nothing
}

} // namespace Sensor
