// Stub implementation of BLDCLever for testing BLDCManager
// This avoids SimpleFOC dependencies in native tests

#include "bldc_lever.h"

namespace Sensors {

// Minimal stub - only updateMotor() is called by BLDCManager
void BLDCLever::updateMotor() {
    // Stub - do nothing
}

} // namespace Sensors
