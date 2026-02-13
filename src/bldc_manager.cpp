#include "bldc_manager.h"
#include "bldc_lever.h"

namespace BLDCManager {

// Maximum number of BLDC levers
constexpr uint8_t MAX_LEVERS = 4;

// Array of registered levers
static Sensor::BLDCLever* g_levers[MAX_LEVERS];
static uint8_t g_lever_count = 0;

void init() {
    for (uint8_t i = 0; i < MAX_LEVERS; i++) {
        g_levers[i] = nullptr;
    }
    g_lever_count = 0;
}

void registerLever(Sensor::BLDCLever* lever) {
    if (lever == nullptr) return;
    if (g_lever_count >= MAX_LEVERS) return;

    // Check if already registered
    for (uint8_t i = 0; i < g_lever_count; i++) {
        if (g_levers[i] == lever) return;
    }

    g_levers[g_lever_count++] = lever;
}

void unregisterLever(Sensor::BLDCLever* lever) {
    if (lever == nullptr) return;

    // Find and remove
    for (uint8_t i = 0; i < g_lever_count; i++) {
        if (g_levers[i] == lever) {
            // Shift remaining levers down
            for (uint8_t j = i; j < g_lever_count - 1; j++) {
                g_levers[j] = g_levers[j + 1];
            }
            g_levers[g_lever_count - 1] = nullptr;
            g_lever_count--;
            return;
        }
    }
}

void updateMotorControl() {
    for (uint8_t i = 0; i < g_lever_count; i++) {
        if (g_levers[i] != nullptr) {
            g_levers[i]->updateMotor();
        }
    }
}

uint8_t getLeverCount() {
    return g_lever_count;
}

} // namespace BLDCManager
