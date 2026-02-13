#include "bldc_lever.h"
#include "board_profiles.h"

using namespace Sensor;

// Mock Arduino functions
static unsigned long mock_millis_value = 0;
unsigned long millis() {
    return mock_millis_value;
}

void pinMode(uint8_t pin, uint8_t mode) {
    // Stub - not used in these tests
}

int analogRead(uint8_t pin) {
    // Stub - not used in these tests
    return 0;
}

int digitalRead(uint8_t pin) {
    // Stub - not used in these tests
    return 0;
}

void digitalWrite(uint8_t pin, uint8_t val) {
    // Stub - not used in these tests
}

void delayMicroseconds(unsigned int us) {
    // Stub - not used in these tests
}

// Include implementation after mocks are defined
#include "../../src/bldc_lever.cpp"
#include <unity.h>

void test_bldc_lever_construction() {
    BLDCLever lever(BoardProfiles::SIMPLEFOC_SHIELD_V2_MEGA);

    TEST_ASSERT_EQUAL(InputType::BLDCLever, lever.getType());
    TEST_ASSERT_FALSE(lever.isCalibrated());
    TEST_ASSERT_FALSE(lever.isProfileActive());
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_bldc_lever_construction);
    return UNITY_END();
}
