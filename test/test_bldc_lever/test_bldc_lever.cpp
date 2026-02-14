#include "bldc_lever.h"

using namespace Sensor;

// Mock Arduino functions
static unsigned long mock_millis_value = 0;
unsigned long millis() {
    return mock_millis_value;
}

void pinMode(uint8_t pin, uint8_t mode) {}
int analogRead(uint8_t pin) { return 0; }
int digitalRead(uint8_t pin) { return 0; }
void digitalWrite(uint8_t pin, uint8_t val) {}
void delayMicroseconds(unsigned int us) {}

// Include implementation after mocks are defined
#include "../../src/bldc_lever.cpp"
#include <unity.h>

void test_bldc_lever_construction() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);

    TEST_ASSERT_EQUAL(InputType::BLDCLever, lever.getType());
    TEST_ASSERT_EQUAL_UINT8(10, lever.getPin());  // encoder_cs is the identifying pin
    TEST_ASSERT_FALSE(lever.isCalibrated());
    TEST_ASSERT_FALSE(lever.isProfileActive());
}

void test_bldc_lever_custom_params() {
    // Different hardware: 7 pole pairs, 24V, 3A limit, 12-bit encoder
    BLDCLever lever(3, 5, 6, 7, 8, 15, 7, 240, 30, 12);

    TEST_ASSERT_EQUAL(InputType::BLDCLever, lever.getType());
    TEST_ASSERT_EQUAL_UINT8(15, lever.getPin());  // encoder_cs pin
}

void test_calibration_success() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();

    bool result = lever.runCalibration();

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(lever.isCalibrated());
}

void test_load_profile() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    BLDC::DetentConfig detents[3];
    detents[0].position_percent = 0;
    detents[0].engagement_strength = 100;
    detents[0].hold_strength = 150;
    detents[0].exit_strength = 100;
    detents[0].spring_back_target = 255;

    detents[1].position_percent = 50;
    detents[1].engagement_strength = 100;
    detents[1].hold_strength = 150;
    detents[1].exit_strength = 100;
    detents[1].spring_back_target = 255;

    detents[2].position_percent = 100;
    detents[2].engagement_strength = 100;
    detents[2].hold_strength = 150;
    detents[2].exit_strength = 100;
    detents[2].spring_back_target = 255;

    bool result = lever.loadProfile(detents, 3, nullptr, 0);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(lever.isProfileActive());
}

void test_detent_state_tracking() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    BLDC::DetentConfig detents[3];
    detents[0].position_percent = 0;
    detents[0].engagement_strength = 100;
    detents[0].hold_strength = 150;
    detents[0].exit_strength = 100;
    detents[0].spring_back_target = 255;

    detents[1].position_percent = 50;
    detents[1].engagement_strength = 100;
    detents[1].hold_strength = 150;
    detents[1].exit_strength = 100;
    detents[1].spring_back_target = 255;

    detents[2].position_percent = 100;
    detents[2].engagement_strength = 100;
    detents[2].hold_strength = 150;
    detents[2].exit_strength = 100;
    detents[2].spring_back_target = 255;

    lever.loadProfile(detents, 3, nullptr, 0);

    Reading reading = lever.getReading();
    TEST_ASSERT_FALSE(reading.has_value);

    lever.updateMotor();

    reading = lever.getReading();
    TEST_ASSERT_TRUE(reading.has_value);
    TEST_ASSERT_EQUAL(InputType::BLDCLever, reading.type);
    TEST_ASSERT_EQUAL(0, reading.value);
}

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_bldc_lever_construction);
    RUN_TEST(test_bldc_lever_custom_params);
    RUN_TEST(test_calibration_success);
    RUN_TEST(test_load_profile);
    RUN_TEST(test_detent_state_tracking);
    return UNITY_END();
}
