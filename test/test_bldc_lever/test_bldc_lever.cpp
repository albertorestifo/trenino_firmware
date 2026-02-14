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
    detents[0].detent_strength = 150;

    detents[1].position_percent = 50;
    detents[1].detent_strength = 150;

    detents[2].position_percent = 100;
    detents[2].detent_strength = 150;

    BLDC::ProfileConfig profile;
    bool result = lever.loadProfile(detents, 3, nullptr, 0, profile);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(lever.isProfileActive());
}

void test_detent_state_tracking() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    BLDC::DetentConfig detents[3];
    detents[0].position_percent = 0;
    detents[0].detent_strength = 150;

    detents[1].position_percent = 50;
    detents[1].detent_strength = 150;

    detents[2].position_percent = 100;
    detents[2].detent_strength = 150;

    BLDC::ProfileConfig profile;
    lever.loadProfile(detents, 3, nullptr, 0, profile);

    Reading reading = lever.getReading();
    TEST_ASSERT_FALSE(reading.has_value);

    lever.updateMotor();

    reading = lever.getReading();
    TEST_ASSERT_TRUE(reading.has_value);
    TEST_ASSERT_EQUAL(InputType::BLDCLever, reading.type);
    TEST_ASSERT_EQUAL(0, reading.value);
}

void test_snap_point_hysteresis() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    BLDC::DetentConfig detents[2];
    detents[0].position_percent = 0;
    detents[0].detent_strength = 200;
    detents[1].position_percent = 100;
    detents[1].detent_strength = 200;

    BLDC::ProfileConfig profile;
    profile.snap_point = 70;
    lever.loadProfile(detents, 2, nullptr, 0, profile);

    MagneticSensorSPI* enc = lever.getEncoder();

    // At detent 0
    enc->setPosition(0);
    mock_millis_value = 100;
    lever.updateMotor();
    TEST_ASSERT_EQUAL(0, lever.getReading().value);

    // 50% of way — should NOT snap (need 70%)
    enc->setPosition(8192);
    mock_millis_value = 101;
    lever.updateMotor();
    TEST_ASSERT_EQUAL(0, lever.getReading().value);

    // 80% of way — SHOULD snap
    enc->setPosition(13106);
    mock_millis_value = 102;
    lever.updateMotor();
    Reading r = lever.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL(1, r.value);
}

void test_linear_range_damping() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    BLDC::DetentConfig detents[3];
    detents[0].position_percent = 0;
    detents[0].detent_strength = 200;
    detents[1].position_percent = 50;
    detents[1].detent_strength = 200;
    detents[2].position_percent = 100;
    detents[2].detent_strength = 200;

    BLDC::LinearRangeConfig ranges[1];
    ranges[0].start_detent_index = 0;
    ranges[0].end_detent_index = 1;
    ranges[0].damping_strength = 128;

    BLDC::ProfileConfig profile;
    lever.loadProfile(detents, 3, ranges, 1, profile);

    MagneticSensorSPI* enc = lever.getEncoder();
    enc->setPosition(0);
    mock_millis_value = 100;
    lever.updateMotor();

    // Move into linear range between detent 0 and 1
    enc->setPosition(4096);
    mock_millis_value = 101;
    lever.updateMotor();

    TEST_ASSERT_EQUAL(0, lever.getReading().value);
    TEST_ASSERT_TRUE(lever.isProfileActive());
}

void test_virtual_endstop() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    BLDC::DetentConfig detents[2];
    detents[0].position_percent = 10;
    detents[0].detent_strength = 200;
    detents[1].position_percent = 90;
    detents[1].detent_strength = 200;

    BLDC::ProfileConfig profile;
    profile.endstop_strength = 200;
    lever.loadProfile(detents, 2, nullptr, 0, profile);

    MagneticSensorSPI* enc = lever.getEncoder();
    uint16_t det0 = static_cast<uint16_t>(16383 * 0.10);
    enc->setPosition(det0);
    mock_millis_value = 100;
    lever.updateMotor();

    // Move beyond detent 0 toward physical endstop
    enc->setPosition(0);
    mock_millis_value = 101;
    lever.updateMotor();

    TEST_ASSERT_TRUE(lever.isProfileActive());
    TEST_ASSERT_EQUAL(0, lever.getReading().value);
}

void test_velocity_cutoff() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    BLDC::DetentConfig detents[1];
    detents[0].position_percent = 50;
    detents[0].detent_strength = 200;

    BLDC::ProfileConfig profile;
    lever.loadProfile(detents, 1, nullptr, 0, profile);

    MagneticSensorSPI* enc = lever.getEncoder();
    enc->setPosition(8192);
    mock_millis_value = 100;
    lever.updateMotor();

    // No crash under any conditions
    mock_millis_value = 101;
    lever.updateMotor();
    TEST_ASSERT_TRUE(lever.isProfileActive());
}

void test_idle_correction() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    BLDC::DetentConfig detents[1];
    detents[0].position_percent = 50;
    detents[0].detent_strength = 200;

    BLDC::ProfileConfig profile;
    lever.loadProfile(detents, 1, nullptr, 0, profile);

    MagneticSensorSPI* enc = lever.getEncoder();
    enc->setPosition(8250); // slightly off from 8192
    mock_millis_value = 100;
    lever.updateMotor();

    // Run many idle iterations past IDLE_CORRECTION_DELAY_MS
    for (uint32_t t = 101; t < 1000; t++) {
        mock_millis_value = t;
        lever.updateMotor();
    }

    TEST_ASSERT_TRUE(lever.isProfileActive());
}

void test_full_detent_traversal() {
    BLDCLever lever(5, 6, 9, 7, 8, 10, 11, 120, 0, 14);
    lever.begin();
    lever.runCalibration();

    // 4 detents: 0%, 33%, 66%, 100%
    BLDC::DetentConfig detents[4];
    detents[0].position_percent = 0;
    detents[0].detent_strength = 200;
    detents[1].position_percent = 33;
    detents[1].detent_strength = 150;
    detents[2].position_percent = 66;
    detents[2].detent_strength = 150;
    detents[3].position_percent = 100;
    detents[3].detent_strength = 200;

    BLDC::ProfileConfig profile;
    profile.snap_point = 55;  // transition just past midpoint
    profile.endstop_strength = 200;

    lever.loadProfile(detents, 4, nullptr, 0, profile);

    MagneticSensorSPI* enc = lever.getEncoder();

    // Start at detent 0
    enc->setPosition(0);
    mock_millis_value = 100;
    lever.updateMotor();
    TEST_ASSERT_EQUAL(0, lever.getReading().value);

    // Move to 60% of way to detent 1 (above snap_point of 0.55)
    // Detent 1 at 33% = position 5406. 60% of 5406 = 3244
    enc->setPosition(3244);
    mock_millis_value = 101;
    lever.updateMotor();
    TEST_ASSERT_EQUAL(1, lever.getReading().value); // snapped to detent 1

    // Move toward detent 2 (66% = 10813)
    // Distance from detent 1 (5406) to detent 2 (10813) = 5407
    // 55% of 5407 = 2974, so threshold at 5406 + 2974 = 8380
    enc->setPosition(8500);
    mock_millis_value = 102;
    lever.updateMotor();
    TEST_ASSERT_EQUAL(2, lever.getReading().value); // snapped to detent 2

    // Move to detent 3 (100% = 16383)
    enc->setPosition(15000);
    mock_millis_value = 103;
    lever.updateMotor();
    TEST_ASSERT_EQUAL(3, lever.getReading().value); // snapped to detent 3

    // Move back — need to cross snap threshold going backward
    // From detent 3 (16383) back toward detent 2 (10813)
    // Distance = 5570, 55% = 3064, threshold at 16383 - 3064 = 13319
    enc->setPosition(13000);
    mock_millis_value = 104;
    lever.updateMotor();
    TEST_ASSERT_EQUAL(2, lever.getReading().value); // snapped back to detent 2
}

void setUp() {
    mock_millis_value = 0;
}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_bldc_lever_construction);
    RUN_TEST(test_bldc_lever_custom_params);
    RUN_TEST(test_calibration_success);
    RUN_TEST(test_load_profile);
    RUN_TEST(test_detent_state_tracking);
    RUN_TEST(test_snap_point_hysteresis);
    RUN_TEST(test_linear_range_damping);
    RUN_TEST(test_virtual_endstop);
    RUN_TEST(test_velocity_cutoff);
    RUN_TEST(test_idle_correction);
    RUN_TEST(test_full_detent_traversal);
    return UNITY_END();
}
