#include <unity.h>
#include "protocol.h"

void test_calibration_error_encode_decode() {
    Protocol::CalibrationError msg_out;
    msg_out.pin = 100;
    msg_out.error_code = 1;

    uint8_t buffer[16];
    size_t len = msg_out.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL_UINT8(3, len);
    TEST_ASSERT_EQUAL_UINT8(Protocol::MESSAGE_TYPE_CALIBRATION_ERROR, buffer[0]);

    Protocol::CalibrationError msg_in;
    bool result = msg_in.decode(buffer, len);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(100, msg_in.pin);
    TEST_ASSERT_EQUAL_UINT8(1, msg_in.error_code);
}

void test_retry_calibration_encode_decode() {
    Protocol::RetryCalibration msg_out;
    msg_out.pin = 100;

    uint8_t buffer[16];
    size_t len = msg_out.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL_UINT8(2, len);

    Protocol::RetryCalibration msg_in;
    bool result = msg_in.decode(buffer, len);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(100, msg_in.pin);
}

void test_load_bldc_profile_header() {
    Protocol::LoadBLDCProfile msg_out;
    msg_out.pin = 100;
    msg_out.num_detents = 5;
    msg_out.num_linear_ranges = 2;

    uint8_t buffer[16];
    size_t len = msg_out.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL_UINT8(4, len);

    Protocol::LoadBLDCProfile msg_in;
    bool result = msg_in.decode(buffer, len);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(100, msg_in.pin);
    TEST_ASSERT_EQUAL_UINT8(5, msg_in.num_detents);
    TEST_ASSERT_EQUAL_UINT8(2, msg_in.num_linear_ranges);
}

void setUp() {}
void tearDown() {}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_calibration_error_encode_decode);
    RUN_TEST(test_retry_calibration_encode_decode);
    RUN_TEST(test_load_bldc_profile_header);
    UNITY_END();
    return 0;
}
