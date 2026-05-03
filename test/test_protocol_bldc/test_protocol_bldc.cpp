#include <unity.h>
#include "protocol.h"

void test_load_bldc_profile_header() {
    Protocol::LoadBLDCProfile msg_out;
    msg_out.pin = 100;
    msg_out.num_detents = 5;
    msg_out.num_linear_ranges = 2;
    msg_out.snap_point = 70;
    msg_out.endstop_strength = 200;
    msg_out.position_start = 1500;   // 1.5 rad
    msg_out.position_end = -500;     // -0.5 rad

    uint8_t buffer[16];
    size_t len = msg_out.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL_UINT8(10, len);

    Protocol::LoadBLDCProfile msg_in;
    bool result = msg_in.decode(buffer, len);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(100, msg_in.pin);
    TEST_ASSERT_EQUAL_UINT8(5, msg_in.num_detents);
    TEST_ASSERT_EQUAL_UINT8(2, msg_in.num_linear_ranges);
    TEST_ASSERT_EQUAL_UINT8(70, msg_in.snap_point);
    TEST_ASSERT_EQUAL_UINT8(200, msg_in.endstop_strength);
    TEST_ASSERT_EQUAL_INT16(1500, msg_in.position_start);
    TEST_ASSERT_EQUAL_INT16(-500, msg_in.position_end);
}

void test_load_bldc_profile_decode_insufficient_data() {
    uint8_t buffer[] = {
        Protocol::MESSAGE_TYPE_LOAD_BLDC_PROFILE,
        100,  // pin
        5,    // num_detents
        2,    // num_linear_ranges
        70,   // snap_point
        200,  // endstop_strength
        // Missing position_start and position_end
    };

    Protocol::LoadBLDCProfile msg;
    bool result = msg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_FALSE(result);
}

void setUp() {}
void tearDown() {}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_load_bldc_profile_header);
    RUN_TEST(test_load_bldc_profile_decode_insufficient_data);
    UNITY_END();
    return 0;
}
