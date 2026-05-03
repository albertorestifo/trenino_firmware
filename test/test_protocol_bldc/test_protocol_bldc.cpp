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

void test_configure_bldc_lever_encode() {
    Protocol::Configure cfg;
    cfg.config_id = 0x00000001;
    cfg.total_parts = 1;
    cfg.part_number = 0;
    cfg.input_type = Protocol::MODULE_TYPE_BLDC_LEVER;
    cfg.bldc_lever.motor_pin_a = 5;
    cfg.bldc_lever.motor_pin_b = 6;
    cfg.bldc_lever.motor_pin_c = 9;
    cfg.bldc_lever.motor_enable = 7;
    cfg.bldc_lever.encoder_cs = 10;
    cfg.bldc_lever.pole_pairs = 11;
    cfg.bldc_lever.voltage = 120;       // 12.0V
    cfg.bldc_lever.current_limit = 0;   // no limit
    cfg.bldc_lever.encoder_bits = 14;

    uint8_t buffer[64];
    size_t size = cfg.encode(buffer, sizeof(buffer));

    // header(8) + 9 payload bytes = 17
    TEST_ASSERT_EQUAL(17, size);
    TEST_ASSERT_EQUAL_UINT8(Protocol::MESSAGE_TYPE_CONFIGURE, buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(Protocol::MODULE_TYPE_BLDC_LEVER, buffer[7]);
    TEST_ASSERT_EQUAL_UINT8(5, buffer[8]);    // motor_pin_a
    TEST_ASSERT_EQUAL_UINT8(6, buffer[9]);    // motor_pin_b
    TEST_ASSERT_EQUAL_UINT8(9, buffer[10]);   // motor_pin_c
    TEST_ASSERT_EQUAL_UINT8(7, buffer[11]);   // motor_enable
    TEST_ASSERT_EQUAL_UINT8(10, buffer[12]);  // encoder_cs
    TEST_ASSERT_EQUAL_UINT8(11, buffer[13]);  // pole_pairs
    TEST_ASSERT_EQUAL_UINT8(120, buffer[14]); // voltage
    TEST_ASSERT_EQUAL_UINT8(0, buffer[15]);   // current_limit
    TEST_ASSERT_EQUAL_UINT8(14, buffer[16]);  // encoder_bits
}

void test_configure_bldc_lever_decode() {
    uint8_t buffer[] = {
        Protocol::MESSAGE_TYPE_CONFIGURE,
        0x01, 0x00, 0x00, 0x00,  // config_id
        0x01,                     // total_parts
        0x00,                     // part_number
        Protocol::MODULE_TYPE_BLDC_LEVER,
        5, 6, 9,                  // motor pins A/B/C
        7,                        // enable pin
        10,                       // encoder_cs
        11,                       // pole_pairs
        120,                      // voltage (12.0V)
        15,                       // current_limit (1.5A)
        14                        // encoder_bits
    };

    Protocol::Configure cfg;
    bool result = cfg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(Protocol::MODULE_TYPE_BLDC_LEVER, cfg.input_type);
    TEST_ASSERT_EQUAL_UINT8(5, cfg.bldc_lever.motor_pin_a);
    TEST_ASSERT_EQUAL_UINT8(6, cfg.bldc_lever.motor_pin_b);
    TEST_ASSERT_EQUAL_UINT8(9, cfg.bldc_lever.motor_pin_c);
    TEST_ASSERT_EQUAL_UINT8(7, cfg.bldc_lever.motor_enable);
    TEST_ASSERT_EQUAL_UINT8(10, cfg.bldc_lever.encoder_cs);
    TEST_ASSERT_EQUAL_UINT8(11, cfg.bldc_lever.pole_pairs);
    TEST_ASSERT_EQUAL_UINT8(120, cfg.bldc_lever.voltage);
    TEST_ASSERT_EQUAL_UINT8(15, cfg.bldc_lever.current_limit);
    TEST_ASSERT_EQUAL_UINT8(14, cfg.bldc_lever.encoder_bits);
}

void test_configure_bldc_lever_roundtrip() {
    Protocol::Configure original;
    original.config_id = 0xDEADBEEF;
    original.total_parts = 2;
    original.part_number = 1;
    original.input_type = Protocol::MODULE_TYPE_BLDC_LEVER;
    original.bldc_lever.motor_pin_a = 3;
    original.bldc_lever.motor_pin_b = 5;
    original.bldc_lever.motor_pin_c = 6;
    original.bldc_lever.motor_enable = 7;
    original.bldc_lever.encoder_cs = 10;
    original.bldc_lever.pole_pairs = 7;
    original.bldc_lever.voltage = 240;       // 24.0V
    original.bldc_lever.current_limit = 30;  // 3.0A
    original.bldc_lever.encoder_bits = 12;

    uint8_t buffer[64];
    size_t size = original.encode(buffer, sizeof(buffer));

    Protocol::Configure decoded;
    bool result = decoded.decode(buffer, size);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.motor_pin_a, decoded.bldc_lever.motor_pin_a);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.motor_pin_b, decoded.bldc_lever.motor_pin_b);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.motor_pin_c, decoded.bldc_lever.motor_pin_c);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.motor_enable, decoded.bldc_lever.motor_enable);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.encoder_cs, decoded.bldc_lever.encoder_cs);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.pole_pairs, decoded.bldc_lever.pole_pairs);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.voltage, decoded.bldc_lever.voltage);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.current_limit, decoded.bldc_lever.current_limit);
    TEST_ASSERT_EQUAL_UINT8(original.bldc_lever.encoder_bits, decoded.bldc_lever.encoder_bits);
}

void test_configure_bldc_lever_decode_insufficient_data() {
    uint8_t buffer[] = {
        Protocol::MESSAGE_TYPE_CONFIGURE,
        0x01, 0x00, 0x00, 0x00,
        0x01, 0x00,
        Protocol::MODULE_TYPE_BLDC_LEVER,
        5, 6, 9  // Only 3 bytes, need 9
    };

    Protocol::Configure cfg;
    bool result = cfg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_FALSE(result);
}

void setUp() {}
void tearDown() {}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_load_bldc_profile_header);
    RUN_TEST(test_load_bldc_profile_decode_insufficient_data);
    RUN_TEST(test_configure_bldc_lever_encode);
    RUN_TEST(test_configure_bldc_lever_decode);
    RUN_TEST(test_configure_bldc_lever_roundtrip);
    RUN_TEST(test_configure_bldc_lever_decode_insufficient_data);
    UNITY_END();
    return 0;
}
