#include "../../src/protocol.h"
#include <string.h>
#include <unity.h>

using namespace Protocol;

void setUp() {}
void tearDown() {}

// ---------- WriteSegments ----------

void test_write_segments_encode()
{
    WriteSegments msg;
    msg.i2c_address = 0x70;
    msg.num_bytes = 4;
    msg.data[0] = 0x11;
    msg.data[1] = 0x22;
    msg.data[2] = 0x33;
    msg.data[3] = 0x44;

    uint8_t buffer[32];
    size_t size = msg.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(7, size);
    TEST_ASSERT_EQUAL_UINT8(MESSAGE_TYPE_WRITE_SEGMENTS, buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(0x70, buffer[1]);
    TEST_ASSERT_EQUAL_UINT8(4, buffer[2]);
    TEST_ASSERT_EQUAL_UINT8(0x11, buffer[3]);
    TEST_ASSERT_EQUAL_UINT8(0x22, buffer[4]);
    TEST_ASSERT_EQUAL_UINT8(0x33, buffer[5]);
    TEST_ASSERT_EQUAL_UINT8(0x44, buffer[6]);
}

void test_write_segments_decode()
{
    uint8_t buffer[] = { MESSAGE_TYPE_WRITE_SEGMENTS, 0x71, 0x02, 0xAB, 0xCD };

    WriteSegments msg;
    bool ok = msg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(0x71, msg.i2c_address);
    TEST_ASSERT_EQUAL_UINT8(2, msg.num_bytes);
    TEST_ASSERT_EQUAL_UINT8(0xAB, msg.data[0]);
    TEST_ASSERT_EQUAL_UINT8(0xCD, msg.data[1]);
}

void test_write_segments_decode_insufficient_data()
{
    uint8_t buffer[] = { MESSAGE_TYPE_WRITE_SEGMENTS, 0x70, 0x04, 0x01, 0x02 };

    WriteSegments msg;
    TEST_ASSERT_FALSE(msg.decode(buffer, sizeof(buffer)));
}

void test_write_segments_decode_too_many_bytes()
{
    uint8_t buffer[20];
    buffer[0] = MESSAGE_TYPE_WRITE_SEGMENTS;
    buffer[1] = 0x70;
    buffer[2] = 17; // > MAX_SEGMENT_BYTES (16)
    for (uint8_t i = 0; i < 17; i++) buffer[3 + i] = i;

    WriteSegments msg;
    TEST_ASSERT_FALSE(msg.decode(buffer, sizeof(buffer)));
}

void test_write_segments_roundtrip()
{
    WriteSegments original;
    original.i2c_address = 0x77;
    original.num_bytes = 8;
    for (uint8_t i = 0; i < 8; i++) original.data[i] = 0xA0 + i;

    uint8_t buffer[32];
    size_t size = original.encode(buffer, sizeof(buffer));

    WriteSegments decoded;
    TEST_ASSERT_TRUE(decoded.decode(buffer, size));
    TEST_ASSERT_EQUAL_UINT8(original.i2c_address, decoded.i2c_address);
    TEST_ASSERT_EQUAL_UINT8(original.num_bytes, decoded.num_bytes);
    for (uint8_t i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL_UINT8(original.data[i], decoded.data[i]);
    }
}

void test_message_decode_write_segments()
{
    uint8_t buffer[] = { MESSAGE_TYPE_WRITE_SEGMENTS, 0x70, 0x02, 0xAB, 0xCD };

    Message msg;
    TEST_ASSERT_TRUE(msg.decode(buffer, sizeof(buffer)));
    TEST_ASSERT_TRUE(msg.isWriteSegments());
    TEST_ASSERT_EQUAL_UINT8(0x70, msg.write_segments.i2c_address);
    TEST_ASSERT_EQUAL_UINT8(2, msg.write_segments.num_bytes);
    TEST_ASSERT_EQUAL_UINT8(0xAB, msg.write_segments.data[0]);
    TEST_ASSERT_EQUAL_UINT8(0xCD, msg.write_segments.data[1]);
}

// ---------- HT16K33 Configure (already plumbed in C1, just verify here) ----------

void test_configure_ht16k33_encode()
{
    Configure cfg;
    cfg.config_id = 0x12345678;
    cfg.total_parts = 1;
    cfg.part_number = 0;
    cfg.module_type = MODULE_TYPE_HT16K33;
    cfg.ht16k33.i2c_address = 0x70;
    cfg.ht16k33.brightness = 8;
    cfg.ht16k33.num_digits = 4;

    uint8_t buffer[32];
    size_t size = cfg.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(11, size); // header(8) + 3 payload
    TEST_ASSERT_EQUAL_UINT8(MODULE_TYPE_HT16K33, buffer[7]);
    TEST_ASSERT_EQUAL_UINT8(0x70, buffer[8]);
    TEST_ASSERT_EQUAL_UINT8(8, buffer[9]);
    TEST_ASSERT_EQUAL_UINT8(4, buffer[10]);
}

void test_configure_ht16k33_roundtrip()
{
    Configure original;
    original.config_id = 0xDEADBEEF;
    original.total_parts = 2;
    original.part_number = 1;
    original.module_type = MODULE_TYPE_HT16K33;
    original.ht16k33.i2c_address = 0x71;
    original.ht16k33.brightness = 12;
    original.ht16k33.num_digits = 8;

    uint8_t buffer[32];
    size_t size = original.encode(buffer, sizeof(buffer));

    Configure decoded;
    TEST_ASSERT_TRUE(decoded.decode(buffer, size));
    TEST_ASSERT_EQUAL_UINT8(MODULE_TYPE_HT16K33, decoded.module_type);
    TEST_ASSERT_EQUAL_UINT8(0x71, decoded.ht16k33.i2c_address);
    TEST_ASSERT_EQUAL_UINT8(12, decoded.ht16k33.brightness);
    TEST_ASSERT_EQUAL_UINT8(8, decoded.ht16k33.num_digits);
}

// ---------- SetModuleBrightness ----------

void test_set_module_brightness_encode()
{
    SetModuleBrightness msg;
    msg.i2c_address = 0x70;
    msg.brightness = 8;

    uint8_t buffer[8];
    size_t size = msg.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(3, size);
    TEST_ASSERT_EQUAL_UINT8(MESSAGE_TYPE_SET_MODULE_BRIGHTNESS, buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(0x70, buffer[1]);
    TEST_ASSERT_EQUAL_UINT8(8, buffer[2]);
}

void test_set_module_brightness_decode()
{
    uint8_t buffer[] = { MESSAGE_TYPE_SET_MODULE_BRIGHTNESS, 0x71, 0x0F };

    SetModuleBrightness msg;
    TEST_ASSERT_TRUE(msg.decode(buffer, sizeof(buffer)));
    TEST_ASSERT_EQUAL_UINT8(0x71, msg.i2c_address);
    TEST_ASSERT_EQUAL_UINT8(15, msg.brightness);
}

void test_set_module_brightness_roundtrip()
{
    SetModuleBrightness original;
    original.i2c_address = 0x77;
    original.brightness = 5;

    uint8_t buffer[8];
    size_t size = original.encode(buffer, sizeof(buffer));

    SetModuleBrightness decoded;
    TEST_ASSERT_TRUE(decoded.decode(buffer, size));
    TEST_ASSERT_EQUAL_UINT8(original.i2c_address, decoded.i2c_address);
    TEST_ASSERT_EQUAL_UINT8(original.brightness, decoded.brightness);
}

void test_message_decode_set_module_brightness()
{
    uint8_t buffer[] = { MESSAGE_TYPE_SET_MODULE_BRIGHTNESS, 0x70, 0x08 };

    Message msg;
    TEST_ASSERT_TRUE(msg.decode(buffer, sizeof(buffer)));
    TEST_ASSERT_TRUE(msg.isSetModuleBrightness());
    TEST_ASSERT_EQUAL_UINT8(0x70, msg.set_module_brightness.i2c_address);
    TEST_ASSERT_EQUAL_UINT8(8, msg.set_module_brightness.brightness);
}

// ---------- ModuleError ----------

void test_module_error_encode()
{
    ModuleError msg;
    msg.i2c_address = 0x70;
    msg.error_code = 0;

    uint8_t buffer[8];
    size_t size = msg.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(3, size);
    TEST_ASSERT_EQUAL_UINT8(MESSAGE_TYPE_MODULE_ERROR, buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(0x70, buffer[1]);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[2]);
}

void test_module_error_decode()
{
    uint8_t buffer[] = { MESSAGE_TYPE_MODULE_ERROR, 0x77, 0x00 };

    ModuleError msg;
    TEST_ASSERT_TRUE(msg.decode(buffer, sizeof(buffer)));
    TEST_ASSERT_EQUAL_UINT8(0x77, msg.i2c_address);
    TEST_ASSERT_EQUAL_UINT8(0, msg.error_code);
}

void test_module_error_roundtrip()
{
    ModuleError original;
    original.i2c_address = 0x71;
    original.error_code = 0;

    uint8_t buffer[8];
    size_t size = original.encode(buffer, sizeof(buffer));

    ModuleError decoded;
    TEST_ASSERT_TRUE(decoded.decode(buffer, size));
    TEST_ASSERT_EQUAL_UINT8(original.i2c_address, decoded.i2c_address);
    TEST_ASSERT_EQUAL_UINT8(original.error_code, decoded.error_code);
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_write_segments_encode);
    RUN_TEST(test_write_segments_decode);
    RUN_TEST(test_write_segments_decode_insufficient_data);
    RUN_TEST(test_write_segments_decode_too_many_bytes);
    RUN_TEST(test_write_segments_roundtrip);
    RUN_TEST(test_message_decode_write_segments);

    RUN_TEST(test_configure_ht16k33_encode);
    RUN_TEST(test_configure_ht16k33_roundtrip);

    RUN_TEST(test_set_module_brightness_encode);
    RUN_TEST(test_set_module_brightness_decode);
    RUN_TEST(test_set_module_brightness_roundtrip);
    RUN_TEST(test_message_decode_set_module_brightness);

    RUN_TEST(test_module_error_encode);
    RUN_TEST(test_module_error_decode);
    RUN_TEST(test_module_error_roundtrip);

    return UNITY_END();
}
