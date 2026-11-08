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

void test_empty_configure_canonical_wire()
{
    const uint8_t expected[] = {2,0x78,0x56,0x34,0x12,0,0,0xff};
    Configure cfg; cfg.config_id = 0x12345678; cfg.total_parts = 0; cfg.part_number = 0; cfg.module_type = 0xff;
    uint8_t encoded[9] = {};
    TEST_ASSERT_EQUAL(8,cfg.encode(encoded,sizeof(encoded)));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected,encoded,8);
    Message decoded;
    TEST_ASSERT_TRUE(decoded.decode(expected,sizeof(expected)));
    TEST_ASSERT_EQUAL_HEX32(0x12345678,decoded.configure.config_id);
    TEST_ASSERT_EQUAL(0,decoded.configure.total_parts);
    TEST_ASSERT_EQUAL(0xff,decoded.configure.module_type);
    TEST_ASSERT_EQUAL(0,cfg.encode(encoded,7));
}
void test_empty_configure_rejects_noncanonical_header_and_lengths()
{
    uint8_t b[] = {2,1,0,0,0,0,0,0xff,0};
    Configure cfg;
    TEST_ASSERT_FALSE(cfg.decode(b,7));
    TEST_ASSERT_FALSE(cfg.decode(b,9));
    b[5] = 1; TEST_ASSERT_FALSE(cfg.decode(b,8));
    b[5] = 0; b[6] = 1; TEST_ASSERT_FALSE(cfg.decode(b,8));
    b[6] = 0; b[7] = 0; TEST_ASSERT_FALSE(cfg.decode(b,8));
    cfg.total_parts = 1; cfg.module_type = 0xff; TEST_ASSERT_EQUAL(0,cfg.encode(b,sizeof(b)));
    cfg.total_parts = 0; cfg.part_number = 1; TEST_ASSERT_EQUAL(0,cfg.encode(b,sizeof(b)));
}

// ---------- Gauge (module type 7, messages 18 and 19) ----------

static GaugeConfig validVid()
{
    GaugeConfig g;
    g.driver = GAUGE_DRIVER_VID6606;
    g.pins[0] = 8; g.pins[1] = 7; g.pins[2] = 6; g.pins[3] = GAUGE_NO_PIN;
    g.travel = 3780; g.max_speed = 5000; g.acceleration = 50000; g.flags = 0;
    return g;
}

static GaugeConfig validDirect()
{
    GaugeConfig g;
    g.driver = GAUGE_DRIVER_DIRECT;
    g.pins[0] = 2; g.pins[1] = 3; g.pins[2] = 4; g.pins[3] = 5;
    g.travel = 945; g.max_speed = 1800; g.acceleration = 1500; g.flags = GAUGE_FLAG_REVERSE;
    return g;
}

void test_configure_gauge_wire_layout()
{
    Configure cfg;
    cfg.config_id = 0x01020304; cfg.total_parts = 1; cfg.part_number = 0;
    cfg.module_type = MODULE_TYPE_GAUGE; cfg.gauge = validVid();
    uint8_t b[32];
    TEST_ASSERT_EQUAL(20, cfg.encode(b, sizeof(b)));
    const uint8_t expected[] = { 2, 4, 3, 2, 1, 1, 0, 7,
        0, 8, 7, 6, 0xff, 0xC4, 0x0E, 0x88, 0x13, 0x50, 0xC3, 0 };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, b, 20);
}

void test_configure_gauge_roundtrip_and_exact_length()
{
    Configure cfg;
    cfg.config_id = 9; cfg.total_parts = 2; cfg.part_number = 1;
    cfg.module_type = MODULE_TYPE_GAUGE; cfg.gauge = validDirect();
    uint8_t b[32];
    size_t n = cfg.encode(b, sizeof(b));
    Configure out;
    TEST_ASSERT_TRUE(out.decode(b, n));
    TEST_ASSERT_EQUAL_UINT8(GAUGE_DRIVER_DIRECT, out.gauge.driver);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(cfg.gauge.pins, out.gauge.pins, 4);
    TEST_ASSERT_EQUAL_UINT16(945, out.gauge.travel);
    TEST_ASSERT_EQUAL_UINT16(1800, out.gauge.max_speed);
    TEST_ASSERT_EQUAL_UINT16(1500, out.gauge.acceleration);
    TEST_ASSERT_EQUAL_UINT8(GAUGE_FLAG_REVERSE, out.gauge.flags);
    TEST_ASSERT_FALSE(out.decode(b, n - 1));
    TEST_ASSERT_FALSE(out.decode(b, n + 1));
}

void test_gauge_config_validation()
{
    TEST_ASSERT_TRUE(isValidGaugeConfig(validVid()));
    TEST_ASSERT_TRUE(isValidGaugeConfig(validDirect()));

    GaugeConfig g = validVid(); g.pins[2] = GAUGE_NO_PIN; TEST_ASSERT_TRUE(isValidGaugeConfig(g)); // no RESET pin
    g = validVid(); g.driver = 2; TEST_ASSERT_FALSE(isValidGaugeConfig(g));
    g = validVid(); g.flags = 2; TEST_ASSERT_FALSE(isValidGaugeConfig(g));
    g = validVid(); g.travel = 0; TEST_ASSERT_FALSE(isValidGaugeConfig(g));
    g = validVid(); g.travel = GAUGE_MAX_TRAVEL + 1; TEST_ASSERT_FALSE(isValidGaugeConfig(g));
    g = validVid(); g.max_speed = 0; TEST_ASSERT_FALSE(isValidGaugeConfig(g));
    g = validVid(); g.max_speed = GAUGE_MAX_SPEED_VID6606 + 1; TEST_ASSERT_FALSE(isValidGaugeConfig(g));
    g = validVid(); g.acceleration = 0; TEST_ASSERT_FALSE(isValidGaugeConfig(g));
    g = validVid(); g.acceleration = GAUGE_MAX_ACCELERATION + 1; TEST_ASSERT_FALSE(isValidGaugeConfig(g));
    g = validVid(); g.pins[0] = GAUGE_NO_PIN; TEST_ASSERT_FALSE(isValidGaugeConfig(g));
    g = validVid(); g.pins[1] = GAUGE_NO_PIN; TEST_ASSERT_FALSE(isValidGaugeConfig(g));
    g = validVid(); g.pins[3] = 9; TEST_ASSERT_FALSE(isValidGaugeConfig(g));
    g = validVid(); g.pins[1] = 8; TEST_ASSERT_FALSE(isValidGaugeConfig(g)); // STEP == DIR
    g = validVid(); g.pins[2] = 7; TEST_ASSERT_FALSE(isValidGaugeConfig(g)); // RESET == DIR

    g = validDirect(); g.max_speed = GAUGE_MAX_SPEED_DIRECT + 1; TEST_ASSERT_FALSE(isValidGaugeConfig(g));
    g = validDirect(); g.pins[3] = GAUGE_NO_PIN; TEST_ASSERT_FALSE(isValidGaugeConfig(g));
    g = validDirect(); g.pins[3] = 2; TEST_ASSERT_FALSE(isValidGaugeConfig(g)); // duplicate
}

void test_set_gauge_position_roundtrip()
{
    SetGaugePosition msg; msg.pin = 21; msg.position = 0x1234;
    uint8_t b[8];
    TEST_ASSERT_EQUAL(4, msg.encode(b, sizeof(b)));
    const uint8_t expected[] = { MESSAGE_TYPE_SET_GAUGE_POSITION, 21, 0x34, 0x12 };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, b, 4);
    Message m;
    TEST_ASSERT_TRUE(m.decode(b, 4));
    TEST_ASSERT_TRUE(m.isSetGaugePosition());
    TEST_ASSERT_EQUAL_UINT8(21, m.set_gauge_position.pin);
    TEST_ASSERT_EQUAL_UINT16(0x1234, m.set_gauge_position.position);
    TEST_ASSERT_FALSE(m.decode(b, 3));
    TEST_ASSERT_FALSE(m.decode(b, 5));
}

void test_zero_gauge_roundtrip()
{
    ZeroGauge msg; msg.pin = 8;
    uint8_t b[4];
    TEST_ASSERT_EQUAL(2, msg.encode(b, sizeof(b)));
    Message m;
    TEST_ASSERT_TRUE(m.decode(b, 2));
    TEST_ASSERT_TRUE(m.isZeroGauge());
    TEST_ASSERT_EQUAL_UINT8(8, m.zero_gauge.pin);
    TEST_ASSERT_FALSE(m.decode(b, 3));
}

void test_removed_stepper_message_types_are_ignored()
{
    // Former stepper command (16) and status (17) packets now decode as unknown.
    uint8_t command[] = { 16, 1, 6, 0, 1, 0 };
    uint8_t status[23] = { 17, 1 };
    Message m;
    TEST_ASSERT_FALSE(m.decode(command, sizeof(command)));
    TEST_ASSERT_FALSE(m.decode(status, sizeof(status)));
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
    RUN_TEST(test_empty_configure_canonical_wire);
    RUN_TEST(test_empty_configure_rejects_noncanonical_header_and_lengths);

    RUN_TEST(test_configure_gauge_wire_layout);
    RUN_TEST(test_configure_gauge_roundtrip_and_exact_length);
    RUN_TEST(test_gauge_config_validation);
    RUN_TEST(test_set_gauge_position_roundtrip);
    RUN_TEST(test_zero_gauge_roundtrip);
    RUN_TEST(test_removed_stepper_message_types_are_ignored);

    return UNITY_END();
}
