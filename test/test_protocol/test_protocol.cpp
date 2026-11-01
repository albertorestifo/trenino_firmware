#include "../../src/protocol.h"
#include <string.h>
#include <unity.h>

using namespace Protocol;

// Test IdentityRequest encoding
void test_identity_request_encode()
{
    IdentityRequest request;
    request.request_id = 0x12345678;

    uint8_t buffer[64];
    size_t size = request.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(5, size);
    TEST_ASSERT_EQUAL_UINT8(MESSAGE_TYPE_IDENTITY_REQUEST, buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(0x78, buffer[1]); // request_id byte 0 (LE)
    TEST_ASSERT_EQUAL_UINT8(0x56, buffer[2]); // request_id byte 1 (LE)
    TEST_ASSERT_EQUAL_UINT8(0x34, buffer[3]); // request_id byte 2 (LE)
    TEST_ASSERT_EQUAL_UINT8(0x12, buffer[4]); // request_id byte 3 (LE)
}

// Test IdentityRequest decoding
void test_identity_request_decode()
{
    uint8_t buffer[] = { MESSAGE_TYPE_IDENTITY_REQUEST, 0x78, 0x56, 0x34, 0x12 };

    IdentityRequest request;
    bool result = request.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(0x12345678, request.request_id);
}

// Test IdentityRequest decode with insufficient data
void test_identity_request_decode_insufficient_data()
{
    uint8_t buffer[] = { MESSAGE_TYPE_IDENTITY_REQUEST, 0x78, 0x56 }; // Only 3 bytes, need 5

    IdentityRequest request;
    bool result = request.decode(buffer, sizeof(buffer));

    TEST_ASSERT_FALSE(result);
}

// Test IdentityRequest roundtrip
void test_identity_request_roundtrip()
{
    IdentityRequest original;
    original.request_id = 0xAABBCCDD;

    uint8_t buffer[64];
    size_t size = original.encode(buffer, sizeof(buffer));

    IdentityRequest decoded;
    bool result = decoded.decode(buffer, size);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(original.request_id, decoded.request_id);
}

// Test IdentityResponse encoding
void test_identity_response_encode()
{
    IdentityResponse response;
    response.request_id = 0xAABBCCDD;
    response.version_major = 1;
    response.version_minor = 2;
    response.version_patch = 3;
    response.config_id = 0x12345678;

    uint8_t buffer[64];
    size_t size = response.encode(buffer, sizeof(buffer));

    // 1 type + 4 request_id + 1 version_major + 1 version_minor + 1 version_patch + 4 config_id = 12
    TEST_ASSERT_EQUAL(12, size);
    TEST_ASSERT_EQUAL_UINT8(MESSAGE_TYPE_IDENTITY_RESPONSE, buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(0xDD, buffer[1]); // request_id byte 0 (LE)
    TEST_ASSERT_EQUAL_UINT8(0xCC, buffer[2]); // request_id byte 1 (LE)
    TEST_ASSERT_EQUAL_UINT8(0xBB, buffer[3]); // request_id byte 2 (LE)
    TEST_ASSERT_EQUAL_UINT8(0xAA, buffer[4]); // request_id byte 3 (LE)
    TEST_ASSERT_EQUAL_UINT8(1, buffer[5]); // version_major
    TEST_ASSERT_EQUAL_UINT8(2, buffer[6]); // version_minor
    TEST_ASSERT_EQUAL_UINT8(3, buffer[7]); // version_patch
    TEST_ASSERT_EQUAL_UINT8(0x78, buffer[8]); // config_id byte 0 (LE)
    TEST_ASSERT_EQUAL_UINT8(0x56, buffer[9]); // config_id byte 1 (LE)
    TEST_ASSERT_EQUAL_UINT8(0x34, buffer[10]); // config_id byte 2 (LE)
    TEST_ASSERT_EQUAL_UINT8(0x12, buffer[11]); // config_id byte 3 (LE)
}

// Test IdentityResponse decoding
void test_identity_response_decode()
{
    // type + request_id (LE) + version_major + version_minor + version_patch + config_id (LE)
    uint8_t buffer[] = { MESSAGE_TYPE_IDENTITY_RESPONSE, 0xDD, 0xCC, 0xBB, 0xAA, 0x01, 0x02, 0x03, 0x78, 0x56, 0x34, 0x12 };

    IdentityResponse response;
    bool result = response.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(0xAABBCCDD, response.request_id);
    TEST_ASSERT_EQUAL_UINT8(1, response.version_major);
    TEST_ASSERT_EQUAL_UINT8(2, response.version_minor);
    TEST_ASSERT_EQUAL_UINT8(3, response.version_patch);
    TEST_ASSERT_EQUAL_UINT32(0x12345678, response.config_id);
}

// Test IdentityResponse decode with insufficient data
void test_identity_response_decode_insufficient_data()
{
    uint8_t buffer[] = { MESSAGE_TYPE_IDENTITY_RESPONSE, 0xDD, 0xCC, 0xBB }; // Only 4 bytes, need 12

    IdentityResponse response;
    bool result = response.decode(buffer, sizeof(buffer));

    TEST_ASSERT_FALSE(result);
}

// Test IdentityResponse roundtrip
void test_identity_response_roundtrip()
{
    IdentityResponse original;
    original.request_id = 0xDEADBEEF;
    original.version_major = 2;
    original.version_minor = 1;
    original.version_patch = 3;
    original.config_id = 0xCAFEBABE;

    uint8_t buffer[64];
    size_t size = original.encode(buffer, sizeof(buffer));

    IdentityResponse decoded;
    bool result = decoded.decode(buffer, size);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(original.request_id, decoded.request_id);
    TEST_ASSERT_EQUAL_UINT8(original.version_major, decoded.version_major);
    TEST_ASSERT_EQUAL_UINT8(original.version_minor, decoded.version_minor);
    TEST_ASSERT_EQUAL_UINT8(original.version_patch, decoded.version_patch);
    TEST_ASSERT_EQUAL_UINT32(original.config_id, decoded.config_id);
}

// Test Message decode for IdentityRequest
void test_message_decode_identity_request()
{
    uint8_t buffer[] = { MESSAGE_TYPE_IDENTITY_REQUEST, 0x78, 0x56, 0x34, 0x12 };

    Message msg;
    bool result = msg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(msg.isIdentityRequest());
    TEST_ASSERT_EQUAL_UINT32(0x12345678, msg.identity_request.request_id);
}

// Test Message decode for IdentityResponse
void test_message_decode_identity_response()
{
    // type + request_id (LE) + version_major + version_minor + version_patch + config_id (LE)
    uint8_t buffer[] = { MESSAGE_TYPE_IDENTITY_RESPONSE, 0xDD, 0xCC, 0xBB, 0xAA, 0x01, 0x00, 0x00, 0x78, 0x56, 0x34, 0x12 };

    Message msg;
    bool result = msg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(msg.isIdentityResponse());
    TEST_ASSERT_EQUAL_UINT32(0xAABBCCDD, msg.identity_response.request_id);
    TEST_ASSERT_EQUAL_UINT8(1, msg.identity_response.version_major);
    TEST_ASSERT_EQUAL_UINT8(0, msg.identity_response.version_minor);
    TEST_ASSERT_EQUAL_UINT8(0, msg.identity_response.version_patch);
    TEST_ASSERT_EQUAL_UINT32(0x12345678, msg.identity_response.config_id);
}

// Test Message decode with invalid message type
void test_message_decode_invalid_type()
{
    uint8_t buffer[] = { 0xFF, 0x42, 0x00 }; // Invalid message type

    Message msg;
    bool result = msg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_FALSE(result);
}

// Test Configure encoding for Analog
void test_configure_encode()
{
    Configure cfg;
    cfg.config_id = 0x00000001;
    cfg.total_parts = 3;
    cfg.part_number = 0;
    cfg.module_type = MODULE_TYPE_ANALOG;
    cfg.analog.pin = 0xA0; // A0
    cfg.analog.sensitivity = 128;

    uint8_t buffer[64];
    size_t size = cfg.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(10, size);
    TEST_ASSERT_EQUAL_UINT8(MESSAGE_TYPE_CONFIGURE, buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(0x01, buffer[1]); // config_id byte 0 (LE)
    TEST_ASSERT_EQUAL_UINT8(0x00, buffer[2]); // config_id byte 1 (LE)
    TEST_ASSERT_EQUAL_UINT8(0x00, buffer[3]); // config_id byte 2 (LE)
    TEST_ASSERT_EQUAL_UINT8(0x00, buffer[4]); // config_id byte 3 (LE)
    TEST_ASSERT_EQUAL_UINT8(3, buffer[5]); // total_parts
    TEST_ASSERT_EQUAL_UINT8(0, buffer[6]); // part_number
    TEST_ASSERT_EQUAL_UINT8(MODULE_TYPE_ANALOG, buffer[7]); // module_type
    TEST_ASSERT_EQUAL_UINT8(0xA0, buffer[8]); // pin
    TEST_ASSERT_EQUAL_UINT8(128, buffer[9]); // sensitivity
}

// Test Configure decoding for Analog
void test_configure_decode()
{
    uint8_t buffer[] = { MESSAGE_TYPE_CONFIGURE, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, MODULE_TYPE_ANALOG, 0xA0, 0x80 };

    Configure cfg;
    bool result = cfg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(0x00000001, cfg.config_id);
    TEST_ASSERT_EQUAL_UINT8(3, cfg.total_parts);
    TEST_ASSERT_EQUAL_UINT8(0, cfg.part_number);
    TEST_ASSERT_EQUAL_UINT8(MODULE_TYPE_ANALOG, cfg.module_type);
    TEST_ASSERT_EQUAL_UINT8(0xA0, cfg.analog.pin);
    TEST_ASSERT_EQUAL_UINT8(0x80, cfg.analog.sensitivity);
}

// Test Configure decode with insufficient data
void test_configure_decode_insufficient_data()
{
    uint8_t buffer[] = { MESSAGE_TYPE_CONFIGURE, 0x01, 0x00, 0x00 }; // Only 4 bytes, need 8+ header

    Configure cfg;
    bool result = cfg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_FALSE(result);
}

// Test Configure roundtrip for Analog
void test_configure_roundtrip()
{
    Configure original;
    original.config_id = 0xDEADBEEF;
    original.total_parts = 5;
    original.part_number = 2;
    original.module_type = MODULE_TYPE_ANALOG;
    original.analog.pin = 0xA1;
    original.analog.sensitivity = 200;

    uint8_t buffer[64];
    size_t size = original.encode(buffer, sizeof(buffer));

    Configure decoded;
    bool result = decoded.decode(buffer, size);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(original.config_id, decoded.config_id);
    TEST_ASSERT_EQUAL_UINT8(original.total_parts, decoded.total_parts);
    TEST_ASSERT_EQUAL_UINT8(original.part_number, decoded.part_number);
    TEST_ASSERT_EQUAL_UINT8(original.module_type, decoded.module_type);
    TEST_ASSERT_EQUAL_UINT8(original.analog.pin, decoded.analog.pin);
    TEST_ASSERT_EQUAL_UINT8(original.analog.sensitivity, decoded.analog.sensitivity);
}

// Test Configure encoding for Button
void test_configure_button_encode()
{
    Configure cfg;
    cfg.config_id = 0x00000002;
    cfg.total_parts = 2;
    cfg.part_number = 1;
    cfg.module_type = MODULE_TYPE_BUTTON;
    cfg.button.pin = 7;
    cfg.button.debounce = 3;

    uint8_t buffer[64];
    size_t size = cfg.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(10, size); // header(8) + pin(1) + debounce(1)
    TEST_ASSERT_EQUAL_UINT8(MESSAGE_TYPE_CONFIGURE, buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(MODULE_TYPE_BUTTON, buffer[7]);
    TEST_ASSERT_EQUAL_UINT8(7, buffer[8]); // pin
    TEST_ASSERT_EQUAL_UINT8(3, buffer[9]); // debounce
}

// Test Configure decoding for Button
void test_configure_button_decode()
{
    uint8_t buffer[] = { MESSAGE_TYPE_CONFIGURE, 0x02, 0x00, 0x00, 0x00, 0x02, 0x01, MODULE_TYPE_BUTTON, 0x07, 0x03 };

    Configure cfg;
    bool result = cfg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(0x00000002, cfg.config_id);
    TEST_ASSERT_EQUAL_UINT8(2, cfg.total_parts);
    TEST_ASSERT_EQUAL_UINT8(1, cfg.part_number);
    TEST_ASSERT_EQUAL_UINT8(MODULE_TYPE_BUTTON, cfg.module_type);
    TEST_ASSERT_EQUAL_UINT8(7, cfg.button.pin);
    TEST_ASSERT_EQUAL_UINT8(3, cfg.button.debounce);
}

// Test Configure roundtrip for Button
void test_configure_button_roundtrip()
{
    Configure original;
    original.config_id = 0xCAFEBABE;
    original.total_parts = 4;
    original.part_number = 3;
    original.module_type = MODULE_TYPE_BUTTON;
    original.button.pin = 12;
    original.button.debounce = 5;

    uint8_t buffer[64];
    size_t size = original.encode(buffer, sizeof(buffer));

    Configure decoded;
    bool result = decoded.decode(buffer, size);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(original.config_id, decoded.config_id);
    TEST_ASSERT_EQUAL_UINT8(original.total_parts, decoded.total_parts);
    TEST_ASSERT_EQUAL_UINT8(original.part_number, decoded.part_number);
    TEST_ASSERT_EQUAL_UINT8(original.module_type, decoded.module_type);
    TEST_ASSERT_EQUAL_UINT8(original.button.pin, decoded.button.pin);
    TEST_ASSERT_EQUAL_UINT8(original.button.debounce, decoded.button.debounce);
}

// Test Configure encoding for Matrix
void test_configure_matrix_encode()
{
    Configure cfg;
    cfg.config_id = 0x00000003;
    cfg.total_parts = 1;
    cfg.part_number = 0;
    cfg.module_type = MODULE_TYPE_MATRIX;
    cfg.matrix.num_row_pins = 3;
    cfg.matrix.num_col_pins = 4;
    // Row pins: 2, 3, 4
    cfg.matrix.pins[0] = 2;
    cfg.matrix.pins[1] = 3;
    cfg.matrix.pins[2] = 4;
    // Col pins: 5, 6, 7, 8
    cfg.matrix.pins[3] = 5;
    cfg.matrix.pins[4] = 6;
    cfg.matrix.pins[5] = 7;
    cfg.matrix.pins[6] = 8;

    uint8_t buffer[64];
    size_t size = cfg.encode(buffer, sizeof(buffer));

    // header(8) + num_row_pins(1) + num_col_pins(1) + pins(7) = 17
    TEST_ASSERT_EQUAL(17, size);
    TEST_ASSERT_EQUAL_UINT8(MESSAGE_TYPE_CONFIGURE, buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(MODULE_TYPE_MATRIX, buffer[7]);
    TEST_ASSERT_EQUAL_UINT8(3, buffer[8]); // num_row_pins
    TEST_ASSERT_EQUAL_UINT8(4, buffer[9]); // num_col_pins
    TEST_ASSERT_EQUAL_UINT8(2, buffer[10]); // row pin 0
    TEST_ASSERT_EQUAL_UINT8(3, buffer[11]); // row pin 1
    TEST_ASSERT_EQUAL_UINT8(4, buffer[12]); // row pin 2
    TEST_ASSERT_EQUAL_UINT8(5, buffer[13]); // col pin 0
    TEST_ASSERT_EQUAL_UINT8(6, buffer[14]); // col pin 1
    TEST_ASSERT_EQUAL_UINT8(7, buffer[15]); // col pin 2
    TEST_ASSERT_EQUAL_UINT8(8, buffer[16]); // col pin 3
}

// Test Configure decoding for Matrix
void test_configure_matrix_decode()
{
    uint8_t buffer[] = {
        MESSAGE_TYPE_CONFIGURE,
        0x03, 0x00, 0x00, 0x00, // config_id
        0x01, // total_parts
        0x00, // part_number
        MODULE_TYPE_MATRIX,
        0x02, // num_row_pins
        0x03, // num_col_pins
        0x0A, 0x0B, // row pins
        0x0C, 0x0D, 0x0E // col pins
    };

    Configure cfg;
    bool result = cfg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(0x00000003, cfg.config_id);
    TEST_ASSERT_EQUAL_UINT8(1, cfg.total_parts);
    TEST_ASSERT_EQUAL_UINT8(0, cfg.part_number);
    TEST_ASSERT_EQUAL_UINT8(MODULE_TYPE_MATRIX, cfg.module_type);
    TEST_ASSERT_EQUAL_UINT8(2, cfg.matrix.num_row_pins);
    TEST_ASSERT_EQUAL_UINT8(3, cfg.matrix.num_col_pins);
    TEST_ASSERT_EQUAL_UINT8(0x0A, cfg.matrix.pins[0]); // row 0
    TEST_ASSERT_EQUAL_UINT8(0x0B, cfg.matrix.pins[1]); // row 1
    TEST_ASSERT_EQUAL_UINT8(0x0C, cfg.matrix.pins[2]); // col 0
    TEST_ASSERT_EQUAL_UINT8(0x0D, cfg.matrix.pins[3]); // col 1
    TEST_ASSERT_EQUAL_UINT8(0x0E, cfg.matrix.pins[4]); // col 2
}

// Test Configure roundtrip for Matrix
void test_configure_matrix_roundtrip()
{
    Configure original;
    original.config_id = 0x11223344;
    original.total_parts = 2;
    original.part_number = 1;
    original.module_type = MODULE_TYPE_MATRIX;
    original.matrix.num_row_pins = 4;
    original.matrix.num_col_pins = 4;
    for (uint8_t i = 0; i < 8; i++) {
        original.matrix.pins[i] = i + 10;
    }

    uint8_t buffer[64];
    size_t size = original.encode(buffer, sizeof(buffer));

    Configure decoded;
    bool result = decoded.decode(buffer, size);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(original.config_id, decoded.config_id);
    TEST_ASSERT_EQUAL_UINT8(original.total_parts, decoded.total_parts);
    TEST_ASSERT_EQUAL_UINT8(original.part_number, decoded.part_number);
    TEST_ASSERT_EQUAL_UINT8(original.module_type, decoded.module_type);
    TEST_ASSERT_EQUAL_UINT8(original.matrix.num_row_pins, decoded.matrix.num_row_pins);
    TEST_ASSERT_EQUAL_UINT8(original.matrix.num_col_pins, decoded.matrix.num_col_pins);
    for (uint8_t i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL_UINT8(original.matrix.pins[i], decoded.matrix.pins[i]);
    }
}

// Test Configure decode with insufficient data for matrix
void test_configure_matrix_decode_insufficient_data()
{
    // Has header but not enough pins
    uint8_t buffer[] = {
        MESSAGE_TYPE_CONFIGURE,
        0x03, 0x00, 0x00, 0x00, // config_id
        0x01, // total_parts
        0x00, // part_number
        MODULE_TYPE_MATRIX,
        0x02, // num_row_pins
        0x03, // num_col_pins
        0x0A // Only 1 pin, need 5
    };

    Configure cfg;
    bool result = cfg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_FALSE(result);
}

// Test Configure decode with too many matrix pins
void test_configure_matrix_decode_too_many_pins()
{
    uint8_t buffer[] = {
        MESSAGE_TYPE_CONFIGURE,
        0x03, 0x00, 0x00, 0x00, // config_id
        0x01, // total_parts
        0x00, // part_number
        MODULE_TYPE_MATRIX,
        0x0A, // num_row_pins = 10
        0x0A, // num_col_pins = 10 (total 20 > MAX_MATRIX_PINS)
    };

    Configure cfg;
    bool result = cfg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_FALSE(result);
}

// 250 + 10 is 260, which wraps to 4 pins in a uint8_t
void test_configure_matrix_decode_pin_count_does_not_wrap()
{
    uint8_t buffer[] = {
        MESSAGE_TYPE_CONFIGURE,
        0x03, 0x00, 0x00, 0x00, // config_id
        0x01, // total_parts
        0x00, // part_number
        MODULE_TYPE_MATRIX,
        250, // num_row_pins
        10, // num_col_pins
        0x02, 0x03, 0x04, 0x05, // the 4 pins of a wrapped count
    };

    Configure cfg;
    TEST_ASSERT_FALSE(cfg.decode(buffer, sizeof(buffer)));
}

// Test Configure decode with unknown input type
void test_configure_decode_unknown_type()
{
    uint8_t buffer[] = {
        MESSAGE_TYPE_CONFIGURE,
        0x01, 0x00, 0x00, 0x00,
        0x01, 0x00,
        0xFF, // Unknown input type
        0x00, 0x00
    };

    Configure cfg;
    bool result = cfg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_FALSE(result);
}

// Test Message decode for Configure (Analog)
void test_message_decode_configure()
{
    uint8_t buffer[] = { MESSAGE_TYPE_CONFIGURE, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, MODULE_TYPE_ANALOG, 0xA0, 0x80 };

    Message msg;
    bool result = msg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(msg.isConfigure());
    TEST_ASSERT_EQUAL_UINT32(0x00000001, msg.configure.config_id);
    TEST_ASSERT_EQUAL_UINT8(3, msg.configure.total_parts);
    TEST_ASSERT_EQUAL_UINT8(0, msg.configure.part_number);
    TEST_ASSERT_EQUAL_UINT8(MODULE_TYPE_ANALOG, msg.configure.module_type);
    TEST_ASSERT_EQUAL_UINT8(0xA0, msg.configure.analog.pin);
    TEST_ASSERT_EQUAL_UINT8(0x80, msg.configure.analog.sensitivity);
}

// Test Message decode for Configure (Button)
void test_message_decode_configure_button()
{
    uint8_t buffer[] = { MESSAGE_TYPE_CONFIGURE, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, MODULE_TYPE_BUTTON, 0x07, 0x03 };

    Message msg;
    bool result = msg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(msg.isConfigure());
    TEST_ASSERT_EQUAL_UINT8(MODULE_TYPE_BUTTON, msg.configure.module_type);
    TEST_ASSERT_EQUAL_UINT8(7, msg.configure.button.pin);
    TEST_ASSERT_EQUAL_UINT8(3, msg.configure.button.debounce);
}

// Test Message decode for Configure (Matrix)
void test_message_decode_configure_matrix()
{
    uint8_t buffer[] = {
        MESSAGE_TYPE_CONFIGURE,
        0x03, 0x00, 0x00, 0x00, // config_id
        0x01, // total_parts
        0x00, // part_number
        MODULE_TYPE_MATRIX,
        0x02, // num_row_pins
        0x02, // num_col_pins
        0x0A, 0x0B, // row pins
        0x0C, 0x0D // col pins
    };

    Message msg;
    bool result = msg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(msg.isConfigure());
    TEST_ASSERT_EQUAL_UINT8(MODULE_TYPE_MATRIX, msg.configure.module_type);
    TEST_ASSERT_EQUAL_UINT8(2, msg.configure.matrix.num_row_pins);
    TEST_ASSERT_EQUAL_UINT8(2, msg.configure.matrix.num_col_pins);
}

// Test encode with buffer too small
void test_encode_buffer_too_small()
{
    IdentityRequest request;
    request.request_id = 0x42;

    uint8_t buffer[2]; // Too small, need 5 bytes
    size_t size = request.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(0, size); // Should return 0 on error
}

// ConfigurationStored tests

void test_configuration_stored_encode()
{
    ConfigurationStored stored;
    stored.config_id = 0x12345678;

    uint8_t buffer[16];
    size_t size = stored.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(5, size);
    TEST_ASSERT_EQUAL(MESSAGE_TYPE_CONFIGURATION_STORED, buffer[0]);
    TEST_ASSERT_EQUAL(0x78, buffer[1]);
    TEST_ASSERT_EQUAL(0x56, buffer[2]);
    TEST_ASSERT_EQUAL(0x34, buffer[3]);
    TEST_ASSERT_EQUAL(0x12, buffer[4]);
}

void test_configuration_stored_decode()
{
    uint8_t buffer[] = { MESSAGE_TYPE_CONFIGURATION_STORED, 0x78, 0x56, 0x34, 0x12 };

    ConfigurationStored stored;
    TEST_ASSERT_TRUE(stored.decode(buffer, sizeof(buffer)));
    TEST_ASSERT_EQUAL(0x12345678, stored.config_id);
}

void test_configuration_stored_roundtrip()
{
    ConfigurationStored original;
    original.config_id = 0xAABBCCDD;

    uint8_t buffer[16];
    size_t size = original.encode(buffer, sizeof(buffer));

    ConfigurationStored decoded;
    TEST_ASSERT_TRUE(decoded.decode(buffer, size));
    TEST_ASSERT_EQUAL(original.config_id, decoded.config_id);
}

// ConfigurationError tests

void test_configuration_error_encode()
{
    ConfigurationError error;
    error.config_id = 0xDEADBEEF;

    uint8_t buffer[16];
    size_t size = error.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(5, size);
    TEST_ASSERT_EQUAL(MESSAGE_TYPE_CONFIGURATION_ERROR, buffer[0]);
    TEST_ASSERT_EQUAL(0xEF, buffer[1]);
    TEST_ASSERT_EQUAL(0xBE, buffer[2]);
    TEST_ASSERT_EQUAL(0xAD, buffer[3]);
    TEST_ASSERT_EQUAL(0xDE, buffer[4]);
}

void test_configuration_error_decode()
{
    uint8_t buffer[] = { MESSAGE_TYPE_CONFIGURATION_ERROR, 0xEF, 0xBE, 0xAD, 0xDE };

    ConfigurationError error;
    TEST_ASSERT_TRUE(error.decode(buffer, sizeof(buffer)));
    TEST_ASSERT_EQUAL(0xDEADBEEF, error.config_id);
}

void test_configuration_error_roundtrip()
{
    ConfigurationError original;
    original.config_id = 0x11223344;

    uint8_t buffer[16];
    size_t size = original.encode(buffer, sizeof(buffer));

    ConfigurationError decoded;
    TEST_ASSERT_TRUE(decoded.decode(buffer, size));
    TEST_ASSERT_EQUAL(original.config_id, decoded.config_id);
}

// Message union tests for new types

void test_message_decode_configuration_stored()
{
    uint8_t buffer[] = { MESSAGE_TYPE_CONFIGURATION_STORED, 0x78, 0x56, 0x34, 0x12 };

    Message msg;
    TEST_ASSERT_TRUE(msg.decode(buffer, sizeof(buffer)));
    TEST_ASSERT_TRUE(msg.isConfigurationStored());
    TEST_ASSERT_EQUAL(0x12345678, msg.configuration_stored.config_id);
}

void test_message_decode_configuration_error()
{
    uint8_t buffer[] = { MESSAGE_TYPE_CONFIGURATION_ERROR, 0xEF, 0xBE, 0xAD, 0xDE };

    Message msg;
    TEST_ASSERT_TRUE(msg.decode(buffer, sizeof(buffer)));
    TEST_ASSERT_TRUE(msg.isConfigurationError());
    TEST_ASSERT_EQUAL(0xDEADBEEF, msg.configuration_error.config_id);
}

// SetOutput tests

void test_set_output_encode()
{
    SetOutput cmd;
    cmd.pin = 13;
    cmd.value = 1;

    uint8_t buffer[16];
    size_t size = cmd.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(3, size);
    TEST_ASSERT_EQUAL_UINT8(MESSAGE_TYPE_SET_OUTPUT, buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(13, buffer[1]);
    TEST_ASSERT_EQUAL_UINT8(1, buffer[2]);
}

void test_set_output_decode()
{
    uint8_t buffer[] = { MESSAGE_TYPE_SET_OUTPUT, 5, 0 };

    SetOutput cmd;
    TEST_ASSERT_TRUE(cmd.decode(buffer, sizeof(buffer)));
    TEST_ASSERT_EQUAL_UINT8(5, cmd.pin);
    TEST_ASSERT_EQUAL_UINT8(0, cmd.value);
}

void test_set_output_roundtrip()
{
    SetOutput original;
    original.pin = 9;
    original.value = 1;

    uint8_t buffer[16];
    size_t size = original.encode(buffer, sizeof(buffer));

    SetOutput decoded;
    TEST_ASSERT_TRUE(decoded.decode(buffer, size));
    TEST_ASSERT_EQUAL_UINT8(original.pin, decoded.pin);
    TEST_ASSERT_EQUAL_UINT8(original.value, decoded.value);
}

void test_set_output_decode_insufficient_data()
{
    uint8_t buffer[] = { MESSAGE_TYPE_SET_OUTPUT, 5 }; // Missing value byte

    SetOutput cmd;
    TEST_ASSERT_FALSE(cmd.decode(buffer, sizeof(buffer)));
}

void test_message_decode_set_output()
{
    uint8_t buffer[] = { MESSAGE_TYPE_SET_OUTPUT, 13, 1 };

    Message msg;
    TEST_ASSERT_TRUE(msg.decode(buffer, sizeof(buffer)));
    TEST_ASSERT_TRUE(msg.isSetOutput());
    TEST_ASSERT_EQUAL_UINT8(13, msg.set_output.pin);
    TEST_ASSERT_EQUAL_UINT8(1, msg.set_output.value);
}

// Main test runner
void setUp(void)
{
    // Set up code (runs before each test)
}

void tearDown(void)
{
    // Clean up code (runs after each test)
}

// ---- MCP23017 / ADS1115 configure ----

static Configure validMcp()
{
    Configure c;
    c.config_id = 7;
    c.total_parts = 1;
    c.part_number = 0;
    c.module_type = MODULE_TYPE_MCP23017;
    c.mcp23017.i2c_address = 0x20;
    c.mcp23017.direct_base = 128;
    c.mcp23017.matrix_base = 144;
    c.mcp23017.debounce = 3;
    c.mcp23017.direct_mask = 0x0F00;
    c.mcp23017.row_mask = 0x0003;
    c.mcp23017.col_mask = 0x001C;
    return c;
}

static Configure validAds()
{
    Configure c;
    c.config_id = 8;
    c.total_parts = 1;
    c.part_number = 0;
    c.module_type = MODULE_TYPE_ADS1115;
    c.ads1115.i2c_address = 0x48;
    c.ads1115.pin_base = 200;
    c.ads1115.channel_mask = 0x07;
    c.ads1115.gain = 1;
    c.ads1115.ref_channel = 3;
    c.ads1115.sensitivity[0] = 1;
    c.ads1115.sensitivity[1] = 5;
    c.ads1115.sensitivity[2] = 10;
    c.ads1115.sensitivity[3] = 99; // disabled channel: ignored
    return c;
}

// Encode then decode: true when the part survives validation.
static bool decodes(const Configure& c)
{
    uint8_t buf[64];
    size_t n = c.encode(buf, sizeof(buf));
    Configure out;
    return n != 0 && out.decode(buf, n);
}

void test_mcp_roundtrip_little_endian()
{
    Configure c = validMcp();
    uint8_t buf[64];
    size_t n = c.encode(buf, sizeof(buf));
    TEST_ASSERT_EQUAL(18, n);
    TEST_ASSERT_EQUAL_UINT8(5, buf[7]);
    TEST_ASSERT_EQUAL_UINT8(0x20, buf[8]);
    TEST_ASSERT_EQUAL_UINT8(128, buf[9]);
    TEST_ASSERT_EQUAL_UINT8(144, buf[10]);
    TEST_ASSERT_EQUAL_UINT8(3, buf[11]);
    TEST_ASSERT_EQUAL_UINT8(0x00, buf[12]);
    TEST_ASSERT_EQUAL_UINT8(0x0F, buf[13]);
    TEST_ASSERT_EQUAL_UINT8(0x03, buf[14]);
    TEST_ASSERT_EQUAL_UINT8(0x00, buf[15]);
    TEST_ASSERT_EQUAL_UINT8(0x1C, buf[16]);
    TEST_ASSERT_EQUAL_UINT8(0x00, buf[17]);

    Configure out;
    TEST_ASSERT_TRUE(out.decode(buf, n));
    TEST_ASSERT_EQUAL_UINT8(MODULE_TYPE_MCP23017, out.module_type);
    TEST_ASSERT_EQUAL_UINT8(0x20, out.mcp23017.i2c_address);
    TEST_ASSERT_EQUAL_UINT8(128, out.mcp23017.direct_base);
    TEST_ASSERT_EQUAL_UINT8(144, out.mcp23017.matrix_base);
    TEST_ASSERT_EQUAL_UINT8(3, out.mcp23017.debounce);
    TEST_ASSERT_EQUAL_HEX16(0x0F00, out.mcp23017.direct_mask);
    TEST_ASSERT_EQUAL_HEX16(0x0003, out.mcp23017.row_mask);
    TEST_ASSERT_EQUAL_HEX16(0x001C, out.mcp23017.col_mask);
}

void test_mcp_rejects_wrong_length()
{
    Configure c = validMcp();
    uint8_t buf[64];
    size_t n = c.encode(buf, sizeof(buf));
    Configure out;
    TEST_ASSERT_FALSE(out.decode(buf, n - 1));
    TEST_ASSERT_FALSE(out.decode(buf, n + 1));
}

void test_mcp_direct_only_ok()
{
    Configure c = validMcp();
    c.mcp23017.matrix_base = 0;
    c.mcp23017.row_mask = 0;
    c.mcp23017.col_mask = 0;
    TEST_ASSERT_TRUE(decodes(c));
}

void test_mcp_matrix_only_ok()
{
    Configure c = validMcp();
    c.mcp23017.direct_base = 0;
    c.mcp23017.direct_mask = 0;
    TEST_ASSERT_TRUE(decodes(c));
}

void test_mcp_rejects_no_enabled_pins()
{
    Configure c = validMcp();
    c.mcp23017.direct_base = 0;
    c.mcp23017.matrix_base = 0;
    c.mcp23017.direct_mask = 0;
    c.mcp23017.row_mask = 0;
    c.mcp23017.col_mask = 0;
    TEST_ASSERT_FALSE(decodes(c));
}

void test_mcp_rejects_overlapping_masks()
{
    Configure c = validMcp();
    c.mcp23017.row_mask = 0x0003;
    c.mcp23017.col_mask = 0x0006;
    TEST_ASSERT_FALSE(decodes(c));
    c = validMcp();
    c.mcp23017.direct_mask = 0x0F01; // overlaps row bit 0
    TEST_ASSERT_FALSE(decodes(c));
    c = validMcp();
    c.mcp23017.direct_mask = 0x0F04; // overlaps col bit 2
    TEST_ASSERT_FALSE(decodes(c));
}

void test_mcp_rejects_gpa7_input()
{
    Configure c = validMcp();
    c.mcp23017.direct_mask = 0x0080;
    TEST_ASSERT_FALSE(decodes(c));
    c = validMcp();
    c.mcp23017.direct_mask = 0x8000; // GPB7
    TEST_ASSERT_FALSE(decodes(c));
}

void test_mcp_rejects_gpb7_column()
{
    Configure c = validMcp();
    c.mcp23017.col_mask = 0x8000;
    TEST_ASSERT_FALSE(decodes(c));
    c = validMcp();
    c.mcp23017.col_mask = 0x0080;
    TEST_ASSERT_FALSE(decodes(c));
}

void test_mcp_allows_gpa7_gpb7_rows()
{
    Configure c = validMcp();
    c.mcp23017.row_mask = 0x8080;
    c.mcp23017.col_mask = 0x0003;
    c.mcp23017.direct_mask = 0x0F00;
    TEST_ASSERT_TRUE(decodes(c));
}

void test_mcp_rejects_nine_rows()
{
    Configure c = validMcp();
    c.mcp23017.direct_base = 0;
    c.mcp23017.direct_mask = 0;
    c.mcp23017.row_mask = 0x01FF;
    c.mcp23017.col_mask = 0x0200;
    TEST_ASSERT_FALSE(decodes(c));
}

void test_mcp_rejects_rows_without_cols()
{
    Configure c = validMcp();
    c.mcp23017.col_mask = 0;
    TEST_ASSERT_FALSE(decodes(c));
    c = validMcp();
    c.mcp23017.row_mask = 0;
    TEST_ASSERT_FALSE(decodes(c));
}

void test_mcp_rejects_zero_base_for_used_block()
{
    Configure c = validMcp();
    c.mcp23017.direct_base = 0;
    TEST_ASSERT_FALSE(decodes(c));
    c = validMcp();
    c.mcp23017.matrix_base = 0;
    TEST_ASSERT_FALSE(decodes(c));
}

void test_mcp_rejects_block_past_255()
{
    Configure c = validMcp();
    c.mcp23017.direct_base = 241; // 16 pins: 241..256
    TEST_ASSERT_FALSE(decodes(c));
    c.mcp23017.direct_base = 240; // 240..255
    TEST_ASSERT_TRUE(decodes(c));

    c = validMcp(); // 2 rows x 3 cols = 6
    c.mcp23017.matrix_base = 251; // 251..256
    TEST_ASSERT_FALSE(decodes(c));
    c.mcp23017.matrix_base = 250; // 250..255
    TEST_ASSERT_TRUE(decodes(c));
}

void test_mcp_rejects_base_below_128()
{
    Configure c = validMcp();
    c.mcp23017.direct_base = 127;
    TEST_ASSERT_FALSE(decodes(c));
    c = validMcp();
    c.mcp23017.matrix_base = 127;
    TEST_ASSERT_FALSE(decodes(c));
}

void test_mcp_rejects_debounce_zero()
{
    Configure c = validMcp();
    c.mcp23017.debounce = 0;
    TEST_ASSERT_FALSE(decodes(c));
}

void test_ads_roundtrip()
{
    Configure c = validAds();
    uint8_t buf[64];
    size_t n = c.encode(buf, sizeof(buf));
    TEST_ASSERT_EQUAL(17, n);
    TEST_ASSERT_EQUAL_UINT8(6, buf[7]);
    TEST_ASSERT_EQUAL_UINT8(0x48, buf[8]);
    TEST_ASSERT_EQUAL_UINT8(200, buf[9]);
    TEST_ASSERT_EQUAL_UINT8(0x07, buf[10]);
    TEST_ASSERT_EQUAL_UINT8(1, buf[11]);
    TEST_ASSERT_EQUAL_UINT8(3, buf[12]);
    TEST_ASSERT_EQUAL_UINT8(10, buf[15]);

    Configure out;
    TEST_ASSERT_TRUE(out.decode(buf, n));
    TEST_ASSERT_EQUAL_UINT8(MODULE_TYPE_ADS1115, out.module_type);
    TEST_ASSERT_EQUAL_UINT8(0x48, out.ads1115.i2c_address);
    TEST_ASSERT_EQUAL_UINT8(200, out.ads1115.pin_base);
    TEST_ASSERT_EQUAL_UINT8(0x07, out.ads1115.channel_mask);
    TEST_ASSERT_EQUAL_UINT8(1, out.ads1115.gain);
    TEST_ASSERT_EQUAL_UINT8(3, out.ads1115.ref_channel);
    TEST_ASSERT_EQUAL_UINT8(5, out.ads1115.sensitivity[1]);
    TEST_ASSERT_FALSE(out.decode(buf, n - 1));
    TEST_ASSERT_FALSE(out.decode(buf, n + 1));
}

void test_ads_rejects_gain_2()
{
    Configure c = validAds();
    c.ads1115.gain = 2;
    TEST_ASSERT_FALSE(decodes(c));
    c.ads1115.gain = 0;
    TEST_ASSERT_TRUE(decodes(c));
}

void test_ads_rejects_ref_in_channel_mask()
{
    Configure c = validAds();
    c.ads1115.ref_channel = 1;
    TEST_ASSERT_FALSE(decodes(c));
}

void test_ads_rejects_ref_4()
{
    Configure c = validAds();
    c.ads1115.ref_channel = 4;
    TEST_ASSERT_FALSE(decodes(c));
}

void test_ads_accepts_no_ref()
{
    Configure c = validAds();
    c.ads1115.ref_channel = ADS_NO_REFERENCE;
    TEST_ASSERT_TRUE(decodes(c));
}

void test_ads_rejects_mask_bits_above_3()
{
    Configure c = validAds();
    c.ads1115.channel_mask = 0x17;
    TEST_ASSERT_FALSE(decodes(c));
}

void test_ads_rejects_empty_channel_mask()
{
    Configure c = validAds();
    c.ads1115.channel_mask = 0;
    TEST_ASSERT_FALSE(decodes(c));
}

void test_ads_rejects_block_past_255()
{
    Configure c = validAds();
    c.ads1115.pin_base = 253; // 253..256
    TEST_ASSERT_FALSE(decodes(c));
    c.ads1115.pin_base = 252; // 252..255
    TEST_ASSERT_TRUE(decodes(c));
    c.ads1115.pin_base = 0;
    TEST_ASSERT_FALSE(decodes(c));
    c.ads1115.pin_base = 127;
    TEST_ASSERT_FALSE(decodes(c));
}

void test_ads_rejects_sensitivity_11_on_enabled_channel()
{
    Configure c = validAds();
    c.ads1115.sensitivity[2] = 11;
    TEST_ASSERT_FALSE(decodes(c));
    c = validAds();
    c.ads1115.sensitivity[0] = 0;
    TEST_ASSERT_FALSE(decodes(c));
}

void test_i2c_union_members_fit_matrix()
{
    TEST_ASSERT_TRUE(sizeof(Mcp23017Config) <= 18);
    TEST_ASSERT_TRUE(sizeof(Ads1115Config) <= 18);
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    // IdentityRequest tests
    RUN_TEST(test_identity_request_encode);
    RUN_TEST(test_identity_request_decode);
    RUN_TEST(test_identity_request_decode_insufficient_data);
    RUN_TEST(test_identity_request_roundtrip);

    // IdentityResponse tests
    RUN_TEST(test_identity_response_encode);
    RUN_TEST(test_identity_response_decode);
    RUN_TEST(test_identity_response_decode_insufficient_data);
    RUN_TEST(test_identity_response_roundtrip);

    // Configure tests (Analog)
    RUN_TEST(test_configure_encode);
    RUN_TEST(test_configure_decode);
    RUN_TEST(test_configure_decode_insufficient_data);
    RUN_TEST(test_configure_roundtrip);

    // Configure tests (Button)
    RUN_TEST(test_configure_button_encode);
    RUN_TEST(test_configure_button_decode);
    RUN_TEST(test_configure_button_roundtrip);

    // Configure tests (Matrix)
    RUN_TEST(test_configure_matrix_encode);
    RUN_TEST(test_configure_matrix_decode);
    RUN_TEST(test_configure_matrix_roundtrip);
    RUN_TEST(test_configure_matrix_decode_insufficient_data);
    RUN_TEST(test_configure_matrix_decode_too_many_pins);
    RUN_TEST(test_configure_matrix_decode_pin_count_does_not_wrap);
    RUN_TEST(test_configure_decode_unknown_type);

    // ConfigurationStored tests
    RUN_TEST(test_configuration_stored_encode);
    RUN_TEST(test_configuration_stored_decode);
    RUN_TEST(test_configuration_stored_roundtrip);

    // ConfigurationError tests
    RUN_TEST(test_configuration_error_encode);
    RUN_TEST(test_configuration_error_decode);
    RUN_TEST(test_configuration_error_roundtrip);

    // SetOutput tests
    RUN_TEST(test_set_output_encode);
    RUN_TEST(test_set_output_decode);
    RUN_TEST(test_set_output_roundtrip);
    RUN_TEST(test_set_output_decode_insufficient_data);

    // Message union tests
    RUN_TEST(test_message_decode_identity_request);
    RUN_TEST(test_message_decode_identity_response);
    RUN_TEST(test_message_decode_configure);
    RUN_TEST(test_message_decode_configure_button);
    RUN_TEST(test_message_decode_configure_matrix);
    RUN_TEST(test_message_decode_configuration_stored);
    RUN_TEST(test_message_decode_configuration_error);
    RUN_TEST(test_message_decode_set_output);
    RUN_TEST(test_message_decode_invalid_type);

    // Error handling tests
    RUN_TEST(test_encode_buffer_too_small);

    RUN_TEST(test_mcp_roundtrip_little_endian);
    RUN_TEST(test_mcp_rejects_wrong_length);
    RUN_TEST(test_mcp_direct_only_ok);
    RUN_TEST(test_mcp_matrix_only_ok);
    RUN_TEST(test_mcp_rejects_no_enabled_pins);
    RUN_TEST(test_mcp_rejects_overlapping_masks);
    RUN_TEST(test_mcp_rejects_gpa7_input);
    RUN_TEST(test_mcp_rejects_gpb7_column);
    RUN_TEST(test_mcp_allows_gpa7_gpb7_rows);
    RUN_TEST(test_mcp_rejects_nine_rows);
    RUN_TEST(test_mcp_rejects_rows_without_cols);
    RUN_TEST(test_mcp_rejects_zero_base_for_used_block);
    RUN_TEST(test_mcp_rejects_block_past_255);
    RUN_TEST(test_mcp_rejects_base_below_128);
    RUN_TEST(test_mcp_rejects_debounce_zero);
    RUN_TEST(test_ads_roundtrip);
    RUN_TEST(test_ads_rejects_gain_2);
    RUN_TEST(test_ads_rejects_ref_in_channel_mask);
    RUN_TEST(test_ads_rejects_ref_4);
    RUN_TEST(test_ads_accepts_no_ref);
    RUN_TEST(test_ads_rejects_mask_bits_above_3);
    RUN_TEST(test_ads_rejects_empty_channel_mask);
    RUN_TEST(test_ads_rejects_block_past_255);
    RUN_TEST(test_ads_rejects_sensitivity_11_on_enabled_channel);
    RUN_TEST(test_i2c_union_members_fit_matrix);

    return UNITY_END();
}
