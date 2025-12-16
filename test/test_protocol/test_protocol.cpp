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
    cfg.input_type = INPUT_TYPE_ANALOG;
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
    TEST_ASSERT_EQUAL_UINT8(INPUT_TYPE_ANALOG, buffer[7]); // input_type
    TEST_ASSERT_EQUAL_UINT8(0xA0, buffer[8]); // pin
    TEST_ASSERT_EQUAL_UINT8(128, buffer[9]); // sensitivity
}

// Test Configure decoding for Analog
void test_configure_decode()
{
    uint8_t buffer[] = { MESSAGE_TYPE_CONFIGURE, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, INPUT_TYPE_ANALOG, 0xA0, 0x80 };

    Configure cfg;
    bool result = cfg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(0x00000001, cfg.config_id);
    TEST_ASSERT_EQUAL_UINT8(3, cfg.total_parts);
    TEST_ASSERT_EQUAL_UINT8(0, cfg.part_number);
    TEST_ASSERT_EQUAL_UINT8(INPUT_TYPE_ANALOG, cfg.input_type);
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
    original.input_type = INPUT_TYPE_ANALOG;
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
    TEST_ASSERT_EQUAL_UINT8(original.input_type, decoded.input_type);
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
    cfg.input_type = INPUT_TYPE_BUTTON;
    cfg.button.pin = 7;
    cfg.button.debounce = 3;

    uint8_t buffer[64];
    size_t size = cfg.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(10, size); // header(8) + pin(1) + debounce(1)
    TEST_ASSERT_EQUAL_UINT8(MESSAGE_TYPE_CONFIGURE, buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(INPUT_TYPE_BUTTON, buffer[7]);
    TEST_ASSERT_EQUAL_UINT8(7, buffer[8]); // pin
    TEST_ASSERT_EQUAL_UINT8(3, buffer[9]); // debounce
}

// Test Configure decoding for Button
void test_configure_button_decode()
{
    uint8_t buffer[] = { MESSAGE_TYPE_CONFIGURE, 0x02, 0x00, 0x00, 0x00, 0x02, 0x01, INPUT_TYPE_BUTTON, 0x07, 0x03 };

    Configure cfg;
    bool result = cfg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(0x00000002, cfg.config_id);
    TEST_ASSERT_EQUAL_UINT8(2, cfg.total_parts);
    TEST_ASSERT_EQUAL_UINT8(1, cfg.part_number);
    TEST_ASSERT_EQUAL_UINT8(INPUT_TYPE_BUTTON, cfg.input_type);
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
    original.input_type = INPUT_TYPE_BUTTON;
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
    TEST_ASSERT_EQUAL_UINT8(original.input_type, decoded.input_type);
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
    cfg.input_type = INPUT_TYPE_MATRIX;
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
    TEST_ASSERT_EQUAL_UINT8(INPUT_TYPE_MATRIX, buffer[7]);
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
        INPUT_TYPE_MATRIX,
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
    TEST_ASSERT_EQUAL_UINT8(INPUT_TYPE_MATRIX, cfg.input_type);
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
    original.input_type = INPUT_TYPE_MATRIX;
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
    TEST_ASSERT_EQUAL_UINT8(original.input_type, decoded.input_type);
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
        INPUT_TYPE_MATRIX,
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
        INPUT_TYPE_MATRIX,
        0x0A, // num_row_pins = 10
        0x0A, // num_col_pins = 10 (total 20 > MAX_MATRIX_PINS)
    };

    Configure cfg;
    bool result = cfg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_FALSE(result);
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
    uint8_t buffer[] = { MESSAGE_TYPE_CONFIGURE, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, INPUT_TYPE_ANALOG, 0xA0, 0x80 };

    Message msg;
    bool result = msg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(msg.isConfigure());
    TEST_ASSERT_EQUAL_UINT32(0x00000001, msg.configure.config_id);
    TEST_ASSERT_EQUAL_UINT8(3, msg.configure.total_parts);
    TEST_ASSERT_EQUAL_UINT8(0, msg.configure.part_number);
    TEST_ASSERT_EQUAL_UINT8(INPUT_TYPE_ANALOG, msg.configure.input_type);
    TEST_ASSERT_EQUAL_UINT8(0xA0, msg.configure.analog.pin);
    TEST_ASSERT_EQUAL_UINT8(0x80, msg.configure.analog.sensitivity);
}

// Test Message decode for Configure (Button)
void test_message_decode_configure_button()
{
    uint8_t buffer[] = { MESSAGE_TYPE_CONFIGURE, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, INPUT_TYPE_BUTTON, 0x07, 0x03 };

    Message msg;
    bool result = msg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(msg.isConfigure());
    TEST_ASSERT_EQUAL_UINT8(INPUT_TYPE_BUTTON, msg.configure.input_type);
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
        INPUT_TYPE_MATRIX,
        0x02, // num_row_pins
        0x02, // num_col_pins
        0x0A, 0x0B, // row pins
        0x0C, 0x0D // col pins
    };

    Message msg;
    bool result = msg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(msg.isConfigure());
    TEST_ASSERT_EQUAL_UINT8(INPUT_TYPE_MATRIX, msg.configure.input_type);
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

// Main test runner
void setUp(void)
{
    // Set up code (runs before each test)
}

void tearDown(void)
{
    // Clean up code (runs after each test)
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
    RUN_TEST(test_configure_decode_unknown_type);

    // ConfigurationStored tests
    RUN_TEST(test_configuration_stored_encode);
    RUN_TEST(test_configuration_stored_decode);
    RUN_TEST(test_configuration_stored_roundtrip);

    // ConfigurationError tests
    RUN_TEST(test_configuration_error_encode);
    RUN_TEST(test_configuration_error_decode);
    RUN_TEST(test_configuration_error_roundtrip);

    // Message union tests
    RUN_TEST(test_message_decode_identity_request);
    RUN_TEST(test_message_decode_identity_response);
    RUN_TEST(test_message_decode_configure);
    RUN_TEST(test_message_decode_configure_button);
    RUN_TEST(test_message_decode_configure_matrix);
    RUN_TEST(test_message_decode_configuration_stored);
    RUN_TEST(test_message_decode_configuration_error);
    RUN_TEST(test_message_decode_invalid_type);

    // Error handling tests
    RUN_TEST(test_encode_buffer_too_small);

    return UNITY_END();
}
