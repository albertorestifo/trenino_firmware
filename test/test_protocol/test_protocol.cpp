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
    response.version = 1;
    response.device_id = 0x01;
    response.config_id = 0x12345678;

    uint8_t buffer[64];
    size_t size = response.encode(buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(11, size);
    TEST_ASSERT_EQUAL_UINT8(MESSAGE_TYPE_IDENTITY_RESPONSE, buffer[0]);
    TEST_ASSERT_EQUAL_UINT8(0xDD, buffer[1]); // request_id byte 0 (LE)
    TEST_ASSERT_EQUAL_UINT8(0xCC, buffer[2]); // request_id byte 1 (LE)
    TEST_ASSERT_EQUAL_UINT8(0xBB, buffer[3]); // request_id byte 2 (LE)
    TEST_ASSERT_EQUAL_UINT8(0xAA, buffer[4]); // request_id byte 3 (LE)
    TEST_ASSERT_EQUAL_UINT8(0x01, buffer[5]); // version
    TEST_ASSERT_EQUAL_UINT8(0x01, buffer[6]); // device_id
    TEST_ASSERT_EQUAL_UINT8(0x78, buffer[7]); // config_id byte 0 (LE)
    TEST_ASSERT_EQUAL_UINT8(0x56, buffer[8]); // config_id byte 1 (LE)
    TEST_ASSERT_EQUAL_UINT8(0x34, buffer[9]); // config_id byte 2 (LE)
    TEST_ASSERT_EQUAL_UINT8(0x12, buffer[10]); // config_id byte 3 (LE)
}

// Test IdentityResponse decoding
void test_identity_response_decode()
{
    uint8_t buffer[] = { MESSAGE_TYPE_IDENTITY_RESPONSE, 0xDD, 0xCC, 0xBB, 0xAA, 0x01, 0x01, 0x78, 0x56, 0x34, 0x12 };

    IdentityResponse response;
    bool result = response.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(0xAABBCCDD, response.request_id);
    TEST_ASSERT_EQUAL_UINT8(1, response.version);
    TEST_ASSERT_EQUAL_UINT8(0x01, response.device_id);
    TEST_ASSERT_EQUAL_UINT32(0x12345678, response.config_id);
}

// Test IdentityResponse decode with insufficient data
void test_identity_response_decode_insufficient_data()
{
    uint8_t buffer[] = { MESSAGE_TYPE_IDENTITY_RESPONSE, 0xDD, 0xCC, 0xBB }; // Only 4 bytes, need 11

    IdentityResponse response;
    bool result = response.decode(buffer, sizeof(buffer));

    TEST_ASSERT_FALSE(result);
}

// Test IdentityResponse roundtrip
void test_identity_response_roundtrip()
{
    IdentityResponse original;
    original.request_id = 0xDEADBEEF;
    original.version = 5;
    original.device_id = 0xAB;
    original.config_id = 0xCAFEBABE;

    uint8_t buffer[64];
    size_t size = original.encode(buffer, sizeof(buffer));

    IdentityResponse decoded;
    bool result = decoded.decode(buffer, size);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(original.request_id, decoded.request_id);
    TEST_ASSERT_EQUAL_UINT8(original.version, decoded.version);
    TEST_ASSERT_EQUAL_UINT8(original.device_id, decoded.device_id);
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
    uint8_t buffer[] = { MESSAGE_TYPE_IDENTITY_RESPONSE, 0xDD, 0xCC, 0xBB, 0xAA, 0x01, 0x01, 0x78, 0x56, 0x34, 0x12 };

    Message msg;
    bool result = msg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(msg.isIdentityResponse());
    TEST_ASSERT_EQUAL_UINT32(0xAABBCCDD, msg.identity_response.request_id);
    TEST_ASSERT_EQUAL_UINT8(1, msg.identity_response.version);
    TEST_ASSERT_EQUAL_UINT8(0x01, msg.identity_response.device_id);
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

// Test Configure encoding
void test_configure_encode()
{
    Configure cfg;
    cfg.config_id = 0x00000001;
    cfg.total_parts = 3;
    cfg.part_number = 0;
    cfg.input_type = INPUT_TYPE_ANALOG;
    cfg.pin = 0xA0; // A0
    cfg.sensitivity = 128;

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

// Test Configure decoding
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
    TEST_ASSERT_EQUAL_UINT8(0xA0, cfg.pin);
    TEST_ASSERT_EQUAL_UINT8(0x80, cfg.sensitivity);
}

// Test Configure decode with insufficient data
void test_configure_decode_insufficient_data()
{
    uint8_t buffer[] = { MESSAGE_TYPE_CONFIGURE, 0x01, 0x00, 0x00 }; // Only 4 bytes, need 12

    Configure cfg;
    bool result = cfg.decode(buffer, sizeof(buffer));

    TEST_ASSERT_FALSE(result);
}

// Test Configure roundtrip
void test_configure_roundtrip()
{
    Configure original;
    original.config_id = 0xDEADBEEF;
    original.total_parts = 5;
    original.part_number = 2;
    original.input_type = INPUT_TYPE_ANALOG;
    original.pin = 0xA1;
    original.sensitivity = 200;

    uint8_t buffer[64];
    size_t size = original.encode(buffer, sizeof(buffer));

    Configure decoded;
    bool result = decoded.decode(buffer, size);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT32(original.config_id, decoded.config_id);
    TEST_ASSERT_EQUAL_UINT8(original.total_parts, decoded.total_parts);
    TEST_ASSERT_EQUAL_UINT8(original.part_number, decoded.part_number);
    TEST_ASSERT_EQUAL_UINT8(original.input_type, decoded.input_type);
    TEST_ASSERT_EQUAL_UINT8(original.pin, decoded.pin);
    TEST_ASSERT_EQUAL_UINT8(original.sensitivity, decoded.sensitivity);
}

// Test Message decode for Configure
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
    TEST_ASSERT_EQUAL_UINT8(0xA0, msg.configure.pin);
    TEST_ASSERT_EQUAL_UINT8(0x80, msg.configure.sensitivity);
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

    // Configure tests
    RUN_TEST(test_configure_encode);
    RUN_TEST(test_configure_decode);
    RUN_TEST(test_configure_decode_insufficient_data);
    RUN_TEST(test_configure_roundtrip);

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
    RUN_TEST(test_message_decode_configuration_stored);
    RUN_TEST(test_message_decode_configuration_error);
    RUN_TEST(test_message_decode_invalid_type);

    // Error handling tests
    RUN_TEST(test_encode_buffer_too_small);

    return UNITY_END();
}
