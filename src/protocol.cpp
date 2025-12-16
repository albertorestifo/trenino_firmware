#include "protocol.h"
#include <string.h>

namespace Protocol {

// IdentityRequest implementation

size_t IdentityRequest::encode(uint8_t* buffer, size_t buffer_size) const
{
    constexpr size_t REQUIRED_SIZE = 5; // 1 byte type + 4 bytes request_id

    if (buffer_size < REQUIRED_SIZE) {
        return 0; // Buffer too small
    }

    size_t offset = 0;

    // Message type (u8)
    buffer[offset++] = MESSAGE_TYPE_IDENTITY_REQUEST;

    // request_id (u32) - little endian
    buffer[offset++] = (request_id >> 0) & 0xFF;
    buffer[offset++] = (request_id >> 8) & 0xFF;
    buffer[offset++] = (request_id >> 16) & 0xFF;
    buffer[offset++] = (request_id >> 24) & 0xFF;

    return offset;
}

bool IdentityRequest::decode(const uint8_t* buffer, size_t length)
{
    constexpr size_t REQUIRED_SIZE = 5;

    if (length < REQUIRED_SIZE) {
        return false; // Not enough data
    }

    if (buffer[0] != MESSAGE_TYPE_IDENTITY_REQUEST) {
        return false; // Wrong message type
    }

    size_t offset = 1;

    // request_id (u32) - little endian
    request_id = ((uint32_t)buffer[offset + 0] << 0) | ((uint32_t)buffer[offset + 1] << 8) | ((uint32_t)buffer[offset + 2] << 16) | ((uint32_t)buffer[offset + 3] << 24);

    return true;
}

// IdentityResponse implementation

size_t IdentityResponse::encode(uint8_t* buffer, size_t buffer_size) const
{
    // 1 type + 4 request_id + 1 version_major + 1 version_minor + 1 version_patch + 4 config_id = 12
    constexpr size_t REQUIRED_SIZE = 12;

    if (buffer_size < REQUIRED_SIZE) {
        return 0; // Buffer too small
    }

    size_t offset = 0;

    // Message type (u8)
    buffer[offset++] = MESSAGE_TYPE_IDENTITY_RESPONSE;

    // request_id (u32) - little endian
    buffer[offset++] = (request_id >> 0) & 0xFF;
    buffer[offset++] = (request_id >> 8) & 0xFF;
    buffer[offset++] = (request_id >> 16) & 0xFF;
    buffer[offset++] = (request_id >> 24) & 0xFF;

    // version_major (u8)
    buffer[offset++] = version_major;

    // version_minor (u8)
    buffer[offset++] = version_minor;

    // version_patch (u8)
    buffer[offset++] = version_patch;

    // config_id (u32) - little endian
    buffer[offset++] = (config_id >> 0) & 0xFF;
    buffer[offset++] = (config_id >> 8) & 0xFF;
    buffer[offset++] = (config_id >> 16) & 0xFF;
    buffer[offset++] = (config_id >> 24) & 0xFF;

    return offset;
}

bool IdentityResponse::decode(const uint8_t* buffer, size_t length)
{
    constexpr size_t REQUIRED_SIZE = 12;

    if (length < REQUIRED_SIZE) {
        return false; // Not enough data
    }

    if (buffer[0] != MESSAGE_TYPE_IDENTITY_RESPONSE) {
        return false; // Wrong message type
    }

    size_t offset = 1;

    // request_id (u32) - little endian
    request_id = ((uint32_t)buffer[offset + 0] << 0) | ((uint32_t)buffer[offset + 1] << 8) | ((uint32_t)buffer[offset + 2] << 16) | ((uint32_t)buffer[offset + 3] << 24);
    offset += 4;

    // version_major (u8)
    version_major = buffer[offset++];

    // version_minor (u8)
    version_minor = buffer[offset++];

    // version_patch (u8)
    version_patch = buffer[offset++];

    // config_id (u32) - little endian
    config_id = ((uint32_t)buffer[offset + 0] << 0) | ((uint32_t)buffer[offset + 1] << 8) | ((uint32_t)buffer[offset + 2] << 16) | ((uint32_t)buffer[offset + 3] << 24);

    return true;
}

// Configure implementation

size_t Configure::encode(uint8_t* buffer, size_t buffer_size) const
{
    // Common header: 1 type + 4 config_id + 1 total_parts + 1 part_number + 1 input_type = 8 bytes
    constexpr size_t HEADER_SIZE = 8;

    // Calculate required size based on input_type
    size_t payload_size = 0;
    switch (input_type) {
    case INPUT_TYPE_ANALOG:
        payload_size = 2; // pin + sensitivity
        break;
    case INPUT_TYPE_BUTTON:
        payload_size = 2; // pin + debounce
        break;
    case INPUT_TYPE_MATRIX:
        payload_size = 2 + matrix.num_row_pins + matrix.num_col_pins; // counts + pins
        break;
    default:
        return 0; // Unknown input type
    }

    size_t required_size = HEADER_SIZE + payload_size;
    if (buffer_size < required_size) {
        return 0; // Buffer too small
    }

    size_t offset = 0;

    // Message type (u8)
    buffer[offset++] = MESSAGE_TYPE_CONFIGURE;

    // config_id (u32) - little endian
    buffer[offset++] = (config_id >> 0) & 0xFF;
    buffer[offset++] = (config_id >> 8) & 0xFF;
    buffer[offset++] = (config_id >> 16) & 0xFF;
    buffer[offset++] = (config_id >> 24) & 0xFF;

    // total_parts (u8)
    buffer[offset++] = total_parts;

    // part_number (u8)
    buffer[offset++] = part_number;

    // input_type (u8)
    buffer[offset++] = input_type;

    // Type-specific payload
    switch (input_type) {
    case INPUT_TYPE_ANALOG:
        buffer[offset++] = analog.pin;
        buffer[offset++] = analog.sensitivity;
        break;

    case INPUT_TYPE_BUTTON:
        buffer[offset++] = button.pin;
        buffer[offset++] = button.debounce;
        break;

    case INPUT_TYPE_MATRIX:
        buffer[offset++] = matrix.num_row_pins;
        buffer[offset++] = matrix.num_col_pins;
        for (uint8_t i = 0; i < matrix.num_row_pins + matrix.num_col_pins; i++) {
            buffer[offset++] = matrix.pins[i];
        }
        break;
    }

    return offset;
}

bool Configure::decode(const uint8_t* buffer, size_t length)
{
    // Minimum size: header (8 bytes)
    constexpr size_t HEADER_SIZE = 8;

    if (length < HEADER_SIZE) {
        return false; // Not enough data for header
    }

    if (buffer[0] != MESSAGE_TYPE_CONFIGURE) {
        return false; // Wrong message type
    }

    size_t offset = 1;

    // config_id (u32) - little endian
    config_id = ((uint32_t)buffer[offset + 0] << 0) | ((uint32_t)buffer[offset + 1] << 8) | ((uint32_t)buffer[offset + 2] << 16) | ((uint32_t)buffer[offset + 3] << 24);
    offset += 4;

    // total_parts (u8)
    total_parts = buffer[offset++];

    // part_number (u8)
    part_number = buffer[offset++];

    // input_type (u8)
    input_type = buffer[offset++];

    // Type-specific payload
    switch (input_type) {
    case INPUT_TYPE_ANALOG:
        if (length < HEADER_SIZE + 2) {
            return false; // Not enough data for analog payload
        }
        analog.pin = buffer[offset++];
        analog.sensitivity = buffer[offset++];
        break;

    case INPUT_TYPE_BUTTON:
        if (length < HEADER_SIZE + 2) {
            return false; // Not enough data for button payload
        }
        button.pin = buffer[offset++];
        button.debounce = buffer[offset++];
        break;

    case INPUT_TYPE_MATRIX: {
        if (length < HEADER_SIZE + 2) {
            return false; // Not enough data for matrix header
        }
        matrix.num_row_pins = buffer[offset++];
        matrix.num_col_pins = buffer[offset++];

        uint8_t total_pins = matrix.num_row_pins + matrix.num_col_pins;
        if (total_pins > MAX_MATRIX_PINS) {
            return false; // Too many pins
        }
        if (length < HEADER_SIZE + 2 + total_pins) {
            return false; // Not enough data for matrix pins
        }
        for (uint8_t i = 0; i < total_pins; i++) {
            matrix.pins[i] = buffer[offset++];
        }
        break;
    }

    default:
        return false; // Unknown input type
    }

    return true;
}

// ConfigurationStored implementation

size_t ConfigurationStored::encode(uint8_t* buffer, size_t buffer_size) const
{
    constexpr size_t REQUIRED_SIZE = 5; // 1 type + 4 config_id

    if (buffer_size < REQUIRED_SIZE) {
        return 0; // Buffer too small
    }

    size_t offset = 0;

    // Message type (u8)
    buffer[offset++] = MESSAGE_TYPE_CONFIGURATION_STORED;

    // config_id (u32) - little endian
    buffer[offset++] = (config_id >> 0) & 0xFF;
    buffer[offset++] = (config_id >> 8) & 0xFF;
    buffer[offset++] = (config_id >> 16) & 0xFF;
    buffer[offset++] = (config_id >> 24) & 0xFF;

    return offset;
}

bool ConfigurationStored::decode(const uint8_t* buffer, size_t length)
{
    constexpr size_t REQUIRED_SIZE = 5;

    if (length < REQUIRED_SIZE) {
        return false; // Not enough data
    }

    if (buffer[0] != MESSAGE_TYPE_CONFIGURATION_STORED) {
        return false; // Wrong message type
    }

    size_t offset = 1;

    // config_id (u32) - little endian
    config_id = ((uint32_t)buffer[offset + 0] << 0) | ((uint32_t)buffer[offset + 1] << 8) | ((uint32_t)buffer[offset + 2] << 16) | ((uint32_t)buffer[offset + 3] << 24);

    return true;
}

// ConfigurationError implementation

size_t ConfigurationError::encode(uint8_t* buffer, size_t buffer_size) const
{
    constexpr size_t REQUIRED_SIZE = 5; // 1 type + 4 config_id

    if (buffer_size < REQUIRED_SIZE) {
        return 0; // Buffer too small
    }

    size_t offset = 0;

    // Message type (u8)
    buffer[offset++] = MESSAGE_TYPE_CONFIGURATION_ERROR;

    // config_id (u32) - little endian
    buffer[offset++] = (config_id >> 0) & 0xFF;
    buffer[offset++] = (config_id >> 8) & 0xFF;
    buffer[offset++] = (config_id >> 16) & 0xFF;
    buffer[offset++] = (config_id >> 24) & 0xFF;

    return offset;
}

bool ConfigurationError::decode(const uint8_t* buffer, size_t length)
{
    constexpr size_t REQUIRED_SIZE = 5;

    if (length < REQUIRED_SIZE) {
        return false; // Not enough data
    }

    if (buffer[0] != MESSAGE_TYPE_CONFIGURATION_ERROR) {
        return false; // Wrong message type
    }

    size_t offset = 1;

    // config_id (u32) - little endian
    config_id = ((uint32_t)buffer[offset + 0] << 0) | ((uint32_t)buffer[offset + 1] << 8) | ((uint32_t)buffer[offset + 2] << 16) | ((uint32_t)buffer[offset + 3] << 24);

    return true;
}

// InputValue implementation

size_t InputValue::encode(uint8_t* buffer, size_t buffer_size) const
{
    constexpr size_t REQUIRED_SIZE = 4;

    if (buffer_size < REQUIRED_SIZE) {
        return 0; // Buffer too small
    }

    size_t offset = 0;

    // Message type (u8)
    buffer[offset++] = MESSAGE_TYPE_INPUT_VALUE;

    // pin (u8)
    buffer[offset++] = pin;

    // value (i16) - little endian
    buffer[offset++] = (value >> 0) & 0xFF;
    buffer[offset++] = (value >> 8) & 0xFF;

    return offset;
}

bool InputValue::decode(const uint8_t* buffer, size_t length)
{
    constexpr size_t REQUIRED_SIZE = 4;

    if (length < REQUIRED_SIZE) {
        return false; // Not enough data
    }

    if (buffer[0] != MESSAGE_TYPE_INPUT_VALUE) {
        return false; // Wrong message type
    }

    size_t offset = 1;

    // pin (u8)
    pin = buffer[offset++];

    // value (i16) - little endian
    value = (int16_t)(((uint16_t)buffer[offset + 0] << 0) | ((uint16_t)buffer[offset + 1] << 8));

    return true;
}

// Heartbeat implementation

size_t Heartbeat::encode(uint8_t* buffer, size_t buffer_size) const
{
    constexpr size_t REQUIRED_SIZE = 1; // Just the message type

    if (buffer_size < REQUIRED_SIZE) {
        return 0; // Buffer too small
    }

    buffer[0] = MESSAGE_TYPE_HEARTBEAT;

    return REQUIRED_SIZE;
}

bool Heartbeat::decode(const uint8_t* buffer, size_t length)
{
    constexpr size_t REQUIRED_SIZE = 1;

    if (length < REQUIRED_SIZE) {
        return false; // Not enough data
    }

    if (buffer[0] != MESSAGE_TYPE_HEARTBEAT) {
        return false; // Wrong message type
    }

    return true;
}

// Message implementation (for generic decoding)

bool Message::decode(const uint8_t* buffer, size_t length)
{
    if (length < 1) {
        return false; // Need at least message type
    }

    message_type = buffer[0];

    switch (message_type) {
    case MESSAGE_TYPE_IDENTITY_REQUEST:
        return identity_request.decode(buffer, length);

    case MESSAGE_TYPE_IDENTITY_RESPONSE:
        return identity_response.decode(buffer, length);

    case MESSAGE_TYPE_CONFIGURE:
        return configure.decode(buffer, length);

    case MESSAGE_TYPE_CONFIGURATION_STORED:
        return configuration_stored.decode(buffer, length);

    case MESSAGE_TYPE_CONFIGURATION_ERROR:
        return configuration_error.decode(buffer, length);

    case MESSAGE_TYPE_INPUT_VALUE:
        return input_value.decode(buffer, length);

    case MESSAGE_TYPE_HEARTBEAT:
        return heartbeat.decode(buffer, length);

    default:
        return false; // Unknown message type
    }
}

} // namespace Protocol
