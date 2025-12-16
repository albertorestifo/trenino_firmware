#pragma once

#include <stddef.h>
#include <stdint.h>

namespace Protocol {

// Message Type constants
constexpr uint8_t MESSAGE_TYPE_IDENTITY_REQUEST = 0;
constexpr uint8_t MESSAGE_TYPE_IDENTITY_RESPONSE = 1;
constexpr uint8_t MESSAGE_TYPE_CONFIGURE = 2;
constexpr uint8_t MESSAGE_TYPE_CONFIGURATION_STORED = 3;
constexpr uint8_t MESSAGE_TYPE_CONFIGURATION_ERROR = 4;
constexpr uint8_t MESSAGE_TYPE_INPUT_VALUE = 5;
constexpr uint8_t MESSAGE_TYPE_HEARTBEAT = 6;

// Input Type constants for Configure message
constexpr uint8_t INPUT_TYPE_ANALOG = 0;
constexpr uint8_t INPUT_TYPE_BUTTON = 1;
constexpr uint8_t INPUT_TYPE_MATRIX = 2;

// Maximum number of pins for matrix configuration (row_pins + col_pins)
constexpr uint8_t MAX_MATRIX_PINS = 16;

// Maximum payload size
constexpr size_t MAX_PAYLOAD_SIZE = 64;

// Identity Request message
struct IdentityRequest {
    uint32_t request_id;

    // Encode to buffer (returns number of bytes written, 0 on error)
    size_t encode(uint8_t* buffer, size_t buffer_size) const;

    // Decode from buffer (returns true on success)
    bool decode(const uint8_t* buffer, size_t length);
};

// Identity Response message
struct IdentityResponse {
    uint32_t request_id;
    uint8_t version_major;
    uint8_t version_minor;
    uint8_t version_patch;
    uint32_t config_id;

    // Encode to buffer (returns number of bytes written, 0 on error)
    size_t encode(uint8_t* buffer, size_t buffer_size) const;

    // Decode from buffer (returns true on success)
    bool decode(const uint8_t* buffer, size_t length);
};

// Configure message - sent by host to configure device inputs
// Uses a discriminated union based on input_type
struct Configure {
    uint32_t config_id;
    uint8_t total_parts;
    uint8_t part_number;
    uint8_t input_type;

    // Type-specific payload (discriminated by input_type)
    union {
        // INPUT_TYPE_ANALOG
        struct {
            uint8_t pin;
            uint8_t sensitivity;
        } analog;

        // INPUT_TYPE_BUTTON
        struct {
            uint8_t pin;
            uint8_t debounce;
        } button;

        // INPUT_TYPE_MATRIX
        struct {
            uint8_t num_row_pins;
            uint8_t num_col_pins;
            uint8_t pins[MAX_MATRIX_PINS]; // row_pins followed by col_pins
        } matrix;
    };

    Configure()
        : config_id(0)
        , total_parts(0)
        , part_number(0)
        , input_type(INPUT_TYPE_ANALOG)
    {
        analog.pin = 0;
        analog.sensitivity = 0;
    }

    // Encode to buffer (returns number of bytes written, 0 on error)
    size_t encode(uint8_t* buffer, size_t buffer_size) const;

    // Decode from buffer (returns true on success)
    bool decode(const uint8_t* buffer, size_t length);
};

// ConfigurationStored message - sent by device when configuration is successfully stored
struct ConfigurationStored {
    uint32_t config_id;

    // Encode to buffer (returns number of bytes written, 0 on error)
    size_t encode(uint8_t* buffer, size_t buffer_size) const;

    // Decode from buffer (returns true on success)
    bool decode(const uint8_t* buffer, size_t length);
};

// ConfigurationError message - sent by device when configuration fails
struct ConfigurationError {
    uint32_t config_id;

    // Encode to buffer (returns number of bytes written, 0 on error)
    size_t encode(uint8_t* buffer, size_t buffer_size) const;

    // Decode from buffer (returns true on success)
    bool decode(const uint8_t* buffer, size_t length);
};

// InputValue message - sent by device when sensor has a value to report
struct InputValue {
    uint8_t pin;
    int16_t value;

    // Encode to buffer (returns number of bytes written, 0 on error)
    size_t encode(uint8_t* buffer, size_t buffer_size) const;

    // Decode from buffer (returns true on success)
    bool decode(const uint8_t* buffer, size_t length);
};

// Heartbeat message - sent periodically by device to keep connection alive
struct Heartbeat {
    // Encode to buffer (returns number of bytes written, 0 on error)
    size_t encode(uint8_t* buffer, size_t buffer_size) const;

    // Decode from buffer (returns true on success)
    bool decode(const uint8_t* buffer, size_t length);
};

// Generic message union for decoding
struct Message {
    uint8_t message_type;

    union {
        IdentityRequest identity_request;
        IdentityResponse identity_response;
        Configure configure;
        ConfigurationStored configuration_stored;
        ConfigurationError configuration_error;
        InputValue input_value;
        Heartbeat heartbeat;
    };

    Message()
        : message_type(MESSAGE_TYPE_IDENTITY_REQUEST)
    {
        identity_request.request_id = 0;
    }

    // Decode message from buffer (returns true on success)
    // Sets message_type and populates the appropriate union member
    bool decode(const uint8_t* buffer, size_t length);

    // Check if this is an IdentityRequest
    bool isIdentityRequest() const { return message_type == MESSAGE_TYPE_IDENTITY_REQUEST; }

    // Check if this is an IdentityResponse
    bool isIdentityResponse() const { return message_type == MESSAGE_TYPE_IDENTITY_RESPONSE; }

    // Check if this is a Configure message
    bool isConfigure() const { return message_type == MESSAGE_TYPE_CONFIGURE; }

    // Check if this is a ConfigurationStored message
    bool isConfigurationStored() const { return message_type == MESSAGE_TYPE_CONFIGURATION_STORED; }

    // Check if this is a ConfigurationError message
    bool isConfigurationError() const { return message_type == MESSAGE_TYPE_CONFIGURATION_ERROR; }

    // Check if this is an InputValue message
    bool isInputValue() const { return message_type == MESSAGE_TYPE_INPUT_VALUE; }

    // Check if this is a Heartbeat message
    bool isHeartbeat() const { return message_type == MESSAGE_TYPE_HEARTBEAT; }
};

} // namespace Protocol
