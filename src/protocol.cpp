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

namespace {

constexpr uint8_t VIRTUAL_PIN_MIN = 128;
constexpr uint8_t MCP_DIRECT_BLOCK = 16;
constexpr uint8_t ADS_BLOCK = 4;
constexpr uint16_t MCP_NO_INPUT_MASK = 0x8080; // GPA7 and GPB7 are output-only

uint8_t popcount16(uint16_t v)
{
    uint8_t n = 0;
    for (; v; v &= v - 1) n++;
    return n;
}

// A used block needs a base from 128 up whose last pin is at most 255.
bool blockFits(uint8_t base, uint8_t size)
{
    return base >= VIRTUAL_PIN_MIN && (uint16_t)base + size <= 256;
}

} // namespace

bool validMcp23017(const Mcp23017Config& c)
{
    if (c.debounce == 0) return false;
    if (!(c.direct_mask | c.row_mask | c.col_mask)) return false;
    if ((c.direct_mask & c.row_mask) || (c.direct_mask & c.col_mask) || (c.row_mask & c.col_mask)) return false;
    if ((c.direct_mask | c.col_mask) & MCP_NO_INPUT_MASK) return false;
    uint8_t rows = popcount16(c.row_mask);
    uint8_t cols = popcount16(c.col_mask);
    if (rows > 8 || cols > 8) return false;
    if ((rows == 0) != (cols == 0)) return false;
    if (c.direct_mask && !blockFits(c.direct_base, MCP_DIRECT_BLOCK)) return false;
    if (rows && !blockFits(c.matrix_base, rows * cols)) return false;
    return true;
}

bool validAds1115(const Ads1115Config& c)
{
    if (c.channel_mask == 0 || (c.channel_mask & 0xF0)) return false;
    if (c.gain > 1) return false;
    if (c.ref_channel != ADS_NO_REFERENCE && (c.ref_channel > 3 || (c.channel_mask & (1 << c.ref_channel)))) return false;
    if (!blockFits(c.pin_base, ADS_BLOCK)) return false;
    for (uint8_t i = 0; i < 4; i++) {
        if ((c.channel_mask & (1 << i)) && (c.sensitivity[i] < 1 || c.sensitivity[i] > 10)) return false;
    }
    return true;
}

size_t Configure::encode(uint8_t* buffer, size_t buffer_size) const
{
    // Common header: 1 type + 4 config_id + 1 total_parts + 1 part_number + 1 module_type = 8 bytes
    constexpr size_t HEADER_SIZE = 8;

    if ((total_parts == 0 || module_type == MODULE_TYPE_EMPTY) &&
        !(total_parts == 0 && part_number == 0 && module_type == MODULE_TYPE_EMPTY)) return 0;

    // Calculate required size based on module_type
    size_t payload_size = 0;
    switch (module_type) {
    case MODULE_TYPE_EMPTY:
        break;
    case MODULE_TYPE_ANALOG:
        payload_size = 2; // pin + sensitivity
        break;
    case MODULE_TYPE_BUTTON:
        payload_size = 2; // pin + debounce
        break;
    case MODULE_TYPE_MATRIX:
        payload_size = 2 + matrix.num_row_pins + matrix.num_col_pins; // counts + pins
        break;
    case MODULE_TYPE_HT16K33:
        payload_size = 3; // i2c_address + brightness + num_digits
        break;
    case MODULE_TYPE_MCP23017:
        payload_size = 10;
        break;
    case MODULE_TYPE_ADS1115:
        payload_size = 9;
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

    // module_type (u8)
    buffer[offset++] = module_type;

    // Type-specific payload
    switch (module_type) {
    case MODULE_TYPE_ANALOG:
        buffer[offset++] = analog.pin;
        buffer[offset++] = analog.sensitivity;
        break;

    case MODULE_TYPE_BUTTON:
        buffer[offset++] = button.pin;
        buffer[offset++] = button.debounce;
        break;

    case MODULE_TYPE_MATRIX:
        buffer[offset++] = matrix.num_row_pins;
        buffer[offset++] = matrix.num_col_pins;
        for (uint8_t i = 0; i < matrix.num_row_pins + matrix.num_col_pins; i++) {
            buffer[offset++] = matrix.pins[i];
        }
        break;

    case MODULE_TYPE_HT16K33:
        buffer[offset++] = ht16k33.i2c_address;
        buffer[offset++] = ht16k33.brightness;
        buffer[offset++] = ht16k33.num_digits;
        break;

    case MODULE_TYPE_MCP23017:
        buffer[offset++] = mcp23017.i2c_address;
        buffer[offset++] = mcp23017.direct_base;
        buffer[offset++] = mcp23017.matrix_base;
        buffer[offset++] = mcp23017.debounce;
        buffer[offset++] = mcp23017.direct_mask & 0xFF;
        buffer[offset++] = mcp23017.direct_mask >> 8;
        buffer[offset++] = mcp23017.row_mask & 0xFF;
        buffer[offset++] = mcp23017.row_mask >> 8;
        buffer[offset++] = mcp23017.col_mask & 0xFF;
        buffer[offset++] = mcp23017.col_mask >> 8;
        break;

    case MODULE_TYPE_ADS1115:
        buffer[offset++] = ads1115.i2c_address;
        buffer[offset++] = ads1115.pin_base;
        buffer[offset++] = ads1115.channel_mask;
        buffer[offset++] = ads1115.gain;
        buffer[offset++] = ads1115.ref_channel;
        for (uint8_t i = 0; i < 4; i++) buffer[offset++] = ads1115.sensitivity[i];
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

    // module_type (u8)
    module_type = buffer[offset++];

    if (total_parts == 0 || module_type == MODULE_TYPE_EMPTY) {
        return total_parts == 0 && part_number == 0 && module_type == MODULE_TYPE_EMPTY && length == HEADER_SIZE;
    }

    // Type-specific payload
    switch (module_type) {
    case MODULE_TYPE_ANALOG:
        if (length < HEADER_SIZE + 2) {
            return false; // Not enough data for analog payload
        }
        analog.pin = buffer[offset++];
        analog.sensitivity = buffer[offset++];
        break;

    case MODULE_TYPE_BUTTON:
        if (length < HEADER_SIZE + 2) {
            return false; // Not enough data for button payload
        }
        button.pin = buffer[offset++];
        button.debounce = buffer[offset++];
        break;

    case MODULE_TYPE_MATRIX: {
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

    case MODULE_TYPE_HT16K33:
        if (length < HEADER_SIZE + 3) {
            return false; // Not enough data for HT16K33 payload
        }
        ht16k33.i2c_address = buffer[offset++];
        ht16k33.brightness = buffer[offset++];
        ht16k33.num_digits = buffer[offset++];
        break;

    case MODULE_TYPE_MCP23017: {
        if (length != HEADER_SIZE + 10) {
            return false;
        }
        Mcp23017Config c;
        c.i2c_address = buffer[offset++];
        c.direct_base = buffer[offset++];
        c.matrix_base = buffer[offset++];
        c.debounce = buffer[offset++];
        c.direct_mask = (uint16_t)(buffer[offset] | (buffer[offset + 1] << 8));
        c.row_mask = (uint16_t)(buffer[offset + 2] | (buffer[offset + 3] << 8));
        c.col_mask = (uint16_t)(buffer[offset + 4] | (buffer[offset + 5] << 8));
        if (!validMcp23017(c)) {
            return false;
        }
        mcp23017 = c;
        break;
    }

    case MODULE_TYPE_ADS1115: {
        if (length != HEADER_SIZE + 9) {
            return false;
        }
        Ads1115Config c;
        c.i2c_address = buffer[offset++];
        c.pin_base = buffer[offset++];
        c.channel_mask = buffer[offset++];
        c.gain = buffer[offset++];
        c.ref_channel = buffer[offset++];
        for (uint8_t i = 0; i < 4; i++) c.sensitivity[i] = buffer[offset++];
        if (!validAds1115(c)) {
            return false;
        }
        ads1115 = c;
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

// SetOutput implementation

size_t SetOutput::encode(uint8_t* buffer, size_t buffer_size) const
{
    constexpr size_t REQUIRED_SIZE = 3; // 1 type + 1 pin + 1 value

    if (buffer_size < REQUIRED_SIZE) {
        return 0; // Buffer too small
    }

    size_t offset = 0;

    // Message type (u8)
    buffer[offset++] = MESSAGE_TYPE_SET_OUTPUT;

    // pin (u8)
    buffer[offset++] = pin;

    // value (u8)
    buffer[offset++] = value;

    return offset;
}

bool SetOutput::decode(const uint8_t* buffer, size_t length)
{
    constexpr size_t REQUIRED_SIZE = 3;

    if (length < REQUIRED_SIZE) {
        return false; // Not enough data
    }

    if (buffer[0] != MESSAGE_TYPE_SET_OUTPUT) {
        return false; // Wrong message type
    }

    size_t offset = 1;

    // pin (u8)
    pin = buffer[offset++];

    // value (u8)
    value = buffer[offset++];

    return true;
}

// EncoderError implementation

size_t EncoderError::encode(uint8_t* buffer, size_t buffer_size) const
{
    constexpr size_t REQUIRED_SIZE = 2; // 1 type + 1 pin

    if (buffer_size < REQUIRED_SIZE) {
        return 0; // Buffer too small
    }

    size_t offset = 0;

    // Message type (u8)
    buffer[offset++] = MESSAGE_TYPE_ENCODER_ERROR;

    // pin (u8)
    buffer[offset++] = pin;

    return offset;
}

bool EncoderError::decode(const uint8_t* buffer, size_t length)
{
    constexpr size_t REQUIRED_SIZE = 2;

    if (length < REQUIRED_SIZE) {
        return false; // Not enough data
    }

    if (buffer[0] != MESSAGE_TYPE_ENCODER_ERROR) {
        return false; // Wrong message type
    }

    size_t offset = 1;

    // pin (u8)
    pin = buffer[offset++];

    return true;
}

// LoadBLDCProfile implementation

size_t LoadBLDCProfile::encode(uint8_t* buffer, size_t buffer_size) const
{
    constexpr size_t REQUIRED_SIZE = 10; // 1 type + 1 pin + 1 num_detents + 1 num_linear_ranges + 1 snap_point + 1 endstop_strength + 2 position_start + 2 position_end

    if (buffer_size < REQUIRED_SIZE) {
        return 0; // Buffer too small
    }

    size_t offset = 0;

    // Message type (u8)
    buffer[offset++] = MESSAGE_TYPE_LOAD_BLDC_PROFILE;

    // pin (u8)
    buffer[offset++] = pin;

    // num_detents (u8)
    buffer[offset++] = num_detents;

    // num_linear_ranges (u8)
    buffer[offset++] = num_linear_ranges;

    // snap_point (u8)
    buffer[offset++] = snap_point;

    // endstop_strength (u8)
    buffer[offset++] = endstop_strength;

    // position_start (i16) - little endian
    buffer[offset++] = (position_start >> 0) & 0xFF;
    buffer[offset++] = (position_start >> 8) & 0xFF;

    // position_end (i16) - little endian
    buffer[offset++] = (position_end >> 0) & 0xFF;
    buffer[offset++] = (position_end >> 8) & 0xFF;

    return offset;
}

bool LoadBLDCProfile::decode(const uint8_t* buffer, size_t length)
{
    constexpr size_t REQUIRED_SIZE = 10;

    if (length < REQUIRED_SIZE) {
        return false; // Not enough data
    }

    if (buffer[0] != MESSAGE_TYPE_LOAD_BLDC_PROFILE) {
        return false; // Wrong message type
    }

    size_t offset = 1;

    // pin (u8)
    pin = buffer[offset++];

    // num_detents (u8)
    num_detents = buffer[offset++];

    // num_linear_ranges (u8)
    num_linear_ranges = buffer[offset++];

    // snap_point (u8)
    snap_point = buffer[offset++];

    // endstop_strength (u8)
    endstop_strength = buffer[offset++];

    // position_start (i16) - little endian
    position_start = (int16_t)(((uint16_t)buffer[offset + 0] << 0) | ((uint16_t)buffer[offset + 1] << 8));
    offset += 2;

    // position_end (i16) - little endian
    position_end = (int16_t)(((uint16_t)buffer[offset + 0] << 0) | ((uint16_t)buffer[offset + 1] << 8));
    offset += 2;

    return true;
}

// DeactivateBLDCProfile implementation

size_t DeactivateBLDCProfile::encode(uint8_t* buffer, size_t buffer_size) const
{
    constexpr size_t REQUIRED_SIZE = 2; // 1 type + 1 pin

    if (buffer_size < REQUIRED_SIZE) {
        return 0; // Buffer too small
    }

    size_t offset = 0;

    // Message type (u8)
    buffer[offset++] = MESSAGE_TYPE_DEACTIVATE_BLDC_PROFILE;

    // pin (u8)
    buffer[offset++] = pin;

    return offset;
}

bool DeactivateBLDCProfile::decode(const uint8_t* buffer, size_t length)
{
    constexpr size_t REQUIRED_SIZE = 2;

    if (length < REQUIRED_SIZE) {
        return false; // Not enough data
    }

    if (buffer[0] != MESSAGE_TYPE_DEACTIVATE_BLDC_PROFILE) {
        return false; // Wrong message type
    }

    size_t offset = 1;

    // pin (u8)
    pin = buffer[offset++];

    return true;
}

// WriteSegments implementation

size_t WriteSegments::encode(uint8_t* buffer, size_t buffer_size) const
{
    if (num_bytes > MAX_SEGMENT_BYTES) {
        return 0;
    }
    size_t required = 3 + num_bytes;
    if (buffer_size < required) {
        return 0;
    }

    size_t offset = 0;
    buffer[offset++] = MESSAGE_TYPE_WRITE_SEGMENTS;
    buffer[offset++] = i2c_address;
    buffer[offset++] = num_bytes;
    for (uint8_t i = 0; i < num_bytes; i++) {
        buffer[offset++] = data[i];
    }
    return offset;
}

bool WriteSegments::decode(const uint8_t* buffer, size_t length)
{
    if (length < 3) return false;
    if (buffer[0] != MESSAGE_TYPE_WRITE_SEGMENTS) return false;

    i2c_address = buffer[1];
    num_bytes = buffer[2];
    if (num_bytes > MAX_SEGMENT_BYTES) return false;
    if (length < (size_t)(3 + num_bytes)) return false;

    for (uint8_t i = 0; i < num_bytes; i++) {
        data[i] = buffer[3 + i];
    }
    return true;
}

// SetModuleBrightness implementation

size_t SetModuleBrightness::encode(uint8_t* buffer, size_t buffer_size) const
{
    if (buffer_size < 3) return 0;
    buffer[0] = MESSAGE_TYPE_SET_MODULE_BRIGHTNESS;
    buffer[1] = i2c_address;
    buffer[2] = brightness;
    return 3;
}

bool SetModuleBrightness::decode(const uint8_t* buffer, size_t length)
{
    if (length < 3) return false;
    if (buffer[0] != MESSAGE_TYPE_SET_MODULE_BRIGHTNESS) return false;
    i2c_address = buffer[1];
    brightness = buffer[2];
    return true;
}

// ModuleError implementation

size_t ModuleError::encode(uint8_t* buffer, size_t buffer_size) const
{
    if (buffer_size < 3) return 0;
    buffer[0] = MESSAGE_TYPE_MODULE_ERROR;
    buffer[1] = i2c_address;
    buffer[2] = error_code;
    return 3;
}

bool ModuleError::decode(const uint8_t* buffer, size_t length)
{
    if (length < 3) return false;
    if (buffer[0] != MESSAGE_TYPE_MODULE_ERROR) return false;
    i2c_address = buffer[1];
    error_code = buffer[2];
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

    case MESSAGE_TYPE_SET_OUTPUT:
        return set_output.decode(buffer, length);

    case MESSAGE_TYPE_ENCODER_ERROR:
        return encoder_error.decode(buffer, length);

    case MESSAGE_TYPE_LOAD_BLDC_PROFILE:
        return load_bldc_profile.decode(buffer, length);

    case MESSAGE_TYPE_DEACTIVATE_BLDC_PROFILE:
        return deactivate_bldc_profile.decode(buffer, length);

    case MESSAGE_TYPE_WRITE_SEGMENTS:
        return write_segments.decode(buffer, length);

    case MESSAGE_TYPE_SET_MODULE_BRIGHTNESS:
        return set_module_brightness.decode(buffer, length);

    case MESSAGE_TYPE_MODULE_ERROR:
        return module_error.decode(buffer, length);

    default:
        return false; // Unknown message type
    }
}

} // namespace Protocol
