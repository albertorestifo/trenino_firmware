# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.0.0] - 2025-12-16

### Added

- **Button input support**: Single digital button with configurable debounce
  - Active-low with internal pullup
  - Reports edge events: press (value=1) and release (value=0)
  - Configurable debounce threshold (number of scan cycles)

- **Matrix input support**: Row/column button grid with N-key rollover
  - Supports arbitrary matrix sizes (limited by 64-byte protocol payload)
  - Virtual pin scheme: `pin = 128 + (row * num_cols + col)`
  - Per-button debouncing
  - Event queue for simultaneous key changes (NKRO)

- **Protocol**: New input type constants
  - `INPUT_TYPE_ANALOG = 0`
  - `INPUT_TYPE_BUTTON = 1`
  - `INPUT_TYPE_MATRIX = 2`

### Changed

- **BREAKING**: `Configure` message format changed to discriminated union
  - Common header: `[type: u8 = 2] [config_id: u32] [total_parts: u8] [part_number: u8] [input_type: u8]`
  - Payload varies by `input_type`:
    - Analog: `[pin: u8] [sensitivity: u8]`
    - Button: `[pin: u8] [debounce: u8]`
    - Matrix: `[num_row_pins: u8] [num_col_pins: u8] [row_pins...] [col_pins...]`

- **BREAKING**: EEPROM format version incremented to 2
  - Existing configurations will be invalidated on firmware upgrade
  - Devices will require reconfiguration after update

### Protocol Migration

**Old Configure format (v1.0.x):**
```
[type: u8 = 2] [config_id: u32] [total_parts: u8] [part_number: u8] [pin: u8] [sensitivity: u8]
```

**New Configure format (v2.0.0):**
```
[type: u8 = 2] [config_id: u32] [total_parts: u8] [part_number: u8] [input_type: u8] [payload...]
```

Hosts must update their protocol implementation to:
1. Include `input_type` field in Configure messages
2. Handle type-specific payloads for analog, button, and matrix inputs
3. Use virtual pins (128+) when receiving InputValue from matrix buttons

## [1.0.1] - 2025-12-10

### Changed

- **BREAKING**: `IdentityResponse` protocol message format changed
  - Version field changed from single `u8` to three separate fields: `version_major`, `version_minor`, `version_patch` (all `u8`)
  - Removed `device_id` field entirely
  - Message size changed from 11 bytes to 12 bytes

### Protocol Migration

**Old format (v1.0.0):**
```
[type: u8 = 1] [request_id: u32] [version: u8] [device_id: u8] [config_id: u32]
```

**New format (v1.0.1):**
```
[type: u8 = 1] [request_id: u32] [version_major: u8] [version_minor: u8] [version_patch: u8] [config_id: u32]
```

Hosts communicating with devices running this firmware version must update their protocol parsers accordingly.

## [1.0.0] - 2025-12-10

### Added

- Initial release
- Binary protocol over serial using COBS framing
- Message types: IdentityRequest, IdentityResponse, Configure, ConfigurationStored, ConfigurationError, InputValue, Heartbeat
- Analog sensor input support with configurable sensitivity
- EEPROM configuration persistence
- Heartbeat keep-alive mechanism
