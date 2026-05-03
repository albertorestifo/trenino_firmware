# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **HT16K33 14-segment LED display support** — new I2C-attached module type (`MODULE_TYPE_HT16K33 = 4`) for driving Holtek HT16K33-based 14-segment LED displays.
  - Configured via the existing `Configure` message with `i2c_address`, initial `brightness` (0–15), and `num_digits` (4 or 8).
  - Three new wire-level messages: `WriteSegments` (13), `SetModuleBrightness` (14), `ModuleError` (15). See `docs/PROTOCOL.md` for full wire format and the new "Driving an HT16K33 from the Host" guide for implementation walkthroughs.
  - Configuration persists across reboots via EEPROM; the chip is re-initialized automatically on power-up.
  - Resilient to transient I2C bus noise: a single retry after ~1 ms on NACK; after three consecutive write failures the firmware silently re-initializes the chip and replays the last cached segment write before retrying.
  - Brightness updates persist through auto-reinit (the cached value is used during the chip's brightness step).

- **I2C-addressable modules** — modules can now be identified by I2C address (the new `i2c_address` field) in addition to hardware pin. This generalizes the configuration pipeline to accept future I2C-attached modules (e.g. MCP23017 GPIO expanders, ADS1115 ADCs) without further architectural changes. Documented in the new "Addressing Scheme" section of `docs/PROTOCOL.md`.

- **`ModuleError (15)` message** — Device → Host. Sent after `applyConfiguration` for each module whose `begin()` failed (e.g. an HT16K33 chip that NACKed because it isn't on the bus or has a wrong address). Lets hosts surface a meaningful error to the user instead of silently leaving the module inoperative.

### Changed

- **BREAKING (protocol)**: terminology renamed throughout from "input" to "module".
  - `INPUT_TYPE_*` constants → `MODULE_TYPE_*`.
  - `input_type` field on `Configure` → `module_type`.
  - The internal `Sensors::` namespace, `ISensor` interface, and `SensorManager` are renamed to `Modules::`, `IModule`, and `ModuleManager`. The header `sensor.h` is now `module.h`.
  - `MAX_INPUTS` / `MAX_SENSORS` constants unified as `MAX_MODULES` (still 8).
- **BREAKING (storage)**: EEPROM format version incremented from 4 to 5. Existing configurations are discarded on upgrade; reconfigure each device after flashing.
- **`IModule` interface (`begin()` now returns `bool`)** — modules whose initialization can fail (currently HT16K33; future I2C input modules) signal failure; modules that cannot fail simply `return true`. Default empty implementations of `scan()` and `getReading()` mean output-only modules don't need stub overrides.
- **`ModuleManager`** gained `getModuleByI2CAddress(addr)` (used to dispatch `WriteSegments` / `SetModuleBrightness`) and `getInitErrors(out, max)` (drained by the message handler after `applyConfiguration` to emit `ModuleError` messages).
- Configuration sequence: after `ConfigurationStored`, the device may now emit zero or more `ModuleError` messages — one per module whose `begin()` failed.

### Removed

- **BLDC haptic lever** — the `BLDCLever` class, `BLDCManager`, SimpleFOC dependency, and all related test mocks have been removed. This was a failed experiment that never tagged a release; everything was on `[Unreleased]` and is now reverted.
  - The wire-level protocol stubs `EncoderError (10)`, `LoadBLDCProfile (11)`, and `DeactivateBLDCProfile (12)` are retained for backwards compatibility — the firmware decodes them but performs no action. Hosts may continue to emit them harmlessly.
  - `MODULE_TYPE_BLDC_LEVER` (was type ID 3) is no longer accepted in `Configure`; type ID 3 is now reserved. The HT16K33 module uses type ID 4 to keep the historical numbering.
  - SimpleFOC is no longer required for any build; the `megaatmega2560` environment no longer pulls it in. All eight supported environments build cleanly without it.

### Protocol Migration

To upgrade an existing host implementation:

1. **Reconfigure devices after flashing.** EEPROM format version went 4 → 5; old configurations are erased.
2. **Update protocol constants.** Rename `INPUT_TYPE_*` → `MODULE_TYPE_*` and `input_type` → `module_type` in your host code. Wire-level layout of `Configure` is otherwise unchanged for analog/button/matrix variants — only the field/constant names changed.
3. **Stop sending `MODULE_TYPE_BLDC_LEVER` (3) Configures.** The firmware no longer accepts BLDC configuration. If your host still sends `LoadBLDCProfile` / `DeactivateBLDCProfile` they will be silently ignored — no harm, but no effect either.
4. **(Optional) Add HT16K33 support.** Send `Configure` with `module_type = 4` and the HT16K33 payload (`i2c_address`, `brightness`, `num_digits`); then drive the display with `WriteSegments`. See the "Driving an HT16K33 from the Host" section in `docs/PROTOCOL.md` for a concrete walkthrough including the segment bit mapping.
5. **(Optional) Handle `ModuleError`.** Hosts that configure I2C modules should listen for type-15 messages after `ConfigurationStored` and surface failures (e.g. wrong I2C address, chip not connected) to the user.

## [2.2.1] - 2026-01-31

### Added

- **Release manifest uploadConfig**: Added upload configuration to release.json manifest
  - Includes upload protocol, MCU identifier, baud rate, and 1200bps touch requirement
  - Enables flashing applications to automatically configure upload parameters
  - Updated RELEASE_MANIFEST.md documentation with uploadConfig schema

## [2.2.0] - 2026-01-17

### Added

- **Arduino Due support**: Added `due` environment for ARM Cortex-M3 based Arduino Due
  - Uses Preferences library for configuration storage instead of EEPROM
  - Cross-platform configuration manager abstraction

- **ESP32 support**: Added `esp32dev` environment for ESP32-based boards
  - Uses Preferences library for configuration storage
  - Supports ESP32 DevKit and compatible boards

- **Release manifest**: Automated generation of `release.json` during GitHub releases
  - Contains firmware metadata for all supported boards
  - Includes download URLs, checksums, and board specifications
  - Documented in docs/RELEASE_MANIFEST.md

### Changed

- **Configuration storage**: Refactored `ConfigManager` to support multiple storage backends
  - AVR boards (Uno, Nano, Mega, Pro Micro, Leonardo, Micro) continue using EEPROM
  - ARM/ESP32 boards (Due, ESP32) use Preferences library
  - Transparent abstraction layer maintains compatibility

- **CI/CD**: Updated workflows to build for all supported platforms including Due and ESP32

### Removed

- **Old bootloader Nano**: Removed `nanoatmega328` environment (old bootloader variant)
  - Use `nanoatmega328new` instead for Arduino Nano boards

## [2.1.0] - 2026-01-07

### Fixed

- **Analog sensor on ATmega328P boards (Nano/Uno)**: Removed erroneous `pinMode()` call in `AnalogSensor::begin()` that was configuring digital pins 0/1 (RX/TX) instead of analog pins when using channel numbers. This was interfering with serial communication on boards that use hardware UART (Nano, Uno, Mega) while working correctly on native USB boards (Pro Micro, Leonardo). `analogRead()` handles analog pin configuration automatically.

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

- **LED output support**: Control output pins directly from host
  - New `SetOutput` message (type 7): `[type: u8 = 7] [pin: u8] [value: u8]`
  - Fire-and-forget (no acknowledgment) for low latency
  - Automatic pinMode configuration on first use
  - No configuration required - device acts as "dumb" output controller

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
4. (Optional) Use new `SetOutput` message to control output pins

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
