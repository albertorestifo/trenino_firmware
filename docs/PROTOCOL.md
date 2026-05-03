# Protocol Specification

Binary protocol over serial using COBS framing (PacketSerial library).

## Transport

- **Baud rate**: 115200
- **Framing**: COBS (Consistent Overhead Byte Stuffing)
- **Byte order**: Little-endian for multi-byte integers

## Message Format

All messages start with a 1-byte message type.

```
[message_type: u8] [payload...]
```

## Message Types

| Type | ID | Direction | Description |
|------|----|-----------|-------------|
| IdentityRequest | 0 | Host → Device | Request device identity |
| IdentityResponse | 1 | Device → Host | Device identity |
| Configure | 2 | Host → Device | Configure a module |
| ConfigurationStored | 3 | Device → Host | Configuration saved |
| ConfigurationError | 4 | Device → Host | Configuration failed |
| InputValue | 5 | Device → Host | Module reading |
| Heartbeat | 6 | Device → Host | Keep-alive |
| SetOutput | 7 | Host → Device | Control an output pin |
| (reserved) | 8 | — | Reserved (formerly RetryCalibration) |
| (reserved) | 9 | — | Reserved (formerly CalibrationError) |
| EncoderError | 10 | Device → Host | Legacy stub; firmware never sends |
| LoadBLDCProfile | 11 | Host → Device | Legacy stub; firmware ignores |
| DeactivateBLDCProfile | 12 | Host → Device | Legacy stub; firmware ignores |
| WriteSegments | 13 | Host → Device | Write segments to I2C display module |
| SetModuleBrightness | 14 | Host → Device | Set brightness on I2C display module |
| ModuleError | 15 | Device → Host | Per-module error (e.g. I2C init failure) |

## Message Definitions

### IdentityRequest (0)

```
[type: u8 = 0] [request_id: u32]
```

### IdentityResponse (1)

```
[type: u8 = 1] [request_id: u32] [version_major: u8] [version_minor: u8] [version_patch: u8] [config_id: u32]
```

| Field | Description |
|-------|-------------|
| version_major | Major version number (semantic versioning) |
| version_minor | Minor version number (semantic versioning) |
| version_patch | Patch version number (semantic versioning) |

### Configure (2)

Multi-part message to configure device modules. Send one message per module.
The message uses a discriminated union based on `module_type`.

**Common Header (8 bytes)**

```
[type: u8 = 2] [config_id: u32] [total_parts: u8] [part_number: u8] [module_type: u8]
```

| Field | Description |
|-------|-------------|
| config_id | Unique configuration identifier |
| total_parts | Total number of modules to configure |
| part_number | This module's index (0-based) |
| module_type | 0 = Analog, 1 = Button, 2 = Matrix, 3 = reserved, 4 = HT16K33 |

**Analog Payload (module_type = 0)**

```
[pin: u8] [sensitivity: u8]
```

| Field | Description |
|-------|-------------|
| pin | Hardware pin number |
| sensitivity | 0-10 (higher = more frequent updates) |

**Button Payload (module_type = 1)**

```
[pin: u8] [debounce: u8]
```

| Field | Description |
|-------|-------------|
| pin | Hardware pin number |
| debounce | Debounce threshold (number of scan cycles, ~10ms each) |

**Matrix Payload (module_type = 2)**

```
[num_row_pins: u8] [num_col_pins: u8] [row_pins: u8[num_row_pins]] [col_pins: u8[num_col_pins]]
```

| Field | Description |
|-------|-------------|
| num_row_pins | Number of row pins |
| num_col_pins | Number of column pins |
| row_pins | Array of row pin numbers |
| col_pins | Array of column pin numbers |

Matrix buttons are reported using virtual pins: `pin = 128 + (row * num_cols + col)`

**HT16K33 Payload (module_type = 4)**

```
[i2c_address: u8] [brightness: u8] [num_digits: u8]
```

| Field | Description |
|-------|-------------|
| i2c_address | I2C bus address of the HT16K33 chip (e.g. 0x70) |
| brightness | Initial display brightness (0–15) |
| num_digits | Number of 14-segment digits driven (determines how many display RAM bytes are valid) |

### ConfigurationStored (3)

```
[type: u8 = 3] [config_id: u32]
```

### ConfigurationError (4)

```
[type: u8 = 4] [config_id: u32]
```

### InputValue (5)

```
[type: u8 = 5] [pin: u8] [value: i16]
```

Value is the raw ADC reading (0-1023 for 10-bit ADC).

### Heartbeat (6)

```
[type: u8 = 6]
```

Sent periodically to indicate device is alive.

### SetOutput (7)

```
[type: u8 = 7] [pin: u8] [value: u8]
```

| Field | Description |
|-------|-------------|
| pin | Output pin number |
| value | 0 = OFF (LOW), 1 = ON (HIGH) |

Controls an output pin directly. The device automatically configures the pin as OUTPUT on first use. No acknowledgment is sent (fire-and-forget for low latency).

### EncoderError (10)

```
[type: u8 = 10] [pin: u8]
```

**Legacy wire-level stub.** The firmware never sends this message. Retained for backwards compatibility with existing host implementations.

### LoadBLDCProfile (11)

```
[type: u8 = 11] [pin: u8] [...]
```

**Legacy wire-level stub.** The firmware ignores this message. Retained for backwards compatibility with existing host implementations.

### DeactivateBLDCProfile (12)

```
[type: u8 = 12] [pin: u8]
```

**Legacy wire-level stub.** The firmware ignores this message. Retained for backwards compatibility with existing host implementations.

### WriteSegments (13)

```
[type: u8 = 13] [i2c_address: u8] [num_bytes: u8] [data: u8[num_bytes]]
```

| Field | Description |
|-------|-------------|
| i2c_address | I2C address of the target display module |
| num_bytes | Number of segment bytes to write (≤ 16) |
| data | Raw segment bytes, written to display RAM starting at register 0x00 |

The firmware silently truncates `data` to `num_digits * 2` bytes (as configured). The host should send no more than 16 bytes regardless of truncation.

### SetModuleBrightness (14)

```
[type: u8 = 14] [i2c_address: u8] [brightness: u8]
```

| Field | Description |
|-------|-------------|
| i2c_address | I2C address of the target display module |
| brightness | New brightness level (0–15) |

Updates the brightness immediately and caches the value so re-initialization uses the latest setting.

### ModuleError (15)

```
[type: u8 = 15] [i2c_address: u8] [error_code: u8]
```

| Field | Description |
|-------|-------------|
| i2c_address | I2C address of the module that failed |
| error_code | 0 = init_failed (chip NACKed during begin()); 1+ reserved |

Sent by the device after `applyConfiguration` completes if one or more I2C modules failed to initialize. The module is registered but inactive; the host may retry by sending a new `Configure` message.

## Configuration Sequence

```
Host                              Device
  |                                  |
  |-- Configure (part 0/3) --------->|
  |-- Configure (part 1/3) --------->|
  |-- Configure (part 2/3) --------->|
  |<-------- ConfigurationStored ----|
  |                                  |
```

If all parts aren't received within 5 seconds, the device sends `ConfigurationError` and discards partial configuration.

After a successful `ConfigurationStored`, the device may send one `ModuleError (15)` per I2C module that failed to initialize.

## Addressing Scheme

Modules are addressed in two ways depending on their type:

- **Hardware-pin modules** (Analog, Button, Matrix): identified by the `pin` field in `Configure` and `InputValue`. Matrix buttons use virtual pin numbers (`128 + row * num_cols + col`).
- **I2C-addressed modules** (HT16K33, and future modules such as MCP23017 or ADS1115): identified by the `i2c_address` field in `Configure`, `WriteSegments`, `SetModuleBrightness`, and `ModuleError`.

**Only one I2C bus is supported.** All I2C modules share the same bus. Each must have a distinct I2C address.

## Adding New Message Types

### Hardware-pin modules

1. Add `MODULE_TYPE_*` constant in `protocol.h`
2. Add a payload variant to the `Configure` union (struct with `pin` + type-specific fields)
3. Create a class implementing `IModule` in a new `*_sensor.h/cpp` (or `*_module.h/cpp`) file
4. Implement `begin()`, `scan()`, `getReading()`, `getType()`, `getPin()`
5. Update `ModuleManager::applyConfiguration()` to instantiate the new type
6. Add the new message struct to `protocol.h`, encode/decode to `protocol.cpp`, and the union to `Message`
7. Add handler in `message_handler.cpp`
8. Add tests in `test/test_protocol.cpp`

### I2C-addressed modules

Follow the hardware-pin steps above, with these differences:

- The `Configure` payload uses `i2c_address` instead of `pin`
- Implement `getPin()` returning `0` and `getI2CAddress()` returning the address
- In `begin()`, call `Wire.begin()` lazily on first use (guard with a static flag)
- Register the module so `ModuleManager::getModuleByI2CAddress()` can find it
- Emit `ModuleError (15)` (via `getInitErrors`) if `begin()` returns false
- Only one I2C bus is supported; do not instantiate `Wire1` or other bus objects
