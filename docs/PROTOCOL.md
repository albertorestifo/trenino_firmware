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
|------|-----|-----------|-------------|
| IdentityRequest | 0 | Host → Device | Request device identity |
| IdentityResponse | 1 | Device → Host | Device identity |
| Configure | 2 | Host → Device | Configure an input |
| ConfigurationStored | 3 | Device → Host | Configuration saved |
| ConfigurationError | 4 | Device → Host | Configuration failed |
| InputValue | 5 | Device → Host | Sensor reading |
| Heartbeat | 6 | Device → Host | Keep-alive |

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

Multi-part message to configure device inputs. Send one message per input.
The message uses a discriminated union based on `input_type`.

**Common Header (8 bytes)**

```
[type: u8 = 2] [config_id: u32] [total_parts: u8] [part_number: u8] [input_type: u8]
```

| Field | Description |
|-------|-------------|
| config_id | Unique configuration identifier |
| total_parts | Total number of inputs to configure |
| part_number | This input's index (0-based) |
| input_type | 0 = Analog, 1 = Button, 2 = Matrix |

**Analog Payload (input_type = 0)**

```
[pin: u8] [sensitivity: u8]
```

| Field | Description |
|-------|-------------|
| pin | Hardware pin number |
| sensitivity | 0-10 (higher = more frequent updates) |

**Button Payload (input_type = 1)**

```
[pin: u8] [debounce: u8]
```

| Field | Description |
|-------|-------------|
| pin | Hardware pin number |
| debounce | Debounce threshold (number of scan cycles, ~10ms each) |

**Matrix Payload (input_type = 2)**

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

## Adding New Message Types

1. Add `MESSAGE_TYPE_*` constant in `protocol.h`
2. Define struct with `encode()` and `decode()` methods
3. Add to `Message` union
4. Implement handler in `message_handler.cpp`
5. Add tests in `test/test_protocol.cpp`
