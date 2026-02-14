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
| SetOutput | 7 | Host → Device | Control an output pin |
| RetryCalibration | 8 | Host → Device | Retry BLDC calibration |
| CalibrationError | 9 | Device → Host | BLDC calibration failed |
| EncoderError | 10 | Device → Host | BLDC encoder error |
| LoadBLDCProfile | 11 | Host → Device | Load BLDC detent profile |
| DeactivateBLDCProfile | 12 | Host → Device | Deactivate BLDC profile |

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
| input_type | 0 = Analog, 1 = Button, 2 = Matrix, 3 = BLDC Lever |

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

**BLDC Lever Payload (input_type = 3)**

```
[motor_pin_a: u8] [motor_pin_b: u8] [motor_pin_c: u8]
[motor_enable_a: u8] [motor_enable_b: u8]
[encoder_cs: u8] [pole_pairs: u8]
[voltage: u8] [current_limit: u8] [encoder_bits: u8]
```

| Field | Description |
|-------|-------------|
| motor_pin_a | PWM phase A pin |
| motor_pin_b | PWM phase B pin |
| motor_pin_c | PWM phase C pin |
| motor_enable_a | Enable pin A |
| motor_enable_b | Enable pin B |
| encoder_cs | SPI chip select pin for magnetic encoder |
| pole_pairs | Motor pole pairs (e.g. 11 for typical gimbal motor) |
| voltage | Supply voltage in 0.1V units (e.g. 120 = 12.0V) |
| current_limit | Max current in 0.1A units (0 = no limit) |
| encoder_bits | Encoder resolution in bits (e.g. 14 for AS5047D) |

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

### RetryCalibration (8)

```
[type: u8 = 8] [pin: u8]
```

### CalibrationError (9)

```
[type: u8 = 9] [pin: u8] [error_code: u8]
```

Error codes: 0=timeout, 1=range_too_small, 2=encoder_error

### EncoderError (10)

```
[type: u8 = 10] [pin: u8]
```

### LoadBLDCProfile (11)

```
[type: u8 = 11] [pin: u8] [num_detents: u8] [num_linear_ranges: u8]
[detent_data: 5 bytes × num_detents]
[range_data: 3 bytes × num_linear_ranges]
```

Detent data (5 bytes each):
```
[position: u8] [engagement: u8] [hold: u8] [exit: u8] [spring_back: u8]
```

Range data (3 bytes each):
```
[start_detent: u8] [end_detent: u8] [damping: u8]
```

### DeactivateBLDCProfile (12)

```
[type: u8 = 12] [pin: u8]
```

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
