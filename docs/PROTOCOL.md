# Protocol Specification

Firmware3.1.0 adds optional message types16/17 for configurable stepper axes.
See [the stepper extension contract](STEPPER_PROTOCOL.md) for exact sizes,
capabilities, referencing, watchdog, retry and commissioning requirements.

Firmware3.1.0 also supports an explicit empty legacy module deployment. Its
entire decoded packet is exactly8 bytes:
`02 <config_id:u32 little endian> 00 00 ff`.
The fields are Configure(type2), config_id, total_parts0, part_number0 and the
Configure-only empty discriminator0xff. There is no payload and no dummy module.
Any other total_parts/part_number paired with0xff, a different discriminator
paired with total_parts0, truncation or trailing bytes is invalid.

An accepted empty deployment cancels pending multipart configuration, removes
all legacy module instances, persists the supplied config_id with zero modules,
and sends ConfigurationStored(type3) with that ID. Identity retains that ID
after reboot. As with every completed legacy deployment, all volatile stepper
axes are reset and release is attempted; any release failure is separately
reported as ModuleError. Shield address registration remains intact.

This configures the **module list**, not every physical output: independent
SetOutput pins retain their last state, and removing a display does not blank
its hardware. Hosts must send explicit output/display commands before removal
if that behaviour is required. Old firmware does not support the empty command;
gate it on version >=3.1.0. Replaying any legacy deployment also resets axes, so
finish deployment/retries before configuring and referencing stepper axes.

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
| module_type | 0 = Analog, 1 = Button, 2 = Matrix, 3 = reserved, 4 = HT16K33, 5 = MCP23017, 6 = ADS1115 |

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

**MCP23017 Payload (module_type = 5)**

Exactly 10 payload bytes (18 bytes in the whole packet); any other length is rejected.

```
[i2c_address: u8] [direct_base: u8] [matrix_base: u8] [debounce: u8]
[direct_mask: u16] [row_mask: u16] [col_mask: u16]
```

| Field | Description |
|-------|-------------|
| i2c_address | I2C bus address (chip range 0x20-0x27) |
| direct_base | First virtual pin of the 16-pin direct block (pin = direct_base + bit) |
| matrix_base | First virtual pin of the matrix block (pin = matrix_base + row_index * col_count + col_index) |
| debounce | Debounce threshold in scans, at least 1 |
| direct_mask | Pins used as direct buttons. Bit 0-7 = GPA0-7, bit 8-15 = GPB0-7 |
| row_mask | Pins driven as matrix rows (same bit layout) |
| col_mask | Pins read as matrix columns |

Row and column indexes count the set bits of the masks from the lowest bit. Inputs use pull-ups; a pressed button reads low and is reported as `1`. Buttons are reported with `InputValue` (1 = pressed, 0 = released).

The decoder rejects the module (the device sends `ConfigurationError`) when:

- `debounce` is 0
- the three masks are all 0 (no enabled pins)
- the masks overlap
- `direct_mask` or `col_mask` contains GPA7 or GPB7 (`0x8080`; output-only on the MCP23017)
- more than 8 rows or 8 columns are set, or only one of `row_mask` / `col_mask` is set
- a used block does not fit in virtual pins 128-255 (`direct_base` / `matrix_base` below 128, or the block extends past 255)

A direct block is 16 pins; a matrix block is `rows * cols` pins.

**ADS1115 Payload (module_type = 6)**

Exactly 9 payload bytes (17 bytes in the whole packet); any other length is rejected.

```
[i2c_address: u8] [pin_base: u8] [channel_mask: u8] [gain: u8] [ref_channel: u8] [sensitivity: u8[4]]
```

| Field | Description |
|-------|-------------|
| i2c_address | I2C bus address (chip range 0x48-0x4B) |
| pin_base | First virtual pin of the 4-pin block (pin = pin_base + channel) |
| channel_mask | Reported channels, bit 0-3 = AIN0-3 |
| gain | Programmable gain: 0 = +/-6.144 V, 1 = +/-4.096 V |
| ref_channel | Channel wired to the analog supply (0-3), or 0xFF for none |
| sensitivity | Per-channel sensitivity, 1-10, same meaning as for Analog |

The decoder rejects the module when `channel_mask` is 0 (no enabled channels) or has bits above 3, `gain` is above 1, `ref_channel` is neither 0-3 nor 0xFF, or is also set in `channel_mask`, the 4-pin block does not fit in virtual pins 128-255, or an enabled channel has a sensitivity outside 1-10.

Values are reported with `InputValue` as 0-1023. With a reference channel they are scaled against its smoothed reading so they are correct on any supply voltage. The reference channel itself is never reported. While the reference reads too low (or has not been read yet) the device converts only the reference channel and sends no values.

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
| error_code | See the table below |

| Code | Name | Meaning |
|------|------|---------|
| 0 | init_failed | The chip NACKed during `begin()` |
| 1 | communication_lost | MCP23017, ADS1115. One chip-reset detection sends code 1, then code 2 immediately once the chip is set up again. Repeated failures, including repeated reset detections, send code 1 alone, and code 2 follows only when a retry succeeds |
| 2 | recovered | Communication with the chip is working again. A retry only counts when the chip reads back what was written to it |
| 3 | supply_reference_lost | ADS1115 only: the supply reference channel reads too low |

Sent by the device in three situations:

1. After `applyConfiguration` completes, for each module whose `begin()` failed (code 0). The module is registered but inactive; the host may retry by sending a new `Configure` message.
2. At run time, when an MCP23017 or ADS1115 changes state (codes 1, 2, 3). Several codes may be sent in a row for one chip.
3. In reply to `IdentityRequest`, after the init errors: one message for every module that is in an error state right now (code 1 or 3), unless that address was already reported in the same reply. A host that connects late therefore learns the current state.

After code 2 an ADS1115 whose reference is still lost reports code 2 and then code 3.

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

After a successful `ConfigurationStored`, the device may send one `ModuleError (15)` per I2C module that failed to initialize. Later, `ModuleError` is also sent when an MCP23017 or ADS1115 loses or regains communication.

## Addressing Scheme

Modules are addressed in two ways depending on their type:

- **Hardware-pin modules** (Analog, Button, Matrix): identified by the `pin` field in `Configure` and `InputValue`. Matrix buttons use virtual pin numbers (`128 + row * num_cols + col`).
- **I2C-addressed modules** (HT16K33, MCP23017, ADS1115): identified by the `i2c_address` field in `Configure` and `ModuleError` (and `WriteSegments` / `SetModuleBrightness` for the HT16K33). MCP23017 and ADS1115 inputs are reported with `InputValue` on virtual pins 128-255 chosen by `direct_base`, `matrix_base` and `pin_base`.

**Only one I2C bus is supported.** All I2C modules share the same bus. Each must have a distinct I2C address.

## Driving an HT16K33 from the Host

This section walks through the message sequence a host needs to issue to drive an HT16K33 14-segment display. All multi-byte integers are little-endian; messages are framed using COBS as described in the Transport section.

### 1. Configure the module (once per session, persisted to EEPROM)

Send a `Configure` message (type 2) with `module_type = 4`. The payload after the 8-byte header is 3 bytes:

```
[type=2] [config_id: u32]
[total_parts: u8] [part_number: u8]
[module_type: u8 = 4]
[i2c_address: u8] [brightness: u8] [num_digits: u8]
```

| Field | Recommended value |
|-------|-------------------|
| i2c_address | `0x70` (default for Adafruit/SparkFun breakouts; A0/A1/A2 jumpers select 0x70–0x77) |
| brightness | `0x08` for half-brightness; full range 0–15 |
| num_digits | `4` for the typical 4-character breakout; `8` for 8-character variants |

The device responds with `ConfigurationStored` (type 3) once all parts of the configuration have arrived. If the chip is not present on the bus (NACK during init), the device additionally emits one `ModuleError` (type 15) with `error_code = 0`. Listen for both.

### 2. Write segments

Once configured, send `WriteSegments` (type 13) any time you want to update the display:

```
[type=13] [i2c_address: u8] [num_bytes: u8] [data: u8[num_bytes]]
```

`data` is written verbatim to HT16K33 display RAM starting at register `0x00`. The chip auto-increments. Each character occupies **two bytes** — the lower 8 segment bits in the first byte, the upper 6 bits (plus optional decimal point) in the second.

For a 4-digit display, send 8 bytes (4 characters × 2 bytes). For an 8-digit display, send 16 bytes. The firmware silently caps `num_bytes` at `num_digits * 2`, so a host can safely send 16 bytes regardless and let the firmware truncate.

#### Segment bit mapping

The HT16K33 maps each character's 14 segments + decimal point into a 16-bit word laid out in two consecutive RAM bytes (low byte then high byte):

```
Byte 0 (low):  [dp] [N] [M] [L] [K] [J] [H] [G2]
Byte 1 (high): [-]  [-] [G1] [F] [E] [D] [C] [B] [A]
```

(Bit names follow the standard 14-segment naming: A–G2 are the seven primary segments and the split middle bar, H/J/K/M are the diagonals, I/L are vertical splits, N is the second middle, dp is the decimal point. Several common 14-segment products use slightly different naming — consult your specific display's datasheet for the authoritative mapping.)

In practice, hosts typically use a precomputed ASCII-to-segments table. The Adafruit LED Backpack library's `alphafonttable[]` (in `Adafruit_LEDBackpack.cpp`) is a widely used reference. A few entries for orientation:

| Character | low byte | high byte | bytes (LE) |
|-----------|----------|-----------|------------|
| ` ` (space) | `0x00` | `0x00` | `0x00 0x00` |
| `0` | `0x3F` | `0x12` | `0x3F 0x12` |
| `1` | `0x06` | `0x10` | `0x06 0x10` |
| `2` | `0xDB` | `0x00` | `0xDB 0x00` |
| `A` | `0xF7` | `0x00` | `0xF7 0x00` |
| `-` | `0xC0` | `0x00` | `0xC0 0x00` |

To display `"42KM"` on a 4-digit module at I2C address 0x70 you would send a `WriteSegments` with `num_bytes = 8` and `data` constructed by concatenating the LE byte pairs for `4`, `2`, `K`, `M`.

`WriteSegments` is fire-and-forget — the device sends no acknowledgment. If the chip NACKs at runtime, the firmware retries once internally; persistent failure triggers a transparent re-initialization on the next write. The host does not need to handle these cases.

### 3. Update brightness at runtime

```
[type=14] [i2c_address: u8] [brightness: u8]
```

The firmware clamps `brightness` to 0–15 and updates the chip immediately. The new value is also cached so any future auto-reinit uses it.

Fire-and-forget; no acknowledgment.

### 4. Handle `ModuleError`

After every `ConfigurationStored`, the device emits zero or more `ModuleError` messages — one per I2C module whose initialization failed. Treat these as actionable: the most common cause is a wrong I2C address or a chip that's not powered/connected. The module remains registered (so subsequent writes attempt to re-initialize), but the host should typically surface the failure to the user.

```
[type=15] [i2c_address: u8] [error_code: u8]
```

`error_code = 0` (init_failed) is the only code an HT16K33 sends. Codes 1-3 come from the MCP23017 and ADS1115 (see `ModuleError (15)`). Other codes are reserved.

### Reset / reconnect behavior

If the device is power-cycled (or the host disconnects and reconnects), the device:

- Re-loads the configuration from EEPROM.
- Re-initializes each configured HT16K33 (oscillator on, display on, brightness, zero display RAM).
- Emits `ModuleError` for any chip that fails to initialize.

The display content is **not** restored from EEPROM (segments are not persisted). If your host wants the display to show specific content after a reconnect, re-send `WriteSegments` after observing `IdentityResponse` or after a fresh `ConfigurationStored`.

### Working reference implementation

A working Python host implementation lives at
[`examples/ht16k33_host_demo.py`](../examples/ht16k33_host_demo.py).
It includes COBS encode/decode, the configure / write-segments /
brightness / module-error handling, the 14-segment font table, and a
short visual demo. See the file's docstring for run instructions.

### Minimal pseudocode example

```
# Configure once
send Configure(config_id=1, total_parts=1, part_number=0,
               module_type=4, i2c_address=0x70,
               brightness=8, num_digits=4)
wait_for ConfigurationStored
on ModuleError(addr=0x70, err=0):
    log "HT16K33 at 0x70 not responding; check wiring/address"

# Show "42  "
segments = encode_ascii("42  ")  # 8 bytes per the segment table
send WriteSegments(i2c_address=0x70, num_bytes=8, data=segments)

# Dim the display for night-mode
send SetModuleBrightness(i2c_address=0x70, brightness=2)
```

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
