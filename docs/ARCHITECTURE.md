# Architecture

## Module Overview

```
src/
├── main.cpp              # Entry point, main loop
├── protocol.h/cpp        # Wire-level message encode/decode
├── message_handler.h/cpp # Routes incoming messages, drains module init errors
├── config_manager.h/cpp  # Configure-message accumulator + EEPROM persistence
├── module.h              # IModule interface, ModuleType enum, Reading struct
├── sensor_manager.h/cpp  # ModuleManager namespace, owns IModule registry
├── analog_sensor.h/cpp   # Analog input module
├── button_sensor.h/cpp   # Digital button module
├── matrix_sensor.h/cpp   # Button matrix module
├── ht16k33_module.h/cpp  # HT16K33 14-segment display module (I2C)
├── output_manager.h/cpp  # Direct GPIO output pin control
├── heartbeat.h/cpp       # Periodic keep-alive
└── device_info.h         # Version + EEPROM format version
```

`ModuleManager` (in `sensor_manager.h/cpp`) is the registry of `IModule*` instances. It exposes:
- `getModuleByPin(pin)` for hardware-pin modules
- `getModuleByI2CAddress(addr)` for I2C modules
- `getInitErrors(out, max)` — drained by `MessageHandler` after `applyConfiguration` to emit `ModuleError` messages

## Data Flow

### Startup

1. Initialize serial at 115200 baud with COBS framing
2. Load configuration from EEPROM
3. Create modules from loaded config
4. Start main loop

### Main Loop

```
loop() {
    PacketSerial.update()     // Process incoming messages
    MessageHandler.update()   // Check timeouts, scan modules, send readings
    delay(1)                  // 1 ms tick
}
```

### Message Handling

```
Packet received → Decode → Route to handler → Send response
```

### Configuration Flow

```
Host sends Configure messages (one per module)
    → Accumulate parts in RAM
    → On complete: store to EEPROM, apply to modules
    → On timeout (5s): discard and send ConfigurationError
    → After apply: drain init errors, send ModuleError for each failed I2C module
```

### Module Scanning

```
For each module:
    → scan() (read value, update state)
    → getReading() — check if a value is ready to report
    → If ready: send InputValue message
```

Output-only modules (e.g. HT16K33) return an empty `Reading` and are never included in `InputValue` traffic.

## Key Constants

| Constant | Value | Description |
|----------|-------|-------------|
| MAX_MODULES | 8 | Maximum configured modules |
| CONFIG_TIMEOUT | 5000ms | Configuration timeout |
| DEAD_ZONE | 2 | ADC noise threshold |

## Adding New Module Types

### Hardware-pin modules (analog, button, matrix style)

1. Create a class implementing `IModule` (defined in `module.h`)
2. Implement `begin()`, `scan()`, `getReading()`, `getType()`, `getPin()`; `getI2CAddress()` defaults to `0`
3. Add `MODULE_TYPE_*` constant in `protocol.h`
4. Add a payload variant to the `Configure` struct union in `protocol.h`; implement encode/decode in `protocol.cpp`
5. Update `ModuleManager::applyConfiguration()` in `sensor_manager.cpp` to instantiate the new type
6. Add tests in `test/test_protocol.cpp` and/or `test/test_module_*.cpp`

### I2C-addressed modules (HT16K33 style)

Follow the hardware-pin steps above, with these differences:

- The `Configure` payload uses `i2c_address` instead of `pin`
- Implement `getPin()` returning `0` and `getI2CAddress()` returning the address
- In `begin()`, call `Wire.begin()` lazily on first use (guard with a static flag, as HT16K33Module does)
- Return `false` from `begin()` if the chip NACKs; `ModuleManager` records this in its init-error list
- `MessageHandler` drains `getInitErrors()` after `applyConfiguration` and sends `ModuleError (15)` to the host
- Only one I2C bus is supported; do not use `Wire1` or other bus objects
- `scan()` and `getReading()` should be no-ops for output-only modules (the defaults in `IModule` are sufficient)
- Control messages (e.g. `WriteSegments`, `SetModuleBrightness`) are dispatched by `MessageHandler` using `getModuleByI2CAddress()`

## Future Expansion (not implemented)

The following capabilities have been deliberately deferred:

- **HT16K33 keypad scanning** — the chip supports 13×3 key scanning; `scan()` and `getReading()` are stubs
- **Multi-bus I2C** — e.g. ESP32 `Wire1`; currently only a single bus is supported
- **Generic I2CBus namespace** — abstraction layer to allow injecting a mock bus in tests without `Wire.h`
- **I2C input modules** — e.g. MCP23017 GPIO expander, ADS1115 ADC; the `IModule` interface already supports them
- **Generic raw I2C passthrough** — a host-driven message to send/receive arbitrary I2C frames
