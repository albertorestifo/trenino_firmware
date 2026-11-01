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
├── mcp23017_module.h/cpp # MCP23017 direct buttons + button matrix (I2C)
├── ads1115_module.h/cpp  # ADS1115 4-channel ADC with supply reference (I2C)
├── matrix_core.h/cpp     # Shared matrix debounce and event queue
├── i2c_bus.h/cpp         # Shared I2C setup, write and register read transactions
├── i2c_health.h          # Failure counting, retry timing, runtime error queue
├── output_manager.h/cpp  # Direct GPIO output pin control
├── heartbeat.h/cpp       # Periodic keep-alive
└── device_info.h         # Version + EEPROM format version
```

`ModuleManager` (in `sensor_manager.h/cpp`) is the registry of `IModule*` instances. It exposes:
- `getModuleByPin(pin)` for hardware-pin modules
- `getModuleByI2CAddress(addr)` for I2C modules
- `getInitErrors(out, max)` — drained by `MessageHandler` after `applyConfiguration` to emit `ModuleError` messages
- `getNextRuntimeError(address, code)` — pops one pending runtime error (`IModule::takeRuntimeError()`); `MessageHandler::update()` sends each as a `ModuleError`
- `getActiveErrors(out, max)` — modules in an error state right now (`IModule::activeError()`, code 1 or 3); not drained, repeated in the reply to every `IdentityRequest`

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
    MessageHandler.update()   // Service steppers, check timeouts, scan modules (steppers serviced
                              // after each bus turn), send readings and runtime errors
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
    → If it used the I2C bus: service steppers
Then for each module:
    → getReading() — check if a value is ready to report
    → If ready: send InputValue message
```

The MCP23017 and ADS1115 never block in `scan()`.

Each MCP23017 matrix tick reads the row selected on the previous tick, compares OLAT against the expected value to detect a chip reset, feeds the row to the debouncer, then selects the next row. A failed select is followed by a full register rewrite, and repeated reset detections count as failures. A reset between the sample and the select write cannot be seen, because that write restores OLAT while IODIR and GPPU stay at reset. So at every 8th wrap to row 0 the tick runs `initRegisters()` instead of selecting: input state is kept, no code is sent on success, and a failure is handled like a failed select. A direct-only chip never writes OLAT after setup, so its sample sees every reset and it has no periodic rewrite.

The ADS1115 starts a conversion, then on a later `scan()` reads the conversion register first and the config register second. Any reset before the second read therefore shows default MUX/PGA/MODE and the result is discarded as a mismatch. If the config read-back has the OS bit clear, the conversion is not finished: the result is discarded, no failure is counted and the read is repeated on the next `scan()`. A fourth clear read counts as a mismatch. While the supply reference is lost the chip converts only the reference channel.

Both use `I2CHealth` to count failures, retry every second and queue `ModuleError` codes (1 communication lost, 2 recovered, 3 supply reference lost). A retry succeeds only if the chip reads back what was written (OLAT on the MCP23017, the config register on the ADS1115), so a wrong device that acknowledges writes stays failed with no code. After recovery with the reference still lost, the ADS1115 reports 2 then 3.

`ModuleManager::scan()` calls a callback after each module that used the bus, detected through `I2CBus::transactionCount()`. `MessageHandler` sets it to `StepperService::update`, so steppers are serviced between module scans, and the callback keeps `sensor_manager` free of stepper code. `applyConfiguration()` passes each module its index to `setFirstSampleOffset()`, delaying its first sample by that many ms so chips created together do not share a bus phase. The bus runs at 100 kHz.

Output-only modules (e.g. HT16K33) return an empty `Reading` and are never included in `InputValue` traffic.

## Key Constants

| Constant | Value | Description |
|----------|-------|-------------|
| MAX_MODULES | 8 | Maximum configured modules |
| CONFIG_TIMEOUT | 5000ms | Configuration timeout |
| DEAD_ZONE | 2 | ADC noise threshold |

## Board Limits

ATmega328P boards (Uno, Nano) have room for one matrix chip, because of their 2 KB of RAM. Static data leaves about 630 bytes of 2048, an 8x8 MCP23017 matrix allocates about 165 bytes, and 300 bytes must stay free for the stack. Nothing enforces this: a second matrix chip is accepted and can run out of RAM. The Pro Micro has 2.5 KB and takes two.

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
- In `begin()`, call `I2CBus::begin()`, which is safe to repeat. Use `I2CBus::write()` for a complete write transaction and `I2CBus::readRegisters()` to set the register pointer and read a block; both report NACKs and short reads as `false`
- Return `false` from `begin()` if the chip NACKs; `ModuleManager` records this in its init-error list
- `MessageHandler` drains `getInitErrors()` after `applyConfiguration` and sends `ModuleError (15)` to the host
- Only one I2C bus is supported; do not use `Wire1` or other bus objects
- `scan()` and `getReading()` should be no-ops for output-only modules (the defaults in `IModule` are sufficient)
- Control messages (e.g. `WriteSegments`, `SetModuleBrightness`) are dispatched by `MessageHandler` using `getModuleByI2CAddress()`

## Future Expansion (not implemented)

The following capabilities have been deliberately deferred:

- **HT16K33 keypad scanning** — the chip supports 13×3 key scanning; `scan()` and `getReading()` are stubs
- **Multi-bus I2C** — e.g. ESP32 `Wire1`; currently only a single bus is supported
- **Injectable I2C bus** — `I2CBus` calls `Wire` directly, so tests mock `Wire.h` rather than the bus
- **Generic raw I2C passthrough** — a host-driven message to send/receive arbitrary I2C frames
