# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Arduino firmware for the TWS-IO device, targeting multiple Arduino-compatible boards. Uses PlatformIO for build management.

## Build Commands

```bash
# Build for default target (SparkFun Pro Micro)
pio run

# Build for specific board
pio run -e uno
pio run -e leonardo
pio run -e megaatmega2560

# Build and upload
pio run -e sparkfun_promicro16 -t upload

# Run unit tests
pio test -e native

# Run specific test file
pio test -e native -f test_protocol
```

## Supported Environments

- `sparkfun_promicro16` - SparkFun Pro Micro (default)
- `leonardo` - Arduino Leonardo
- `micro` - Arduino Micro
- `uno` - Arduino Uno
- `nanoatmega328` / `nanoatmega328new` - Arduino Nano
- `megaatmega2560` - Arduino Mega 2560
- `native` - Unit testing

## Architecture

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for module overview and data flow.

See [docs/PROTOCOL.md](docs/PROTOCOL.md) for the serial communication protocol specification.

### Key Files

- **main.cpp** - Entry point, main loop
- **protocol.h/cpp** - Binary message encoding/decoding
- **message_handler.h/cpp** - Serial communication via PacketSerial
- **config_manager.h/cpp** - Configuration and EEPROM persistence
- **sensor_manager.h/cpp** - Sensor lifecycle management
- **analog_sensor.h/cpp** - Analog input implementation

## Testing

Tests run on the native platform using mock Arduino headers in `test/`. The native environment excludes hardware-dependent files.

Mock headers:
- `test/Arduino.h` - millis(), analogRead() stubs
- `test/EEPROM.h` - In-memory EEPROM simulation
