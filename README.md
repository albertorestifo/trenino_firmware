# TWS-IO Device Firmware

Arduino firmware for the TWS-IO device - a configurable I/O device for train simulator cockpits that communicates over serial.

## Roadmap

### Inputs

- [x] Dynamic configuration via serial
- [x] Analog inputs (levers, rotary controls)
- [ ] Switches (single and multi-position)
- [ ] Button matrix

### Outputs

- [ ] LED outputs (indicator lights)
- [ ] PWM outputs (dimmable lights, gauge needles)
- [ ] Servo outputs (physical gauge needles)
- [ ] 7-segment displays (speed/pressure readouts)

## Supported Boards

| Board | Environment | Notes |
|-------|-------------|-------|
| SparkFun Pro Micro | `sparkfun_promicro16` | Native USB |
| Arduino Leonardo | `leonardo` | Native USB |
| Arduino Micro | `micro` | Native USB |
| Arduino Uno | `uno` | USB-to-Serial |
| Arduino Nano | `nanoatmega328new` | USB-to-Serial |
| Arduino Nano (Old Bootloader) | `nanoatmega328` | USB-to-Serial |
| Arduino Mega 2560 | `megaatmega2560` | USB-to-Serial |

## Installation

### From Release

1. Download the `.hex` file for your board from the [Releases](../../releases) page
2. Flash using avrdude or your preferred tool:
   ```bash
   avrdude -p atmega32u4 -c avr109 -P /dev/ttyACM0 -U flash:w:tws-io-sparkfun-pro-micro.hex:i
   ```
   Or use the Arduino IDE: **Sketch → Upload Using Programmer**

### Building from Source

Requires [PlatformIO](https://platformio.org/).

```bash
# Install PlatformIO
pip install platformio

# Build for your board
pio run -e sparkfun_promicro16

# Build and upload
pio run -e sparkfun_promicro16 -t upload
```

## Development

### Running Tests

```bash
pio test -e native
```

### Project Structure

```
src/           # Firmware source code
test/          # Unit tests and Arduino mocks
docs/          # Documentation
  ARCHITECTURE.md  # Code architecture
  PROTOCOL.md      # Communication protocol
```

## Documentation

- [Architecture](docs/ARCHITECTURE.md) - Code structure and data flow
- [Protocol](docs/PROTOCOL.md) - Serial communication protocol specification

## License

Licensed under [CC BY-NC](https://creativecommons.org/licenses/by-nc/4.0/).

For commercial use, contact me.
