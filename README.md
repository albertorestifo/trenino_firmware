# TWS-IO Device Firmware

Arduino firmware for the TWS-IO device - a configurable sensor input device that communicates over serial.

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

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/my-feature`)
3. Make your changes
4. Run tests (`pio test -e native`)
5. Build for at least one target (`pio run -e uno`)
6. Commit your changes (`git commit -am 'Add my feature'`)
7. Push to the branch (`git push origin feature/my-feature`)
8. Open a Pull Request

### Code Style

- Use consistent indentation (4 spaces)
- Keep functions focused and small
- Add tests for new protocol messages
- Update documentation for user-facing changes

## License

[Add your license here]
