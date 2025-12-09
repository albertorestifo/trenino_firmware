# Architecture

## Module Overview

```
src/
├── main.cpp              # Entry point, main loop
├── protocol.h/cpp        # Message encoding/decoding
├── message_handler.h/cpp # Serial communication routing
├── config_manager.h/cpp  # Configuration and EEPROM persistence
├── sensor_manager.h/cpp  # Sensor lifecycle management
├── sensor.h              # ISensor interface
└── analog_sensor.h/cpp   # Analog input implementation
```

## Data Flow

### Startup

1. Initialize serial at 115200 baud with COBS framing
2. Load configuration from EEPROM
3. Create sensors from loaded config
4. Start main loop

### Main Loop

```
loop() {
    PacketSerial.update()     // Process incoming messages
    MessageHandler.update()   // Check timeouts, scan sensors, send readings
    delay(10)                 // ~100 Hz scan rate
}
```

### Message Handling

```
Packet received → Decode → Route to handler → Send response
```

### Configuration Flow

```
Host sends Configure messages (one per input)
    → Accumulate parts in RAM
    → On complete: store to EEPROM, apply to sensors
    → On timeout (5s): discard and send error
```

### Sensor Scanning

```
For each sensor:
    → Read value (analogRead)
    → Check send conditions (interval, dead zone)
    → If ready: send InputValue message
```

## Key Constants

| Constant | Value | Description |
|----------|-------|-------------|
| MAX_INPUTS | 8 | Maximum configured inputs |
| CONFIG_TIMEOUT | 5000ms | Configuration timeout |
| DEAD_ZONE | 2 | ADC noise threshold |

## Adding New Sensor Types

1. Create class implementing `ISensor` interface in `sensor.h`
2. Implement `begin()`, `scan()`, `getReading()`, `getType()`, `getPin()`
3. Add input type constant in `protocol.h`
4. Update `SensorManager::applyConfiguration()` to create instances
