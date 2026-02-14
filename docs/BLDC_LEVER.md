# BLDC Haptic Lever

## Overview

BLDC (Brushless DC) motor-based haptic lever with configurable virtual detents using SimpleFOC library.

## Hardware

- **Motor:** 3-phase BLDC motor (7 pole pairs typical)
- **Encoder:** AS5047D 14-bit magnetic encoder (SPI mode)
- **Driver:** SimpleFOCShield v2 on Arduino Mega 2560
- **Pins:** See `board_profiles.h` for pinout

## Configuration Levels

### Level 1: Hardware Config (EEPROM-persisted)

Configure once via standard `Configure` message (INPUT_TYPE_BLDC_LEVER = 3):
- Board profile (0 = SimpleFOCShield v2 on Mega)
- Triggers automatic calibration to find physical endstops
- Stored in EEPROM, restored on boot

### Level 2: Detent Profile (Runtime, volatile)

Loaded via `LoadBLDCProfile` message when train/scenario starts:
- Detent positions (0-100% of calibrated range)
- Detent strength per detent (scales PD controller gains)
- Snap point hysteresis and virtual endstop strength
- Linear ranges with damping between detents

Profile can be changed instantly without recalibration.

## Usage Flow

1. **Initial setup:** Host sends Configure (BLDC) → firmware auto-calibrates → enters freewheel
2. **Load train:** Host sends LoadBLDCProfile → firmware activates haptics
3. **Unload train:** Host sends DeactivateBLDCProfile → firmware enters freewheel
4. **Switch train:** Host sends new LoadBLDCProfile → firmware switches haptics

## Reported Values

Firmware reports **detent index** (0-based), not raw encoder position.

**Rule:** While moving through a linear range between detents, the *start detent* is reported until the *end detent* is fully engaged.

Example: Moving from detent 2 to detent 3 through a linear range:
- In linear range: reports detent 2
- Click into detent 3: reports detent 3

## Error Handling

- **Calibration fails:** Sends CalibrationError, enters freewheel, waits for RetryCalibration
- **Encoder fails during operation:** Sends EncoderError, enters safe freewheel immediately
- **Invalid profile:** Sends ConfigurationError, keeps previous profile (or freewheel)

## Testing

Run unit tests:
```bash
pio test -e native -f test_bldc_lever
pio test -e native -f test_protocol_bldc
```

Build for hardware:
```bash
pio run -e megaatmega2560
```

## References

- [Design Document](plans/2026-02-12-bldc-lever-design.md)
- [SimpleFOC Documentation](https://simplefoc.com/)
- [SmartKnob Project](https://github.com/scottbez1/smartknob)
