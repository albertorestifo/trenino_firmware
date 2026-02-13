# BLDC Lever Testing Guide

## Hardware Setup

1. **Connect SimpleFOCShield v2 to Arduino Mega 2560**
   - Shield stacks on top of Mega
   - Motor connects to shield terminals (phases A, B, C)
   - AS5047D encoder connects via SPI:
     - CS → Pin 10
     - MOSI → Pin 51
     - MISO → Pin 50
     - SCK → Pin 52
     - VCC → 5V
     - GND → GND

2. **Power:**
   - USB for Arduino
   - External 12V for motor (via shield power jack)

## Test Sequence

### Test 1: Hardware Configuration and Calibration

1. Flash firmware: `pio run -e megaatmega2560 -t upload`
2. Connect to serial: `pio device monitor -e megaatmega2560`
3. Send Configure message (INPUT_TYPE_BLDC_LEVER, board_profile=0)
4. **Expected:** Lever slowly sweeps to both endstops (~10-30 seconds)
5. **Expected:** Receives ConfigurationStored message
6. **Expected:** Motor enters freewheel (no resistance)

### Test 2: Load Simple Detent Profile

1. Send LoadBLDCProfile with 3 detents at 0%, 50%, 100%
   - All strengths = 100
   - No spring-back (all targets = 255)
   - No linear ranges
2. **Expected:** Motor provides resistance at 3 positions
3. Manually move lever through positions
4. **Expected:** Distinct "click" feel at each detent
5. **Expected:** InputValue messages report detent 0, 1, 2 as you click in

### Test 3: Spring-Back Behavior

1. Send LoadBLDCProfile with 2 detents:
   - Detent 0: position=0%, spring_back=255 (no spring-back)
   - Detent 1: position=100%, spring_back=0 (springs back to detent 0)
2. Push lever to 100% position
3. **Expected:** Clicks into detent 1, reports detent 1
4. Release lever
5. **Expected:** Motor pulls back to 0%, clicks into detent 0, reports detent 0

### Test 4: Linear Range

1. Send LoadBLDCProfile with 2 detents and 1 linear range:
   - Detent 0: position=0%
   - Detent 1: position=100%
   - Linear range: start=0, end=1, damping=50
2. Move lever slowly from 0% to 100%
3. **Expected:** Light resistance (damping) between detents
4. **Expected:** Reports detent 0 until clicking into detent 1
5. **Expected:** Reports detent 1 once engaged

### Test 5: Profile Switching

1. Load profile A (3 detents)
2. Verify detent positions
3. Send new LoadBLDCProfile with profile B (5 detents)
4. **Expected:** Immediate switch to new detent positions
5. **Expected:** No recalibration needed

### Test 6: Deactivation

1. Load any profile
2. Send DeactivateBLDCProfile
3. **Expected:** Motor enters freewheel immediately
4. **Expected:** No more InputValue messages

### Test 7: Error Recovery

1. Block lever mechanically during calibration
2. **Expected:** CalibrationError message after timeout
3. Clear obstruction
4. Send RetryCalibration
5. **Expected:** Successful calibration, ConfigurationStored

### Test 8: Encoder Failure

1. Disconnect encoder SPI during operation (carefully!)
2. **Expected:** EncoderError message
3. **Expected:** Motor enters safe freewheel
4. Reconnect encoder
5. **Expected:** Auto-recovery, ConfigurationStored

## Success Criteria

- [ ] Calibration finds endstops reliably
- [ ] Detents feel distinct and repeatable
- [ ] Spring-back returns smoothly to target
- [ ] Linear ranges feel smooth, not jerky
- [ ] Profile switching is instant
- [ ] No unexpected motor movement
- [ ] All error states enter freewheel
- [ ] InputValue messages report correct detent indices

## Troubleshooting

**Motor doesn't move during calibration:**
- Check power supply to shield
- Verify motor phases connected correctly
- Check enable pins in board_profiles.h

**Erratic encoder readings:**
- Verify SPI connections
- Check encoder magnet alignment with sensor IC
- Ensure encoder is getting clean 5V power

**Weak detent feel:**
- Increase engagement/hold/exit strengths in profile
- Check motor power supply voltage
- Verify pole pairs setting in bldc_lever.cpp (default 7)

**Calibration timeout:**
- Increase CALIBRATION_TIMEOUT_MS in bldc_config.h
- Check for mechanical binding
- Verify encoder is reading position changes
