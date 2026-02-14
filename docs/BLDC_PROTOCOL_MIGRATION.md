# BLDC Protocol Migration Guide

This document describes the breaking changes to the BLDC lever protocol introduced by the detent simulation redesign. Use this as the reference when updating host software.

## What Changed and Why

The BLDC lever's motor control was redesigned to use SmartKnob-style PD (proportional-derivative) haptic simulation. The old protocol had three separate strength fields per detent (`engagement_strength`, `hold_strength`, `exit_strength`) and a `spring_back_target` field, but these were never properly implemented in firmware — the motor control was a stub. The new protocol simplifies to a single `detent_strength` per detent and adds two profile-level parameters (`snap_point`, `endstop_strength`) that the new PD controller requires.

## Messages Affected

Only **LoadBLDCProfile (type 11)** changed. All other messages are identical:
- Configure (type 2) — unchanged
- RetryCalibration (type 8) — unchanged
- CalibrationError (type 9) — unchanged
- EncoderError (type 10) — unchanged
- DeactivateBLDCProfile (type 12) — unchanged
- InputValue (type 5) — unchanged (still reports detent index)

## LoadBLDCProfile — Old Format

```
Byte 0:    [type: u8 = 11]
Byte 1:    [pin: u8]
Byte 2:    [num_detents: u8]
Byte 3:    [num_linear_ranges: u8]

Per detent (5 bytes each):
  [position_percent: u8]        0-100
  [engagement_strength: u8]     0-255 (REMOVED)
  [hold_strength: u8]           0-255 (REMOVED)
  [exit_strength: u8]           0-255 (REMOVED)
  [spring_back_target: u8]      detent index or 255 (REMOVED)

Per linear range (3 bytes each):
  [start_detent_index: u8]
  [end_detent_index: u8]
  [damping_strength: u8]

Total size: 4 + (5 * num_detents) + (3 * num_linear_ranges)
```

## LoadBLDCProfile — New Format

```
Byte 0:    [type: u8 = 11]
Byte 1:    [pin: u8]
Byte 2:    [num_detents: u8]
Byte 3:    [num_linear_ranges: u8]
Byte 4:    [snap_point: u8]           NEW
Byte 5:    [endstop_strength: u8]     NEW

Per detent (2 bytes each):
  [position_percent: u8]        0-100
  [detent_strength: u8]         0-255

Per linear range (3 bytes each, unchanged):
  [start_detent_index: u8]
  [end_detent_index: u8]
  [damping_strength: u8]

Total size: 6 + (2 * num_detents) + (3 * num_linear_ranges)
```

## Field-by-Field Changes

### Removed Fields

| Field | Was | Rationale |
|---|---|---|
| `engagement_strength` | Per-detent, 0-255 | Replaced by single `detent_strength`. The PD controller uses one strength to derive both P and D gains. Separate engagement/hold/exit phases were never implemented. |
| `hold_strength` | Per-detent, 0-255 | Same as above. |
| `exit_strength` | Per-detent, 0-255 | Same as above. |
| `spring_back_target` | Per-detent, detent index or 255 | Feature dropped. If needed in the future, host software can simulate it by sending a new profile on detent change events. |

### New Fields

| Field | Location | Range | Description |
|---|---|---|---|
| `snap_point` | Profile header, byte 4 | 50-150 | Hysteresis threshold. Maps to 0.50-1.50 fractional value. Controls how far past the midpoint between detents the lever must travel before the detent index transitions. **50** = transitions at midpoint (easy to move). **100** = must reach next detent center exactly. **110** = must overshoot past next detent (resists accidental changes). Recommended starting value: **70**. |
| `endstop_strength` | Profile header, byte 5 | 0-255 | Virtual endstop resistance. When the lever moves beyond the first or last detent toward the physical endstop, the motor pushes back with this strength. **0** = no virtual endstop (physical stop only). **255** = maximum resistance. Recommended starting value: **200**. |

### Changed Fields

| Field | Old | New | Notes |
|---|---|---|---|
| `detent_strength` | Was three fields: engagement (0-255), hold (0-255), exit (0-255) | Single field: detent_strength (0-255) | Scales both P and D gains of the PD controller. **0** = no haptic effect (free-spinning past this point). **128** = moderate detent. **255** = maximum strength. |

## Migration Examples

### Old: 3-detent throttle profile

```python
# Old format
message = bytes([
    11,         # type = LoadBLDCProfile
    10,         # pin (encoder CS)
    3,          # num_detents
    0,          # num_linear_ranges
    # Detent 0: Idle (0%)
    0,          # position_percent
    100,        # engagement_strength
    150,        # hold_strength
    100,        # exit_strength
    255,        # spring_back_target (none)
    # Detent 1: Half speed (50%)
    50,         # position_percent
    80,         # engagement_strength
    120,        # hold_strength
    80,         # exit_strength
    255,        # spring_back_target (none)
    # Detent 2: Full speed (100%)
    100,        # position_percent
    100,        # engagement_strength
    150,        # hold_strength
    100,        # exit_strength
    255,        # spring_back_target (none)
])
```

### New: Same profile

```python
# New format
message = bytes([
    11,         # type = LoadBLDCProfile
    10,         # pin (encoder CS)
    3,          # num_detents
    0,          # num_linear_ranges
    70,         # snap_point (0.70 threshold)
    200,        # endstop_strength
    # Detent 0: Idle (0%)
    0,          # position_percent
    150,        # detent_strength (was hold_strength)
    # Detent 1: Half speed (50%)
    50,         # position_percent
    120,        # detent_strength
    # Detent 2: Full speed (100%)
    100,        # position_percent
    150,        # detent_strength
])
```

### Migration rule for detent_strength

When converting from the old three-strength model, use the old `hold_strength` value as the new `detent_strength`. The engagement and exit strengths have no equivalent — the PD controller handles the feel of entering and leaving detents through its dynamic derivative gain.

## Parameter Tuning Guide

### snap_point

| Value | Mapped | Behavior |
|---|---|---|
| 50 | 0.50 | Transitions at midpoint between detents. Easy to move. Good for fine-grained control. |
| 70 | 0.70 | Must travel 70% of the way. Good default — prevents accidental transitions. |
| 100 | 1.00 | Must reach the next detent center exactly. Firm positions. |
| 110 | 1.10 | Must overshoot past the next detent. Maximum resistance to accidental changes. |

### endstop_strength

| Value | Behavior |
|---|---|
| 0 | No motor resistance at boundaries. Lever hits physical stops. |
| 128 | Moderate pushback before physical limit. |
| 200 | Strong virtual wall. Recommended for protecting hardware. |
| 255 | Maximum resistance. |

### detent_strength

| Value | Behavior |
|---|---|
| 0 | No detent at all. Lever slides past this position freely. |
| 64 | Light detent. Gentle resistance. |
| 128 | Medium detent. Clear haptic click. |
| 200 | Strong detent. Firm click. |
| 255 | Maximum strength. May cause motor whine on some hardware. |

## Behavioral Differences

### What feels different

1. **Detent feel**: The new PD controller produces a "click" sensation through the derivative term. Fine detents (closely spaced) have stronger clicks. Coarse detents (widely spaced) have softer, broader resistance.

2. **Dead zone**: Near the center of a detent, the motor produces no torque. This prevents the audible hum that a pure proportional controller causes. The dead zone is automatic — no configuration needed.

3. **Snap point hysteresis**: The lever must travel past a threshold before the detent index transitions. This prevents the reported value from flickering when the lever is near a boundary.

4. **Virtual endstops**: The motor resists before the physical limit, providing a soft wall feel and protecting hardware from repeated impacts.

5. **Idle correction**: When the lever is stationary but not perfectly centered on a detent, the controller slowly adjusts its reference point. This prevents a persistent low torque that could drain power or cause a faint hum.

### What stays the same

- InputValue messages still report detent index (0-based integer)
- The overall flow: Configure -> Calibrate -> LoadProfile -> Report detent changes -> DeactivateProfile
- Linear ranges still provide smooth movement with damping between detents
- All other message types are byte-identical
