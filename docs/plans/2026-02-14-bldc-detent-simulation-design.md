# BLDC Detent Simulation Design

**Date:** 2026-02-14
**Status:** Approved
**Reference:** SmartKnob detent simulation (see `smartknob/docs/detent-simulation-parameters.md`)

## Overview

Replace the stub proportional spring in `bldc_lever.cpp` with a SmartKnob-style PD controller for haptic detent simulation. This brings proper dead zones, snap point hysteresis, dynamic derivative gains, idle correction, velocity safety cutoff, virtual endstops, and linear range damping.

## Design Decisions

- **Fixed detent positions** (keep `position_percent` model, not SmartKnob's evenly-spaced virtual detents)
- **Single `detent_strength`** per detent (replace `engagement_strength` / `hold_strength` / `exit_strength`)
- **Global `snap_point`** for hysteresis (profile-level, not per-detent)
- **Keep linear ranges** with per-range `damping_strength`
- **Drop `spring_back_target`** (removed from DetentConfig)
- **Add virtual endstops** with profile-level `endstop_strength`
- **Include idle correction** (EWMA-based center drift compensation)
- **Work in encoder ticks** (not radians — avoids needing lever geometry)
- **Breaking protocol change** (pre-release firmware, clean up now)

## Data Model

### DetentConfig (simplified, was 5 bytes, now 2)

```cpp
struct DetentConfig {
    uint8_t position_percent;    // 0-100% of calibrated range
    uint8_t detent_strength;     // 0-255: scales PD gains
};
```

### ProfileConfig (new, profile-level parameters)

```cpp
struct ProfileConfig {
    uint8_t snap_point;          // 50-150 -> 0.50-1.50 fractional threshold
    uint8_t endstop_strength;    // 0-255: P gain at calibration boundaries
};
```

### LinearRangeConfig (unchanged)

```cpp
struct LinearRangeConfig {
    uint8_t start_detent_index;
    uint8_t end_detent_index;
    uint8_t damping_strength;    // 0-255: velocity-proportional resistance
};
```

## Protocol Changes

### LoadBLDCProfile (message type 11) — BREAKING

**Old format (header = 4 bytes, detent = 5 bytes):**

```
[type: u8 = 11] [pin: u8] [num_detents: u8] [num_linear_ranges: u8]
[position: u8] [engagement: u8] [hold: u8] [exit: u8] [spring_back: u8]  x num_detents
[start: u8] [end: u8] [damping: u8]  x num_linear_ranges
```

**New format (header = 6 bytes, detent = 2 bytes):**

```
[type: u8 = 11] [pin: u8] [num_detents: u8] [num_linear_ranges: u8]
[snap_point: u8] [endstop_strength: u8]
[position: u8] [detent_strength: u8]  x num_detents
[start: u8] [end: u8] [damping: u8]  x num_linear_ranges
```

All other messages (Configure, DeactivateBLDCProfile, RetryCalibration, etc.) are unchanged.

See `docs/BLDC_PROTOCOL_MIGRATION.md` for complete migration guide for host software.

## PD Controller

### P Gain

```
P = (detent_strength / 255.0) * P_SCALE_FACTOR
```

Starting `P_SCALE_FACTOR = 4.0` (from SmartKnob). Tuned on hardware.

### Dynamic D Gain

Piecewise linear interpolation based on the angular width between adjacent detents (distance to nearest neighbor of the current detent, in ticks).

```
D_LOWER_STRENGTH = (detent_strength / 255.0) * 0.08   // narrow detents
D_UPPER_STRENGTH = (detent_strength / 255.0) * 0.02   // wide detents

D_WIDTH_LOWER = ticks_per_degree * 3     // 3 degrees equivalent
D_WIDTH_UPPER = ticks_per_degree * 8     // 8 degrees equivalent

raw_D = D_LOWER + (D_UPPER - D_LOWER)
        / (D_WIDTH_UPPER - D_WIDTH_LOWER)
        * (detent_width_ticks - D_WIDTH_LOWER)

D = clamp(raw_D, min(D_LOWER, D_UPPER), max(D_LOWER, D_UPPER))
```

`ticks_per_degree = calibrated_range / lever_arc_degrees`. Since lever arc is unknown, this constant is defined in `bldc_config.h` and tuned on hardware. Starting estimate: `calibrated_range / 90.0` (assume 90-degree arc).

D is recalculated whenever the active detent changes.

### Dead Zone

```
dead_zone_fraction = 0.2
dead_zone_abs = ticks_per_degree * 1.0   // 1 degree equivalent
dead_zone = min(detent_width * dead_zone_fraction, dead_zone_abs)
```

Within the dead zone, the error fed to the PD controller is reduced to zero.

### Torque Computation (per loop iteration)

```
1. angle_error = current_position - current_detent_center
2. dead_zone_adj = clamp(angle_error, -dead_zone, +dead_zone)
3. pid_input = -(angle_error - dead_zone_adj)
4. If in linear range: pid_input = 0, apply damping instead
5. If out of bounds: P = (endstop_strength / 255.0) * P_SCALE_FACTOR
6. torque = P * pid_input + D * (-current_velocity)
7. If abs(velocity) > MAX_SAFE_VELOCITY: torque = 0
```

## Snap Point / Hysteresis

Replaces "closest detent wins" in `updateDetentState()`.

For each direction, compute a snap boundary:

```
distance_to_next = abs(next_detent_pos - current_detent_pos)
snap_boundary = current_detent_pos + distance_to_next * (snap_point / 100.0)
```

- `snap_point = 50` (0.50): transition at midpoint
- `snap_point = 100` (1.00): must reach next detent center
- `snap_point = 110` (1.10): must overshoot past next detent

On transition: reset `detent_center_offset` to 0, recalculate P and D, set `detent_changed` flag.

At boundaries (first/last detent), no neighbor exists in one direction — handled by virtual endstop.

In linear ranges, snap point still governs when the detent index transitions. Reported detent stays at the range's start until the snap threshold to the end is crossed.

## Virtual Endstops

When position goes beyond the outermost detents:

```
P = (endstop_strength / 255.0) * P_SCALE_FACTOR
angle_error = current_position - nearest_boundary
torque = -P * angle_error
```

No dead zone in endstop region. D gain kept at nearest detent's value. Velocity cutoff still applies.

## Idle Correction

When the lever is stationary but not perfectly centered on a detent, slowly drift the virtual detent center toward the shaft.

### State

```cpp
float velocity_ewma_;
uint32_t idle_start_time_;
float detent_center_offset_;
```

### Constants

```
IDLE_VELOCITY_EWMA_ALPHA   = 0.001
IDLE_VELOCITY_THRESHOLD    = 0.05    // ticks/ms
IDLE_CORRECTION_DELAY_MS   = 500
IDLE_CORRECTION_MAX_ANGLE  = ticks_per_degree * 5   // 5 degrees
IDLE_CORRECTION_RATE_ALPHA = 0.0005
```

### Logic

```
1. velocity_ewma = velocity_ewma * (1 - ALPHA) + abs(velocity) * ALPHA
2. If velocity_ewma > THRESHOLD: idle_start_time = now; return
3. If (now - idle_start_time) < DELAY: return
4. If abs(angle_to_detent_center) > MAX_ANGLE: return
5. detent_center_offset += (current_pos - nominal_detent_center) * RATE_ALPHA
```

Offset resets to 0 on detent transitions.

## Velocity Tracking

Finite difference with low-pass filter:

```cpp
float prev_position_;
float current_velocity_;
uint32_t prev_update_time_;
```

Per iteration:

```
dt = (now - prev_update_time) / 1000.0
raw_velocity = (current_position - prev_position) / dt
current_velocity = current_velocity * (1 - LPF_ALPHA) + raw_velocity * LPF_ALPHA
```

Constants:

```
VELOCITY_LPF_ALPHA  = 0.1
MAX_SAFE_VELOCITY   = 60.0   // ticks/ms (tune on hardware)
```

## Linear Range Damping

When position is inside a defined linear range between two detents:

```
damping_torque = -(damping_strength / 255.0) * current_velocity * DAMPING_SCALE
```

PD detent pull is zeroed (pid_input = 0). Only damping is applied.

## Files Changed

| File | Changes |
|---|---|
| `src/bldc_config.h` | Simplify `DetentConfig`, add `ProfileConfig`, add PD/dead zone/idle constants, remove `spring_back_target` |
| `src/bldc_lever.h` | Add velocity/idle state vars, update `loadProfile` signature, add PD helpers |
| `src/bldc_lever.cpp` | Rewrite `calculateTargetTorque()`, `updateDetentState()`, `isInLinearRange()`. Add velocity tracking, dead zone, idle correction, virtual endstops |
| `src/protocol.h` | Update `LoadBLDCProfile` struct (add snap_point, endstop_strength fields) |
| `src/protocol.cpp` | Update encode/decode for new format |
| `src/message_handler.cpp` | Update `LoadBLDCProfile` parsing |
| `test/BLDCMotor.h` | Add `shaft_velocity` to mock |
| `test/test_bldc_lever/test_bldc_lever.cpp` | Update existing tests, add PD/dead zone/snap/endstop/damping/velocity/idle tests |
| `test/test_protocol_bldc/test_protocol_bldc.cpp` | Update for new message format |
| `docs/PROTOCOL.md` | Update LoadBLDCProfile specification |
| `docs/BLDC_LEVER.md` | Update for simplified config and PD motor control |
| `docs/BLDC_PROTOCOL_MIGRATION.md` | NEW: migration guide for host software |

## Files NOT Changed

- `bldc_manager.h/cpp` — just calls `updateMotor()`
- `main.cpp` — unchanged
- `sensor_manager.cpp` — hardware config path unchanged
- `config_manager.cpp` — EEPROM stores hardware config, not profiles

## Test Strategy

Tests verify the math using mock BLDCMotor/encoder:

- **PD torque direction**: position away from detent -> torque pulls toward
- **Dead zone**: position within dead zone -> near-zero torque
- **Snap point**: gradual movement -> detent index unchanged until threshold
- **Virtual endstop**: position beyond last detent -> pushback torque
- **Linear range damping**: between range detents -> damping torque, no PD pull
- **Velocity cutoff**: high velocity -> zero torque
- **Idle correction**: stationary off-center + time advance -> center drifts

## Tuning Constants

All constants are `constexpr` in `bldc_config.h`. Starting values from SmartKnob, to be tuned on hardware:

| Constant | Starting Value | Source |
|---|---|---|
| `P_SCALE_FACTOR` | 4.0 | SmartKnob |
| `D_LOWER_FACTOR` | 0.08 | SmartKnob |
| `D_UPPER_FACTOR` | 0.02 | SmartKnob |
| `D_WIDTH_LOWER_DEG` | 3.0 | SmartKnob |
| `D_WIDTH_UPPER_DEG` | 8.0 | SmartKnob |
| `DEAD_ZONE_FRACTION` | 0.2 | SmartKnob |
| `DEAD_ZONE_MAX_DEG` | 1.0 | SmartKnob |
| `VELOCITY_LPF_ALPHA` | 0.1 | Estimate |
| `MAX_SAFE_VELOCITY` | 60.0 | SmartKnob |
| `IDLE_VELOCITY_EWMA_ALPHA` | 0.001 | SmartKnob |
| `IDLE_VELOCITY_THRESHOLD` | 0.05 | SmartKnob |
| `IDLE_CORRECTION_DELAY_MS` | 500 | SmartKnob |
| `IDLE_CORRECTION_MAX_DEG` | 5.0 | SmartKnob |
| `IDLE_CORRECTION_RATE_ALPHA` | 0.0005 | SmartKnob |
| `DAMPING_SCALE` | 1.0 | Estimate |
| `LEVER_ARC_DEGREES` | 90.0 | Estimate |
