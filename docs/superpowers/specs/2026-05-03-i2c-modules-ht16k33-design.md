# I2C Modules and HT16K33 14-Segment Display — Design

**Status:** Design approved, ready for implementation planning
**Date:** 2026-05-03

## Goal

Add support for I2C-attached peripheral modules to the Trenino firmware. The HT16K33 14-segment LED display is the first module driver. The design generalizes so future I2C modules — both output (more displays, GPIO expanders) and input (ADCs, GPIO expanders read as buttons) — can be added without further architectural changes.

The firmware stays minimal: it owns chip initialization, write retries, and bus management. Content (formatting, glyph translation, animation) lives entirely on the host.

## Scope

In scope:
- HT16K33 driver: init, write display RAM, runtime brightness, retry, auto-reinit on repeated failure.
- Renaming the protocol's "input" terminology to "module" (breaking change).
- Refactoring `ISensor`/`SensorManager` to `IModule`/`ModuleManager` so output modules fit naturally.
- Three new protocol messages plus a new module type.
- Single I2C bus support only.

Out of scope (deferred, documented for future):
- HT16K33 keypad scanning (the chip supports 3×13 key matrix on the same address — would be a separate module type or config extension).
- Multi-bus support (e.g. ESP32's `Wire1`).
- Generic `I2CBus` namespace with shared error reporting and configurable clock speed.
- I2C input modules (MCP23017, ADS1115). The architecture accommodates them but no driver is built.
- Generic raw I2C passthrough (`I2CWrite`/`I2CRead`/poll-slot machinery).

## Breaking changes

- `INPUT_TYPE_*` constants in `protocol.h` rename to `MODULE_TYPE_*`. Same numeric values; HT16K33 is the new value `4`.
- `Configure` message payload union members keep their names (`analog`, `button`, `matrix`, `bldc_lever`); a new `ht16k33` member joins them.
- `ConfigManager::InputConfig` renames to `ConfigManager::ModuleConfig`. Same union shape plus the new HT16K33 variant.
- `MAX_INPUTS` renames to `MAX_MODULES` (still 8).
- `Sensors::ISensor` renames to `Modules::IModule`. `Sensors::InputType` renames to `Modules::ModuleType`. `SensorManager` renames to `ModuleManager`. `getSensorByPin` renames to `getModuleByPin`.
- EEPROM magic value bumps from `0xC0FF1234` to `0xC0FF1235` so old configs are discarded on first boot. No migration code.
- `Sensors::Reading` keeps its name and shape — it still describes what input modules emit; output modules simply never produce one.
- Per-class filenames (`analog_sensor.cpp` etc.) are not renamed — churn for no benefit.

## Architecture

### Module abstraction

```cpp
namespace Modules {

enum class ModuleType : uint8_t {
    Analog = 0,
    Button = 1,
    Matrix = 2,
    BLDCLever = 3,
    HT16K33 = 4,
};

struct Reading {
    bool has_value;
    int16_t value;
    ModuleType type;
    uint8_t pin;
    // unchanged from current Sensors::Reading
};

class IModule {
public:
    virtual ~IModule() {}
    virtual bool begin() = 0;                          // returns true on successful init
    virtual void scan() {}                             // default no-op (output modules)
    virtual Reading getReading() { return Reading(); } // default: no value
    virtual ModuleType getType() const = 0;
    virtual uint8_t getPin() const = 0;                // 0 for I2C-only modules
    virtual uint8_t getI2CAddress() const { return 0; } // 0 for hardware-pin modules
};

} // namespace Modules
```

`begin()` returns `bool` so modules that can fail at init (HT16K33 today, future I2C input modules) can signal it. Existing modules that can't fail (analog, button, matrix) return `true` unconditionally. BLDC retains its existing init semantics and returns `true`.

`scan()` and `getReading()` become virtual-with-default rather than pure-virtual so output modules don't need degenerate overrides. Cost is ~4 bytes vtable per class — negligible on AVR.

`getI2CAddress()` is added so `ModuleManager` can look up I2C modules by address. It returns 0 for hardware-pin modules.

### ModuleManager

`ModuleManager` (renamed from `SensorManager`) keeps its current shape:

- Stores up to `MAX_MODULES` (8) `IModule*` instances.
- `applyConfiguration()` deletes existing modules, instantiates new ones from `ModuleConfig` array, calls `begin()` on each, registers BLDC levers with `BLDCManager` as today. For each module whose `begin()` returns `false`, records the module's identifying handle (i2c_address for HT16K33, pin for future input modules) and type so `MessageHandler` can emit a `ModuleError` for it after `applyConfiguration` returns. Modules that fail `begin()` are still added to the registry (so subsequent writes can attempt re-init), but they are tagged as failed so the host gets one error notification.
- `scan()` calls `scan()` on every module (no-op for HT16K33).
- `getNextReading()` round-robins through modules calling `getReading()` (no-op for HT16K33).
- `getModuleByPin(uint8_t pin)` — unchanged behavior, renamed.
- **New** `getModuleByI2CAddress(uint8_t address)` — iterates modules and matches by `getI2CAddress()`.
- **New** `getInitErrors(InitError* out, uint8_t max_count) -> uint8_t` — drains the per-`applyConfiguration` failure list. `MessageHandler` calls this after `applyConfiguration` to send `ModuleError` messages for each failure. The list is cleared after each drain.

```cpp
struct InitError {
    ModuleType type;       // module type that failed
    uint8_t i2c_address;   // populated for I2C modules; 0 otherwise
    uint8_t pin;           // populated for hardware-pin modules; 0 otherwise
    uint8_t error_code;    // matches ModuleError protocol error codes
};
```

The list is fixed-size (`MAX_MODULES`) and lives inside `ModuleManager` — no dynamic allocation.

### HT16K33 module

**File:** `src/ht16k33_module.h/cpp`

Owns:
- I2C address (`uint8_t`) — set at construction, immutable
- Cached brightness (`uint8_t`, 0–15) — updated by `setBrightness` so re-init restores the latest value
- `num_digits` (`uint8_t`, 4 or 8 — sets the cap on `WriteSegments` data length)
- Cached segment buffer (`uint8_t[16]`) so re-init can restore display state
- `cached_segment_bytes` (`uint8_t`) — count of bytes in the segment buffer that are valid
- Consecutive-failure counter (`uint8_t`)
- `needs_reinit` flag (`bool`) — set when failure counter reaches threshold; checked at the start of every write/setBrightness call

Public methods:
- `bool begin()` — runs the init sequence. Returns `true` if every transaction was ACKed; `false` otherwise. Resets the failure counter and `needs_reinit` on success. The init sequence's brightness step always uses the *cached* brightness, so a re-init naturally restores the latest brightness the host set. Restoring cached segments after a re-init is the runtime write flow's responsibility (see below), not `begin()`'s.
- `bool writeSegments(const uint8_t* data, uint8_t num_bytes)` — see runtime write flow below. Updates the cached segment buffer on every call (regardless of success) so a later re-init restores the host's most recent intent.
- `bool setBrightness(uint8_t level)` — clamps to 0–15, **updates the cached brightness field first**, then writes `0xE0 | level` to the device using the runtime write flow.

**Init sequence (executed by `begin()`):**
1. `Wire.beginTransmission(addr); Wire.write(0x21); Wire.endTransmission();` — turn on system oscillator
2. `Wire.beginTransmission(addr); Wire.write(0x81); Wire.endTransmission();` — display on, no blink
3. `Wire.beginTransmission(addr); Wire.write(0xE0 | cached_brightness); Wire.endTransmission();` — set brightness
4. Write 16 bytes of zeros to display RAM starting at register `0x00` (single transaction)

If any step's `endTransmission()` returns non-zero (NACK), `begin()` sets `needs_reinit = true` and returns `false` immediately without attempting subsequent steps. The next runtime call will retry init via the runtime write flow.

**Runtime write flow** (used by `writeSegments` and `setBrightness`):
1. If `needs_reinit` is set, run `begin()` first. On success, also rewrite the cached segment buffer (one transaction: register `0x00` + `cached_segment_bytes` bytes). If re-init fails, return `false` immediately; `needs_reinit` stays set, and the next call will retry re-init.
2. Attempt the requested write. If it ACKs, reset the failure counter and return `true`.
3. On NACK, delay ~1 ms and retry once. If the retry ACKs, reset the failure counter and return `true`.
4. If the retry also NACKs, increment the failure counter. If the counter reaches `3`, set `needs_reinit` and reset the counter to `0`. Return `false`.

The host is never notified of runtime write failures by design — the host re-sends segment state on the next change, and re-init self-heals power blips.

**Threshold rationale:** 3 consecutive failures keeps recovery responsive without paying the cost of a re-init for every transient glitch. Re-init runs at most once per ~3 host-driven writes in pathological conditions, which is acceptable.

### I2C bus initialization

`Wire.begin()` is called lazily on the first HT16K33 module's `begin()`, guarded by a static flag in `ht16k33_module.cpp`:

```cpp
static bool g_wire_initialized = false;
// inside HT16K33Module::begin():
if (!g_wire_initialized) {
    Wire.begin();
    g_wire_initialized = true;
}
```

Bus clock left at platform default (100 kHz), comfortably within HT16K33's 400 kHz limit.

When a second I2C module type is added, this lazy init is hoisted into a small `I2CBus` namespace — but not now (YAGNI).

## Protocol additions

### MODULE_TYPE_HT16K33 = 4

Configure payload (3 bytes):
```
[i2c_address: u8]    // 0x70-0x77 typical, no enforcement in firmware
[brightness: u8]     // 0-15 (clamped)
[num_digits: u8]     // 4 or 8 — caps WriteSegments data length at num_digits * 2
```

### MESSAGE_TYPE_WRITE_SEGMENTS = 13 (Host → Device)

```
[type: u8 = 13] [i2c_address: u8] [num_bytes: u8] [data: u8[num_bytes]]
```

`num_bytes` ≤ 16. Firmware caps `num_bytes` at the configured `num_digits * 2` (silently truncates if oversized). Fire-and-forget; no response. If `i2c_address` doesn't match any configured HT16K33 module, silently dropped.

### MESSAGE_TYPE_SET_MODULE_BRIGHTNESS = 14 (Host → Device)

```
[type: u8 = 14] [i2c_address: u8] [brightness: u8]
```

`brightness` clamped to 0–15. Fire-and-forget; no response.

### MESSAGE_TYPE_MODULE_ERROR = 15 (Device → Host)

```
[type: u8 = 15] [i2c_address: u8] [error_code: u8]
```

Error codes:
- `0` = `init_failed` — chip NACKed during `begin()`. Module is added to the registry but in a failed state; subsequent writes will trigger re-init attempts.
- `1` = `unsupported_module` — reserved for future use.
- `2+` = reserved for future module-type-specific errors.

Sent only at module init time. Runtime write failures do not generate `ModuleError`.

### Addressing rule

The protocol now has two addressing schemes, picked by message:
- **Hardware-pin-based modules** (analog, button, matrix, BLDC): use `pin` (real Arduino pin, or virtual pin in matrix's 128–223 range).
- **I2C-addressed modules** (HT16K33 today, future MCP23017 / ADS1115 / etc.): use `i2c_address`.

Each message's field name makes the addressing unambiguous. No `pin`/`i2c_address` overlap.

## Configuration sequence (HT16K33)

Identical to existing input modules — host sends one `Configure` message per module as part of a normal multi-part configuration. HT16K33 is configured alongside any other modules in the same configuration round.

```
Host                              Device
  |                                  |
  |-- Configure (HT16K33 part) ----->| (other parts may also be sent)
  |                                  | (on completion of all parts:)
  |                                  | - persist to EEPROM
  |                                  | - call applyConfiguration:
  |                                  |   - HT16K33Module::begin()
  |                                  |     - lazy Wire.begin()
  |                                  |     - chip init sequence
  |<-------- ConfigurationStored ----| (if all modules init OK)
  |<-------- ModuleError ------------| (per HT16K33 that NACKed at init)
  |                                  |
  |-- WriteSegments ---------------->| (any time after init)
  |-- SetModuleBrightness ---------->| (any time after init)
```

`ConfigurationStored` is sent regardless of per-module init outcome (the configuration is stored; individual modules may have failed). After `applyConfiguration` returns, `MessageHandler` drains `ModuleManager::getInitErrors()` and sends one `ModuleError` per failed module.

## Memory and platform impact

Estimates per Pro Micro (ATmega32U4, 2.5 KB SRAM, 28 KB flash):

| Item | Flash | SRAM |
|------|-------|------|
| Wire library (one-time) | ~600 B | ~32 B (twin buffers) |
| `HT16K33Module` instance | — | ~24 B (addr + brightness + num_digits + segment buffer + failure counter + bookkeeping) |
| `IModule` virtual default thunks | ~16 B | — |
| New protocol message encode/decode | ~200 B | — |

Worst case with one HT16K33 configured: ~56 B SRAM, ~800 B flash. Comfortable on Pro Micro. No conditional compilation needed.

## Testing

### Mock additions

New `test/Wire.h` modeled after `test/EEPROM.h`:
- Records `beginTransmission(addr)` / `write(byte)` / `endTransmission()` sequences into a per-address transcript the test can inspect.
- Configurable NACK behavior per address (so tests can simulate init failure, transient write failure, and recovery).
- `requestFrom`/`read` stubs for future I2C input modules (returning canned responses).

### New test files

- `test/test_ht16k33_module/` — covers:
  - `begin()` emits the four init transactions in order on success.
  - `begin()` returns `false`, sets `needs_reinit`, and skips remaining steps when an early `endTransmission()` returns non-zero.
  - `writeSegments()` writes register `0x00` followed by the data bytes; caps at `num_digits * 2`.
  - `setBrightness(level)` clamps to 0–15, updates the cached brightness, then writes `0xE0 | level`.
  - Single retry after ~1 ms on transient NACK; success on retry resets the failure counter.
  - Failure counter increments on persistent NACK; reaches 3 → sets `needs_reinit`, resets counter.
  - When `needs_reinit` is set, the next write runs `begin()` first, then rewrites cached segments and the requested write.
  - After a successful re-init triggered by a failed `setBrightness`, the *new* (cached) brightness value is used during the init's brightness step.
  - Cached segment buffer is updated on every `writeSegments` call regardless of success.

- `test/test_protocol_module/` — covers:
  - Encode/decode for `Configure` with HT16K33 payload.
  - Encode/decode for `WriteSegments`, `SetModuleBrightness`, `ModuleError`.
  - Rename smoke test: `Modules::ModuleType::HT16K33 == 4`, `MAX_MODULES == 8`.

### Existing test updates

Mechanical rename across all existing tests:
- `Sensors::` → `Modules::`
- `getSensorByPin` → `getModuleByPin`
- `INPUT_TYPE_*` → `MODULE_TYPE_*`
- `MAX_INPUTS` → `MAX_MODULES`
- `Sensors::ISensor` → `Modules::IModule`
- `Sensors::InputType` → `Modules::ModuleType`
- `Sensors::Reading` unchanged

### platformio.ini

Update `build_src_filter` for the `native` env to exclude `ht16k33_module.cpp` (Wire is hardware-only).

## Documentation updates

- `docs/PROTOCOL.md`:
  - Section "Module Types" replaces "Input Types" with the renamed constants.
  - New section documenting the `pin` vs `i2c_address` addressing rule.
  - New message definitions for `WriteSegments`, `SetModuleBrightness`, `ModuleError`.
  - HT16K33 Configure payload definition.
  - Note: only one I2C bus is supported.
- `docs/ARCHITECTURE.md`:
  - Module overview updated for renames and the new `ht16k33_module.h/cpp` files.
  - "Adding new module types" section replaces "Adding new sensor types"; covers both pin-based and I2C-addressed modules.
  - "Future expansion" section notes deferred work: HT16K33 keypad, multi-bus, generic `I2CBus` namespace, I2C input modules.

## Implementation ordering (suggested for the plan)

1. Rename pass: `Sensors::*` → `Modules::*`, `SensorManager` → `ModuleManager`, `INPUT_TYPE_*` → `MODULE_TYPE_*`, `MAX_INPUTS` → `MAX_MODULES`. EEPROM magic bump. Update existing tests. Verify all tests pass.
2. `IModule` interface reshape: `begin()` returns `bool` (existing modules updated to `return true`), `scan()` and `getReading()` become virtual-with-default, add `getI2CAddress()`. Update `ModuleManager::applyConfiguration` to track failed-init modules and add `getInitErrors()`. Verify existing modules still pass tests.
3. Add `Modules::ModuleType::HT16K33`, `Configure` payload variant, `ConfigManager::ModuleConfig::ht16k33`. Update `ConfigState::addPart` for the new variant.
4. Mock `test/Wire.h`. Add `test_ht16k33_module` covering init, writes, brightness, retry, re-init.
5. Implement `HT16K33Module` until tests pass.
6. Add `WriteSegments`, `SetModuleBrightness`, `ModuleError` to `protocol.h/cpp` and `Message` union, with encode/decode tests.
7. Wire new messages into `MessageHandler`: dispatch `WriteSegments` and `SetModuleBrightness` via `ModuleManager::getModuleByI2CAddress`; after `applyConfiguration` returns, drain `ModuleManager::getInitErrors()` and emit one `ModuleError` per failure.
8. Wire HT16K33 instantiation into `ModuleManager::applyConfiguration`. Update `platformio.ini` to exclude `ht16k33_module.cpp` from native env.
9. Update `docs/PROTOCOL.md` and `docs/ARCHITECTURE.md`.
10. Manual verification on hardware: configure an HT16K33 via host, write segments, change brightness, power-cycle the chip and confirm auto-reinit recovers.
