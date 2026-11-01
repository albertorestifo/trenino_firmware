# I2C Input Modules Firmware Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add MCP23017 and ADS1115 input modules to the firmware and release them as 3.2.0.

**Architecture:** Each chip is one `IModule`, polled on `millis()` timers so it never blocks for long. Modules report through the existing `Reading` path using host-assigned virtual pins. A shared health helper handles failure, retry and error reporting.

**Tech Stack:** C++11, Arduino, PlatformIO, Unity tests on the `native` environment.

**Spec:** `/Users/alberto/repos/arestifo/trenino/docs/superpowers/specs/2026-11-01-i2c-input-modules-design.md`

**Companion plan:** `trenino/docs/superpowers/plans/2026-11-01-i2c-input-modules.md`. This plan ships first.

## Global Constraints

- Firmware version 3.2.0. `EEPROM_FORMAT_VERSION` stays 5.
- `InputValue` and every existing `Configure` payload stay byte-identical.
- `MCP23017 = 5` with a 10-byte payload, `ADS1115 = 6` with a 9-byte payload.
- GPA7 and GPB7 (bits 7 and 15, mask `0x8080`) are never inputs.
- The bus stays at 100 kHz. `setWireTimeout(25000, true)` on AVR.
- Timer comparisons use unsigned subtraction, so they survive `millis()` wrap.
- A failed transaction changes no input state and produces no events.
- `ModuleError` codes: 0 init failed, 1 communication lost, 2 recovered, 3 supply reference lost.
- Run tests with `pio test -e native`. Build with `pio run`.
- Tests include the `.cpp` under test directly, and `test/Wire.h` is included by one translation unit per test binary.
- Every new hardware `.cpp` is excluded from `build_src_filter` in `[env:native]`.

## Review Focus

1. A chip absent at boot sends no events, and starts working when connected later, without a reconfigure.
2. A short read, where `requestFrom` returns fewer bytes than asked, counts as a failed transaction.
3. A latched switch held at boot is reported as pressed once debounced, as `ButtonSensor` does.
4. A direct-only chip still has a working reset sentinel.
5. A reference of 0, or a negative ADS reading, never divides by zero or produces a value above 1023 or below 0.

## File Structure

| File | Responsibility |
|---|---|
| `src/i2c_bus.h/.cpp` (new) | One-time `Wire` setup |
| `src/i2c_health.h` (new) | Failure counting, retry timing, pending error codes |
| `src/matrix_core.h/.cpp` (new) | Matrix debounce and event queue, no hardware access |
| `src/mcp23017_module.h/.cpp` (new) | MCP23017 direct and matrix scanning |
| `src/ads1115_module.h/.cpp` (new) | ADS1115 conversion state machine and reporting |
| `src/module.h` | Runtime-error virtuals, new `ModuleType` values |
| `src/protocol.h/.cpp`, `src/config_manager.h/.cpp` | New `Configure` types and EEPROM records |
| `src/sensor_manager.h/.cpp`, `src/message_handler.cpp` | Instantiation, runtime error sending |
| `test/Wire.h` | Scripted reads |

---

### Task 1: Shared I2C setup and scripted mock reads

**Files:**
- Create: `src/i2c_bus.h`, `src/i2c_bus.cpp`
- Modify: `src/ht16k33_module.cpp`, `src/stepper_driver.cpp`, `test/Wire.h`, `platformio.ini`, `test/test_ht16k33_module/test_ht16k33_module.cpp`, `test/test_stepper_driver/test_stepper_driver.cpp`

**Interfaces:**
- Produces:
  - `void I2CBus::begin()`, safe to call repeatedly.
  - `void I2CBus::resetForTest()` under `UNIT_TEST`, replacing `ht16k33_reset_wire_state()`.
  - `MockWire::queueRead(uint8_t address, const uint8_t* bytes, uint8_t length)`: the next `requestFrom` to that address returns these bytes in order.
  - `MockWire::read_request_count`, the number of `requestFrom` calls.
  - With nothing queued, `requestFrom` returns 0.

- [ ] **Step 1: Write failing tests** in `test_ht16k33_module.cpp`: `test_begin_twice_initializes_wire_once` asserts `MockWire::wire_timeout_call_count == 1` after two modules call `begin()`. Add a new test in the same file, `test_mock_wire_returns_queued_read`, asserting `requestFrom` returns the queued length and `read()` yields those bytes.
- [ ] **Step 2: Run** `pio test -e native -f test_ht16k33_module`. Expected: FAIL, `queueRead` undefined.
- [ ] **Step 3: Implement** `I2CBus` and extend the mock. Replace the guard in `HT16K33Module::begin()` and `startWire()` in `stepper_driver.cpp` with `I2CBus::begin()`. `ShieldDriver::shutdown` must still work when it runs first at boot. Add `-<i2c_bus.cpp>` to the native filter, and include the `.cpp` from the tests that need it.
- [ ] **Step 4: Run** `pio test -e native`. Expected: all pass.
- [ ] **Step 5: Run** `pio run -e sparkfun_promicro16`. Expected: SUCCESS.
- [ ] **Step 6: Commit** `refactor: share I2C bus setup`.

### Task 2: Extract matrix debounce

**Files:**
- Create: `src/matrix_core.h`, `src/matrix_core.cpp`, `test/test_matrix_core/test_matrix_core.cpp`
- Modify: `src/matrix_sensor.h`, `src/matrix_sensor.cpp`, `platformio.ini`

**Interfaces:**
- Produces, in namespace `Modules`:
  ```cpp
  class MatrixCore {
  public:
      static constexpr uint8_t MAX_ROWS = 8, MAX_COLS = 8, EVENT_QUEUE_SIZE = 8;
      MatrixCore(uint8_t rows, uint8_t cols, uint8_t debounce_threshold);
      void reset();
      void sample(uint8_t row, uint8_t col, bool pressed);
      bool pop(uint8_t& button_index, bool& pressed);
  };
  ```
  `button_index` is `row * cols + col`.

- [ ] **Step 1: Write failing tests** in `test_matrix_core.cpp`: `test_press_needs_threshold_samples`, `test_bounce_resets_count`, `test_queue_full_drops_oldest`, `test_pop_returns_row_col_index`.
- [ ] **Step 2: Run** `pio test -e native -f test_matrix_core`. Expected: FAIL to compile.
- [ ] **Step 3: Implement** by moving the logic from `MatrixSensor::scanButton`, `enqueueEvent` and `getReading`. Store debounced and reported state as one `uint8_t` bitmap per row. `MatrixSensor` keeps `DEFAULT_DEBOUNCE`, `VIRTUAL_PIN_BASE` and its public behaviour. Exclude the new `.cpp` in the native filter.
- [ ] **Step 4: Run** `pio test -e native -f test_matrix_core` and `-f test_matrix_sensor`. Expected: PASS, with `test_matrix_sensor.cpp` changed only to include `matrix_core.cpp`.
- [ ] **Step 5: Commit** `refactor: extract matrix debounce into MatrixCore`.

### Task 3: Protocol and EEPROM records

**Files:**
- Modify: `src/protocol.h`, `src/protocol.cpp`, `src/config_manager.h`, `src/config_manager.cpp`, `src/stepper_service.cpp`, `test/test_protocol/test_protocol.cpp`, `test/test_config_manager/test_config_manager.cpp`

**Interfaces:**
- Produces:
  - `MODULE_TYPE_MCP23017 = 5`, `MODULE_TYPE_ADS1115 = 6`, `ADS_NO_REFERENCE = 0xFF`.
  - `Configure` and `ModuleConfig` union members:
    ```cpp
    struct { uint8_t i2c_address, direct_base, matrix_base, debounce;
             uint16_t direct_mask, row_mask, col_mask; } mcp23017;
    struct { uint8_t i2c_address, pin_base, channel_mask, gain, ref_channel;
             uint8_t sensitivity[4]; } ads1115;
    ```
  - `bool Protocol::validMcp23017(const ...)` and `validAds1115(...)`, used by `decode`.

- [ ] **Step 1: Write failing protocol tests:** round trips for both types with masks little-endian on the wire, and one test per spec rejection:
  - MCP: `test_mcp_rejects_overlapping_masks`, `test_mcp_rejects_gpa7_input`, `test_mcp_rejects_gpb7_column`, `test_mcp_allows_gpa7_gpb7_rows`, `test_mcp_rejects_nine_rows`, `test_mcp_rejects_rows_without_cols`, `test_mcp_rejects_zero_base_for_used_block`, `test_mcp_rejects_block_past_255`, `test_mcp_rejects_base_below_128`, `test_mcp_rejects_debounce_zero`.
  - ADS: `test_ads_rejects_gain_2`, `test_ads_rejects_ref_in_channel_mask`, `test_ads_rejects_ref_4`, `test_ads_accepts_no_ref`, `test_ads_rejects_mask_bits_above_3`, `test_ads_rejects_block_past_255`, `test_ads_rejects_sensitivity_11_on_enabled_channel`.
  - Block sizes come from the spec: direct is 16, matrix is `rows * cols`, ADS is 4.
- [ ] **Step 2: Write failing config tests:** `test_eeprom_round_trip_mcp23017`, `test_eeprom_round_trip_ads1115`, and `test_eeprom_loads_3_1_0_record`, which writes a format 5 analog/button/HT16K33 layout byte by byte and asserts it loads unchanged.
- [ ] **Step 3: Run** `pio test -e native -f test_protocol` and `-f test_config_manager`. Expected: FAIL.
- [ ] **Step 4: Implement** encode, decode, `ConfigState::addPart`, `storeToEEPROM`, `loadFromEEPROM`, and the new cases in `StepperService::conflicts` and `moduleConflicts`, comparing `i2c_address` with the shield address. Do not change `EEPROM_FORMAT_VERSION`. Add a `static_assert` that the new union members fit within the matrix member's size, keeping `ModuleConfig` the same size.
- [ ] **Step 5: Run** `pio test -e native`. Expected: all pass.
- [ ] **Step 6: Commit** `feat: add MCP23017 and ADS1115 configuration`.

### Task 4: Health helper and MCP23017 direct buttons

**Files:**
- Create: `src/i2c_health.h`, `src/mcp23017_module.h`, `src/mcp23017_module.cpp`, `test/test_mcp23017_module/test_mcp23017_module.cpp`
- Modify: `src/module.h`, `platformio.ini`

**Interfaces:**
- Consumes: `I2CBus::begin()`, `MockWire::queueRead`.
- Produces:
  - In `module.h`: `ModuleType::MCP23017 = 5`, `ModuleType::ADS1115 = 6`, `MODULE_ERROR_NONE = 0xFF`, and two `IModule` virtuals with defaults:
    `virtual uint8_t takeRuntimeError() { return MODULE_ERROR_NONE; }` and `virtual uint8_t activeError() const { return MODULE_ERROR_NONE; }`.
  - `class I2CHealth`:
    `bool noteFailure(unsigned long now)`, true when the threshold of 3 is reached; `bool noteSuccess()`, true when this ended a failed state; `bool failed() const`; `bool retryDue(unsigned long now) const`, every 1000 ms; `void push(uint8_t code)`; `uint8_t take()`.
  - `Mcp23017Module(uint8_t address, uint8_t direct_base, uint8_t matrix_base, uint8_t debounce, uint16_t direct_mask, uint16_t row_mask, uint16_t col_mask)`.

**Registers (BANK = 0):** IODIRA `0x00`, IOCON `0x0A`, GPPUA `0x0C`, GPIOA `0x12`, OLATA `0x14`. Paired A/B registers are written and read sequentially, low byte first.

**Init order:** IOCON = 0, then OLAT, then GPPU, then IODIR, so that outputs never glitch.

```
sentinel = lowest bit clear in row_mask
OLAT     = row_mask | sentinel       (row bits high = idle)
IODIR    = ~(row_mask | 0x8080)      (1 = input)
GPPU     = ~(row_mask | 0x8080)
```

Unused pins stay inputs with pull-ups, except bits 7 and 15, which are always outputs.

- [ ] **Step 1: Write failing tests:**
  - `test_begin_writes_expected_registers`: asserts the four transactions for `direct_mask = 0x0003`, and that IODIR bit 7 and 15 are clear.
  - `test_begin_nack_returns_false`.
  - `test_no_read_before_5ms` and `test_reads_four_bytes_from_gpioa`.
  - `test_press_reported_after_debounce_samples`: active low, reported as `Reading(1, ModuleType::MCP23017, direct_base + bit)`.
  - `test_release_reported`.
  - `test_two_pins_change_two_readings`.
  - `test_held_at_boot_reports_press` (Review Focus 3).
  - `test_pins_outside_mask_ignored`.
  - `test_short_read_counts_as_failure` (Review Focus 2).
  - `test_sentinel_exists_for_direct_only_chip` (Review Focus 4).
  - `test_timer_survives_millis_wrap`.
- [ ] **Step 2: Run** `pio test -e native -f test_mcp23017_module`. Expected: FAIL to compile.
- [ ] **Step 3: Implement** the module without a matrix. `scan()` returns unless 5 ms have passed. One sample writes the pointer `0x12`, then reads 4 bytes. Debounce with a per-pin counter, and hold changed bits in a `uint16_t` pending mask that `getReading()` drains one at a time. Failure handling beyond the short-read count arrives in Task 6.
- [ ] **Step 4: Run** the tests. Expected: PASS.
- [ ] **Step 5: Commit** `feat: add MCP23017 direct inputs`.

### Task 5: MCP23017 matrix

**Files:**
- Modify: `src/mcp23017_module.h`, `src/mcp23017_module.cpp`, `test/test_mcp23017_module/test_mcp23017_module.cpp`

**Interfaces:**
- Consumes: `MatrixCore`.
- Produces: no new public names.

- [ ] **Step 1: Write failing tests:**
  - `test_matrix_scans_one_row_every_2ms`: exactly one OLAT write and one read per tick, and none before 2 ms.
  - `test_matrix_row_low_others_high`: the written OLAT has the selected row clear, other rows set, and the sentinel set.
  - `test_matrix_cell_pin_number`: `matrix_base + row * cols + col`, with row and col as indexes into the set bits.
  - `test_matrix_uses_rows_on_gpa7_gpb7_with_cols_on_both_ports`.
  - `test_matrix_debounce_counts_full_scans`.
  - `test_direct_pins_debounce_once_per_cycle_on_mixed_chip`.
  - `test_direct_and_matrix_readings_both_drain`.
- [ ] **Step 2: Run** the tests. Expected: FAIL.
- [ ] **Step 3: Implement.** With a matrix, the tick is 2 ms: write OLAT for the current row, read the 4 bytes, feed that row's columns to `MatrixCore`, then advance. There is no settle delay. Allocate `MatrixCore` only when `row_mask != 0`, so direct-only chips use less RAM.
- [ ] **Step 4: Run** the tests. Expected: PASS.
- [ ] **Step 5: Commit** `feat: add MCP23017 matrix scanning`.

### Task 6: MCP23017 failure and reset handling

**Files:**
- Modify: `src/mcp23017_module.h`, `src/mcp23017_module.cpp`, `test/test_mcp23017_module/test_mcp23017_module.cpp`

**Interfaces:**
- Consumes: `I2CHealth`.
- Produces: overrides of `takeRuntimeError()` and `activeError()`.

- [ ] **Step 1: Write failing tests:**
  - `test_nack_during_poll_produces_no_event`.
  - `test_held_button_stays_held_through_outage`.
  - `test_three_failures_report_code_1_once`.
  - `test_failed_chip_stops_polling_and_retries_every_1000ms`.
  - `test_recovery_reports_code_2_and_reinitializes`.
  - `test_change_during_outage_reported_after_recovery`.
  - `test_absent_at_boot_then_connected_recovers`, with no events before it connects (Review Focus 1). This reports code 2 because `begin()` failed.
  - `test_olat_mismatch_discards_sample_and_reinitializes`: the read returns pins all low with OLAT zero, and no press is reported.
  - `test_olat_mismatch_reports_code_1_then_2`.
  - `test_active_error_is_1_while_failed`.
- [ ] **Step 2: Run** the tests. Expected: FAIL.
- [ ] **Step 3: Implement** per the spec's Failure handling section. Compare the OLAT bytes on every sample before using the GPIO bytes. A `begin()` failure marks the chip failed immediately, so that it retries.
- [ ] **Step 4: Run** the tests. Expected: PASS.
- [ ] **Step 5: Commit** `feat: recover MCP23017 from failures and resets`.

### Task 7: ADS1115 module

**Files:**
- Create: `src/ads1115_module.h`, `src/ads1115_module.cpp`, `test/test_ads1115_module/test_ads1115_module.cpp`
- Modify: `platformio.ini`

**Interfaces:**
- Consumes: `I2CHealth`, `I2CBus::begin()`.
- Produces:
  - `Ads1115Module(uint8_t address, uint8_t pin_base, uint8_t channel_mask, uint8_t gain, uint8_t ref_channel, const uint8_t* sensitivity)`.
  - `static uint16_t Ads1115Module::scaled(int16_t raw, uint16_t ref)`, where `ref == 0` means no reference.

**Config register:** `0xC1A3 | (channel << 12) | (gain << 9)`, sent MSB first to pointer `0x01`. This is OS start, single-ended, single-shot, 250 SPS, comparator off. The result is read from pointer `0x00`, MSB first.

**Reference:** smoothed as `ref = (ref * 3 + raw) / 4`, seeded by the first reading.

```
with reference:    min(1023, max(raw, 0) * 1023UL / ref)
without reference: max(raw, 0) >> 5
```

- [ ] **Step 1: Write failing tests:**
  - `test_scaled_without_ref`: 26666 gives 833, 32767 gives 1023.
  - `test_scaled_with_ref`: raw equal to ref gives 1023, half gives 511.
  - `test_scaled_clamps`: raw above ref gives 1023, negative raw gives 0 (Review Focus 5).
  - `test_config_word_channel_2_gain_1`: writes `0x01, 0xE3, 0xA3`.
  - `test_no_read_before_5ms`, `test_rotation_skips_disabled_channels`.
  - `test_sends_on_change_after_rate_limit`: sensitivity 10 sends on the next conversion, sensitivity 1 waits 10 conversions.
  - `test_change_within_dead_zone_not_sent`.
  - `test_still_channel_sends_every_100ms`.
  - `test_reference_channel_never_reported`.
  - `test_values_held_until_first_reference`.
  - `test_reference_below_4000_stops_values_reports_code_3_once`.
  - `test_reference_returning_reports_code_2`.
  - `test_nack_produces_no_value`, `test_three_failures_report_code_1`, `test_recovery_reports_code_2`, `test_absent_at_boot_then_connected_recovers`.
- [ ] **Step 2: Run** `pio test -e native -f test_ads1115_module`. Expected: FAIL to compile.
- [ ] **Step 3: Implement** the state machine, `IDLE` then `CONVERTING`. Never block in `scan()`. Each enabled channel holds its current value, last sent value, conversion count and last send time.
- [ ] **Step 4: Run** the tests. Expected: PASS.
- [ ] **Step 5: Commit** `feat: add ADS1115 analog inputs`.

### Task 8: Wire modules in, report runtime errors, release

**Files:**
- Modify: `src/sensor_manager.h`, `src/sensor_manager.cpp`, `src/message_handler.cpp`, `src/device_info.h`, `docs/PROTOCOL.md`, `docs/ARCHITECTURE.md`, `CHANGELOG.md`, `CLAUDE.md`, `test/test_module_manager_init_errors/test_module_manager_init_errors.cpp`

**Interfaces:**
- Consumes: `Mcp23017Module`, `Ads1115Module`, the `IModule` error virtuals.
- Produces:
  - `bool ModuleManager::getNextRuntimeError(uint8_t& address, uint8_t& code)`.
  - `uint8_t ModuleManager::getActiveErrors(InitError* out, uint8_t max_count)`.

- [ ] **Step 1: Write failing tests:** `test_runtime_errors_drain_with_address` and `test_active_errors_list_failed_modules`, using a fake `IModule`.
- [ ] **Step 2: Run** `pio test -e native -f test_module_manager_init_errors`. Expected: FAIL.
- [ ] **Step 3: Implement.**
  - `applyConfiguration` creates both modules, passing `debounce` and `sensitivity` from `ModuleConfig`.
  - `MessageHandler::update()` sends `getNextRuntimeError` results after the reading loop.
  - `handleIdentityRequest` sends one `ModuleError` per `getActiveErrors` entry, after the init errors, without duplicating an address already sent in that response.
- [ ] **Step 4: Update docs.** In `PROTOCOL.md`, add both payloads and the error codes. In `ARCHITECTURE.md`, add the files, remove I2C input modules from Future Expansion, and correct the main loop description, which has no `delay(1)`. List the new modules in `CLAUDE.md` and add a `CHANGELOG.md` entry.
- [ ] **Step 5: Set version** 3.2.0 in `device_info.h`. Leave `EEPROM_FORMAT_VERSION` at 5.
- [ ] **Step 6: Run** `pio test -e native`, then `pio run`. Expected: all tests pass and all environments build.
- [ ] **Step 7: Check size.** `pio run -e sparkfun_promicro16` must report flash under 95%. Record static RAM in the PR. Add a temporary measurement of `sizeof(Mcp23017Module)` and confirm two 8×8 chips leave over 300 bytes free. If not, shrink the debounce counters before continuing.
- [ ] **Step 8: Commit** `feat: enable I2C input modules for 3.2.0`.

### Task 9: Bench acceptance

No code unless a check fails. Needs the owner's hardware, so hand this to them with the checklist.

- [ ] **Step 1:** Run the spec's Acceptance bench checks that need no host UI, using `examples/` or a small host script: a press on each chip, a 3.1.0 config surviving the update, and a gauge moving during a full matrix scan.
- [ ] **Step 2:** If gauges stutter, try `Wire.setClock(400000)` in `I2CBus::begin()` per the spec, after checking the panel wiring length.
