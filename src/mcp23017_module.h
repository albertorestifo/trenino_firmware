#pragma once

#include "i2c_health.h"
#include "matrix_core.h"
#include "module.h"
#include <stdint.h>

namespace Modules {

// MCP23017 I/O expander: direct buttons plus an optional button matrix
// (row_mask/col_mask).
//
// GPA7/GPB7 (0x8080) are never inputs. Pull-ups are on for every input.
// Row bits in OLAT idle high; one non-row bit carries a sentinel that is
// always written high, so a chip reset (which clears OLAT) can be detected.
class Mcp23017Module : public IModule {
public:
    static constexpr uint16_t OUTPUT_ONLY_MASK = 0x8080;
    static constexpr unsigned long SAMPLE_INTERVAL_MS = 5; // no matrix
    static constexpr unsigned long MATRIX_ROW_INTERVAL_MS = 2; // one row per tick

    Mcp23017Module(uint8_t address, uint8_t direct_base, uint8_t matrix_base,
        uint8_t debounce, uint16_t direct_mask, uint16_t row_mask, uint16_t col_mask);

    // A copy does not share the matrix; it allocates its own in begin().
    Mcp23017Module(const Mcp23017Module& other);
    Mcp23017Module& operator=(const Mcp23017Module&) = delete;
    ~Mcp23017Module() override;

    bool begin() override;
    void scan() override;
    Reading getReading() override;
    ModuleType getType() const override { return ModuleType::MCP23017; }
    uint8_t getPin() const override { return 0; }
    uint8_t getI2CAddress() const override { return address_; }
    uint8_t takeRuntimeError() override;
    uint8_t activeError() const override;

#ifdef UNIT_TEST
    const I2CHealth& health() const { return health_; }
    uint16_t sentinelBit() const { return sentinel_; }
    bool reinitRegistersForTest() { return initRegisters(); }
    uint16_t nextOlatForTest() const { return expectedOlat(); }
#endif

private:
    bool writeRegisters(uint8_t reg, uint16_t value);
    bool writeIocon();
    bool initRegisters();
    bool ensureMatrix();
    void markFailed(unsigned long now);
    uint16_t expectedOlat() const;
    void recover(unsigned long now);
    void handleReset(unsigned long now);
    void noteFailure(unsigned long now);
    bool olatMatches(const uint8_t olat[2]);
    bool sample(uint16_t& gpio, uint8_t olat[2]);
    void debounceDirect(uint16_t pressed);
    void scanMatrixRow(unsigned long now);
    static uint16_t nthSetBit(uint16_t mask, uint8_t n);

    uint8_t address_;
    uint8_t direct_base_;
    uint8_t matrix_base_;
    uint8_t debounce_;
    uint16_t direct_mask_; // effective: GPA7/GPB7 and row bits removed
    uint16_t row_mask_;
    uint16_t col_mask_;
    uint16_t sentinel_;
    uint16_t olat_expected_; // OLAT value last written by the scan

    unsigned long last_sample_ms_;
    uint8_t olat_read_[2]; // OLATA/OLATB from the last good sample
    I2CHealth health_;

    uint16_t state_; // debounced, 1 = pressed
    uint16_t reported_; // last state handed out by getReading()
    uint8_t counts_[16];

    MatrixCore* matrix_; // allocated in begin(), only when row_mask_ != 0
    uint8_t row_count_;
    uint8_t col_count_;
    uint8_t current_row_; // row index (not bit) to sample next
};

} // namespace Modules
