#include "mcp23017_module.h"
#include "i2c_bus.h"
#include <Arduino.h>
#include <Wire.h>

namespace Modules {

namespace {
constexpr uint8_t REG_IODIR = 0x00;
constexpr uint8_t REG_IOCON = 0x0A;
constexpr uint8_t REG_GPPU = 0x0C;
constexpr uint8_t REG_GPIO = 0x12;
constexpr uint8_t REG_OLAT = 0x14;
constexpr uint8_t SAMPLE_BYTES = 4; // GPIOA, GPIOB, OLATA, OLATB
}

Mcp23017Module::Mcp23017Module(uint8_t address, uint8_t direct_base, uint8_t matrix_base,
    uint8_t debounce, uint16_t direct_mask, uint16_t row_mask, uint16_t col_mask)
    : address_(address)
    , direct_base_(direct_base)
    , matrix_base_(matrix_base)
    , debounce_(debounce == 0 ? 1 : debounce)
    , direct_mask_(direct_mask & ~(row_mask | OUTPUT_ONLY_MASK))
    , row_mask_(row_mask)
    , col_mask_(col_mask)
    , sentinel_(0)
    , olat_expected_(0)
    , last_sample_ms_(0)
    , state_(0)
    , reported_(0)
    , matrix_(nullptr)
    , row_count_(0)
    , col_count_(0)
    , current_row_(0)
    , row_selected_(false)
{
    // Lowest bit that is not a row. For a direct-only chip this is bit 0.
    uint16_t bit = 1;
    while (bit != 0 && (row_mask_ & bit)) bit <<= 1;
    sentinel_ = bit;
    olat_expected_ = row_mask_ | sentinel_;
    for (uint8_t i = 0; i < 16; i++) counts_[i] = 0;
    for (uint16_t m = row_mask_; m; m &= m - 1) row_count_++;
    for (uint16_t m = col_mask_ & ~(row_mask_ | OUTPUT_ONLY_MASK); m; m &= m - 1) col_count_++;
    if (row_count_ > MatrixCore::MAX_ROWS) row_count_ = MatrixCore::MAX_ROWS;
    if (col_count_ > MatrixCore::MAX_COLS) col_count_ = MatrixCore::MAX_COLS;
}

// matrix_ is owned by the original; a copy allocates its own in begin().
Mcp23017Module::Mcp23017Module(const Mcp23017Module& o)
    : address_(o.address_)
    , direct_base_(o.direct_base_)
    , matrix_base_(o.matrix_base_)
    , debounce_(o.debounce_)
    , direct_mask_(o.direct_mask_)
    , row_mask_(o.row_mask_)
    , col_mask_(o.col_mask_)
    , sentinel_(o.sentinel_)
    , olat_expected_(o.olat_expected_)
    , last_sample_ms_(o.last_sample_ms_)
    , health_(o.health_)
    , state_(o.state_)
    , reported_(o.reported_)
    , matrix_(nullptr)
    , row_count_(o.row_count_)
    , col_count_(o.col_count_)
    , current_row_(o.current_row_)
    , row_selected_(o.row_selected_)
{
    for (uint8_t i = 0; i < 16; i++) counts_[i] = o.counts_[i];
}

Mcp23017Module::~Mcp23017Module()
{
    delete matrix_;
}

// Position of the n-th (0-based) set bit of mask, as a one-bit mask; 0 if none.
uint16_t Mcp23017Module::nthSetBit(uint16_t mask, uint8_t n)
{
    for (uint16_t bit = 1; bit; bit <<= 1) {
        if (!(mask & bit)) continue;
        if (n == 0) return bit;
        n--;
    }
    return 0;
}

bool Mcp23017Module::writeRegisters(uint8_t reg, uint16_t value)
{
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.write((uint8_t)(value & 0xFF));
    Wire.write((uint8_t)(value >> 8));
    return Wire.endTransmission() == 0;
}

bool Mcp23017Module::writeIocon()
{
    Wire.beginTransmission(address_);
    Wire.write(REG_IOCON);
    Wire.write((uint8_t)0);
    return Wire.endTransmission() == 0;
}

bool Mcp23017Module::begin()
{
    I2CBus::begin();

    state_ = 0;
    reported_ = 0;
    for (uint8_t i = 0; i < 16; i++) counts_[i] = 0;
    current_row_ = 0;
    row_selected_ = false;
    unsigned long now = millis();
    last_sample_ms_ = now;

    // No runtime code 1 here: ModuleManager reports a failed begin() as code 0.
    // The chip is failed at once so that it retries; recovery reports code 2.
    // Matrix state is cleared here only, never on recovery, and even when the
    // chip is absent.
    bool allocated = ensureMatrix();
    if (matrix_) matrix_->reset();
    if (!allocated || !initRegisters()) {
        markFailed(now);
        return false;
    }
    return true;
}

// Allocation only: recovery must not touch matrix state.
bool Mcp23017Module::ensureMatrix()
{
    if (row_mask_ == 0) return true;
    if (!matrix_) {
        matrix_ = new MatrixCore(row_count_, col_count_, debounce_);
        if (matrix_) matrix_->reset();
    }
    return matrix_ != nullptr;
}

void Mcp23017Module::markFailed(unsigned long now)
{
    health_.markFailed(now);
}

uint16_t Mcp23017Module::expectedOlat() const
{
    if (!matrix_) return (uint16_t)(row_mask_ | sentinel_);
    return (uint16_t)((row_mask_ & ~nthSetBit(row_mask_, current_row_)) | sentinel_);
}

// Retry while failed. Uses initRegisters(), never begin(): begin() would clear
// input state and lose held buttons and their release events.
void Mcp23017Module::recover(unsigned long now)
{
    if (ensureMatrix() && initRegisters()) {
        health_.noteSuccess();
        health_.push(2);
        last_sample_ms_ = now;
    } else {
        health_.noteFailure(now); // restarts the retry timer
    }
}

// The chip answered but OLAT is not what was written: it was reset. The sample
// is discarded and the scan restarts at row 0
// (initRegisters leaves row 0 selected).
void Mcp23017Module::handleReset(unsigned long now)
{
    health_.push(1);
    last_sample_ms_ = now;
    if (initRegisters()) {
        health_.push(2);
    } else {
        markFailed(now); // code 1 already queued; recovers through the retry
    }
}

uint8_t Mcp23017Module::takeRuntimeError()
{
    return health_.take();
}

uint8_t Mcp23017Module::activeError() const
{
    return health_.failed() ? 1 : MODULE_ERROR_NONE;
}

// Register writes only: safe to repeat after an outage without disturbing
// debounced input state. A matrix chip is left with row 0 selected, and that
// OLAT value is the expected one.
bool Mcp23017Module::initRegisters()
{
    // Order matters: outputs get their level before they become outputs.
    uint16_t inputs = (uint16_t) ~(row_mask_ | OUTPUT_ONLY_MASK);
    current_row_ = 0;
    row_selected_ = false;
    uint16_t olat = expectedOlat();
    if (!writeIocon()) return false;
    if (!writeRegisters(REG_OLAT, olat)) return false;
    olat_expected_ = olat;
    if (!writeRegisters(REG_GPPU, inputs)) return false;
    if (!writeRegisters(REG_IODIR, inputs)) return false;
    row_selected_ = true;
    return true;
}

bool Mcp23017Module::sample(uint16_t& gpio, uint8_t olat[2])
{
    Wire.beginTransmission(address_);
    Wire.write(REG_GPIO);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom(address_, SAMPLE_BYTES) != SAMPLE_BYTES) return false;
    uint8_t b[SAMPLE_BYTES];
    for (uint8_t i = 0; i < SAMPLE_BYTES; i++) {
        int v = Wire.read();
        if (v < 0) return false;
        b[i] = (uint8_t)v;
    }
    gpio = (uint16_t)(b[0] | (b[1] << 8));
    olat[0] = b[2];
    olat[1] = b[3];
    return true;
}

void Mcp23017Module::scan()
{
    unsigned long now = millis();
    if (health_.failed()) {
        // No polling while failed; retry once per interval.
        if (health_.retryDue(now)) recover(now);
        return;
    }
    unsigned long interval = matrix_ ? MATRIX_ROW_INTERVAL_MS : SAMPLE_INTERVAL_MS;
    if ((now - last_sample_ms_) < interval) return;
    last_sample_ms_ = now;

    if (matrix_) {
        scanMatrixRow(now);
        return;
    }

    uint16_t gpio;
    uint8_t olat[2];
    if (!sample(gpio, olat)) {
        // A failed transaction changes no input state.
        noteFailure(now);
        return;
    }
    if (!olatMatches(olat)) {
        // A mismatch counts as a failure so that repeated resets reach the
        // threshold; only a good sample clears the count.
        if (health_.noteFailure(now)) {
            health_.push(1);
        } else {
            handleReset(now);
        }
        return;
    }
    health_.noteSuccess();

    debounceDirect(~gpio); // pull-ups: pressed = low
}

// Reads the row selected on the previous tick, checks OLAT against the value
// last written, feeds that row, then selects the next row. Reading before
// selecting means a chip reset is seen before any write can hide it. A failed
// read leaves the row selected and is retried; a failed select write is retried
// on the next tick by rewriting every register (the chip may be mid-reset), not
// just OLAT, and without sampling, so no row is fed twice. Direct pins are
// debounced once per full scan.
void Mcp23017Module::scanMatrixRow(unsigned long now)
{
    if (!row_selected_) {
        // No success is noted here: a chip that keeps NACKing must still reach
        // the failure threshold, so only a good sample resets the count.
        if (!initRegisters()) noteFailure(now);
        return;
    }

    uint16_t gpio;
    uint8_t olat[2];
    if (!sample(gpio, olat)) {
        noteFailure(now);
        return;
    }
    if (!olatMatches(olat)) {
        // A mismatch counts as a failure so that repeated resets reach the
        // threshold; only a good sample clears the count.
        if (health_.noteFailure(now)) {
            health_.push(1);
        } else {
            handleReset(now);
        }
        return;
    }
    health_.noteSuccess();

    uint16_t pressed = ~gpio; // pull-ups: pressed = low
    for (uint8_t c = 0; c < col_count_; c++) {
        uint16_t col_bit = nthSetBit(col_mask_ & ~(row_mask_ | OUTPUT_ONLY_MASK), c);
        matrix_->sample(current_row_, c, (pressed & col_bit) != 0);
    }

    if (++current_row_ >= row_count_) {
        current_row_ = 0;
        debounceDirect(pressed);
    }
    selectRow(now);
}

// Writes OLAT for current_row_. The expected value only changes when the write
// succeeds.
void Mcp23017Module::selectRow(unsigned long now)
{
    uint16_t olat = expectedOlat();
    if (!writeRegisters(REG_OLAT, olat)) {
        row_selected_ = false;
        noteFailure(now);
        return;
    }
    olat_expected_ = olat;
    row_selected_ = true;
}

void Mcp23017Module::noteFailure(unsigned long now)
{
    if (health_.noteFailure(now)) health_.push(1);
}

bool Mcp23017Module::olatMatches(const uint8_t olat[2]) const
{
    return olat[0] == (uint8_t)(olat_expected_ & 0xFF) && olat[1] == (uint8_t)(olat_expected_ >> 8);
}

void Mcp23017Module::debounceDirect(uint16_t pressed)
{
    for (uint8_t bit = 0; bit < 16; bit++) {
        uint16_t m = (uint16_t)1 << bit;
        if (!(direct_mask_ & m)) continue;
        bool raw = (pressed & m) != 0;
        bool cur = (state_ & m) != 0;
        if (raw == cur) {
            counts_[bit] = 0;
        } else if (++counts_[bit] >= debounce_) {
            counts_[bit] = 0;
            state_ ^= m;
        }
    }
}

Reading Mcp23017Module::getReading()
{
    uint16_t pending = state_ ^ reported_;
    for (uint8_t bit = 0; pending && bit < 16; bit++) {
        uint16_t m = (uint16_t)1 << bit;
        if (!(pending & m)) continue;
        reported_ ^= m;
        return Reading((state_ & m) ? 1 : 0, ModuleType::MCP23017, (uint8_t)(direct_base_ + bit));
    }
    uint8_t index;
    bool pressed;
    if (matrix_ && matrix_->pop(index, pressed)) {
        return Reading(pressed ? 1 : 0, ModuleType::MCP23017, (uint8_t)(matrix_base_ + index));
    }
    return Reading();
}

} // namespace Modules
