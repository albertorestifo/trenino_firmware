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
{
    // Lowest bit that is not a row. For a direct-only chip this is bit 0.
    uint16_t bit = 1;
    while (bit != 0 && (row_mask_ & bit)) bit <<= 1;
    sentinel_ = bit;
    olat_expected_ = row_mask_ | sentinel_;
    olat_read_[0] = olat_read_[1] = 0;
    for (uint8_t i = 0; i < 16; i++) counts_[i] = 0;
    for (uint16_t m = row_mask_; m; m &= m - 1) row_count_++;
    for (uint16_t m = col_mask_ & ~(row_mask_ | OUTPUT_ONLY_MASK); m; m &= m - 1) col_count_++;
    if (row_count_ > MatrixCore::MAX_ROWS) row_count_ = MatrixCore::MAX_ROWS;
    if (col_count_ > MatrixCore::MAX_COLS) col_count_ = MatrixCore::MAX_COLS;
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
    if (row_mask_ != 0) {
        if (!matrix_) matrix_ = new MatrixCore(row_count_, col_count_, debounce_);
        matrix_->reset();
    }
    last_sample_ms_ = millis();

    return initRegisters();
}

// Register writes only: safe to repeat after an outage without disturbing
// debounced input state.
bool Mcp23017Module::initRegisters()
{
    // Order matters: outputs get their idle level before they become outputs.
    uint16_t inputs = (uint16_t) ~(row_mask_ | OUTPUT_ONLY_MASK);
    if (!writeIocon()) return false;
    if (!writeRegisters(REG_OLAT, (uint16_t)(row_mask_ | sentinel_))) return false; // rows idle high
    if (!writeRegisters(REG_GPPU, inputs)) return false;
    if (!writeRegisters(REG_IODIR, inputs)) return false;
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
        // A failed transaction changes no input state. Threshold handling
        // and recovery are added with the failure-handling step.
        health_.noteFailure(now);
        return;
    }
    health_.noteSuccess();
    // Place for the OLAT-vs-olat_expected_ check (reset detection).
    olat_read_[0] = olat[0];
    olat_read_[1] = olat[1];

    debounceDirect(~gpio); // pull-ups: pressed = low
}

// Selects the current row (OLAT write), reads the inputs, and feeds that row's
// columns to the matrix. If either transaction fails the row is not fed and
// is retried on the next tick. Direct pins are debounced once per full scan.
void Mcp23017Module::scanMatrixRow(unsigned long now)
{
    uint16_t row_bit = nthSetBit(row_mask_, current_row_);
    olat_expected_ = (uint16_t)((row_mask_ & ~row_bit) | sentinel_);

    uint16_t gpio;
    uint8_t olat[2];
    if (!writeRegisters(REG_OLAT, olat_expected_) || !sample(gpio, olat)) {
        health_.noteFailure(now);
        return;
    }
    health_.noteSuccess();
    // Place for the OLAT-vs-olat_expected_ check (reset detection).
    olat_read_[0] = olat[0];
    olat_read_[1] = olat[1];

    uint16_t pressed = ~gpio; // pull-ups: pressed = low
    for (uint8_t c = 0; c < col_count_; c++) {
        uint16_t col_bit = nthSetBit(col_mask_ & ~(row_mask_ | OUTPUT_ONLY_MASK), c);
        matrix_->sample(current_row_, c, (pressed & col_bit) != 0);
    }

    if (++current_row_ >= row_count_) {
        current_row_ = 0;
        debounceDirect(pressed);
    }
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
