#define ARDUINO_ARCH_AVR
#ifndef UNIT_TEST
#define UNIT_TEST
#endif
#include "../Wire.h"
#include "../../src/i2c_bus.cpp"
#include "../../src/matrix_core.cpp"
#include "../../src/mcp23017_module.cpp"
#include <new>
#include <type_traits>
#include <unity.h>

using namespace Modules;

static_assert(!std::is_copy_constructible<Mcp23017Module>::value,
    "a copy would not own the matrix");

static unsigned long g_now = 0;
unsigned long millis() { return g_now; }
void delay(unsigned long) {}

static const uint8_t ADDR = 0x20;
static const uint8_t BASE = 200;

// Simulated chip: OLAT, GPPU and IODIR are latched from the successful register
// writes recorded by the mock, and a reset returns them to their power-on
// values. `g_seen` is how many recorded transactions have been applied.
static uint16_t g_olat = 0;
static uint16_t g_gppu = 0;
static uint16_t g_iodir = 0xFFFF;
static uint8_t g_seen = 0;
static bool g_reset_after_read = false; // reset right after the next sample

static void syncChip()
{
    if (MockWire::transaction_count < g_seen) g_seen = 0;
    for (; g_seen < MockWire::transaction_count; g_seen++) {
        const MockWire::Transaction& t = MockWire::transactions[g_seen];
        if (t.end_status != 0 || t.length != 3) continue;
        uint16_t value = (uint16_t)(t.data[1] | (t.data[2] << 8));
        if (t.data[0] == 0x14) g_olat = value;
        if (t.data[0] == 0x0C) g_gppu = value;
        if (t.data[0] == 0x00) g_iodir = value;
    }
}

// Clears the recorded transactions without losing the simulated chip state.
static void resetTransactions()
{
    syncChip();
    MockWire::transaction_count = 0;
    g_seen = 0;
}

static void simulateChipReset()
{
    syncChip();
    g_olat = 0;
    g_gppu = 0;
    g_iodir = 0xFFFF;
}

// OLAT bytes of 0xDE 0xAD at the end of a scripted read mean "whatever the
// simulated chip holds when the read is served".
static void fixupRead(uint8_t, uint8_t* bytes, uint8_t length)
{
    if (length >= 2 && bytes[length - 2] == 0xDE && bytes[length - 1] == 0xAD) {
        syncChip();
        bytes[length - 2] = (uint8_t)(g_olat & 0xFF);
        bytes[length - 1] = (uint8_t)(g_olat >> 8);
    }
    if (g_reset_after_read) {
        g_reset_after_read = false;
        simulateChipReset();
    }
}

void setUp()
{
    MockWire::reset();
    MockWire::read_fixup = fixupRead;
    I2CBus::resetForTest();
    g_now = 0;
    g_olat = 0;
    g_gppu = 0;
    g_iodir = 0xFFFF;
    g_seen = 0;
    g_reset_after_read = false;
}

// Modules cannot be copied, so make() and makeMatrix() build them in static
// storage. A test holds at most two at once.
static const uint8_t SLOTS = 2;
alignas(Mcp23017Module) static unsigned char g_storage[SLOTS][sizeof(Mcp23017Module)];
static Mcp23017Module* g_built[SLOTS] = { nullptr, nullptr };
static uint8_t g_next_slot = 0;

static void destroySlot(uint8_t slot)
{
    if (g_built[slot]) g_built[slot]->~Mcp23017Module();
    g_built[slot] = nullptr;
}

static Mcp23017Module& build(uint8_t matrix_base, uint8_t debounce, uint16_t direct_mask,
    uint16_t row_mask, uint16_t col_mask)
{
    uint8_t slot = g_next_slot;
    g_next_slot = (uint8_t)((slot + 1) % SLOTS);
    destroySlot(slot);
    g_built[slot] = new (g_storage[slot])
        Mcp23017Module(ADDR, BASE, matrix_base, debounce, direct_mask, row_mask, col_mask);
    return *g_built[slot];
}

void tearDown()
{
    for (uint8_t i = 0; i < SLOTS; i++) destroySlot(i);
}

// One scripted sample: GPIOA, GPIOB, then the OLAT bytes a healthy chip returns.
static void queueSample(uint8_t gpioA, uint8_t gpioB)
{
    const uint8_t bytes[4] = { gpioA, gpioB, 0x01, 0x00 };
    MockWire::queueRead(ADDR, bytes, 4);
}

// Sample whose OLAT bytes are what the simulated chip holds when it is read.
static void queueNext(const Mcp23017Module&, uint8_t gpioA, uint8_t gpioB)
{
    const uint8_t bytes[4] = { gpioA, gpioB, 0xDE, 0xAD };
    MockWire::queueRead(ADDR, bytes, 4);
}

// The read-back of a recovery attempt on a healthy chip.
static void queueOlatReadback()
{
    const uint8_t bytes[2] = { 0xDE, 0xAD };
    MockWire::queueRead(ADDR, bytes, 2);
}

static void queueShort()
{
    const uint8_t bytes[3] = { 0xFF, 0xFF, 0x01 };
    MockWire::queueRead(ADDR, bytes, 3);
}

// Advances 5 ms and scans once.
static void tick(Mcp23017Module& m)
{
    g_now += 5;
    m.scan();
}

static Mcp23017Module& make(uint16_t direct_mask = 0x0003, uint8_t debounce = 3, uint16_t row_mask = 0)
{
    return build(0, debounce, direct_mask, row_mask, 0);
}

void test_begin_writes_expected_registers()
{
    Mcp23017Module& m = make(0x0003);
    TEST_ASSERT_TRUE(m.begin());
    TEST_ASSERT_EQUAL_UINT8(4, MockWire::transaction_count);

    const uint8_t expect[4][3] = {
        { 0x0A, 0x00, 0x00 }, // IOCON
        { 0x14, 0x01, 0x00 }, // OLAT: sentinel bit 0
        { 0x0C, 0x7F, 0x7F }, // GPPU
        { 0x00, 0x7F, 0x7F }, // IODIR
    };
    const uint8_t len[4] = { 2, 3, 3, 3 };
    for (uint8_t i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_UINT8(ADDR, MockWire::transactions[i].address);
        TEST_ASSERT_EQUAL_UINT8(len[i], MockWire::transactions[i].length);
        for (uint8_t j = 0; j < len[i]; j++) {
            TEST_ASSERT_EQUAL_UINT8(expect[i][j], MockWire::transactions[i].data[j]);
        }
    }
    // GPA7 and GPB7 are never inputs.
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::transactions[3].data[1] & 0x80);
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::transactions[3].data[2] & 0x80);
}

void test_begin_nack_returns_false()
{
    MockWire::scheduleNacks(ADDR, 1);
    Mcp23017Module& m = make();
    TEST_ASSERT_FALSE(m.begin());
}

void test_no_read_before_5ms()
{
    Mcp23017Module& m = make();
    m.begin();
    g_now = 4;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::read_request_count);
    g_now = 5;
    queueSample(0xFF, 0xFF);
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::read_request_count);
}

void test_reads_four_bytes_from_gpioa()
{
    Mcp23017Module& m = make();
    m.begin();
    resetTransactions();
    queueSample(0xFF, 0xFF);
    tick(m);
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transactions[0].length);
    TEST_ASSERT_EQUAL_UINT8(0x12, MockWire::transactions[0].data[0]);
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::read_request_count);
    TEST_ASSERT_EQUAL_UINT8(4, MockWire::last_request_quantity);
}

void test_press_reported_after_debounce_samples()
{
    Mcp23017Module& m = make(0x0003, 3);
    m.begin();
    for (uint8_t i = 0; i < 2; i++) {
        queueSample(0xFE, 0xFF);
        tick(m);
        TEST_ASSERT_FALSE(m.getReading().has_value);
    }
    queueSample(0xFE, 0xFF);
    tick(m);
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(1, r.value);
    TEST_ASSERT_TRUE(r.type == ModuleType::MCP23017);
    TEST_ASSERT_EQUAL_UINT8(BASE + 0, r.pin);
    TEST_ASSERT_FALSE(m.getReading().has_value);
}

void test_release_reported()
{
    Mcp23017Module& m = make(0x0003, 2);
    m.begin();
    for (uint8_t i = 0; i < 2; i++) { queueSample(0xFD, 0xFF); tick(m); }
    TEST_ASSERT_TRUE(m.getReading().has_value);
    for (uint8_t i = 0; i < 2; i++) { queueSample(0xFF, 0xFF); tick(m); }
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(0, r.value);
    TEST_ASSERT_EQUAL_UINT8(BASE + 1, r.pin);
}

void test_two_pins_change_two_readings()
{
    Mcp23017Module& m = make(0x0103, 1);
    m.begin();
    queueSample(0xFE, 0xFE); // GPA0 and GPB0 low
    tick(m);
    Reading a = m.getReading();
    Reading b = m.getReading();
    TEST_ASSERT_TRUE(a.has_value);
    TEST_ASSERT_TRUE(b.has_value);
    TEST_ASSERT_EQUAL_UINT8(BASE + 0, a.pin);
    TEST_ASSERT_EQUAL_UINT8(BASE + 8, b.pin);
    TEST_ASSERT_FALSE(m.getReading().has_value);
}

void test_held_at_boot_reports_press()
{
    g_now = 1000;
    Mcp23017Module& m = make(0x0001, 3);
    m.begin();
    for (uint8_t i = 0; i < 3; i++) { queueSample(0xFE, 0xFF); tick(m); }
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(1, r.value);
    TEST_ASSERT_EQUAL_UINT8(BASE, r.pin);
}

void test_register_reinit_keeps_held_state()
{
    Mcp23017Module& m = make(0x0001, 2);
    m.begin();
    for (uint8_t i = 0; i < 2; i++) { queueSample(0xFE, 0xFF); tick(m); }
    TEST_ASSERT_TRUE(m.getReading().has_value); // press reported

    resetTransactions();
    TEST_ASSERT_TRUE(m.reinitRegistersForTest());
    TEST_ASSERT_EQUAL_UINT8(4, MockWire::transaction_count);

    // Still held after reinit: no new event, and the release debounces normally.
    queueSample(0xFE, 0xFF);
    tick(m);
    TEST_ASSERT_FALSE(m.getReading().has_value);
    for (uint8_t i = 0; i < 2; i++) { queueSample(0xFF, 0xFF); tick(m); }
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(0, r.value);
}

void test_pins_outside_mask_ignored()
{
    Mcp23017Module& m = make(0x0001, 1);
    m.begin();
    queueSample(0xFE & 0x7F, 0x00); // bit 0 low, everything else low including GPA7/GPB7
    tick(m);
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_UINT8(BASE, r.pin);
    TEST_ASSERT_FALSE(m.getReading().has_value);

    // GPA7 in direct_mask is stripped as well.
    Mcp23017Module& m2 = make(0x8080, 1);
    m2.begin();
    queueSample(0x00, 0x00);
    tick(m2);
    TEST_ASSERT_FALSE(m2.getReading().has_value);
}

void test_short_read_counts_as_failure()
{
    Mcp23017Module& m = make(0x0001, 3);
    m.begin();
    for (uint8_t i = 0; i < 2; i++) { queueSample(0xFE, 0xFF); tick(m); }
    queueShort();
    tick(m);
    TEST_ASSERT_EQUAL_UINT8(1, m.health().failures());
    TEST_ASSERT_FALSE(m.getReading().has_value);
    // The failed sample left the debounce counter alone.
    queueSample(0xFE, 0xFF);
    tick(m);
    TEST_ASSERT_TRUE(m.getReading().has_value);
    TEST_ASSERT_EQUAL_UINT8(0, m.health().failures());
}

void test_pointer_write_nack_counts_as_failure()
{
    Mcp23017Module& m = make(0x0001, 1);
    m.begin();
    MockWire::scheduleNacks(ADDR, 1);
    queueSample(0xFE, 0xFF);
    tick(m);
    TEST_ASSERT_EQUAL_UINT8(1, m.health().failures());
    TEST_ASSERT_FALSE(m.getReading().has_value);
}

void test_sentinel_exists_for_direct_only_chip()
{
    Mcp23017Module& m = make(0x0003, 3, 0);
    TEST_ASSERT_EQUAL_UINT16(0x0001, m.sentinelBit());
    m.begin();
    TEST_ASSERT_EQUAL_UINT8(0x01, MockWire::transactions[1].data[1]);

    // With rows on bits 0-2 the sentinel is the next free bit; row 0 is selected
    // (low) and the other rows stay high.
    MockWire::reset();
    I2CBus::resetForTest();
    Mcp23017Module& m2 = make(0x0000, 2, 0x0007);
    TEST_ASSERT_EQUAL_UINT16(0x0008, m2.sentinelBit());
    m2.begin();
    TEST_ASSERT_EQUAL_UINT8(0x0E, MockWire::transactions[1].data[1]); // row 0 selected, others high
}

void test_timer_survives_millis_wrap()
{
    g_now = ~0UL - 2;
    Mcp23017Module& m = make(0x0001, 1);
    m.begin();
    g_now += 4; // 4 ms later, counter has wrapped
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::read_request_count);
    g_now += 1;
    queueSample(0xFE, 0xFF);
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::read_request_count);
    TEST_ASSERT_TRUE(m.getReading().has_value);
}

// --- Matrix ---

// Advances 2 ms (the matrix row interval) and scans once.
static void tick2(Mcp23017Module& m)
{
    g_now += 2;
    m.scan();
}

static Mcp23017Module& makeMatrix(uint16_t row_mask, uint16_t col_mask, uint8_t debounce = 1,
    uint16_t direct_mask = 0, uint8_t matrix_base = 100)
{
    return build(matrix_base, debounce, direct_mask, row_mask, col_mask);
}

void test_matrix_scans_one_row_every_2ms()
{
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100);
    m.begin();
    resetTransactions();
    g_now = 1;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::read_request_count);

    queueNext(m, 0xFF, 0xFF);
    g_now = 2;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(2, MockWire::transaction_count); // pointer write + next-row select
    TEST_ASSERT_EQUAL_UINT8(0x12, MockWire::transactions[0].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x14, MockWire::transactions[1].data[0]);
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::read_request_count);
    TEST_ASSERT_EQUAL_UINT8(4, MockWire::last_request_quantity);

    g_now = 3;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(2, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::read_request_count);

    queueNext(m, 0xFF, 0xFF);
    g_now = 4;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(4, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(2, MockWire::read_request_count);
}

void test_matrix_row_low_others_high()
{
    Mcp23017Module& m = makeMatrix(0x0007, 0x0100); // sentinel is bit 3
    m.begin();
    TEST_ASSERT_EQUAL_UINT8(0x0E, MockWire::transactions[1].data[1]); // begin leaves row 0 selected
    resetTransactions();
    // Each tick reads the selected row, then selects the next: 1, 2, 0, 1.
    const uint8_t expect_low[4] = { 0x0D, 0x0B, 0x0E, 0x0D };
    for (uint8_t i = 0; i < 4; i++) {
        queueNext(m, 0xFF, 0xFF);
        tick2(m);
        const MockWire::Transaction& t = MockWire::transactions[i * 2 + 1];
        TEST_ASSERT_EQUAL_UINT8(0x14, t.data[0]);
        TEST_ASSERT_EQUAL_UINT8(expect_low[i], t.data[1]);
        TEST_ASSERT_EQUAL_UINT8(0x00, t.data[2]);
        TEST_ASSERT_EQUAL_UINT8(0x08, t.data[1] & 0x08); // sentinel high
    }
}

void test_matrix_cell_pin_number()
{
    // Rows on bits 3 and 1 (set bits, in order: bit 1 = row 0, bit 3 = row 1);
    // columns on bits 9 and 12 (GPB1, GPB4).
    Mcp23017Module& m = makeMatrix(0x000A, 0x1200, 1, 0, 100);
    m.begin();
    queueNext(m, 0xFF, 0xFF); // row 0: nothing
    tick2(m);
    queueNext(m, 0xFF, 0xEF); // row 1: column 1 (bit 12 = GPB4) low
    tick2(m);
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(1, r.value);
    TEST_ASSERT_TRUE(r.type == ModuleType::MCP23017);
    TEST_ASSERT_EQUAL_UINT8(100 + 1 * 2 + 1, r.pin);
    TEST_ASSERT_FALSE(m.getReading().has_value);

    // Row 0, column 0 (bit 9 = GPB1).
    queueNext(m, 0xFF, 0xFD);
    tick2(m);
    queueNext(m, 0xFF, 0xEF);
    tick2(m);
    r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_UINT8(100, r.pin);
}

void test_matrix_uses_rows_on_gpa7_gpb7_with_cols_on_both_ports()
{
    // Rows GPA7 and GPB7; columns GPA1 and GPB1. Sentinel is bit 0.
    Mcp23017Module& m = makeMatrix(0x8080, 0x0202, 1, 0, 50);
    m.begin();
    TEST_ASSERT_EQUAL_UINT8(0x7F, MockWire::transactions[3].data[1]); // IODIR: GPA7 output
    TEST_ASSERT_EQUAL_UINT8(0x7F, MockWire::transactions[3].data[2]); // IODIR: GPB7 output
    TEST_ASSERT_EQUAL_UINT8(0x01, MockWire::transactions[1].data[1]); // begin selects row 0 (GPA7 low)
    TEST_ASSERT_EQUAL_UINT8(0x80, MockWire::transactions[1].data[2]);
    resetTransactions();

    queueNext(m, 0xFD, 0xFF); // row 0: GPA1 = column 0
    tick2(m);
    TEST_ASSERT_EQUAL_UINT8(0x81, MockWire::transactions[1].data[1]); // selects row 1 (GPB7 low)
    TEST_ASSERT_EQUAL_UINT8(0x00, MockWire::transactions[1].data[2]);
    queueNext(m, 0xFF, 0xFD); // row 1: GPB1 = column 1
    tick2(m);
    TEST_ASSERT_EQUAL_UINT8(0x01, MockWire::transactions[3].data[1]);
    TEST_ASSERT_EQUAL_UINT8(0x80, MockWire::transactions[3].data[2]);

    Reading a = m.getReading();
    Reading b = m.getReading();
    TEST_ASSERT_TRUE(a.has_value);
    TEST_ASSERT_TRUE(b.has_value);
    TEST_ASSERT_EQUAL_UINT8(50 + 0 * 2 + 0, a.pin);
    TEST_ASSERT_EQUAL_UINT8(50 + 1 * 2 + 1, b.pin);
    TEST_ASSERT_FALSE(m.getReading().has_value);
}

// Runs one full scan of a 2-row matrix. `pressed_row1` holds the column on row 1.
static void scanCycle(Mcp23017Module& m, bool pressed_row1, uint8_t gpioA = 0xFF)
{
    queueNext(m, gpioA, 0xFF);
    tick2(m);
    queueNext(m, gpioA, pressed_row1 ? 0xFE : 0xFF);
    tick2(m);
}

void test_matrix_debounce_counts_full_scans()
{
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100, 2);
    m.begin();
    scanCycle(m, true);
    TEST_ASSERT_FALSE(m.getReading().has_value); // one scan is not enough
    scanCycle(m, true);
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(1, r.value);
    TEST_ASSERT_EQUAL_UINT8(100 + 1 * 1 + 0, r.pin);

    scanCycle(m, false);
    TEST_ASSERT_FALSE(m.getReading().has_value);
    scanCycle(m, false);
    r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(0, r.value);
}

void test_direct_pins_debounce_once_per_cycle_on_mixed_chip()
{
    // Direct pin GPA4 held low throughout; debounce 2 counts scans, not samples.
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100, 2, 0x0010);
    m.begin();
    queueNext(m, 0xEF, 0xFF);
    tick2(m);
    queueNext(m, 0xEF, 0xFF);
    tick2(m); // two samples, but only one full scan
    TEST_ASSERT_FALSE(m.getReading().has_value);
    queueNext(m, 0xEF, 0xFF);
    tick2(m);
    TEST_ASSERT_FALSE(m.getReading().has_value);
    queueNext(m, 0xEF, 0xFF);
    tick2(m); // second full scan
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(1, r.value);
    TEST_ASSERT_EQUAL_UINT8(BASE + 4, r.pin);
}

void test_direct_and_matrix_readings_both_drain()
{
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100, 1, 0x0010);
    m.begin();
    scanCycle(m, true, 0xEF);
    Reading d = m.getReading();
    Reading x = m.getReading();
    TEST_ASSERT_TRUE(d.has_value);
    TEST_ASSERT_EQUAL_UINT8(BASE + 4, d.pin); // direct first
    TEST_ASSERT_TRUE(x.has_value);
    TEST_ASSERT_EQUAL_UINT8(100 + 1, x.pin);
    TEST_ASSERT_FALSE(m.getReading().has_value);
}

void test_matrix_failed_row_is_retried_without_events()
{
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100, 1);
    m.begin();
    resetTransactions();
    MockWire::scheduleNacks(ADDR, 1); // pointer write for the row 0 read fails
    tick2(m);
    TEST_ASSERT_EQUAL_UINT8(1, m.health().failures());
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::read_request_count);
    TEST_ASSERT_FALSE(m.getReading().has_value);

    // Row 0 is read again (it is still selected), not skipped.
    queueNext(m, 0xFF, 0xFE); // column pressed on row 0
    tick2(m);
    TEST_ASSERT_EQUAL_UINT8(0x12, MockWire::transactions[1].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x01, MockWire::transactions[2].data[1] & 0x03); // row 1 selected next
    TEST_ASSERT_EQUAL_UINT8(0, m.health().failures());
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_UINT8(100, r.pin);
}

void test_matrix_select_write_failure_retries_without_feeding_twice()
{
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100, 1);
    m.begin();
    resetTransactions();
    queueNext(m, 0xFF, 0xFE); // row 0 pressed
    MockWire::scheduleNacks(ADDR, 1, 2, 1); // pointer write passes, row-1 select NACKs
    tick2(m);
    TEST_ASSERT_EQUAL_UINT8(1, m.health().failures());
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_UINT8(100, r.pin);

    // Retry: every register is rewritten, nothing is read or fed, and the
    // partly successful init does not reset the failure count.
    uint8_t reads = MockWire::read_request_count;
    resetTransactions();
    tick2(m);
    TEST_ASSERT_EQUAL_UINT8(reads, MockWire::read_request_count);
    TEST_ASSERT_EQUAL_UINT8(4, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(0x0A, MockWire::transactions[0].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x0C, MockWire::transactions[2].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, MockWire::transactions[3].data[0]);
    TEST_ASSERT_EQUAL_UINT8(1, m.health().failures());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
    TEST_ASSERT_FALSE(m.getReading().has_value);

    // Row 0 is selected again and read next. Its key is still held: no event.
    queueNext(m, 0xFF, 0xFE);
    tick2(m);
    TEST_ASSERT_EQUAL_UINT8(0, m.health().failures());
    TEST_ASSERT_FALSE(m.getReading().has_value);
}

void test_failing_select_retries_reach_the_threshold()
{
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100, 1);
    m.begin();
    queueNext(m, 0xFF, 0xFF);
    // Select NACKs, then every init attempt gets as far as IOCON and fails.
    MockWire::scheduleNacks(ADDR, 3, 2, 1);
    tick2(m);
    tick2(m);
    TEST_ASSERT_EQUAL_UINT8(2, m.health().failures());
    tick2(m);
    TEST_ASSERT_TRUE(m.health().failed());
    TEST_ASSERT_EQUAL_UINT8(1, m.takeRuntimeError());
}

void test_reset_during_failed_select_is_not_hidden()
{
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100, 1);
    m.begin();
    queueNext(m, 0xFF, 0xFF);
    MockWire::scheduleNacks(ADDR, 1, 2, 1); // read passes, select NACKs: the chip is resetting
    tick2(m);
    simulateChipReset(); // OLAT, pull-ups and directions back to defaults

    resetTransactions();
    tick2(m); // a bare OLAT write here would hide the reset
    TEST_ASSERT_EQUAL_UINT8(4, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(0x0A, MockWire::transactions[0].data[0]); // IOCON
    TEST_ASSERT_EQUAL_UINT8(0x14, MockWire::transactions[1].data[0]); // OLAT
    TEST_ASSERT_EQUAL_UINT8(0x0C, MockWire::transactions[2].data[0]); // GPPU
    TEST_ASSERT_EQUAL_UINT8(0x00, MockWire::transactions[3].data[0]); // IODIR
    TEST_ASSERT_FALSE(m.getReading().has_value);

    queueNext(m, 0xFF, 0xFF);
    tick2(m);
    queueNext(m, 0xFF, 0xFF);
    tick2(m);
    TEST_ASSERT_FALSE(m.getReading().has_value);
}

void test_repeated_olat_mismatch_reaches_failed_state()
{
    Mcp23017Module& m = make(0x0003);
    m.begin();
    const uint8_t bad[4] = { 0xFF, 0xFF, 0x77, 0x77 }; // not what was written
    for (int i = 0; i < 2; i++) {
        MockWire::queueRead(ADDR, bad, 4);
        tick(m);
        TEST_ASSERT_EQUAL_UINT8(1, m.takeRuntimeError()); // a single mismatch: 1 then 2
        TEST_ASSERT_EQUAL_UINT8(2, m.takeRuntimeError());
        TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
        TEST_ASSERT_FALSE(m.health().failed());
    }
    resetTransactions();
    MockWire::queueRead(ADDR, bad, 4);
    tick(m);
    TEST_ASSERT_TRUE(m.health().failed());
    TEST_ASSERT_EQUAL_UINT8(1, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(1, m.activeError());
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transaction_count); // no reinit; just the sample pointer write

    // No polling while failed; the normal retry reinitializes and reports 2.
    resetTransactions();
    g_now += 500;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::transaction_count);
    g_now += 500;
    queueOlatReadback();
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(5, MockWire::transaction_count); // 4 writes, read-back pointer
    TEST_ASSERT_EQUAL_UINT8(2, m.takeRuntimeError());
}

void test_matrix_reset_detected_with_no_failed_transaction()
{
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100, 1);
    m.begin();
    queueNext(m, 0xFF, 0xFF);
    tick2(m);
    simulateChipReset(); // OLAT cleared, pull-ups gone, every transaction still ACKs
    resetTransactions();
    queueNext(m, 0x00, 0x00); // floating pins read low
    tick2(m);
    TEST_ASSERT_FALSE(m.getReading().has_value);
    TEST_ASSERT_EQUAL_UINT8(1, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(2, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
    // pointer write, then IOCON, OLAT, GPPU, IODIR rewritten.
    TEST_ASSERT_EQUAL_UINT8(5, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(0x0A, MockWire::transactions[1].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x0C, MockWire::transactions[3].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, MockWire::transactions[4].data[0]);

    // Scan restarts at row 0, which is selected again.
    queueNext(m, 0xFF, 0xFE);
    tick2(m);
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_UINT8(100, r.pin);
}

// Loses the chip for 3 polls, then lets the 1000 ms retry recover it.
static void matrixOutage(Mcp23017Module& m)
{
    MockWire::scheduleNacks(ADDR, 3);
    for (uint8_t i = 0; i < 3; i++) tick2(m);
    TEST_ASSERT_TRUE(m.health().failed());
    g_now += 1000;
    queueOlatReadback();
    m.scan();
    TEST_ASSERT_FALSE(m.health().failed());
}

void test_matrix_held_key_stays_held_through_outage()
{
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100, 1);
    m.begin();
    scanCycle(m, true);
    TEST_ASSERT_TRUE(m.getReading().has_value); // press
    TEST_ASSERT_FALSE(m.getReading().has_value);

    matrixOutage(m);
    TEST_ASSERT_FALSE(m.getReading().has_value);
    scanCycle(m, true); // still held
    TEST_ASSERT_FALSE(m.getReading().has_value); // no duplicate press
    scanCycle(m, false);
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(0, r.value);
    TEST_ASSERT_EQUAL_UINT8(101, r.pin);
}

void test_matrix_key_released_during_outage_reports_release()
{
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100, 1);
    m.begin();
    scanCycle(m, true);
    TEST_ASSERT_TRUE(m.getReading().has_value); // press

    matrixOutage(m);
    scanCycle(m, false); // released while the chip was gone
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(0, r.value);
    TEST_ASSERT_EQUAL_UINT8(101, r.pin);
    TEST_ASSERT_FALSE(m.getReading().has_value);
}

// --- I2CHealth ---

void test_health_threshold_and_recovery()
{
    I2CHealth h;
    TEST_ASSERT_FALSE(h.noteFailure(0));
    TEST_ASSERT_FALSE(h.noteFailure(1));
    TEST_ASSERT_FALSE(h.failed());
    TEST_ASSERT_TRUE(h.noteFailure(2));
    TEST_ASSERT_TRUE(h.failed());
    TEST_ASSERT_FALSE(h.noteFailure(3)); // already failed
    TEST_ASSERT_TRUE(h.noteSuccess());
    TEST_ASSERT_FALSE(h.failed());
    TEST_ASSERT_FALSE(h.noteSuccess());
}

void test_health_success_resets_count()
{
    I2CHealth h;
    h.noteFailure(0);
    h.noteFailure(0);
    h.noteSuccess();
    TEST_ASSERT_FALSE(h.noteFailure(0));
    TEST_ASSERT_FALSE(h.noteFailure(0));
    TEST_ASSERT_FALSE(h.failed());
}

void test_health_retry_every_1000ms_across_wrap()
{
    I2CHealth h;
    unsigned long t = ~0UL - 500;
    h.noteFailure(t);
    h.noteFailure(t);
    h.noteFailure(t);
    TEST_ASSERT_FALSE(h.retryDue(t + 999));
    TEST_ASSERT_TRUE(h.retryDue(t + 1000));
    h.noteFailure(t + 1000); // failed retry restarts the timer
    TEST_ASSERT_FALSE(h.retryDue(t + 1999));
    TEST_ASSERT_TRUE(h.retryDue(t + 2000));
    h.noteSuccess();
    TEST_ASSERT_FALSE(h.retryDue(t + 5000));
}

void test_health_error_queue()
{
    I2CHealth h;
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, h.take());
    h.push(1);
    h.push(2);
    TEST_ASSERT_EQUAL_UINT8(1, h.take());
    TEST_ASSERT_EQUAL_UINT8(2, h.take());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, h.take());
}

// --- Failure and reset handling ---

// Three failed polls: from g_now 0 the chip becomes failed at 15.
static void failChip(Mcp23017Module& m)
{
    MockWire::scheduleNacks(ADDR, 3);
    for (uint8_t i = 0; i < 3; i++) tick(m);
}

static void queueOlatZero()
{
    const uint8_t bytes[4] = { 0x00, 0x00, 0x00, 0x00 };
    MockWire::queueRead(ADDR, bytes, 4);
}

void test_nack_during_poll_produces_no_event()
{
    Mcp23017Module& m = make(0x0001, 1);
    m.begin();
    MockWire::scheduleNacks(ADDR, 1);
    queueSample(0xFE, 0xFF);
    tick(m);
    TEST_ASSERT_FALSE(m.getReading().has_value);
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
}

void test_held_button_stays_held_through_outage()
{
    Mcp23017Module& m = make(0x0001, 2);
    m.begin();
    for (uint8_t i = 0; i < 2; i++) { queueSample(0xFE, 0xFF); tick(m); }
    TEST_ASSERT_TRUE(m.getReading().has_value); // press reported

    failChip(m); // failed at 25
    TEST_ASSERT_FALSE(m.getReading().has_value);
    g_now = 1025;
    queueOlatReadback();
    m.scan(); // recovers
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.activeError());
    TEST_ASSERT_FALSE(m.getReading().has_value); // no phantom release

    queueSample(0xFE, 0xFF);
    tick(m);
    TEST_ASSERT_FALSE(m.getReading().has_value); // still held, no new press
    for (uint8_t i = 0; i < 2; i++) { queueSample(0xFF, 0xFF); tick(m); }
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(0, r.value); // release debounces normally
}

void test_three_failures_report_code_1_once()
{
    Mcp23017Module& m = make(0x0001, 1);
    m.begin();
    MockWire::scheduleNacks(ADDR, 2);
    tick(m);
    tick(m);
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
    MockWire::scheduleNacks(ADDR, 1);
    tick(m);
    TEST_ASSERT_EQUAL_UINT8(1, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());

    // A retry that fails does not report code 1 again.
    MockWire::scheduleNacks(ADDR, 1);
    g_now = 1015;
    m.scan();
    TEST_ASSERT_TRUE(m.health().failed());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
}

void test_failed_chip_stops_polling_and_retries_every_1000ms()
{
    Mcp23017Module& m = make(0x0001, 1);
    m.begin();
    failChip(m); // failed at 15
    resetTransactions();
    MockWire::read_request_count = 0;
    MockWire::scheduleNacks(ADDR, 1); // first retry fails
    for (g_now = 20; g_now < 1015; g_now += 5) m.scan();
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::read_request_count);

    g_now = 1015;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transaction_count); // IOCON NACKed
    g_now = 2014;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transaction_count);
    g_now = 2015;
    queueOlatReadback();
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(6, MockWire::transaction_count); // full reinit, read-back pointer
    TEST_ASSERT_FALSE(m.health().failed());
}

void test_recovery_reports_code_2_and_reinitializes()
{
    Mcp23017Module& m = make(0x0001, 1);
    m.begin();
    failChip(m);
    TEST_ASSERT_EQUAL_UINT8(1, m.takeRuntimeError());
    resetTransactions();
    g_now = 1015;
    queueOlatReadback();
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(5, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(0x0A, MockWire::transactions[0].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x14, MockWire::transactions[1].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x0C, MockWire::transactions[2].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, MockWire::transactions[3].data[0]);
    TEST_ASSERT_EQUAL_UINT8(2, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.activeError());
}

void test_change_during_outage_reported_after_recovery()
{
    Mcp23017Module& m = make(0x0001, 2);
    m.begin();
    failChip(m);
    // The button is pressed while the chip is gone; nothing is visible yet.
    g_now = 500;
    m.scan();
    TEST_ASSERT_FALSE(m.getReading().has_value);
    g_now = 1015;
    queueOlatReadback();
    m.scan(); // recovers
    TEST_ASSERT_FALSE(m.getReading().has_value);
    for (uint8_t i = 0; i < 2; i++) { queueSample(0xFE, 0xFF); tick(m); }
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(1, r.value);
    TEST_ASSERT_EQUAL_UINT8(BASE, r.pin);
}

void test_absent_at_boot_then_connected_recovers()
{
    MockWire::scheduleNacks(ADDR, 1);
    Mcp23017Module& m = make(0x0001, 1);
    TEST_ASSERT_FALSE(m.begin());
    TEST_ASSERT_EQUAL_UINT8(1, m.activeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError()); // no runtime code 1

    resetTransactions();
    for (g_now = 5; g_now < 1000; g_now += 5) {
        queueSample(0xFE, 0xFF); // chip not there yet: never read
        m.scan();
        TEST_ASSERT_FALSE(m.getReading().has_value);
    }
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::transaction_count);
    MockWire::read_request_count = 0;
    MockWire::read_queue_count = 0; // drop the samples that were never read

    g_now = 1000;
    queueOlatReadback();
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(5, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(2, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
    TEST_ASSERT_FALSE(m.getReading().has_value);
}

void test_olat_mismatch_discards_sample_and_reinitializes()
{
    Mcp23017Module& m = make(0x0001, 1);
    m.begin();
    resetTransactions();
    queueOlatZero(); // pins read low: would be a press if it were used
    tick(m);
    TEST_ASSERT_FALSE(m.getReading().has_value);
    TEST_ASSERT_EQUAL_UINT8(5, MockWire::transaction_count); // pointer write + 4 reinit
    TEST_ASSERT_EQUAL_UINT8(0x0A, MockWire::transactions[1].data[0]);

    queueSample(0xFE, 0xFF);
    tick(m);
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_INT16(1, r.value);
}

void test_olat_mismatch_reports_code_1_then_2()
{
    Mcp23017Module& m = make(0x0001, 1);
    m.begin();
    queueOlatZero();
    tick(m);
    TEST_ASSERT_EQUAL_UINT8(1, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(2, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.activeError());
}

void test_olat_mismatch_with_failed_reinit_reports_code_1_once_then_2()
{
    Mcp23017Module& m = make(0x0001, 1);
    m.begin();
    queueOlatZero();
    MockWire::scheduleNacks(ADDR, 1, 2, 1); // pointer write passes, reinit IOCON NACKs
    tick(m); // at 5
    TEST_ASSERT_FALSE(m.getReading().has_value);
    TEST_ASSERT_TRUE(m.health().failed());
    TEST_ASSERT_EQUAL_UINT8(1, m.activeError());
    TEST_ASSERT_EQUAL_UINT8(1, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());

    g_now = 1004;
    m.scan();
    TEST_ASSERT_TRUE(m.health().failed());
    g_now = 1005;
    queueOlatReadback();
    m.scan();
    TEST_ASSERT_FALSE(m.health().failed());
    TEST_ASSERT_EQUAL_UINT8(2, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
}

void test_active_error_is_1_while_failed()
{
    Mcp23017Module& m = make(0x0001, 1);
    m.begin();
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.activeError());
    failChip(m);
    TEST_ASSERT_EQUAL_UINT8(1, m.activeError());
    g_now = 1015;
    queueOlatReadback();
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.activeError());
}

void test_failed_read_after_good_olat_write_changes_nothing()
{
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100, 1);
    m.begin();
    resetTransactions();
    queueShort(); // the read of row 0 comes back short
    tick2(m);
    TEST_ASSERT_EQUAL_UINT8(1, m.health().failures());
    TEST_ASSERT_FALSE(m.getReading().has_value);
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());

    // Row 0 is retried (not skipped) and its press is seen normally.
    queueNext(m, 0xFF, 0xFE);
    tick2(m);
    TEST_ASSERT_EQUAL_UINT8(0x12, MockWire::transactions[0].data[0]); // short read, nothing selected
    TEST_ASSERT_EQUAL_UINT8(0x12, MockWire::transactions[1].data[0]); // same row read again
    TEST_ASSERT_EQUAL_UINT8(0x14, MockWire::transactions[2].data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x01, MockWire::transactions[2].data[1] & 0x03); // then row 1
    Reading r = m.getReading();
    TEST_ASSERT_TRUE(r.has_value);
    TEST_ASSERT_EQUAL_UINT8(100, r.pin);
}

// One matrix tick that keeps the mock's transaction log from filling up.
static void matrixTick(Mcp23017Module& m, uint8_t gpioA = 0xFF, uint8_t gpioB = 0xFF)
{
    resetTransactions();
    queueNext(m, gpioA, gpioB);
    tick2(m);
    syncChip();
}

// True when the tick just run wrote IOCON, GPPU and IODIR.
static bool rewroteRegisters()
{
    bool iocon = false, gppu = false, iodir = false;
    for (uint8_t i = 0; i < MockWire::transaction_count; i++) {
        const MockWire::Transaction& t = MockWire::transactions[i];
        if (t.length == 2 && t.data[0] == 0x0A) iocon = true;
        if (t.length == 3 && t.data[0] == 0x0C) gppu = true;
        if (t.length == 3 && t.data[0] == 0x00) iodir = true;
    }
    return iocon && gppu && iodir;
}

void test_matrix_periodic_rewrite_catches_reset_during_select()
{
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100, 3);
    m.begin();
    syncChip();
    const uint16_t iodir = g_iodir;
    const uint16_t gppu = g_gppu;

    // Reset right after a sample: the select write puts the sentinel back, so
    // OLAT no longer shows the reset but IODIR and GPPU stay at power-on values.
    g_reset_after_read = true;
    matrixTick(m);
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, g_iodir);
    TEST_ASSERT_EQUAL_HEX16(0, g_gppu);

    // Worst case: 8 scans of 2 rows, less the tick that took the reset.
    for (uint8_t i = 0; i < 15; i++) matrixTick(m);
    TEST_ASSERT_EQUAL_HEX16(iodir, g_iodir);
    TEST_ASSERT_EQUAL_HEX16(gppu, g_gppu);
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
    TEST_ASSERT_FALSE(m.health().failed());
}

void test_matrix_periodic_rewrite_keeps_input_state()
{
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100, 2, 0x0010);
    m.begin();
    // Keys in both rows and the direct pin stay held through two rewrites.
    uint8_t rewrites = 0;
    for (uint8_t i = 0; i < 32; i++) {
        matrixTick(m, 0xEF, 0xFE);
        if (rewroteRegisters()) rewrites++;
        if (i == 7) {
            for (uint8_t n = 0; n < 3; n++) TEST_ASSERT_TRUE(m.getReading().has_value);
            TEST_ASSERT_FALSE(m.getReading().has_value);
        }
    }
    TEST_ASSERT_EQUAL_UINT8(2, rewrites);
    TEST_ASSERT_FALSE(m.getReading().has_value);
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
    // The scan restarts at row 0, with row 1 idle high and the sentinel set.
    TEST_ASSERT_EQUAL_HEX16(0x0006, g_olat);
}

void test_matrix_periodic_rewrite_failure_takes_failure_path()
{
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100, 1);
    m.begin();
    for (uint8_t i = 0; i < 15; i++) matrixTick(m);

    MockWire::scheduleNacks(ADDR, 1, 2, 1); // pointer write passes, IOCON NACKs
    matrixTick(m);
    TEST_ASSERT_EQUAL_UINT8(1, m.health().failures());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());

    // The next tick rewrites everything and does not sample.
    resetTransactions();
    MockWire::read_request_count = 0;
    tick2(m);
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::read_request_count);
    TEST_ASSERT_TRUE(rewroteRegisters());
    matrixTick(m);
    TEST_ASSERT_EQUAL_UINT8(0, m.health().failures());

    // Repeated failures reach the threshold and report code 1 once.
    for (uint8_t i = 0; i < 14; i++) matrixTick(m);
    MockWire::scheduleNacks(ADDR, 1, 2, 1);
    matrixTick(m);
    MockWire::scheduleNacks(ADDR, 2);
    tick2(m);
    tick2(m);
    TEST_ASSERT_TRUE(m.health().failed());
    TEST_ASSERT_EQUAL_UINT8(1, m.takeRuntimeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());
}

// What a wrong device answers when recovery reads OLAT back.
static void queueOlatBytes(uint8_t low, uint8_t high)
{
    const uint8_t bytes[2] = { low, high };
    MockWire::queueRead(ADDR, bytes, 2);
}

void test_recovery_with_wrong_olat_stays_failed()
{
    Mcp23017Module& m = make(0x0001, 1);
    m.begin();
    failChip(m);
    TEST_ASSERT_EQUAL_UINT8(1, m.takeRuntimeError());

    g_now = 1015;
    queueOlatBytes(0xFF, 0xFF); // acknowledges everything, keeps nothing
    m.scan();
    TEST_ASSERT_TRUE(m.health().failed());
    TEST_ASSERT_EQUAL_UINT8(1, m.activeError());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());

    // Still no flapping a second later, and no read-back at all fails too.
    g_now = 2015;
    m.scan();
    TEST_ASSERT_TRUE(m.health().failed());
    TEST_ASSERT_EQUAL_UINT8(MODULE_ERROR_NONE, m.takeRuntimeError());

    // The right chip comes back.
    g_now = 3015;
    queueOlatBytes(0x01, 0x00);
    m.scan();
    TEST_ASSERT_FALSE(m.health().failed());
    TEST_ASSERT_EQUAL_UINT8(2, m.takeRuntimeError());
}

void test_recovery_reads_olat_back_after_the_writes()
{
    Mcp23017Module& m = make(0x0001, 1);
    m.begin();
    failChip(m);
    resetTransactions();
    MockWire::read_request_count = 0;
    g_now = 1015;
    queueOlatBytes(0x01, 0x00);
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(5, MockWire::transaction_count);
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::transactions[4].length);
    TEST_ASSERT_EQUAL_HEX8(0x14, MockWire::transactions[4].data[0]);
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::read_request_count);
    TEST_ASSERT_EQUAL_UINT8(2, MockWire::last_request_quantity);
}

void test_first_sample_offset_delays_a_direct_chip()
{
    Mcp23017Module& m = make();
    m.begin();
    m.setFirstSampleOffset(3);
    g_now = 7;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::read_request_count);
    g_now = 8;
    queueSample(0xFF, 0xFF);
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::read_request_count);
}

void test_first_sample_offset_delays_a_matrix_chip()
{
    Mcp23017Module& m = makeMatrix(0x0003, 0x0100, 1);
    m.begin();
    m.setFirstSampleOffset(5); // longer than the 2 ms row interval
    g_now = 6;
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::read_request_count);
    g_now = 7;
    queueNext(m, 0xFF, 0xFF);
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::read_request_count);
}

void test_first_sample_offset_survives_millis_wrap()
{
    g_now = (unsigned long)0 - 2;
    Mcp23017Module& m = make();
    m.begin();
    m.setFirstSampleOffset(1);
    g_now = 2; // 4 ms later
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(0, MockWire::read_request_count);
    g_now = 4; // 6 ms later
    queueSample(0xFF, 0xFF);
    m.scan();
    TEST_ASSERT_EQUAL_UINT8(1, MockWire::read_request_count);
}

int main(int, char**)
{
    UNITY_BEGIN();
    RUN_TEST(test_begin_writes_expected_registers);
    RUN_TEST(test_begin_nack_returns_false);
    RUN_TEST(test_no_read_before_5ms);
    RUN_TEST(test_reads_four_bytes_from_gpioa);
    RUN_TEST(test_press_reported_after_debounce_samples);
    RUN_TEST(test_release_reported);
    RUN_TEST(test_two_pins_change_two_readings);
    RUN_TEST(test_held_at_boot_reports_press);
    RUN_TEST(test_register_reinit_keeps_held_state);
    RUN_TEST(test_pins_outside_mask_ignored);
    RUN_TEST(test_short_read_counts_as_failure);
    RUN_TEST(test_pointer_write_nack_counts_as_failure);
    RUN_TEST(test_sentinel_exists_for_direct_only_chip);
    RUN_TEST(test_timer_survives_millis_wrap);
    RUN_TEST(test_matrix_scans_one_row_every_2ms);
    RUN_TEST(test_matrix_row_low_others_high);
    RUN_TEST(test_matrix_cell_pin_number);
    RUN_TEST(test_matrix_uses_rows_on_gpa7_gpb7_with_cols_on_both_ports);
    RUN_TEST(test_matrix_debounce_counts_full_scans);
    RUN_TEST(test_direct_pins_debounce_once_per_cycle_on_mixed_chip);
    RUN_TEST(test_direct_and_matrix_readings_both_drain);
    RUN_TEST(test_matrix_failed_row_is_retried_without_events);
    RUN_TEST(test_matrix_select_write_failure_retries_without_feeding_twice);
    RUN_TEST(test_failing_select_retries_reach_the_threshold);
    RUN_TEST(test_reset_during_failed_select_is_not_hidden);
    RUN_TEST(test_repeated_olat_mismatch_reaches_failed_state);
    RUN_TEST(test_matrix_reset_detected_with_no_failed_transaction);
    RUN_TEST(test_matrix_held_key_stays_held_through_outage);
    RUN_TEST(test_matrix_key_released_during_outage_reports_release);
    RUN_TEST(test_health_threshold_and_recovery);
    RUN_TEST(test_health_success_resets_count);
    RUN_TEST(test_health_retry_every_1000ms_across_wrap);
    RUN_TEST(test_health_error_queue);
    RUN_TEST(test_nack_during_poll_produces_no_event);
    RUN_TEST(test_held_button_stays_held_through_outage);
    RUN_TEST(test_three_failures_report_code_1_once);
    RUN_TEST(test_failed_chip_stops_polling_and_retries_every_1000ms);
    RUN_TEST(test_recovery_reports_code_2_and_reinitializes);
    RUN_TEST(test_change_during_outage_reported_after_recovery);
    RUN_TEST(test_absent_at_boot_then_connected_recovers);
    RUN_TEST(test_olat_mismatch_discards_sample_and_reinitializes);
    RUN_TEST(test_olat_mismatch_reports_code_1_then_2);
    RUN_TEST(test_olat_mismatch_with_failed_reinit_reports_code_1_once_then_2);
    RUN_TEST(test_active_error_is_1_while_failed);
    RUN_TEST(test_failed_read_after_good_olat_write_changes_nothing);
    RUN_TEST(test_matrix_periodic_rewrite_catches_reset_during_select);
    RUN_TEST(test_matrix_periodic_rewrite_keeps_input_state);
    RUN_TEST(test_matrix_periodic_rewrite_failure_takes_failure_path);
    RUN_TEST(test_recovery_with_wrong_olat_stays_failed);
    RUN_TEST(test_recovery_reads_olat_back_after_the_writes);
    RUN_TEST(test_first_sample_offset_delays_a_direct_chip);
    RUN_TEST(test_first_sample_offset_delays_a_matrix_chip);
    RUN_TEST(test_first_sample_offset_survives_millis_wrap);
    return UNITY_END();
}
