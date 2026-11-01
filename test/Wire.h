// Mock Wire.h for native testing (header-only)
//
// Records every I2C transaction so tests can assert byte-level correctness.
// Per-address scheduled NACKs let tests simulate init failure, transient
// write failures, and recovery.
//
// Usage in tests:
//   MockWire::reset();                       // clear state
//   MockWire::scheduleNacks(0x70, 1);        // next endTransmission to 0x70 NACKs
//   // ... run code under test ...
//   MockWire::transactions[i].address;       // inspect what was sent
//   MockWire::transactions[i].data[j];
//   MockWire::queueRead(0x20, bytes, 2);     // next requestFrom(0x20, n) returns these bytes
//   MockWire::read_request_count;            // number of requestFrom calls
//   MockWire::last_request_quantity;         // byte count of the latest requestFrom
//
// IMPORTANT — single-TU constraint:
// All mock state (transactions, nack_table, Wire instance) is declared `static`,
// which means each translation unit that includes this header gets its own private
// copy. This is a C++11 header-only limitation. Therefore this header MUST be
// included by exactly one test translation unit per test binary. If a future test
// suite also needs Wire mock support, either (a) merge it into the same binary so
// only one TU includes Wire.h, or (b) refactor the mock into a .cpp/.h pair with
// a single definition and extern declarations.
#pragma once

#include <stdint.h>
#include <stddef.h>

namespace MockWire {

constexpr uint8_t MAX_TRANSACTIONS = 32;
constexpr uint8_t MAX_BYTES_PER_TRANSACTION = 32;
constexpr uint8_t MAX_ADDRESSES = 8;

struct Transaction {
    uint8_t address;
    uint8_t data[MAX_BYTES_PER_TRANSACTION];
    uint8_t length;
    uint8_t end_status;
};

struct AddressNackBehavior {
    uint8_t address;
    uint8_t nack_count_remaining;
    uint8_t nack_status;
};

constexpr uint8_t MAX_READ_QUEUES = 8;
constexpr uint8_t MAX_READ_BYTES = 32;

struct ReadQueue {
    uint8_t address;
    uint8_t data[MAX_READ_BYTES];
    uint8_t length;
};

static Transaction transactions[MAX_TRANSACTIONS];
static uint8_t transaction_count = 0;
static AddressNackBehavior nack_table[MAX_ADDRESSES];
static uint8_t nack_table_count = 0;
static bool wire_begin_called = false;
static unsigned long wire_timeout_us = 0;
static bool wire_timeout_reset_on_timeout = false;
static uint8_t wire_timeout_call_count = 0;

static ReadQueue read_queues[MAX_READ_QUEUES];
static uint8_t read_queue_count = 0;
static uint8_t read_request_count = 0;
static uint8_t last_request_quantity = 0;
static uint8_t rx_buffer[MAX_READ_BYTES];
static uint8_t rx_length = 0;
static uint8_t rx_pos = 0;

static uint8_t current_address = 0;
static uint8_t current_buffer[MAX_BYTES_PER_TRANSACTION];
static uint8_t current_length = 0;
static bool transaction_active = false;

static inline void reset() {
    transaction_count = 0;
    nack_table_count = 0;
    wire_begin_called = false;
    wire_timeout_us = 0;
    wire_timeout_reset_on_timeout = false;
    wire_timeout_call_count = 0;
    transaction_active = false;
    current_length = 0;
    read_queue_count = 0;
    read_request_count = 0;
    last_request_quantity = 0;
    rx_length = 0;
    rx_pos = 0;
}

// Scripts the bytes returned by the next requestFrom() to `address`. Queued
// reads for one address are served in the order they were queued.
static inline void queueRead(uint8_t address, const uint8_t* bytes, uint8_t length) {
    if (read_queue_count >= MAX_READ_QUEUES) return;
    ReadQueue& q = read_queues[read_queue_count++];
    q.address = address;
    q.length = length > MAX_READ_BYTES ? MAX_READ_BYTES : length;
    for (uint8_t i = 0; i < q.length; i++) q.data[i] = bytes[i];
}

static inline void scheduleNacks(uint8_t address, uint8_t count, uint8_t status = 2) {
    if (nack_table_count >= MAX_ADDRESSES) return;
    nack_table[nack_table_count++] = { address, count, status };
}

static uint8_t consumeNackStatus(uint8_t address) {
    for (uint8_t i = 0; i < nack_table_count; i++) {
        if (nack_table[i].address == address && nack_table[i].nack_count_remaining > 0) {
            nack_table[i].nack_count_remaining--;
            return nack_table[i].nack_status;
        }
    }
    return 0;
}

static bool wireBeginCalled() { return wire_begin_called; }

} // namespace MockWire

class TwoWire {
public:
    void begin() {
        MockWire::wire_begin_called = true;
    }

    void setWireTimeout(unsigned long timeout, bool reset_on_timeout) {
        MockWire::wire_timeout_us = timeout;
        MockWire::wire_timeout_reset_on_timeout = reset_on_timeout;
        MockWire::wire_timeout_call_count++;
    }

    void beginTransmission(uint8_t address) {
        MockWire::current_address = address;
        MockWire::current_length = 0;
        MockWire::transaction_active = true;
    }

    size_t write(uint8_t byte) {
        if (!MockWire::transaction_active) return 0;
        if (MockWire::current_length >= MockWire::MAX_BYTES_PER_TRANSACTION) return 0;
        MockWire::current_buffer[MockWire::current_length++] = byte;
        return 1;
    }

    size_t write(const uint8_t* data, size_t length) {
        size_t written = 0;
        for (size_t i = 0; i < length; i++) {
            if (write(data[i]) == 0) break;
            written++;
        }
        return written;
    }

    uint8_t endTransmission() {
        return endTransmission(true);
    }

    uint8_t endTransmission(bool /*stop*/) {
        if (!MockWire::transaction_active) return 4;
        MockWire::transaction_active = false;

        uint8_t status = MockWire::consumeNackStatus(MockWire::current_address);

        if (MockWire::transaction_count < MockWire::MAX_TRANSACTIONS) {
            MockWire::Transaction& t = MockWire::transactions[MockWire::transaction_count++];
            t.address = MockWire::current_address;
            t.length = MockWire::current_length;
            t.end_status = status;
            for (uint8_t i = 0; i < MockWire::current_length; i++) {
                t.data[i] = MockWire::current_buffer[i];
            }
        }

        return status;
    }

    uint8_t requestFrom(uint8_t address, uint8_t quantity) {
        MockWire::read_request_count++;
        MockWire::last_request_quantity = quantity;
        MockWire::rx_length = 0;
        MockWire::rx_pos = 0;
        for (uint8_t i = 0; i < MockWire::read_queue_count; i++) {
            if (MockWire::read_queues[i].address != address) continue;
            MockWire::ReadQueue& q = MockWire::read_queues[i];
            uint8_t n = q.length < quantity ? q.length : quantity;
            for (uint8_t j = 0; j < n; j++) MockWire::rx_buffer[j] = q.data[j];
            MockWire::rx_length = n;
            // Consume this entry.
            for (uint8_t k = i; k + 1 < MockWire::read_queue_count; k++) {
                MockWire::read_queues[k] = MockWire::read_queues[k + 1];
            }
            MockWire::read_queue_count--;
            break;
        }
        return MockWire::rx_length;
    }
    int available() { return MockWire::rx_length - MockWire::rx_pos; }
    int read() {
        if (MockWire::rx_pos >= MockWire::rx_length) return -1;
        return MockWire::rx_buffer[MockWire::rx_pos++];
    }
};

static TwoWire Wire;
