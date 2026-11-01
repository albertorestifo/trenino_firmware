#pragma once

#include <stdint.h>

namespace Modules {

// Hardware-independent matrix debounce and event queue.
// Callers feed raw samples; debounced edges are queued for pop().
class MatrixCore {
public:
    static constexpr uint8_t MAX_ROWS = 8;
    static constexpr uint8_t MAX_COLS = 8;
    static constexpr uint8_t EVENT_QUEUE_SIZE = 8;

    MatrixCore(uint8_t rows, uint8_t cols, uint8_t debounce_threshold);

    // Clear all debounce state and pending events
    void reset();

    // Feed one raw sample for a button
    void sample(uint8_t row, uint8_t col, bool pressed);

    // Pop the oldest pending edge. button_index = row * cols + col.
    // Marks the edge as reported. Returns false if the queue is empty.
    bool pop(uint8_t& button_index, bool& pressed);

private:
    struct PendingEvent {
        uint8_t button_index;
        bool pressed;
    };

    void enqueue(uint8_t button_index, bool pressed);

    uint8_t num_rows;
    uint8_t num_cols;
    uint8_t debounce_threshold;

    uint8_t current_state[MAX_ROWS];   // Debounced state, one bit per column
    uint8_t last_reported[MAX_ROWS];   // Last reported state, one bit per column
    uint8_t debounce_count[MAX_ROWS * MAX_COLS];

    PendingEvent event_queue[EVENT_QUEUE_SIZE];
    uint8_t queue_head;
    uint8_t queue_tail;
};

} // namespace Modules
