#include "matrix_core.h"
#include <string.h>

namespace Modules {

MatrixCore::MatrixCore(uint8_t rows, uint8_t cols, uint8_t threshold)
    : num_rows(rows < MAX_ROWS ? rows : MAX_ROWS)
    , num_cols(cols < MAX_COLS ? cols : MAX_COLS)
    , debounce_threshold(threshold)
{
    reset();
}

void MatrixCore::reset()
{
    memset(current_state, 0, sizeof(current_state));
    memset(last_reported, 0, sizeof(last_reported));
    memset(debounce_count, 0, sizeof(debounce_count));
    queue_head = 0;
    queue_tail = 0;
}

void MatrixCore::sample(uint8_t row, uint8_t col, bool pressed)
{
    if (row >= num_rows || col >= num_cols) {
        return;
    }

    uint8_t idx = row * num_cols + col;
    uint8_t mask = 1 << col;
    bool current = (current_state[row] & mask) != 0;

    // Counter-based debounce algorithm
    if (pressed == current) {
        debounce_count[idx] = 0;
        return;
    }

    debounce_count[idx]++;
    if (debounce_count[idx] >= debounce_threshold) {
        // Stable new state detected
        if (pressed) {
            current_state[row] |= mask;
        } else {
            current_state[row] &= ~mask;
        }
        debounce_count[idx] = 0;

        // Queue only if it differs from the last reported state
        bool reported = (last_reported[row] & mask) != 0;
        if (pressed != reported) {
            enqueue(idx, pressed);
        }
    }
}

bool MatrixCore::pop(uint8_t& button_index, bool& pressed)
{
    if (queue_head == queue_tail) {
        return false;
    }

    const PendingEvent& event = event_queue[queue_head];
    queue_head = (queue_head + 1) % EVENT_QUEUE_SIZE;

    button_index = event.button_index;
    pressed = event.pressed;

    uint8_t row = button_index / num_cols;
    uint8_t mask = 1 << (button_index % num_cols);
    if (pressed) {
        last_reported[row] |= mask;
    } else {
        last_reported[row] &= ~mask;
    }
    return true;
}

void MatrixCore::enqueue(uint8_t button_index, bool pressed)
{
    uint8_t next_tail = (queue_tail + 1) % EVENT_QUEUE_SIZE;

    // Queue full, drop oldest event
    if (next_tail == queue_head) {
        queue_head = (queue_head + 1) % EVENT_QUEUE_SIZE;
    }

    event_queue[queue_tail].button_index = button_index;
    event_queue[queue_tail].pressed = pressed;
    queue_tail = next_tail;
}

} // namespace Modules
