#pragma once

#include "module.h"
#include <stdint.h>

namespace Modules {

// Consecutive-failure tracking, retry timing and a small queue of runtime
// error codes, shared by the I2C input modules. All timer comparisons use
// unsigned subtraction so they survive millis() wrap.
class I2CHealth {
public:
    static constexpr uint8_t FAILURE_THRESHOLD = 3;
    static constexpr unsigned long RETRY_INTERVAL_MS = 1000;
    static constexpr uint8_t QUEUE_SIZE = 4;

    // Records a failed transaction. Returns true exactly when this failure
    // reaches the threshold and the module becomes failed. While already
    // failed it only restarts the retry timer.
    bool noteFailure(unsigned long now)
    {
        if (failed_) {
            last_attempt_ = now;
            return false;
        }
        if (failures_ < 255) failures_++;
        if (failures_ < FAILURE_THRESHOLD) return false;
        failed_ = true;
        last_attempt_ = now;
        return true;
    }

    // Records a good transaction. Returns true if this ended a failed state.
    bool noteSuccess()
    {
        failures_ = 0;
        bool was_failed = failed_;
        failed_ = false;
        return was_failed;
    }

    // Enters the failed state at once (a failed begin()); no code is queued.
    void markFailed(unsigned long now)
    {
        failures_ = FAILURE_THRESHOLD;
        failed_ = true;
        last_attempt_ = now;
    }

    bool failed() const { return failed_; }

    // True once per RETRY_INTERVAL_MS while failed.
    bool retryDue(unsigned long now) const
    {
        return failed_ && (now - last_attempt_) >= RETRY_INTERVAL_MS;
    }

    // Queues an error code; the oldest is dropped if the queue is full.
    void push(uint8_t code)
    {
        if (count_ == QUEUE_SIZE) {
            head_ = (head_ + 1) % QUEUE_SIZE;
            count_--;
        }
        queue_[(head_ + count_) % QUEUE_SIZE] = code;
        count_++;
    }

    // Pops the oldest code, or MODULE_ERROR_NONE.
    uint8_t take()
    {
        if (count_ == 0) return MODULE_ERROR_NONE;
        uint8_t code = queue_[head_];
        head_ = (head_ + 1) % QUEUE_SIZE;
        count_--;
        return code;
    }

    uint8_t failures() const { return failures_; }

private:
    uint8_t failures_ = 0;
    bool failed_ = false;
    unsigned long last_attempt_ = 0;
    uint8_t queue_[QUEUE_SIZE] = { 0 };
    uint8_t head_ = 0;
    uint8_t count_ = 0;
};

} // namespace Modules
