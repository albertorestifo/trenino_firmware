#pragma once
#include <stddef.h>
#include <stdint.h>

// Hardware units only. No simulator, train, dial or angular calibration belongs here.
namespace Stepper {
constexpr uint8_t COMMAND_TYPE = 16, STATUS_TYPE = 17, REVISION = 1, MAX_AXES = 2;
enum Operation : uint8_t { CONFIGURE = 0, MOVE_TO = 1, HOME = 2, STOP = 3,
    ENABLE = 4, RELEASE = 5, STATUS_QUERY = 6, KEEP_ALIVE = 7, RESET = 8, REFERENCE = 9, FORGET_DRIVER = 10 };
enum Result : uint8_t { OK = 0, INVALID = 1, BAD_AXIS = 2, UNSUPPORTED = 3,
    CONFLICT = 4, UNCONFIGURED = 5, UNREFERENCED = 6, LIMIT = 7, BUSY = 8, IO_ERROR = 9 };
enum State : uint8_t { AXIS_DISABLED = 0, AXIS_IDLE = 1, AXIS_MOVING = 2, AXIS_HOMING = 3, AXIS_FAULT = 4 };
enum Fault : uint8_t { NO_FAULT = 0, DRIVER_ERROR = 1, HOME_TIMEOUT = 2, HOST_LOST = 3 };
struct Config {
    uint8_t driver, mode, address, port, reverse, hold, loss_policy, home_strategy;
    int32_t minimum, maximum;
    uint16_t max_speed, acceleration;
    int8_t home_direction;
    uint16_t home_steps, home_speed, home_timeout_ms;
    int32_t home_position;
    uint16_t watchdog_ms, home_backoff;
};
struct Command {
    uint8_t operation, axis;
    uint16_t request_id;
    Config config;
    int32_t position;
    bool decode(const uint8_t* bytes, size_t length);
};
struct Axis {
    Config config;
    int32_t position, target;
    uint32_t last_step, home_started, speed_squared;
    uint16_t home_remaining;
    int8_t direction;
    uint8_t phase;
    uint8_t reference_kind;
    State state;
    Fault fault;
    bool configured, known, enabled, backing_off;
};
// Future drivers implement this small electrical interface. begin must not reset
// another active port. Every method reports acknowledged writes, never motion feedback.
class Driver {
public:
    virtual ~Driver() {}
    virtual bool begin(uint8_t address) = 0;
    virtual uint8_t phaseCount(uint8_t mode) const = 0;
    virtual bool write(uint8_t address, uint8_t port, uint8_t mode, uint8_t phase) = 0;
    virtual bool release(uint8_t address, uint8_t port) = 0;
};
class Controller {
public:
    explicit Controller(Driver& driver);
    bool reset();
    Result configure(uint8_t id, const Config& config, uint32_t now);
    Result moveTo(uint8_t id, int32_t position, uint32_t now);
    Result home(uint8_t id, uint32_t now);
    Result stop(uint8_t id);
    Result enable(uint8_t id);
    Result release(uint8_t id);
    Result reference(uint8_t id, int32_t position, uint32_t now);
    void keepAlive(uint32_t now) { last_contact_ = now; }
    void update(uint32_t now);
    const Axis& axis(uint8_t id) const { return axes_[id]; }
private:
    Driver& driver_;
    Axis axes_[MAX_AXES];
    uint32_t last_contact_;
    Result check(uint8_t id) const;
    void fail(Axis& axis, Fault reason, bool release_output = true);
    bool energize(Axis& axis);
    void idle(Axis& axis);
};
// Status is exactly 23 bytes. 'result' acknowledges acceptance; state/flags
// describe completion. Asynchronous notifications use operation=STATUS_QUERY, id=0.
size_t encodeStatus(uint8_t* bytes, size_t capacity, uint8_t axis, uint8_t operation,
    uint16_t request_id, Result result, const Axis* value);
} // namespace Stepper
