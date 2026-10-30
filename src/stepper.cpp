#include "stepper.h"
#include <math.h>
#include <string.h>

namespace Stepper {
namespace {
uint16_t read16(const uint8_t* p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
int32_t read32(const uint8_t* p) {
    uint32_t u = uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
    // Avoid implementation-defined unsigned-to-signed conversion on native compilers.
    return u <= 0x7fffffffUL ? int32_t(u) : -1 - int32_t(0xffffffffUL - u);
}
void put32(uint8_t* p, int32_t value) {
    uint32_t u = uint32_t(value);
    for (uint8_t i = 0; i < 4; ++i) p[i] = uint8_t(u >> (8 * i));
}
uint32_t smaller(uint32_t a, uint32_t b) { return a < b ? a : b; }
}
bool Command::decode(const uint8_t* b, size_t n) {
    if (!b || n < 6 || b[0] != COMMAND_TYPE || b[1] != REVISION || b[2] > FORGET_DRIVER) return false;
    operation = b[2]; axis = b[3]; request_id = read16(b + 4);
    size_t expected = operation == CONFIGURE ? 41 : ((operation == MOVE_TO || operation == REFERENCE) ? 10 : 6);
    if (n != expected) return false;
    if (operation == MOVE_TO || operation == REFERENCE) position = read32(b + 6);
    if (operation == CONFIGURE) {
        config.driver = b[6]; config.mode = b[7]; config.address = b[8]; config.port = b[9];
        config.reverse = b[10]; config.hold = b[11]; config.loss_policy = b[12]; config.home_strategy = b[13];
        config.minimum = read32(b + 14); config.maximum = read32(b + 18);
        config.max_speed = read16(b + 22); config.acceleration = read16(b + 24);
        config.home_direction = b[26] == 255 ? -1 : int8_t(b[26]);
        config.home_steps = read16(b + 27); config.home_speed = read16(b + 29);
        config.home_timeout_ms = read16(b + 31); config.home_position = read32(b + 33);
        config.watchdog_ms = read16(b + 37);
        config.home_backoff = read16(b + 39);
    }
    return true;
}
Controller::Controller(Driver& driver) : driver_(driver), last_contact_(0) { memset(axes_, 0, sizeof(axes_)); }
Result Controller::check(uint8_t id) const {
    if (id >= MAX_AXES) return BAD_AXIS;
    return axes_[id].configured ? OK : UNCONFIGURED;
}
bool Controller::reset() {
    bool ok = true;
    for (uint8_t i = 0; i < MAX_AXES; ++i) {
        if (axes_[i].configured && !driver_.release(axes_[i].config.address, axes_[i].config.port)) ok = false;
    }
    memset(axes_, 0, sizeof(axes_));
    return ok;
}
Result Controller::configure(uint8_t id, const Config& c, uint32_t now) {
    if (id >= MAX_AXES) return BAD_AXIS;
    if (c.driver != 1 || !driver_.phaseCount(c.mode) || c.home_strategy > 1) return UNSUPPORTED;
    if (c.address < 0x60 || c.address > 0x77 || c.port < 1 || c.port > 2 || c.reverse > 1 ||
        c.hold > 1 || c.loss_policy > 1 || c.minimum < -1000000 || c.maximum > 1000000 ||
        c.minimum >= c.maximum || !c.max_speed || c.max_speed > 200 || !c.acceleration ||
        c.acceleration > 10000 || c.watchdog_ms < 100 || c.watchdog_ms > 60000 ||
        c.home_position < c.minimum || c.home_position > c.maximum ||
        (c.home_strategy == 1 && ((c.home_direction != -1 && c.home_direction != 1) ||
        !c.home_steps || c.home_backoff >= c.home_steps || !c.home_speed || c.home_speed > c.max_speed ||
        !c.home_timeout_ms || c.home_timeout_ms > 60000))) return INVALID;
    for (uint8_t i = 0; i < MAX_AXES; ++i) {
        if (i != id && axes_[i].configured && (axes_[i].config.address != c.address || axes_[i].config.port == c.port)) return CONFLICT;
    }
    Axis& a = axes_[id];
    if (a.configured && !driver_.release(a.config.address, a.config.port)) { fail(a, DRIVER_ERROR); return IO_ERROR; }
    memset(&a, 0, sizeof(a)); a.config = c; a.configured = true;
    if (!driver_.begin(c.address) || !driver_.release(c.address, c.port)) { fail(a, DRIVER_ERROR); return IO_ERROR; }
    a.last_step = now; last_contact_ = now;
    return OK;
}
void Controller::fail(Axis& a, Fault reason, bool release_output) {
    if (release_output) driver_.release(a.config.address, a.config.port);
    a.known = false; a.reference_kind = 0; a.enabled = !release_output && a.enabled; a.state = FAULT; a.fault = reason;
    a.speed_squared = 0; a.home_remaining = 0;
    if (reason == DRIVER_ERROR) {
        for (uint8_t i = 0; i < MAX_AXES; ++i) {
            Axis& other = axes_[i];
            if (&other != &a && other.configured && other.config.address == a.config.address) {
                driver_.release(other.config.address, other.config.port);
                other.known = false; other.reference_kind = 0; other.enabled = false;
                other.state = FAULT; other.fault = DRIVER_ERROR; other.speed_squared = 0; other.home_remaining = 0;
            }
        }
    }
}
bool Controller::energize(Axis& a) {
    if (!driver_.write(a.config.address, a.config.port, a.config.mode, a.phase)) { fail(a, DRIVER_ERROR); return false; }
    a.enabled = true; return true;
}
Result Controller::enable(uint8_t id) {
    Result r = check(id); if (r != OK) return r;
    Axis& a = axes_[id];
    if (a.state == FAULT) return IO_ERROR; // reconfigure acknowledges and clears fault
    if (a.enabled) return OK;
    if (!energize(a)) return IO_ERROR;
    a.state = IDLE; return OK;
}
Result Controller::reference(uint8_t id, int32_t p, uint32_t now) {
    Result r = check(id); if (r != OK) return r;
    Axis& a = axes_[id];
    if (a.config.home_strategy != 0) return UNSUPPORTED;
    if (a.state == MOVING || a.state == HOMING) return BUSY;
    if (p < a.config.minimum || p > a.config.maximum) return LIMIT;
    if ((r = enable(id)) != OK) return r;
    a.position = a.target = p; a.known = true; a.reference_kind = 2; a.last_step = now;
    return OK;
}
Result Controller::moveTo(uint8_t id, int32_t p, uint32_t now) {
    Result r = check(id); if (r != OK) return r;
    Axis& a = axes_[id];
    if (a.state == HOMING) return BUSY;
    if (!a.known) return UNREFERENCED;
    if (p < a.config.minimum || p > a.config.maximum) return LIMIT;
    if ((r = enable(id)) != OK) return r;
    a.target = p;
    if (a.state != MOVING && p != a.position) {
        a.direction = p > a.position ? 1 : -1;
        a.speed_squared = smaller(uint32_t(a.config.max_speed) * a.config.max_speed, 2UL * a.config.acceleration);
        a.last_step = now; a.state = MOVING;
    }
    return OK;
}
Result Controller::home(uint8_t id, uint32_t now) {
    Result r = check(id); if (r != OK) return r;
    Axis& a = axes_[id];
    if (a.config.home_strategy != 1) return UNSUPPORTED;
    if (a.state == HOMING || a.state == MOVING) return BUSY;
    if ((r = enable(id)) != OK) return r;
    a.known = false; a.reference_kind = 0; a.backing_off = false;
    a.home_remaining = a.config.home_steps; a.home_started = a.last_step = now;
    a.state = HOMING; return OK;
}
Result Controller::release(uint8_t id) {
    Result r = check(id); if (r != OK) return r;
    Axis& a = axes_[id];
    bool ok = driver_.release(a.config.address, a.config.port);
    a.enabled = false; a.known = false; a.reference_kind = 0; a.speed_squared = 0; a.home_remaining = 0;
    if (!ok) { fail(a, DRIVER_ERROR, false); return IO_ERROR; }
    if (a.state != FAULT) a.state = DISABLED;
    return OK;
}
void Controller::idle(Axis& a) {
    a.state = IDLE; a.speed_squared = 0; a.direction = 0;
    if (!a.config.hold) {
        if (!driver_.release(a.config.address, a.config.port)) { fail(a, DRIVER_ERROR, false); return; }
        a.enabled = false; a.known = false; a.reference_kind = 0; a.state = DISABLED;
    }
}
Result Controller::stop(uint8_t id) {
    Result r = check(id); if (r != OK) return r;
    Axis& a = axes_[id];
    // Immediate halt is explicitly not a controlled deceleration. A moving rotor
    // can coast: require a fresh reference rather than claiming exact position.
    if (a.state == MOVING || a.state == HOMING) { a.known = false; a.reference_kind = 0; }
    a.target = a.position; a.home_remaining = 0;
    if (a.state != FAULT) idle(a);
    return OK;
}
void Controller::update(uint32_t now) {
    for (uint8_t i = 0; i < MAX_AXES; ++i) {
        Axis& a = axes_[i]; if (!a.configured || a.state == FAULT) continue;
        if (uint32_t(now - last_contact_) >= uint32_t(a.config.watchdog_ms) * 1000UL) {
            // Policy 0 releases; policy 1 holds the last phase but never claims
            // a reference after an emergency halt. Explicit reconfigure required.
            fail(a, HOST_LOST, a.config.loss_policy == 0); continue;
        }
        if (a.state != MOVING && a.state != HOMING) continue;
        if (a.state == HOMING && uint32_t(now - a.home_started) >= uint32_t(a.config.home_timeout_ms) * 1000UL) {
            fail(a, HOME_TIMEOUT); continue;
        }
        uint32_t speed = a.state == HOMING ? a.config.home_speed : uint32_t(sqrt(double(a.speed_squared)));
        if (!speed) speed = 1;
        if (uint32_t(now - a.last_step) < (1000000UL + speed - 1) / speed) continue;
        // No catch-up burst: the next deadline is based on the actual service time.
        a.last_step = now;
        int8_t direction = a.state == HOMING ? a.config.home_direction : a.direction;
        if (a.state == HOMING && a.backing_off) direction = -direction;
        if (a.config.reverse) direction = -direction;
        uint8_t count = driver_.phaseCount(a.config.mode);
        uint8_t phase = (a.phase + (direction > 0 ? 1 : count - 1)) % count;
        if (!driver_.write(a.config.address, a.config.port, a.config.mode, phase)) { fail(a, DRIVER_ERROR); continue; }
        a.phase = phase;
        if (a.state == HOMING) {
            if (--a.home_remaining == 0) {
                if (!a.backing_off && a.config.home_backoff) {
                    a.backing_off = true; a.home_remaining = a.config.home_backoff;
                } else {
                    a.position = a.target = a.config.home_position; a.known = true;
                    a.reference_kind = 1; idle(a);
                }
            }
            continue;
        }
        a.position += a.direction;
        int32_t distance = (a.target - a.position) * a.direction;
        uint32_t quantum = 2UL * a.config.acceleration;
        // Compare by division: full position span times acceleration can overflow uint32_t.
        bool braking = distance <= 0 || (a.speed_squared + quantum - 1) / quantum >= uint32_t(distance);
        if (a.position == a.config.minimum || a.position == a.config.maximum ||
            (braking && a.speed_squared <= quantum)) {
            a.speed_squared = 0;
            if (a.position == a.target) { idle(a); continue; }
            a.direction = a.target > a.position ? 1 : -1;
            a.speed_squared = smaller(uint32_t(a.config.max_speed) * a.config.max_speed, quantum);
        } else if (braking) a.speed_squared -= quantum;
        else a.speed_squared = smaller(a.speed_squared + quantum, uint32_t(a.config.max_speed) * a.config.max_speed);
    }
}
size_t encodeStatus(uint8_t* b, size_t n, uint8_t id, uint8_t op, uint16_t request, Result result, const Axis* a) {
    if (!b || n < 23) return 0;
    memset(b, 0, 23); b[0] = STATUS_TYPE; b[1] = REVISION; b[2] = op; b[3] = id;
    b[4] = uint8_t(request); b[5] = uint8_t(request >> 8); b[6] = result;
    if (a) {
        b[7] = a->state; b[8] = (a->configured ? 1 : 0) | (a->known ? 2 : 0) | (a->enabled ? 4 : 0) | (a->reference_kind << 3);
        b[9] = a->fault; put32(b + 10, a->position); put32(b + 14, a->target); b[18] = a->phase;
    }
    b[19] = MAX_AXES; b[20] = 1; b[21] = 7; b[22] = 1; // driver1; modes1..3; bounded-stop homing
    return 23;
}
} // namespace Stepper
