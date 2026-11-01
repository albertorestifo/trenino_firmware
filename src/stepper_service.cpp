#include "stepper_service.h"
#include "stepper_driver.h"
#include "config_manager.h"
#include "output_manager.h"
#include <Arduino.h>

namespace StepperService {
namespace {
Stepper::ShieldDriver driver;
Stepper::Controller controller(driver);
Sender send = nullptr;
bool boot_safe = true;
struct Previous { uint16_t request; uint8_t operation, result; bool valid; } previous[Stepper::MAX_AXES];
uint8_t snapshots[Stepper::MAX_AXES];
void status(uint8_t id, uint8_t op, uint16_t request, Stepper::Result result) {
    uint8_t bytes[23];
    size_t length = Stepper::encodeStatus(bytes, sizeof(bytes), id, op, request, result,
        id < Stepper::MAX_AXES ? &controller.axis(id) : nullptr);
    if (send) send(bytes, length);
}
bool busPin(uint8_t pin) { return pin == SDA || pin == SCL; }
bool analogBusPin(uint8_t pin) {
#if defined(ARDUINO_ARCH_AVR)
    // AVR analogRead accepts both channel numbers and digital Ax aliases.
    // On the328P channels4/5 are the same pads as A4/A5 and SDA/SCL.
#if defined(analogInputToDigitalPin)
    if (pin < NUM_ANALOG_INPUTS) {
        int digital = analogInputToDigitalPin(pin);
        return digital >= 0 && (digital == SDA || digital == SCL);
    }
#endif
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // These AVR variants have no ADC channel on either TWI pad. In particular
    // Leonardo analog channels2/3 are not digital SDA2/SCL3.
    return false;
#endif
#endif
    return busPin(pin);
}
bool moduleConflicts(const ConfigManager::ModuleConfig& c, uint8_t address) {
    switch (c.module_type) {
    case Protocol::MODULE_TYPE_ANALOG: return analogBusPin(c.analog.pin);
    case Protocol::MODULE_TYPE_BUTTON: return busPin(c.button.pin);
    case Protocol::MODULE_TYPE_MATRIX:
        for (uint8_t i = 0; i < c.matrix.num_row_pins + c.matrix.num_col_pins && i < Protocol::MAX_MATRIX_PINS; ++i)
            if (busPin(c.matrix.pins[i])) return true;
        return false;
    case Protocol::MODULE_TYPE_HT16K33: return c.ht16k33.i2c_address == address;
    case Protocol::MODULE_TYPE_MCP23017: return c.mcp23017.i2c_address == address;
    case Protocol::MODULE_TYPE_ADS1115: return c.ads1115.i2c_address == address;
    default: return false;
    }
}
}
void init(Sender sender) {
    send = sender;
    uint8_t registered = ConfigManager::getRegisteredShieldAddress();
    // Only touch explicitly registered hardware, before EEPROM display init.
    // A failed shutdown leaves the bus unsafe: main defers module init below.
    if (registered) boot_safe = driver.shutdown(registered);
}
bool canApplyModules() {
    if (!boot_safe) return false;
    uint8_t address = ConfigManager::getRegisteredShieldAddress();
    if (!address) return true;
    uint8_t count = 0;
    const ConfigManager::ModuleConfig* modules = ConfigManager::getCurrentConfig(count);
    for (uint8_t i = 0; i < count; ++i) if (moduleConflicts(modules[i], address)) return false;
    return true;
}
uint8_t bootFaultAddress() { return canApplyModules() ? 0 : ConfigManager::getRegisteredShieldAddress(); }
bool reservesPin(uint8_t pin) { return ConfigManager::getRegisteredShieldAddress() && busPin(pin); }
bool conflicts(const Protocol::Configure& cfg) {
    uint8_t address = ConfigManager::getRegisteredShieldAddress();
    if (!address) return false;
    ConfigManager::ModuleConfig c;
    c.assign(cfg);
    return moduleConflicts(c, address);
}
bool reset() {
    bool ok = controller.reset();
    for (uint8_t i = 0; i < Stepper::MAX_AXES; ++i) { previous[i].valid = false; snapshots[i] = 0xff; }
    return ok;
}
bool onPacket(const uint8_t* bytes, size_t length) {
    if (!length || bytes[0] != Stepper::COMMAND_TYPE) return false;
    Stepper::Command c;
    if (!c.decode(bytes, length)) return true; // malformed extensions never refresh watchdog
    uint32_t now = micros();
    if (c.operation == Stepper::RESET || c.operation == Stepper::KEEP_ALIVE || c.operation == Stepper::FORGET_DRIVER) {
        if (c.axis != 255) { status(c.axis, c.operation, c.request_id, Stepper::BAD_AXIS); return true; }
        if (c.operation == Stepper::RESET && !reset()) {
            status(255, c.operation, c.request_id, Stepper::IO_ERROR); return true;
        }
        if (c.operation == Stepper::FORGET_DRIVER) {
            for (uint8_t i = 0; i < Stepper::MAX_AXES; ++i)
                if (controller.axis(i).configured) { status(255, c.operation, c.request_id, Stepper::BUSY); return true; }
            uint8_t registered = ConfigManager::getRegisteredShieldAddress();
            if (registered && !driver.shutdown(registered)) { status(255, c.operation, c.request_id, Stepper::IO_ERROR); return true; }
            ConfigManager::registerShieldAddress(0); boot_safe = true;
        }
        controller.keepAlive(now); status(255, c.operation, c.request_id, Stepper::OK); return true;
    }
    if (c.axis >= Stepper::MAX_AXES) { status(c.axis, c.operation, c.request_id, Stepper::BAD_AXIS); return true; }
    if (c.operation == Stepper::STATUS_QUERY) { status(c.axis, c.operation, c.request_id, Stepper::OK); return true; }
    if (!c.request_id) { status(c.axis, c.operation, c.request_id, Stepper::INVALID); return true; }
    Previous& p = previous[c.axis];
    if (p.valid && p.request == c.request_id) {
        status(c.axis, c.operation, c.request_id, c.operation == p.operation ? Stepper::Result(p.result) : Stepper::INVALID);
        return true; // in particular never restart a retried Home
    }
    Stepper::Result result = Stepper::INVALID;
    switch (c.operation) {
    case Stepper::CONFIGURE: {
        bool conflict = OutputManager::isOutputPin(SDA) || OutputManager::isOutputPin(SCL);
        uint8_t count = 0;
        const ConfigManager::ModuleConfig* modules = ConfigManager::getCurrentConfig(count);
        for (uint8_t i = 0; i < count; ++i) conflict |= moduleConflicts(modules[i], c.config.address);
        uint8_t registered = ConfigManager::getRegisteredShieldAddress();
        // Registration persists across reset/reconnect. Changing physical address
        // requires explicit commissioning, rather than silently abandoning a shield.
        if (registered && registered != c.config.address) conflict = true;
        result = conflict ? Stepper::CONFLICT : controller.configure(c.axis, c.config, now);
        if (result == Stepper::OK) { ConfigManager::registerShieldAddress(c.config.address); boot_safe = true; }
        break;
    }
    case Stepper::MOVE_TO: result = controller.moveTo(c.axis, c.position, now); break;
    case Stepper::HOME: result = controller.home(c.axis, now); break;
    case Stepper::STOP: result = controller.stop(c.axis); break;
    case Stepper::ENABLE: result = controller.enable(c.axis); break;
    case Stepper::RELEASE: result = controller.release(c.axis); break;
    case Stepper::REFERENCE: result = controller.reference(c.axis, c.position, now); break;
    default: break;
    }
    p = {c.request_id, c.operation, uint8_t(result), true};
    if (result == Stepper::OK) controller.keepAlive(now);
    status(c.axis, c.operation, c.request_id, result);
    return true;
}
void update() {
    controller.update(micros());
    for (uint8_t i = 0; i < Stepper::MAX_AXES; ++i) {
        const Stepper::Axis& a = controller.axis(i);
        uint8_t snapshot = uint8_t(a.state) | (a.known ? 8 : 0) | (uint8_t(a.fault) << 4);
        if (a.configured && snapshots[i] != snapshot) status(i, Stepper::STATUS_QUERY, 0, Stepper::OK);
        snapshots[i] = snapshot;
    }
}
}
