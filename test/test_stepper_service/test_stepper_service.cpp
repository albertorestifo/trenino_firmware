#include <stdint.h>
#include <string.h>
#include <vector>
uint8_t mock_eeprom_storage[1024];
#include "../EEPROM.h"
EEPROMClass EEPROM;
#define SDA 18
#define SCL 19
#define ARDUINO_ARCH_AVR
#define NUM_ANALOG_INPUTS 8
#define analogInputToDigitalPin(p) ((p) < 6 ? (p) + 14 : -1)
static uint32_t now_us;
unsigned long micros() { return now_us; }
unsigned long millis() { return now_us / 1000; }
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t, uint8_t) {}
#include "../../src/config_manager.cpp"
#include "../../src/output_manager.cpp"
#include "../../src/stepper_driver.cpp"
#include "../../src/stepper_service.cpp"
#include <unity.h>
std::vector<std::vector<uint8_t>> packets;
void capture(const uint8_t* b, size_t n) { packets.emplace_back(b, b+n); }
void setUp() {
    StepperService::reset(); MockWire::reset(); memset(mock_eeprom_storage, 0, sizeof(mock_eeprom_storage));
    ConfigManager::init(); OutputManager::init(); packets.clear(); now_us = 0; StepperService::init(capture);
}
void tearDown() {}
void command(uint8_t op, uint8_t axis, uint16_t id) {
    uint8_t b[] = {16,1,op,axis,uint8_t(id),uint8_t(id >> 8)};
    TEST_ASSERT_TRUE(StepperService::onPacket(b, sizeof(b)));
}
void configure(uint8_t axis = 0, uint16_t id = 1, uint8_t port = 1) {
    // min0 max100 speed100 accel100 home-1 sweep2 speed20 timeout1000 pos0 watchdog1000 backoff1
    uint8_t b[41] = {16,1,0,axis,uint8_t(id),uint8_t(id >> 8),1,1,0x60,port,0,1,0,1,
        0,0,0,0,100,0,0,0,100,0,100,0,255,2,0,20,0,0xe8,3,0,0,0,0,0xe8,3,1,0};
    StepperService::onPacket(b, sizeof(b));
}
uint8_t result() { return packets.back()[6]; }
void test_home_retry_is_not_reexecuted_and_status_is_completion() {
    configure(); TEST_ASSERT_EQUAL(0, result()); TEST_ASSERT_EQUAL(0x60, ConfigManager::getRegisteredShieldAddress());
    command(2,0,2); TEST_ASSERT_EQUAL(3, packets.back()[7]); TEST_ASSERT_EQUAL(0, packets.back()[8] & 2);
    now_us = 50000; StepperService::update();
    command(2,0,2); TEST_ASSERT_EQUAL(0, result());
    now_us = 100000; StepperService::update();
    now_us = 150000; StepperService::update();
    TEST_ASSERT_EQUAL(1, packets.back()[7]); TEST_ASSERT_EQUAL(0x0f, packets.back()[8]);
    uint8_t count = MockWire::transaction_count;
    command(2,0,2); TEST_ASSERT_EQUAL(count, MockWire::transaction_count);
    TEST_ASSERT_EQUAL(1, packets.back()[7]);
    command(3,0,2); TEST_ASSERT_EQUAL(1, result());
}
void test_registration_boot_shutdown_and_forget_workflow() {
    configure();
    command(10,255,3); TEST_ASSERT_EQUAL(8, result());
    command(8,255,4); command(10,255,5); TEST_ASSERT_EQUAL(0, result());
    TEST_ASSERT_EQUAL(0, ConfigManager::getRegisteredShieldAddress());
    ConfigManager::registerShieldAddress(0x60); MockWire::reset();
    StepperService::init(capture);
    TEST_ASSERT_EQUAL(0x60, MockWire::transactions[0].address);
    TEST_ASSERT_EQUAL(0x20, MockWire::transactions[0].data[1]);
    TEST_ASSERT_TRUE(StepperService::canApplyModules());
    MockWire::scheduleNacks(0x60,1); StepperService::init(capture);
    TEST_ASSERT_FALSE(StepperService::canApplyModules());
}
void test_conflicts_and_watchdog_are_stepper_specific() {
    OutputManager::setOutput(SDA, 1); configure(); TEST_ASSERT_EQUAL(4, result());
    OutputManager::init(); configure(0,2); TEST_ASSERT_EQUAL(0, result());
    Protocol::Configure cfg; cfg.module_type = Protocol::MODULE_TYPE_BUTTON; cfg.button.pin = SCL;
    TEST_ASSERT_TRUE(StepperService::conflicts(cfg));
    cfg.module_type = Protocol::MODULE_TYPE_ANALOG; cfg.analog.pin = 4;
    TEST_ASSERT_TRUE(StepperService::conflicts(cfg));
    cfg.analog.pin = 5; TEST_ASSERT_TRUE(StepperService::conflicts(cfg));
    cfg.module_type = Protocol::MODULE_TYPE_HT16K33; cfg.ht16k33.i2c_address = 0x70;
    TEST_ASSERT_FALSE(StepperService::conflicts(cfg));
    TEST_ASSERT_TRUE(StepperService::reservesPin(SDA));
    now_us = 999000; command(6,0,4); // status does not keep host lease alive
    now_us = 1000000; StepperService::update();
    TEST_ASSERT_EQUAL(4, packets.back()[7]); TEST_ASSERT_EQUAL(3, packets.back()[9]);
}
void test_existing_analog_channel_conflict_and_reset_nack() {
    ConfigManager::ModuleConfig mod; mod.module_type = Protocol::MODULE_TYPE_ANALOG; mod.analog.pin = 4;
    ConfigManager::storeToEEPROM(100,&mod,1); ConfigManager::init();
    configure(); TEST_ASSERT_EQUAL(4,result());
    TEST_ASSERT_EQUAL(0,ConfigManager::getRegisteredShieldAddress());
    ConfigManager::storeToEEPROM(101,nullptr,0); ConfigManager::init();
    configure(0,2); TEST_ASSERT_EQUAL(0,result());
    MockWire::scheduleNacks(0x60,1); command(8,255,3);
    TEST_ASSERT_EQUAL(9,result());
    command(6,0,4); TEST_ASSERT_EQUAL(0,packets.back()[8]);
}
int main() {
    UNITY_BEGIN(); RUN_TEST(test_home_retry_is_not_reexecuted_and_status_is_completion);
    RUN_TEST(test_registration_boot_shutdown_and_forget_workflow);
    RUN_TEST(test_conflicts_and_watchdog_are_stepper_specific);
    RUN_TEST(test_existing_analog_channel_conflict_and_reset_nack); return UNITY_END();
}
