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
int analogRead(uint8_t) { return 0; }
int digitalRead(uint8_t) { return 0; }
void delay(unsigned long) {}
void delayMicroseconds(unsigned int) {}
#ifndef UNIT_TEST
#define UNIT_TEST
#endif
#include "../../src/i2c_bus.cpp"
#include "../../src/config_manager.cpp"
#include "../../src/output_manager.cpp"
#include "../../src/stepper_driver.cpp"
#include "../../src/stepper_service.cpp"
#include "../../src/analog_sensor.cpp"
#include "../../src/button_sensor.cpp"
#include "../../src/matrix_core.cpp"
#include "../../src/matrix_sensor.cpp"
#include "../../src/ht16k33_module.cpp"
#include "../../src/mcp23017_module.cpp"
#include "../../src/ads1115_module.cpp"
#include "../../src/sensor_manager.cpp"
#include "../../src/message_handler.cpp"
#include <unity.h>
std::vector<std::vector<uint8_t>> packets;
void capture(const uint8_t* b, size_t n) { packets.emplace_back(b, b+n); }
void setUp() {
    StepperService::reset(); MockWire::reset(); I2CBus::resetForTest(); memset(mock_eeprom_storage, 0, sizeof(mock_eeprom_storage));
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
void test_empty_deployment_ack_identity_modules_axes_and_output_semantics() {
    PacketSerial_<COBS> serial;
    ModuleManager::init(); MessageHandler::init(&serial);
    const uint8_t populated[] = {2,100,0,0,0,1,0,1,7,3}; // button pin7
    MessageHandler::onPacketReceived(populated,sizeof(populated));
    TEST_ASSERT_NOT_NULL(ModuleManager::getModuleByPin(7));
    configure(); command(2,0,2); // axis has begun a home sweep
    OutputManager::setOutput(8,1);
    const uint8_t empty[] = {2,0x78,0x56,0x34,0x12,0,0,0xff};
    MessageHandler::onPacketReceived(empty,sizeof(empty));
    TEST_ASSERT_NULL(ModuleManager::getModuleByPin(7));
    TEST_ASSERT_EQUAL(3,serial.packets.back()[0]); // ConfigurationStored
    const uint8_t stored[] = {3,0x78,0x56,0x34,0x12};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(stored,serial.packets.back().data(),sizeof(stored));
    command(6,0,5); // now uses MessageHandler transport callback
    TEST_ASSERT_EQUAL(17,serial.packets.back()[0]);
    TEST_ASSERT_EQUAL(0,serial.packets.back()[8]); // axis config/reference/enable cleared
    TEST_ASSERT_TRUE(OutputManager::isOutputPin(8)); // legacy config does not clear independent outputs
    TEST_ASSERT_EQUAL(0x60,ConfigManager::getRegisteredShieldAddress());
    const uint8_t identity[] = {0,1,0,0,0};
    MessageHandler::onPacketReceived(identity,sizeof(identity));
    TEST_ASSERT_EQUAL(1,serial.packets.back()[0]);
    const uint8_t expected_id[] = {0x78,0x56,0x34,0x12};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_id,serial.packets.back().data()+8,4);
    ModuleManager::init();
}
int main() {
    UNITY_BEGIN(); RUN_TEST(test_home_retry_is_not_reexecuted_and_status_is_completion);
    RUN_TEST(test_registration_boot_shutdown_and_forget_workflow);
    RUN_TEST(test_conflicts_and_watchdog_are_stepper_specific);
    RUN_TEST(test_existing_analog_channel_conflict_and_reset_nack);
    RUN_TEST(test_empty_deployment_ack_identity_modules_axes_and_output_semantics); return UNITY_END();
}
