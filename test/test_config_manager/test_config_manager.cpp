#include "../../src/config_manager.h"
#include "../../src/device_info.h"
#include "../EEPROM.h"
#include <unity.h>

// Define mock EEPROM storage
uint8_t mock_eeprom_storage[1024];
EEPROMClass EEPROM;

// Mock Arduino functions
static unsigned long mock_millis_value = 0;
unsigned long millis()
{
    return mock_millis_value;
}

void pinMode(uint8_t pin, uint8_t mode)
{
    // Stub - not used in config_manager tests
}

int analogRead(uint8_t pin)
{
    // Stub - not used in config_manager tests
    return 0;
}

void setUp()
{
    // Clear mock EEPROM
    for (int i = 0; i < 1024; i++) {
        mock_eeprom_storage[i] = 0xFF;
    }
}

void tearDown()
{
    // Nothing to clean up
}

// Test that storeToEEPROM writes the device version
void test_store_writes_version()
{
    ConfigManager::InputConfig inputs[2];
    inputs[0].input_type = 0; // Analog
    inputs[0].pin = 14;
    inputs[0].sensitivity = 5;
    inputs[1].input_type = 0; // Analog
    inputs[1].pin = 15;
    inputs[1].sensitivity = 8;

    uint32_t config_id = 12345;

    ConfigManager::storeToEEPROM(config_id, inputs, 2);

    // Verify magic number
    uint32_t magic;
    EEPROM.get(ConfigManager::EEPROM_MAGIC_ADDR, magic);
    TEST_ASSERT_EQUAL_UINT32(ConfigManager::EEPROM_MAGIC, magic);

    // Verify version
    uint8_t version;
    EEPROM.get(ConfigManager::EEPROM_VERSION_ADDR, version);
    TEST_ASSERT_EQUAL_UINT8(DEVICE_VERSION, version);

    // Verify config_id
    uint32_t stored_config_id;
    EEPROM.get(ConfigManager::EEPROM_CONFIG_ID_ADDR, stored_config_id);
    TEST_ASSERT_EQUAL_UINT32(config_id, stored_config_id);

    // Verify number of inputs
    uint8_t num_inputs;
    EEPROM.get(ConfigManager::EEPROM_NUM_INPUTS_ADDR, num_inputs);
    TEST_ASSERT_EQUAL_UINT8(2, num_inputs);
}

// Test that loadFromEEPROM succeeds with matching version
void test_load_succeeds_with_matching_version()
{
    // Store a valid configuration
    ConfigManager::InputConfig inputs[1];
    inputs[0].input_type = 0;
    inputs[0].pin = 14;
    inputs[0].sensitivity = 5;

    uint32_t config_id = 54321;
    ConfigManager::storeToEEPROM(config_id, inputs, 1);

    // Load it back
    bool result = ConfigManager::loadFromEEPROM();
    TEST_ASSERT_TRUE(result);

    // Verify loaded config_id
    TEST_ASSERT_EQUAL_UINT32(config_id, ConfigManager::getCurrentConfigId());

    // Verify loaded inputs
    uint8_t num_inputs = 0;
    const ConfigManager::InputConfig* loaded = ConfigManager::getCurrentConfig(num_inputs);
    TEST_ASSERT_EQUAL_UINT8(1, num_inputs);
    TEST_ASSERT_EQUAL_UINT8(0, loaded[0].input_type);
    TEST_ASSERT_EQUAL_UINT8(14, loaded[0].pin);
    TEST_ASSERT_EQUAL_UINT8(5, loaded[0].sensitivity);
}

// Test that loadFromEEPROM fails and clears EEPROM with mismatched version
void test_load_fails_with_mismatched_version()
{
    // Manually write EEPROM with wrong version
    EEPROM.put(ConfigManager::EEPROM_MAGIC_ADDR, ConfigManager::EEPROM_MAGIC);
    uint8_t wrong_version = DEVICE_VERSION + 1;
    EEPROM.put(ConfigManager::EEPROM_VERSION_ADDR, wrong_version);
    uint32_t config_id = 99999;
    EEPROM.put(ConfigManager::EEPROM_CONFIG_ID_ADDR, config_id);
    uint8_t num_inputs = 1;
    EEPROM.put(ConfigManager::EEPROM_NUM_INPUTS_ADDR, num_inputs);

    // Try to load - should fail
    bool result = ConfigManager::loadFromEEPROM();
    TEST_ASSERT_FALSE(result);

    // Verify magic number was cleared
    uint32_t magic;
    EEPROM.get(ConfigManager::EEPROM_MAGIC_ADDR, magic);
    TEST_ASSERT_EQUAL_UINT32(0, magic);
}

// Test that loadFromEEPROM fails with no magic number
void test_load_fails_with_no_magic()
{
    // EEPROM is already cleared in setUp
    bool result = ConfigManager::loadFromEEPROM();
    TEST_ASSERT_FALSE(result);
}

// Test that loadFromEEPROM fails with invalid number of inputs
void test_load_fails_with_invalid_num_inputs()
{
    // Write valid magic and version but invalid num_inputs
    EEPROM.put(ConfigManager::EEPROM_MAGIC_ADDR, ConfigManager::EEPROM_MAGIC);
    EEPROM.put(ConfigManager::EEPROM_VERSION_ADDR, DEVICE_VERSION);
    uint32_t config_id = 12345;
    EEPROM.put(ConfigManager::EEPROM_CONFIG_ID_ADDR, config_id);
    uint8_t num_inputs = 0; // Invalid: 0 inputs
    EEPROM.put(ConfigManager::EEPROM_NUM_INPUTS_ADDR, num_inputs);

    bool result = ConfigManager::loadFromEEPROM();
    TEST_ASSERT_FALSE(result);

    // Try with too many inputs
    num_inputs = ConfigManager::MAX_INPUTS + 1;
    EEPROM.put(ConfigManager::EEPROM_NUM_INPUTS_ADDR, num_inputs);

    result = ConfigManager::loadFromEEPROM();
    TEST_ASSERT_FALSE(result);
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_store_writes_version);
    RUN_TEST(test_load_succeeds_with_matching_version);
    RUN_TEST(test_load_fails_with_mismatched_version);
    RUN_TEST(test_load_fails_with_no_magic);
    RUN_TEST(test_load_fails_with_invalid_num_inputs);

    return UNITY_END();
}
