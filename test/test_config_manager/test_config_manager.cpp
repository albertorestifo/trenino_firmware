// Define mock EEPROM storage first (before including EEPROM.h)
#include <stdint.h>
uint8_t mock_eeprom_storage[1024];

#include "../../src/device_info.h"
#include "../EEPROM.h"

// Define EEPROM instance
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

// Include config_manager implementation after mocks are defined
#include "../../src/config_manager.cpp"
#include <unity.h>

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
    ConfigManager::ModuleConfig modules[2];
    modules[0].module_type = Protocol::MODULE_TYPE_ANALOG;
    modules[0].analog.pin = 14;
    modules[0].analog.sensitivity = 5;
    modules[1].module_type = Protocol::MODULE_TYPE_ANALOG;
    modules[1].analog.pin = 15;
    modules[1].analog.sensitivity = 8;

    uint32_t config_id = 12345;

    ConfigManager::storeToEEPROM(config_id, modules, 2);

    // Verify magic number
    uint32_t magic;
    EEPROM.get(ConfigManager::EEPROM_MAGIC_ADDR, magic);
    TEST_ASSERT_EQUAL_UINT32(ConfigManager::EEPROM_MAGIC, magic);

    // Verify EEPROM format version
    uint8_t version;
    EEPROM.get(ConfigManager::EEPROM_VERSION_ADDR, version);
    TEST_ASSERT_EQUAL_UINT8(EEPROM_FORMAT_VERSION, version);

    // Verify config_id
    uint32_t stored_config_id;
    EEPROM.get(ConfigManager::EEPROM_CONFIG_ID_ADDR, stored_config_id);
    TEST_ASSERT_EQUAL_UINT32(config_id, stored_config_id);

    // Verify number of modules
    uint8_t num_modules;
    EEPROM.get(ConfigManager::EEPROM_NUM_INPUTS_ADDR, num_modules);
    TEST_ASSERT_EQUAL_UINT8(2, num_modules);
}

// Test that loadFromEEPROM succeeds with matching version
void test_load_succeeds_with_matching_version()
{
    // Store a valid configuration
    ConfigManager::ModuleConfig modules[1];
    modules[0].module_type = Protocol::MODULE_TYPE_ANALOG;
    modules[0].analog.pin = 14;
    modules[0].analog.sensitivity = 5;

    uint32_t config_id = 54321;
    ConfigManager::storeToEEPROM(config_id, modules, 1);

    // Load it back
    bool result = ConfigManager::loadFromEEPROM();
    TEST_ASSERT_TRUE(result);

    // Verify loaded config_id
    TEST_ASSERT_EQUAL_UINT32(config_id, ConfigManager::getCurrentConfigId());

    // Verify loaded modules
    uint8_t num_modules = 0;
    const ConfigManager::ModuleConfig* loaded = ConfigManager::getCurrentConfig(num_modules);
    TEST_ASSERT_EQUAL_UINT8(1, num_modules);
    TEST_ASSERT_EQUAL_UINT8(Protocol::MODULE_TYPE_ANALOG, loaded[0].module_type);
    TEST_ASSERT_EQUAL_UINT8(14, loaded[0].analog.pin);
    TEST_ASSERT_EQUAL_UINT8(5, loaded[0].analog.sensitivity);
}

// Test that loadFromEEPROM fails and clears EEPROM with mismatched version
void test_load_fails_with_mismatched_version()
{
    // Manually write EEPROM with wrong version
    EEPROM.put(ConfigManager::EEPROM_MAGIC_ADDR, ConfigManager::EEPROM_MAGIC);
    uint8_t wrong_version = EEPROM_FORMAT_VERSION + 1;
    EEPROM.put(ConfigManager::EEPROM_VERSION_ADDR, wrong_version);
    uint32_t config_id = 99999;
    EEPROM.put(ConfigManager::EEPROM_CONFIG_ID_ADDR, config_id);
    uint8_t num_modules = 1;
    EEPROM.put(ConfigManager::EEPROM_NUM_INPUTS_ADDR, num_modules);

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
    EEPROM.put(ConfigManager::EEPROM_VERSION_ADDR, EEPROM_FORMAT_VERSION);
    uint32_t config_id = 12345;
    EEPROM.put(ConfigManager::EEPROM_CONFIG_ID_ADDR, config_id);
    uint8_t num_modules = ConfigManager::MAX_MODULES + 1;
    EEPROM.put(ConfigManager::EEPROM_NUM_INPUTS_ADDR, num_modules);

    bool result = ConfigManager::loadFromEEPROM();
    TEST_ASSERT_FALSE(result);

    // Another out-of-range count must also fail.
    num_modules = 255;
    EEPROM.put(ConfigManager::EEPROM_NUM_INPUTS_ADDR, num_modules);

    result = ConfigManager::loadFromEEPROM();
    TEST_ASSERT_FALSE(result);
}

void test_shield_registration_roundtrip_and_corruption()
{
    TEST_ASSERT_EQUAL(0, ConfigManager::getRegisteredShieldAddress());
    ConfigManager::registerShieldAddress(0x61);
    TEST_ASSERT_EQUAL(0x61, ConfigManager::getRegisteredShieldAddress());
    TEST_ASSERT_EQUAL(0x53, mock_eeprom_storage[500]);
    TEST_ASSERT_EQUAL(1, mock_eeprom_storage[502]);
    mock_eeprom_storage[504] ^= 1;
    TEST_ASSERT_EQUAL(0, ConfigManager::getRegisteredShieldAddress());
    ConfigManager::registerShieldAddress(0x60);
    ConfigManager::storeToEEPROM(12345, nullptr, 0);
    TEST_ASSERT_EQUAL(0x60, ConfigManager::getRegisteredShieldAddress());
    ConfigManager::registerShieldAddress(0);
    TEST_ASSERT_EQUAL(0, ConfigManager::getRegisteredShieldAddress());
}

void test_empty_configuration_retains_identity_after_reboot_and_cancels_partial()
{
    ConfigManager::ModuleConfig mod; mod.module_type = Protocol::MODULE_TYPE_ANALOG; mod.analog.pin = 1;
    ConfigManager::storeToEEPROM(100,&mod,1); ConfigManager::init();
    uint8_t count = 0; ConfigManager::getCurrentConfig(count); TEST_ASSERT_EQUAL(1,count);
    ConfigManager::g_config_state.start(200,2);
    Protocol::Configure cfg; cfg.config_id = 0x12345678; cfg.module_type = 0xff; cfg.total_parts = 0; cfg.part_number = 0;
    bool complete = false, error = false;
    TEST_ASSERT_TRUE(ConfigManager::handleConfigure(cfg,complete,error));
    TEST_ASSERT_TRUE(complete); TEST_ASSERT_FALSE(error);
    TEST_ASSERT_FALSE(ConfigManager::g_config_state.isActive());
    ConfigManager::getCurrentConfig(count); TEST_ASSERT_EQUAL(0,count);
    TEST_ASSERT_EQUAL_HEX32(0x12345678,ConfigManager::getCurrentConfigId());
    ConfigManager::init(); // loads EEPROM, rather than preserving volatile globals
    ConfigManager::getCurrentConfig(count); TEST_ASSERT_EQUAL(0,count);
    TEST_ASSERT_EQUAL_HEX32(0x12345678,ConfigManager::getCurrentConfigId());
}
void test_noncanonical_empty_config_does_not_clear_modules()
{
    ConfigManager::ModuleConfig mod; ConfigManager::storeToEEPROM(100,&mod,1); ConfigManager::init();
    Protocol::Configure cfg; cfg.config_id = 200; cfg.module_type = 0xff; cfg.total_parts = 0; cfg.part_number = 1;
    bool complete = true, error = false;
    ConfigManager::handleConfigure(cfg,complete,error);
    TEST_ASSERT_FALSE(complete); TEST_ASSERT_TRUE(error);
    uint8_t count; ConfigManager::getCurrentConfig(count); TEST_ASSERT_EQUAL(1,count);
    TEST_ASSERT_EQUAL(100,ConfigManager::getCurrentConfigId());
}

void test_eeprom_round_trip_mcp23017()
{
    ConfigManager::ModuleConfig m[2];
    m[0].module_type = Protocol::MODULE_TYPE_MCP23017;
    m[0].mcp23017.i2c_address = 0x21;
    m[0].mcp23017.direct_base = 130;
    m[0].mcp23017.matrix_base = 150;
    m[0].mcp23017.debounce = 4;
    m[0].mcp23017.direct_mask = 0x0E0F;
    m[0].mcp23017.row_mask = 0x8030;
    m[0].mcp23017.col_mask = 0x0140;
    m[1].module_type = Protocol::MODULE_TYPE_ANALOG;
    m[1].analog.pin = 3;
    m[1].analog.sensitivity = 6;
    ConfigManager::storeToEEPROM(77, m, 2);
    TEST_ASSERT_TRUE(ConfigManager::loadFromEEPROM());
    uint8_t n;
    const ConfigManager::ModuleConfig* l = ConfigManager::getCurrentConfig(n);
    TEST_ASSERT_EQUAL(2, n);
    TEST_ASSERT_EQUAL_UINT8(Protocol::MODULE_TYPE_MCP23017, l[0].module_type);
    TEST_ASSERT_EQUAL_UINT8(0x21, l[0].mcp23017.i2c_address);
    TEST_ASSERT_EQUAL_UINT8(130, l[0].mcp23017.direct_base);
    TEST_ASSERT_EQUAL_UINT8(150, l[0].mcp23017.matrix_base);
    TEST_ASSERT_EQUAL_UINT8(4, l[0].mcp23017.debounce);
    TEST_ASSERT_EQUAL_HEX16(0x0E0F, l[0].mcp23017.direct_mask);
    TEST_ASSERT_EQUAL_HEX16(0x8030, l[0].mcp23017.row_mask);
    TEST_ASSERT_EQUAL_HEX16(0x0140, l[0].mcp23017.col_mask);
    TEST_ASSERT_EQUAL_UINT8(3, l[1].analog.pin);
    TEST_ASSERT_EQUAL_UINT8(6, l[1].analog.sensitivity);
}

void test_eeprom_round_trip_ads1115()
{
    ConfigManager::ModuleConfig m[1];
    m[0].module_type = Protocol::MODULE_TYPE_ADS1115;
    m[0].ads1115.i2c_address = 0x49;
    m[0].ads1115.pin_base = 220;
    m[0].ads1115.channel_mask = 0x0B;
    m[0].ads1115.gain = 1;
    m[0].ads1115.ref_channel = 2;
    m[0].ads1115.sensitivity[0] = 2;
    m[0].ads1115.sensitivity[1] = 3;
    m[0].ads1115.sensitivity[2] = 4;
    m[0].ads1115.sensitivity[3] = 5;
    ConfigManager::storeToEEPROM(78, m, 1);
    TEST_ASSERT_TRUE(ConfigManager::loadFromEEPROM());
    uint8_t n;
    const ConfigManager::ModuleConfig* l = ConfigManager::getCurrentConfig(n);
    TEST_ASSERT_EQUAL(1, n);
    TEST_ASSERT_EQUAL_UINT8(Protocol::MODULE_TYPE_ADS1115, l[0].module_type);
    TEST_ASSERT_EQUAL_UINT8(0x49, l[0].ads1115.i2c_address);
    TEST_ASSERT_EQUAL_UINT8(220, l[0].ads1115.pin_base);
    TEST_ASSERT_EQUAL_UINT8(0x0B, l[0].ads1115.channel_mask);
    TEST_ASSERT_EQUAL_UINT8(1, l[0].ads1115.gain);
    TEST_ASSERT_EQUAL_UINT8(2, l[0].ads1115.ref_channel);
    for (int i = 0; i < 4; i++) TEST_ASSERT_EQUAL_UINT8(2 + i, l[0].ads1115.sensitivity[i]);
}

void test_eeprom_loads_3_1_0_record()
{
    // Format 5 layout written by firmware 3.1.0: analog, button, matrix, HT16K33.
    const uint8_t record[] = {
        0x34, 0x12, 0xFF, 0xC0,       // magic 0xC0FF1234 LE
        EEPROM_FORMAT_VERSION,
        0x2A, 0x00, 0x00, 0x00,       // config_id 42
        4,                            // inputs
        0, 14, 7,                     // analog pin 14 sens 7
        1, 5, 20,                     // button pin 5 debounce 20
        2, 2, 1, 6, 7, 8,             // matrix 2 rows 1 col pins 6 7 8
        4, 0x70, 9, 4                 // ht16k33 addr 0x70 brightness 9 digits 4
    };
    for (size_t i = 0; i < sizeof(record); i++) mock_eeprom_storage[i] = record[i];
    TEST_ASSERT_TRUE(ConfigManager::loadFromEEPROM());
    uint8_t n;
    const ConfigManager::ModuleConfig* l = ConfigManager::getCurrentConfig(n);
    TEST_ASSERT_EQUAL(4, n);
    TEST_ASSERT_EQUAL_HEX32(42, ConfigManager::getCurrentConfigId());
    TEST_ASSERT_EQUAL_UINT8(14, l[0].analog.pin);
    TEST_ASSERT_EQUAL_UINT8(7, l[0].analog.sensitivity);
    TEST_ASSERT_EQUAL_UINT8(5, l[1].button.pin);
    TEST_ASSERT_EQUAL_UINT8(20, l[1].button.debounce);
    TEST_ASSERT_EQUAL_UINT8(2, l[2].matrix.num_row_pins);
    TEST_ASSERT_EQUAL_UINT8(1, l[2].matrix.num_col_pins);
    TEST_ASSERT_EQUAL_UINT8(8, l[2].matrix.pins[2]);
    TEST_ASSERT_EQUAL_UINT8(0x70, l[3].ht16k33.i2c_address);
    TEST_ASSERT_EQUAL_UINT8(9, l[3].ht16k33.brightness);
    TEST_ASSERT_EQUAL_UINT8(4, l[3].ht16k33.num_digits);
}

void test_eeprom_rejects_wrapped_matrix_pin_count()
{
    const uint8_t record[] = {
        0x34, 0x12, 0xFF, 0xC0,       // magic 0xC0FF1234 LE
        EEPROM_FORMAT_VERSION,
        0x2A, 0x00, 0x00, 0x00,       // config_id 42
        1,                            // inputs
        2, 250, 10, 6, 7, 8, 9,       // matrix 250 + 10 pins wraps to 4
    };
    for (size_t i = 0; i < sizeof(record); i++) mock_eeprom_storage[i] = record[i];
    TEST_ASSERT_FALSE(ConfigManager::loadFromEEPROM());
}

void test_max_records_stay_below_shield_registration()
{
    ConfigManager::ModuleConfig m[ConfigManager::MAX_MODULES];
    for (auto& x : m) {
        x.module_type = Protocol::MODULE_TYPE_MATRIX;
        x.matrix.num_row_pins = 8;
        x.matrix.num_col_pins = 8;
    }
    ConfigManager::registerShieldAddress(0x60);
    ConfigManager::storeToEEPROM(1, m, ConfigManager::MAX_MODULES);
    TEST_ASSERT_EQUAL_UINT8(0x60, ConfigManager::getRegisteredShieldAddress());
    for (auto& x : m) x.module_type = Protocol::MODULE_TYPE_MCP23017;
    ConfigManager::storeToEEPROM(1, m, ConfigManager::MAX_MODULES);
    TEST_ASSERT_EQUAL_UINT8(0x60, ConfigManager::getRegisteredShieldAddress());
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_store_writes_version);
    RUN_TEST(test_load_succeeds_with_matching_version);
    RUN_TEST(test_load_fails_with_mismatched_version);
    RUN_TEST(test_load_fails_with_no_magic);
    RUN_TEST(test_load_fails_with_invalid_num_inputs);
    RUN_TEST(test_shield_registration_roundtrip_and_corruption);
    RUN_TEST(test_empty_configuration_retains_identity_after_reboot_and_cancels_partial);
    RUN_TEST(test_noncanonical_empty_config_does_not_clear_modules);

    RUN_TEST(test_eeprom_round_trip_mcp23017);
    RUN_TEST(test_eeprom_round_trip_ads1115);
    RUN_TEST(test_eeprom_loads_3_1_0_record);
    RUN_TEST(test_eeprom_rejects_wrapped_matrix_pin_count);
    RUN_TEST(test_max_records_stay_below_shield_registration);

    return UNITY_END();
}
