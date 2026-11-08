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

void test_load_rejects_wrapped_matrix_pin_count()
{
    EEPROM.put(ConfigManager::EEPROM_MAGIC_ADDR, ConfigManager::EEPROM_MAGIC);
    EEPROM.put(ConfigManager::EEPROM_VERSION_ADDR, EEPROM_FORMAT_VERSION);
    uint32_t config_id = 42;
    EEPROM.put(ConfigManager::EEPROM_CONFIG_ID_ADDR, config_id);
    uint8_t num_modules = 1;
    EEPROM.put(ConfigManager::EEPROM_NUM_INPUTS_ADDR, num_modules);
    // Matrix with 250 + 10 pins, which wraps to 4 in a uint8_t
    const uint8_t record[] = { Protocol::MODULE_TYPE_MATRIX, 250, 10, 6, 7, 8, 9 };
    for (size_t i = 0; i < sizeof(record); i++) {
        mock_eeprom_storage[ConfigManager::EEPROM_INPUTS_ADDR + i] = record[i];
    }

    TEST_ASSERT_FALSE(ConfigManager::loadFromEEPROM());
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

static ConfigManager::ModuleConfig gaugeModule(uint8_t step)
{
    ConfigManager::ModuleConfig m;
    m.module_type = Protocol::MODULE_TYPE_GAUGE;
    m.gauge.driver = Protocol::GAUGE_DRIVER_VID6606;
    m.gauge.pins[0] = step;
    m.gauge.pins[1] = step + 1;
    m.gauge.pins[2] = 6;
    m.gauge.pins[3] = Protocol::GAUGE_NO_PIN;
    m.gauge.travel = 3780;
    m.gauge.max_speed = 4000;
    m.gauge.acceleration = 0xABCD;
    m.gauge.flags = Protocol::GAUGE_FLAG_REVERSE;
    return m;
}

static Protocol::Configure gaugePart(uint32_t id, uint8_t total, uint8_t part, uint8_t step)
{
    Protocol::Configure cfg;
    cfg.config_id = id;
    cfg.total_parts = total;
    cfg.part_number = part;
    cfg.module_type = Protocol::MODULE_TYPE_GAUGE;
    cfg.gauge = gaugeModule(step).gauge;
    return cfg;
}

void test_gauge_record_eeprom_roundtrip_is_field_by_field()
{
    ConfigManager::ModuleConfig modules[2];
    modules[0] = gaugeModule(8);
    modules[1].module_type = Protocol::MODULE_TYPE_BUTTON;
    modules[1].button.pin = 9;
    modules[1].button.debounce = 3;
    ConfigManager::storeToEEPROM(77, modules, 2);

    // 1 type byte + 12 payload bytes, little endian, then the next record.
    const uint8_t expected[] = { Protocol::MODULE_TYPE_GAUGE, 0, 8, 9, 6, 0xff,
        0xC4, 0x0E, 0xA0, 0x0F, 0xCD, 0xAB, 1, Protocol::MODULE_TYPE_BUTTON, 9, 3 };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, &mock_eeprom_storage[ConfigManager::EEPROM_INPUTS_ADDR], sizeof(expected));

    TEST_ASSERT_TRUE(ConfigManager::loadFromEEPROM());
    uint8_t count = 0;
    const ConfigManager::ModuleConfig* loaded = ConfigManager::getCurrentConfig(count);
    TEST_ASSERT_EQUAL(2, count);
    TEST_ASSERT_EQUAL_UINT8(Protocol::MODULE_TYPE_GAUGE, loaded[0].module_type);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(modules[0].gauge.pins, loaded[0].gauge.pins, 4);
    TEST_ASSERT_EQUAL_UINT16(3780, loaded[0].gauge.travel);
    TEST_ASSERT_EQUAL_UINT16(4000, loaded[0].gauge.max_speed);
    TEST_ASSERT_EQUAL_UINT16(0xABCD, loaded[0].gauge.acceleration);
    TEST_ASSERT_EQUAL_UINT8(Protocol::GAUGE_FLAG_REVERSE, loaded[0].gauge.flags);
    TEST_ASSERT_EQUAL_UINT8(9, loaded[1].button.pin);
}

void test_load_rejects_corrupt_gauge_record()
{
    ConfigManager::ModuleConfig m = gaugeModule(8);
    ConfigManager::storeToEEPROM(77, &m, 1);
    mock_eeprom_storage[ConfigManager::EEPROM_INPUTS_ADDR + 1] = 9; // driver 9
    TEST_ASSERT_FALSE(ConfigManager::loadFromEEPROM());

    ConfigManager::storeToEEPROM(77, &m, 1);
    mock_eeprom_storage[ConfigManager::EEPROM_INPUTS_ADDR + 6] = 0; // travel 0
    mock_eeprom_storage[ConfigManager::EEPROM_INPUTS_ADDR + 7] = 0;
    TEST_ASSERT_FALSE(ConfigManager::loadFromEEPROM());
}

void test_load_rejects_more_than_max_gauges()
{
    ConfigManager::ModuleConfig m[5];
    for (uint8_t i = 0; i < 5; i++) m[i] = gaugeModule(20 + 2 * i);
    ConfigManager::storeToEEPROM(77, m, 5);
    TEST_ASSERT_FALSE(ConfigManager::loadFromEEPROM());
    ConfigManager::storeToEEPROM(77, m, 4);
    TEST_ASSERT_TRUE(ConfigManager::loadFromEEPROM());
}

void test_configure_rejects_invalid_gauge_part()
{
    ConfigManager::g_config_state.reset();
    Protocol::Configure cfg = gaugePart(300, 1, 0, 8);
    cfg.gauge.max_speed = Protocol::GAUGE_MAX_SPEED_VID6606 + 1;
    bool complete = false;
    bool error = false;
    ConfigManager::handleConfigure(cfg, complete, error);
    TEST_ASSERT_TRUE(error);
    TEST_ASSERT_FALSE(complete);
}

void test_configure_rejects_fifth_gauge_at_completion_in_any_order()
{
    ConfigManager::g_config_state.reset();
    const uint8_t order[] = { 4, 0, 2, 2, 1, 3 }; // out of order, one repeat
    bool complete = false;
    bool error = false;
    for (uint8_t i = 0; i < sizeof(order); i++) {
        complete = error = false;
        ConfigManager::handleConfigure(gaugePart(400, 5, order[i], 20 + 2 * order[i]), complete, error);
        if (i < sizeof(order) - 1) TEST_ASSERT_FALSE(error);
    }
    TEST_ASSERT_TRUE(error);
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_FALSE(ConfigManager::g_config_state.isActive());

    // Four gauges plus another module is fine.
    ConfigManager::g_config_state.reset();
    for (uint8_t p = 0; p < 4; p++) {
        complete = error = false;
        ConfigManager::handleConfigure(gaugePart(401, 5, p, 20 + 2 * p), complete, error);
    }
    Protocol::Configure button;
    button.config_id = 401; button.total_parts = 5; button.part_number = 4;
    button.module_type = Protocol::MODULE_TYPE_BUTTON; button.button.pin = 2; button.button.debounce = 1;
    complete = error = false;
    ConfigManager::handleConfigure(button, complete, error);
    TEST_ASSERT_FALSE(error);
    TEST_ASSERT_TRUE(complete);
}

void test_configure_rejects_gauges_sharing_pins()
{
    bool complete = false;
    bool error = false;
    // Same STEP pin on two gauges.
    ConfigManager::g_config_state.reset();
    ConfigManager::handleConfigure(gaugePart(500, 2, 0, 8), complete, error);
    ConfigManager::handleConfigure(gaugePart(500, 2, 1, 8), complete, error);
    TEST_ASSERT_TRUE(error);
    TEST_ASSERT_FALSE(complete);

    // One gauge's DIR is another's STEP.
    ConfigManager::g_config_state.reset();
    complete = error = false;
    ConfigManager::handleConfigure(gaugePart(501, 2, 0, 8), complete, error);
    ConfigManager::handleConfigure(gaugePart(501, 2, 1, 9), complete, error);
    TEST_ASSERT_TRUE(error);

    // A shared RESET_N pin (6 in gaugePart) is fine.
    ConfigManager::g_config_state.reset();
    complete = error = false;
    ConfigManager::handleConfigure(gaugePart(502, 2, 0, 8), complete, error);
    ConfigManager::handleConfigure(gaugePart(502, 2, 1, 20), complete, error);
    TEST_ASSERT_FALSE(error);
    TEST_ASSERT_TRUE(complete);

    // A RESET_N pin that is another gauge's STEP is not.
    ConfigManager::g_config_state.reset();
    complete = error = false;
    ConfigManager::handleConfigure(gaugePart(503, 2, 0, 8), complete, error);
    ConfigManager::handleConfigure(gaugePart(503, 2, 1, 6), complete, error);
    TEST_ASSERT_TRUE(error);
}

void test_shield_registration_bytes_are_left_alone()
{
    // EEPROM 500..504 held the 3.1.0 shield registration; they are reserved.
    const uint8_t old_record[] = { 0x53, 0x54, 1, 0x60, 0x9F };
    for (uint8_t i = 0; i < 5; i++) mock_eeprom_storage[500 + i] = old_record[i];
    ConfigManager::ModuleConfig m = gaugeModule(8);
    ConfigManager::storeToEEPROM(1, &m, 1);
    TEST_ASSERT_TRUE(ConfigManager::loadFromEEPROM());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(old_record, &mock_eeprom_storage[500], 5);
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_store_writes_version);
    RUN_TEST(test_load_succeeds_with_matching_version);
    RUN_TEST(test_load_fails_with_mismatched_version);
    RUN_TEST(test_load_fails_with_no_magic);
    RUN_TEST(test_load_fails_with_invalid_num_inputs);
    RUN_TEST(test_load_rejects_wrapped_matrix_pin_count);
    RUN_TEST(test_empty_configuration_retains_identity_after_reboot_and_cancels_partial);
    RUN_TEST(test_noncanonical_empty_config_does_not_clear_modules);
    RUN_TEST(test_gauge_record_eeprom_roundtrip_is_field_by_field);
    RUN_TEST(test_load_rejects_corrupt_gauge_record);
    RUN_TEST(test_load_rejects_more_than_max_gauges);
    RUN_TEST(test_configure_rejects_invalid_gauge_part);
    RUN_TEST(test_configure_rejects_fifth_gauge_at_completion_in_any_order);
    RUN_TEST(test_configure_rejects_gauges_sharing_pins);
    RUN_TEST(test_shield_registration_bytes_are_left_alone);

    return UNITY_END();
}
