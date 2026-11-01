#include "config_manager.h"
#include "device_info.h"

// Platform-specific EEPROM instance for Arduino Due
#ifdef EEPROM_USE_DUE_FLASH
DueFlashStorage dueFlashStorage;
#endif

// Platform-agnostic EEPROM access helpers
namespace {

void eeprom_put_bytes(int address, const void* bytes, size_t count)
{
    const uint8_t* ptr = static_cast<const uint8_t*>(bytes);
    for (size_t i = 0; i < count; i++) {
#ifdef EEPROM_USE_DUE_FLASH
        dueFlashStorage.write(address + i, ptr[i]);
#else
        EEPROM.put(address + i, ptr[i]);
#endif
    }
}

void eeprom_get_bytes(int address, void* bytes, size_t count)
{
    uint8_t* ptr = static_cast<uint8_t*>(bytes);
    for (size_t i = 0; i < count; i++) {
#ifdef EEPROM_USE_DUE_FLASH
        ptr[i] = dueFlashStorage.read(address + i);
#else
        EEPROM.get(address + i, ptr[i]);
#endif
    }
}

template <typename T>
void eeprom_put(int address, const T& value)
{
    eeprom_put_bytes(address, &value, sizeof(T));
}

template <typename T>
void eeprom_get(int address, T& value)
{
    eeprom_get_bytes(address, &value, sizeof(T));
}

void eeprom_commit()
{
#ifdef EEPROM_NEEDS_COMMIT
    EEPROM.commit();
#endif
}

} // anonymous namespace

namespace ConfigManager {

uint8_t getRegisteredShieldAddress()
{
    uint8_t data[5];
    eeprom_get(500, data);
    if (data[0] != 0x53 || data[1] != 0x54 || data[2] != 1 ||
        data[3] < 0x60 || data[3] > 0x77 || data[4] != uint8_t(~data[3])) return 0;
    return data[3];
}

void registerShieldAddress(uint8_t address)
{
    if (getRegisteredShieldAddress() == address) return;
    // Commit marker last; interrupted writes cannot authorize an unrelated address.
    const uint8_t invalid = 0;
    eeprom_put(500, invalid);
    const uint8_t tail[4] = {0x54, 1, address, uint8_t(~address)};
    eeprom_put(501, tail);
    const uint8_t marker = 0x53;
    eeprom_put(500, marker);
    eeprom_commit();
}

// Global state
ConfigState g_config_state;
uint32_t g_current_config_id = 0;
static ModuleConfig g_current_modules[MAX_MODULES];
static uint8_t g_current_num_modules = 0;

void init()
{
#ifdef EEPROM_NEEDS_BEGIN
    // ESP32 requires explicit EEPROM initialization
    EEPROM.begin(EEPROM_SIZE);
#endif

    // Try to load configuration from EEPROM
    if (loadFromEEPROM()) {
        // Configuration loaded successfully
    } else {
        // No valid configuration in EEPROM
        g_current_config_id = 0;
        g_current_num_modules = 0;
    }
}

bool handleConfigure(const Protocol::Configure& cfg, bool& complete, bool& error)
{
    complete = false;
    error = false;

    // An explicit empty deployment is one complete command, never a multipart
    // fragment or a dummy module. It supersedes any in-flight deployment.
    if (cfg.total_parts == 0 || cfg.module_type == Protocol::MODULE_TYPE_EMPTY) {
        g_config_state.reset();
        if (cfg.total_parts != 0 || cfg.part_number != 0 || cfg.module_type != Protocol::MODULE_TYPE_EMPTY) {
            error = true; return true;
        }
        storeToEEPROM(cfg.config_id, nullptr, 0);
        g_current_config_id = cfg.config_id;
        g_current_num_modules = 0;
        complete = true;
        return true;
    }

    // Check if this is a new configuration or continuation of existing one
    if (!g_config_state.isActive()) {
        // Start new configuration
        if (cfg.total_parts == 0 || cfg.total_parts > MAX_MODULES) {
            error = true;
            return true; // Invalid total_parts
        }
        g_config_state.start(cfg.config_id, cfg.total_parts);
    } else {
        // Check if this is for the same configuration
        if (g_config_state.getConfigId() != cfg.config_id) {
            // Different config_id - discard old configuration and start new one
            g_config_state.reset();
            if (cfg.total_parts == 0 || cfg.total_parts > MAX_MODULES) {
                error = true;
                return true;
            }
            g_config_state.start(cfg.config_id, cfg.total_parts);
        }
    }

    // Add this part to the configuration
    if (!g_config_state.addPart(cfg)) {
        error = true;
        g_config_state.reset();
        return true;
    }

    // Check if configuration is complete
    if (g_config_state.isComplete()) {
        // Store to EEPROM
        storeToEEPROM(
            g_config_state.getConfigId(),
            g_config_state.getModules(),
            g_config_state.getNumModules());

        // Update current configuration
        g_current_config_id = g_config_state.getConfigId();
        g_current_num_modules = g_config_state.getNumModules();
        for (uint8_t i = 0; i < g_current_num_modules; i++) {
            g_current_modules[i] = g_config_state.getModules()[i];
        }

        // Reset state
        g_config_state.reset();

        complete = true;
        return true;
    }

    return false; // Configuration in progress
}

bool checkTimeout()
{
    if (g_config_state.isActive() && g_config_state.hasTimedOut()) {
        g_config_state.reset();
        return true;
    }
    return false;
}

void storeToEEPROM(uint32_t config_id, const ModuleConfig* modules, uint8_t num_modules)
{
    int addr = 0;

    // Write magic number
    eeprom_put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);

    // Write EEPROM format version (for compatibility checking)
    eeprom_put(EEPROM_VERSION_ADDR, EEPROM_FORMAT_VERSION);

    // Write config_id
    eeprom_put(EEPROM_CONFIG_ID_ADDR, config_id);

    // Write number of modules
    eeprom_put(EEPROM_NUM_INPUTS_ADDR, num_modules);

    // Write module configurations: the type, then the payload bytes in wire order
    addr = EEPROM_INPUTS_ADDR;
    for (uint8_t i = 0; i < num_modules; i++) {
        const ModuleConfig& module = modules[i];
        eeprom_put(addr, module.module_type);
        addr += sizeof(uint8_t);

        uint8_t total_pins = module.matrix.num_row_pins + module.matrix.num_col_pins;
        uint8_t payload_size = Protocol::modulePayloadSize(module.module_type, total_pins);
        eeprom_put_bytes(addr, &module.matrix, payload_size);
        addr += payload_size;
    }

    // Commit changes for platforms that require it
    eeprom_commit();
}

bool loadFromEEPROM()
{
    // Read and verify magic number
    uint32_t magic;
    eeprom_get(EEPROM_MAGIC_ADDR, magic);
    if (magic != EEPROM_MAGIC) {
        return false; // No valid configuration
    }

    // Read and verify EEPROM format version
    uint8_t stored_version;
    eeprom_get(EEPROM_VERSION_ADDR, stored_version);
    if (stored_version != EEPROM_FORMAT_VERSION) {
        // Version mismatch - clear EEPROM to invalidate old config
        // This prevents loading incompatible configurations after firmware updates
        uint32_t zero = 0;
        eeprom_put(EEPROM_MAGIC_ADDR, zero);
        eeprom_commit();
        return false;
    }

    // Read config_id
    eeprom_get(EEPROM_CONFIG_ID_ADDR, g_current_config_id);

    // Read number of inputs
    eeprom_get(EEPROM_NUM_INPUTS_ADDR, g_current_num_modules);

    // Validate number of inputs
    if (g_current_num_modules > MAX_MODULES) {
        return false;
    }

    // Read input configurations (variable size based on type)
    int addr = EEPROM_INPUTS_ADDR;
    for (uint8_t i = 0; i < g_current_num_modules; i++) {
        ModuleConfig& module = g_current_modules[i];
        eeprom_get(addr, module.module_type);
        addr += sizeof(uint8_t);

        // The matrix payload length depends on its own first two bytes.
        uint8_t total_pins = 0;
        if (module.module_type == Protocol::MODULE_TYPE_MATRIX) {
            eeprom_get_bytes(addr, &module.matrix, 2);
            total_pins = module.matrix.num_row_pins + module.matrix.num_col_pins;
            if (total_pins > MAX_MATRIX_PINS) {
                return false; // Invalid matrix config
            }
        }

        uint8_t payload_size = Protocol::modulePayloadSize(module.module_type, total_pins);
        if (payload_size == 0) {
            return false; // Unknown input type
        }
        eeprom_get_bytes(addr, &module.matrix, payload_size);
        addr += payload_size;

        if (module.module_type == Protocol::MODULE_TYPE_MCP23017 && !Protocol::validMcp23017(module.mcp23017)) {
            return false;
        }
        if (module.module_type == Protocol::MODULE_TYPE_ADS1115 && !Protocol::validAds1115(module.ads1115)) {
            return false;
        }
    }

    return true;
}

uint32_t getCurrentConfigId()
{
    return g_current_config_id;
}

const ModuleConfig* getCurrentConfig(uint8_t& num_modules)
{
    num_modules = g_current_num_modules;
    return g_current_modules;
}

} // namespace ConfigManager
