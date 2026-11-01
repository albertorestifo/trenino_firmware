#include "config_manager.h"
#include "device_info.h"

// Platform-specific EEPROM instance for Arduino Due
#ifdef EEPROM_USE_DUE_FLASH
DueFlashStorage dueFlashStorage;
#endif

// Platform-agnostic EEPROM access helpers
namespace {

template <typename T>
void eeprom_put(int address, const T& value)
{
#ifdef EEPROM_USE_DUE_FLASH
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&value);
    for (size_t i = 0; i < sizeof(T); i++) {
        dueFlashStorage.write(address + i, ptr[i]);
    }
#else
    EEPROM.put(address, value);
#endif
}

template <typename T>
void eeprom_get(int address, T& value)
{
#ifdef EEPROM_USE_DUE_FLASH
    uint8_t* ptr = reinterpret_cast<uint8_t*>(&value);
    for (size_t i = 0; i < sizeof(T); i++) {
        ptr[i] = dueFlashStorage.read(address + i);
    }
#else
    EEPROM.get(address, value);
#endif
}

// Little-endian u16, byte by byte so records do not depend on struct layout.
void eeprom_put_u16(int address, uint16_t value)
{
    eeprom_put(address, uint8_t(value & 0xFF));
    eeprom_put(address + 1, uint8_t(value >> 8));
}

uint16_t eeprom_get_u16(int address)
{
    uint8_t lo, hi;
    eeprom_get(address, lo);
    eeprom_get(address + 1, hi);
    return uint16_t(lo | (hi << 8));
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

    // Write module configurations (variable size based on type)
    addr = EEPROM_INPUTS_ADDR;
    for (uint8_t i = 0; i < num_modules; i++) {
        // Write input type
        eeprom_put(addr, modules[i].module_type);
        addr += sizeof(uint8_t);

        switch (modules[i].module_type) {
        case Protocol::MODULE_TYPE_ANALOG:
            eeprom_put(addr, modules[i].analog.pin);
            addr += sizeof(uint8_t);
            eeprom_put(addr, modules[i].analog.sensitivity);
            addr += sizeof(uint8_t);
            break;

        case Protocol::MODULE_TYPE_BUTTON:
            eeprom_put(addr, modules[i].button.pin);
            addr += sizeof(uint8_t);
            eeprom_put(addr, modules[i].button.debounce);
            addr += sizeof(uint8_t);
            break;

        case Protocol::MODULE_TYPE_MATRIX: {
            eeprom_put(addr, modules[i].matrix.num_row_pins);
            addr += sizeof(uint8_t);
            eeprom_put(addr, modules[i].matrix.num_col_pins);
            addr += sizeof(uint8_t);
            uint8_t total_pins = modules[i].matrix.num_row_pins + modules[i].matrix.num_col_pins;
            for (uint8_t p = 0; p < total_pins; p++) {
                eeprom_put(addr, modules[i].matrix.pins[p]);
                addr += sizeof(uint8_t);
            }
            break;
        }

        case Protocol::MODULE_TYPE_HT16K33:
            eeprom_put(addr, modules[i].ht16k33.i2c_address);
            addr += sizeof(uint8_t);
            eeprom_put(addr, modules[i].ht16k33.brightness);
            addr += sizeof(uint8_t);
            eeprom_put(addr, modules[i].ht16k33.num_digits);
            addr += sizeof(uint8_t);
            break;

        case Protocol::MODULE_TYPE_MCP23017:
            eeprom_put(addr, modules[i].mcp23017.i2c_address); addr += 1;
            eeprom_put(addr, modules[i].mcp23017.direct_base); addr += 1;
            eeprom_put(addr, modules[i].mcp23017.matrix_base); addr += 1;
            eeprom_put(addr, modules[i].mcp23017.debounce); addr += 1;
            eeprom_put_u16(addr, modules[i].mcp23017.direct_mask); addr += 2;
            eeprom_put_u16(addr, modules[i].mcp23017.row_mask); addr += 2;
            eeprom_put_u16(addr, modules[i].mcp23017.col_mask); addr += 2;
            break;

        case Protocol::MODULE_TYPE_ADS1115:
            eeprom_put(addr, modules[i].ads1115.i2c_address); addr += 1;
            eeprom_put(addr, modules[i].ads1115.pin_base); addr += 1;
            eeprom_put(addr, modules[i].ads1115.channel_mask); addr += 1;
            eeprom_put(addr, modules[i].ads1115.gain); addr += 1;
            eeprom_put(addr, modules[i].ads1115.ref_channel); addr += 1;
            for (uint8_t c = 0; c < 4; c++) {
                eeprom_put(addr, modules[i].ads1115.sensitivity[c]); addr += 1;
            }
            break;
        }
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
        eeprom_get(addr, g_current_modules[i].module_type);
        addr += sizeof(uint8_t);

        switch (g_current_modules[i].module_type) {
        case Protocol::MODULE_TYPE_ANALOG:
            eeprom_get(addr, g_current_modules[i].analog.pin);
            addr += sizeof(uint8_t);
            eeprom_get(addr, g_current_modules[i].analog.sensitivity);
            addr += sizeof(uint8_t);
            break;

        case Protocol::MODULE_TYPE_BUTTON:
            eeprom_get(addr, g_current_modules[i].button.pin);
            addr += sizeof(uint8_t);
            eeprom_get(addr, g_current_modules[i].button.debounce);
            addr += sizeof(uint8_t);
            break;

        case Protocol::MODULE_TYPE_MATRIX: {
            eeprom_get(addr, g_current_modules[i].matrix.num_row_pins);
            addr += sizeof(uint8_t);
            eeprom_get(addr, g_current_modules[i].matrix.num_col_pins);
            addr += sizeof(uint8_t);
            uint8_t total_pins = g_current_modules[i].matrix.num_row_pins + g_current_modules[i].matrix.num_col_pins;
            if (total_pins > MAX_MATRIX_PINS) {
                return false; // Invalid matrix config
            }
            for (uint8_t p = 0; p < total_pins; p++) {
                eeprom_get(addr, g_current_modules[i].matrix.pins[p]);
                addr += sizeof(uint8_t);
            }
            break;
        }

        case Protocol::MODULE_TYPE_HT16K33:
            eeprom_get(addr, g_current_modules[i].ht16k33.i2c_address);
            addr += sizeof(uint8_t);
            eeprom_get(addr, g_current_modules[i].ht16k33.brightness);
            addr += sizeof(uint8_t);
            eeprom_get(addr, g_current_modules[i].ht16k33.num_digits);
            addr += sizeof(uint8_t);
            break;

        case Protocol::MODULE_TYPE_MCP23017: {
            Protocol::Mcp23017Config& m = g_current_modules[i].mcp23017;
            eeprom_get(addr, m.i2c_address); addr += 1;
            eeprom_get(addr, m.direct_base); addr += 1;
            eeprom_get(addr, m.matrix_base); addr += 1;
            eeprom_get(addr, m.debounce); addr += 1;
            m.direct_mask = eeprom_get_u16(addr); addr += 2;
            m.row_mask = eeprom_get_u16(addr); addr += 2;
            m.col_mask = eeprom_get_u16(addr); addr += 2;
            if (!Protocol::validMcp23017(m)) {
                return false;
            }
            break;
        }

        case Protocol::MODULE_TYPE_ADS1115: {
            Protocol::Ads1115Config& a = g_current_modules[i].ads1115;
            eeprom_get(addr, a.i2c_address); addr += 1;
            eeprom_get(addr, a.pin_base); addr += 1;
            eeprom_get(addr, a.channel_mask); addr += 1;
            eeprom_get(addr, a.gain); addr += 1;
            eeprom_get(addr, a.ref_channel); addr += 1;
            for (uint8_t c = 0; c < 4; c++) {
                eeprom_get(addr, a.sensitivity[c]); addr += 1;
            }
            if (!Protocol::validAds1115(a)) {
                return false;
            }
            break;
        }

        default:
            return false; // Unknown input type
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
