// Minimal MCP23017 scaffold implementation
#include "MCP23017.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "mcp23017";

const char *mcp23017_version(void)
{
    return "mcp23017 alpha";
}

// internal device struct
struct mcp23017_t {
    void *bus; // i2c_master_bus_handle_t
    void *dev_handles[8]; // i2c_master_dev_handle_t
    uint8_t addresses[8];
    int addr_count;
    bool owns_bus;
    // defaults flag
    bool defaults_applied;
    // register cache for each attached device (0x00..0x15)
    uint8_t reg_cache[8][0x16];
    bool reg_cache_valid[8];
};

static const uint8_t DEFAULT_REGS[] = {
    // 0x00..0x15 sequential defaults (IODIRA..OLATB)
    0xFF, // IODIRA
    0xFF, // IODIRB
    0x00, // IPOLA
    0x00, // IPOLB
    0x00, // GPINTENA
    0x00, // GPINTENB
    0x00, // DEFVALA
    0x00, // DEFVALB
    0x00, // INTCONA
    0x00, // INTCONB
    0x00, // IOCON (safe default)
    0x00, // (0x0B) IOCON alias / reserved
    0x00, // GPPUA
    0x00, // GPPUB
    0x00, // INTFA (read-only; write ignored)
    0x00, // INTFB (read-only; write ignored)
    0x00, // INTCAPA (read-only)
    0x00, // INTCAPB (read-only)
    0x00, // GPIOA
    0x00, // GPIOB
    0x00, // OLATA
    0x00, // OLATB
};

// ----------------------- Bus-keyed mutex registry -----------------------
// Serializes multi-register/block I2C transactions for all handles sharing the same bus.
#define MCP_BUS_REG_MAX 8

typedef struct {
    void *bus;
    SemaphoreHandle_t lock;
    int refcount;
} bus_entry_t;

static bus_entry_t s_bus_registry[MCP_BUS_REG_MAX];
static SemaphoreHandle_t s_bus_registry_lock = NULL;

static esp_err_t bus_registry_add(void *bus)
{
    if (!bus) return ESP_ERR_INVALID_ARG;
    if (!s_bus_registry_lock) {
        s_bus_registry_lock = xSemaphoreCreateMutex();
        if (!s_bus_registry_lock) return ESP_ERR_NO_MEM;
    }
    if (xSemaphoreTake(s_bus_registry_lock, pdMS_TO_TICKS(200)) != pdTRUE) return ESP_ERR_TIMEOUT;
    // find existing
    for (int i = 0; i < MCP_BUS_REG_MAX; ++i) {
        if (s_bus_registry[i].bus == bus) {
            s_bus_registry[i].refcount++;
            xSemaphoreGive(s_bus_registry_lock);
            return ESP_OK;
        }
    }
    // find empty slot
    for (int i = 0; i < MCP_BUS_REG_MAX; ++i) {
        if (s_bus_registry[i].bus == NULL) {
            s_bus_registry[i].bus = bus;
            s_bus_registry[i].lock = xSemaphoreCreateMutex();
            if (!s_bus_registry[i].lock) {
                s_bus_registry[i].bus = NULL;
                xSemaphoreGive(s_bus_registry_lock);
                return ESP_ERR_NO_MEM;
            }
            s_bus_registry[i].refcount = 1;
            xSemaphoreGive(s_bus_registry_lock);
            return ESP_OK;
        }
    }
    xSemaphoreGive(s_bus_registry_lock);
    return ESP_ERR_NO_MEM;
}

static void bus_registry_release(void *bus)
{
    if (!bus || !s_bus_registry_lock) return;
    if (xSemaphoreTake(s_bus_registry_lock, pdMS_TO_TICKS(200)) != pdTRUE) return;
    for (int i = 0; i < MCP_BUS_REG_MAX; ++i) {
        if (s_bus_registry[i].bus == bus) {
            s_bus_registry[i].refcount--;
            if (s_bus_registry[i].refcount <= 0) {
                if (s_bus_registry[i].lock) vSemaphoreDelete(s_bus_registry[i].lock);
                s_bus_registry[i].lock = NULL;
                s_bus_registry[i].bus = NULL;
                s_bus_registry[i].refcount = 0;
            }
            break;
        }
    }
    xSemaphoreGive(s_bus_registry_lock);
}

static esp_err_t bus_lock(void *bus, TickType_t tmo)
{
    if (!bus) return ESP_ERR_INVALID_ARG;
    if (!s_bus_registry_lock) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_bus_registry_lock, pdMS_TO_TICKS(200)) != pdTRUE) return ESP_ERR_TIMEOUT;
    SemaphoreHandle_t m = NULL;
    for (int i = 0; i < MCP_BUS_REG_MAX; ++i) {
        if (s_bus_registry[i].bus == bus) { m = s_bus_registry[i].lock; break; }
    }
    xSemaphoreGive(s_bus_registry_lock);
    if (!m) return ESP_ERR_NOT_FOUND;
    if (xSemaphoreTake(m, tmo) != pdTRUE) return ESP_ERR_TIMEOUT;
    return ESP_OK;
}

static void bus_unlock(void *bus)
{
    if (!bus || !s_bus_registry_lock) return;
    if (xSemaphoreTake(s_bus_registry_lock, pdMS_TO_TICKS(200)) != pdTRUE) return;
    SemaphoreHandle_t m = NULL;
    for (int i = 0; i < MCP_BUS_REG_MAX; ++i) {
        if (s_bus_registry[i].bus == bus) { m = s_bus_registry[i].lock; break; }
    }
    xSemaphoreGive(s_bus_registry_lock);
    if (m) xSemaphoreGive(m);
}

// Internal low-level helpers using i2c_master APIs
static esp_err_t read8_device(void *dev, uint8_t reg, uint8_t *out)
{
    if (!dev || !out) return ESP_ERR_INVALID_ARG;
    return i2c_master_transmit_receive((i2c_master_dev_handle_t)dev, &reg, 1, out, 1, pdMS_TO_TICKS(200));
}

// Block read: send start reg then read `len` bytes into buf in a single I2C transaction
static esp_err_t readn_device(void *dev, uint8_t start_reg, uint8_t *buf, size_t len, TickType_t tmo)
{
    if (!dev || !buf || len == 0) return ESP_ERR_INVALID_ARG;
    return i2c_master_transmit_receive((i2c_master_dev_handle_t)dev, &start_reg, 1, buf, len, tmo);
}

// Populate the register cache for a specific device index. Reads 0x00..0x15
// in a single block under the bus mutex and marks the cache valid on success.
static esp_err_t populate_register_cache(mcp23017_handle_t h, int dev_idx)
{
    if (!h || dev_idx < 0 || dev_idx >= h->addr_count) return ESP_ERR_INVALID_ARG;
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)h->dev_handles[dev_idx];
    if (!dev) return ESP_ERR_INVALID_ARG;
    const uint8_t start_reg = 0x00;
    const size_t n = 0x16;
    esp_err_t r = bus_lock(h->bus, pdMS_TO_TICKS(500));
    if (r != ESP_OK) return r;
    r = readn_device(dev, start_reg, h->reg_cache[dev_idx], n, pdMS_TO_TICKS(500));
    if (r == ESP_OK) h->reg_cache_valid[dev_idx] = true;
    bus_unlock(h->bus);
    return r;
}

// Public: re-populate the register cache for a device index
esp_err_t mcp23017_sync_registers(mcp23017_handle_t h, int dev_idx)
{
    return populate_register_cache(h, dev_idx);
}

// Public: invalidate the register cache for a device index
esp_err_t mcp23017_invalidate_register_cache(mcp23017_handle_t h, int dev_idx)
{
    if (!h || dev_idx < 0 || dev_idx >= h->addr_count) return ESP_ERR_INVALID_ARG;
    h->reg_cache_valid[dev_idx] = false;
    return ESP_OK;
}

static esp_err_t write8_device(void *dev, uint8_t reg, uint8_t val)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit((i2c_master_dev_handle_t)dev, buf, 2, pdMS_TO_TICKS(200));
}

// Create a handle from an existing bus and a list of addresses (already attached)
static mcp23017_handle_t handle_create_from_devices(void *bus, uint8_t *addrs, i2c_master_dev_handle_t *devs, int count, bool owns_bus)
{
    mcp23017_handle_t h = calloc(1, sizeof(struct mcp23017_t));
    if (!h) return NULL;
    h->bus = bus;
    h->addr_count = count;
    h->owns_bus = owns_bus;
    for (int i = 0; i < count; ++i) {
        h->addresses[i] = addrs[i];
        h->dev_handles[i] = devs[i];
    }
    h->defaults_applied = false;
    // register bus for mutexing multi-register operations
    (void)bus_registry_add(h->bus);
    return h;
}

esp_err_t mcp23017_reg_read8(mcp23017_handle_t h, int dev_idx, uint8_t reg_addr, uint8_t *value)
{
    if (!h || dev_idx < 0 || dev_idx >= h->addr_count) return ESP_ERR_INVALID_ARG;
    // Use register cache for configuration registers (0x00..0x0D) when valid.
    if (reg_addr < MCP_REG_INTFA && h->reg_cache_valid[dev_idx]) {
        *value = h->reg_cache[dev_idx][reg_addr];
        return ESP_OK;
    }
    // Otherwise perform a live read under the bus lock
    esp_err_t r = bus_lock(h->bus, pdMS_TO_TICKS(300));
    if (r != ESP_OK) return r;
    r = read8_device(h->dev_handles[dev_idx], reg_addr, value);
    bus_unlock(h->bus);
    return r;
}

// Block read wrapper exposed in header
esp_err_t mcp23017_reg_read_block(mcp23017_handle_t h, int dev_idx, uint8_t reg_addr, uint8_t *buf, size_t len)
{
    if (!h || dev_idx < 0 || dev_idx >= h->addr_count) return ESP_ERR_INVALID_ARG;
    // If the requested block is fully within the cached range and cache is valid,
    // copy from cache to avoid I2C transfer.
    const size_t CACHE_N = 0x16;
    if (reg_addr < MCP_REG_INTFA && (size_t)reg_addr + len <= CACHE_N && h->reg_cache_valid[dev_idx]) {
        memcpy(buf, &h->reg_cache[dev_idx][reg_addr], len);
        return ESP_OK;
    }
    esp_err_t r = bus_lock(h->bus, pdMS_TO_TICKS(500));
    if (r != ESP_OK) return r;
    r = readn_device(h->dev_handles[dev_idx], reg_addr, buf, len, pdMS_TO_TICKS(300));
    bus_unlock(h->bus);
    return r;
}

esp_err_t mcp23017_reg_write8(mcp23017_handle_t h, int dev_idx, uint8_t reg_addr, uint8_t value)
{
    if (!h || dev_idx < 0 || dev_idx >= h->addr_count) return ESP_ERR_INVALID_ARG;
    esp_err_t r = bus_lock(h->bus, pdMS_TO_TICKS(300));
    if (r != ESP_OK) return r;
    r = write8_device(h->dev_handles[dev_idx], reg_addr, value);
    if (r == ESP_OK) {
        if (reg_addr < MCP_REG_INTFA) {
            h->reg_cache[dev_idx][reg_addr] = value;
            h->reg_cache_valid[dev_idx] = true;
        }
    }
    bus_unlock(h->bus);
    return r;
}

// Helper to attach a device at address 'a' to bus and return dev handle (or NULL)
static i2c_master_dev_handle_t attach_device(void *bus, uint8_t a, int freq_hz)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = a,
        .scl_speed_hz = freq_hz > 0 ? freq_hz : 400000,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device((i2c_master_bus_handle_t)bus, &dev_cfg, &dev) == ESP_OK) {
        return dev;
    }
    return NULL;
}

// Discover devices on an existing bus: try addresses 0x20..0x27 and attach devices that respond
static esp_err_t discover_devices_on_bus(void *bus, uint8_t *out_addrs, i2c_master_dev_handle_t *out_devs, int *out_count)
{
    if (!bus || !out_addrs || !out_devs || !out_count) return ESP_ERR_INVALID_ARG;
    int found = 0;
    for (uint8_t a = 0x20; a <= 0x27 && found < 8; ++a) {
        i2c_master_dev_handle_t dev = attach_device(bus, a, 400000);
        if (!dev) continue;
        // quick probe: read IOCON (0x0A) to confirm device presence
        uint8_t reg = 0x0A;
        uint8_t tmp;
        if (i2c_master_transmit_receive(dev, &reg, 1, &tmp, 1, pdMS_TO_TICKS(200)) == ESP_OK) {
            out_addrs[found] = a;
            out_devs[found] = dev;
            found++;
        } else {
            i2c_master_bus_rm_device(dev);
        }
    }
    *out_count = found;
    return ESP_OK;
}

// Apply datasheet defaults quickly by writing sequential block starting at 0x00 (IODIRA)
// Apply defaults to device using the device handle index from our mcp handle so
// we can serialize with the bus mutex. This writes the DEFAULT_REGS sequential
// in a single I2C transaction under the bus lock.
static esp_err_t apply_defaults_to_device(mcp23017_handle_t h, int dev_idx)
{
    if (!h || dev_idx < 0 || dev_idx >= h->addr_count) return ESP_ERR_INVALID_ARG;
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)h->dev_handles[dev_idx];
    if (!dev) return ESP_ERR_INVALID_ARG;
    const uint8_t start_reg = 0x00;
    const size_t n = sizeof(DEFAULT_REGS);
    uint8_t *buf = malloc(n + 1);
    if (!buf) return ESP_ERR_NO_MEM;
    buf[0] = start_reg;
    memcpy(buf + 1, DEFAULT_REGS, n);
    esp_err_t rc = bus_lock(h->bus, pdMS_TO_TICKS(500));
    if (rc != ESP_OK) { free(buf); return rc; }
    rc = i2c_master_transmit(dev, buf, n + 1, pdMS_TO_TICKS(500));
    bus_unlock(h->bus);
    free(buf);
    return rc;
}

// delete handle
esp_err_t mcp23017_delete(mcp23017_handle_t h)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    for (int i = 0; i < h->addr_count; ++i) {
        if (h->dev_handles[i]) i2c_master_bus_rm_device((i2c_master_dev_handle_t)h->dev_handles[i]);
    }
    // release bus registry entry created at handle creation
    if (h->bus) bus_registry_release(h->bus);
    if (h->owns_bus && h->bus) i2c_del_master_bus((i2c_master_bus_handle_t)h->bus);
    free(h);
    return ESP_OK;
}

// Updated high-level helpers that operate on handle
esp_err_t mcp23017_port_read(void *handle, int dev_idx, mcp23017_port_t port, uint8_t *value)
{
    if (!handle || !value) return ESP_ERR_INVALID_ARG;
    return mcp23017_reg_read8((mcp23017_handle_t)handle, dev_idx, (port==MCP_PORT_A)?MCP_REG_GPIOA:MCP_REG_GPIOB, value);
}

esp_err_t mcp23017_port_write(void *handle, int dev_idx, mcp23017_port_t port, uint8_t value)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    return mcp23017_reg_write8((mcp23017_handle_t)handle, dev_idx, (port==MCP_PORT_A)?MCP_REG_OLATA:MCP_REG_OLATB, value);
}

esp_err_t mcp23017_port_masked_write(void *handle, int dev_idx, mcp23017_port_t port, uint8_t mask, uint8_t value)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    uint8_t reg = (port==MCP_PORT_A)?MCP_REG_GPIOA:MCP_REG_GPIOB;
    uint8_t cur = 0;
    esp_err_t r = mcp23017_reg_read8((mcp23017_handle_t)handle, dev_idx, reg, &cur);
    if (r != ESP_OK) return r;
    uint8_t next = (cur & (uint8_t)(~mask)) | (value & mask);
    // write to OLAT to update outputs
    uint8_t reg_olat = (port==MCP_PORT_A)?MCP_REG_OLATA:MCP_REG_OLATB;
    return mcp23017_reg_write8((mcp23017_handle_t)handle, dev_idx, reg_olat, next);
}

esp_err_t mcp23017_port_set_dir(void *handle, int dev_idx, mcp23017_port_t port, uint8_t dir_mask)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    uint8_t reg = (port == MCP_PORT_A) ? MCP_REG_IODIRA : MCP_REG_IODIRB;
    return mcp23017_reg_write8((mcp23017_handle_t)handle, dev_idx, reg, dir_mask);
}

esp_err_t mcp23017_port_set_pullup(void *handle, int dev_idx, mcp23017_port_t port, uint8_t mask, mcp23017_pull_t pull)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    uint8_t reg = (port == MCP_PORT_A) ? MCP_REG_GPPUA : MCP_REG_GPPUB;
    if (pull == MCP_PULLUP_ENABLE) {
        return mcp23017_reg_write8((mcp23017_handle_t)handle, dev_idx, reg, mask);
    } else {
        uint8_t cur = 0;
        esp_err_t r = mcp23017_reg_read8((mcp23017_handle_t)handle, dev_idx, reg, &cur);
        if (r != ESP_OK) return r;
        cur &= (uint8_t)(~mask);
        return mcp23017_reg_write8((mcp23017_handle_t)handle, dev_idx, reg, cur);
    }
}

esp_err_t mcp23017_read_gpio16(void *handle, int dev_idx, uint16_t *value)
{
    if (!handle || !value) return ESP_ERR_INVALID_ARG;
    uint8_t a,b;
    esp_err_t r1 = mcp23017_reg_read8((mcp23017_handle_t)handle, dev_idx, MCP_REG_GPIOA, &a);
    if (r1 != ESP_OK) return r1;
    esp_err_t r2 = mcp23017_reg_read8((mcp23017_handle_t)handle, dev_idx, MCP_REG_GPIOB, &b);
    if (r2 != ESP_OK) return r2;
    *value = (((uint16_t)a) << 8) | b;
    return ESP_OK;
}

// Inline bit-reverse helper (compact, used to initialize lookup table)
static inline uint8_t rev8_inline(uint8_t x)
{
    x = (x >> 4) | (x << 4);
    x = (uint8_t)(((x & 0xCC) >> 2) | ((x & 0x33) << 2));
    x = (uint8_t)(((x & 0xAA) >> 1) | ((x & 0x55) << 1));
    return x;
}

// Lazily-initialized 256-entry reverse lookup table
static uint8_t rev8_table[256];
static bool rev8_table_inited = false;
static void rev8_table_init(void)
{
    if (rev8_table_inited) return;
    for (int i = 0; i < 256; ++i) rev8_table[i] = rev8_inline((uint8_t)i);
    rev8_table_inited = true;
}

// Read a port and return the reversed-bit value via out_rev
esp_err_t mcp23017_port_read_reversed(void *handle, int dev_idx, mcp23017_port_t port, uint8_t *out_rev)
{
    if (!handle || !out_rev) return ESP_ERR_INVALID_ARG;
    uint8_t v = 0;
    esp_err_t r = mcp23017_port_read(handle, dev_idx, port, &v);
    if (r != ESP_OK) return r;
    rev8_table_init();
    *out_rev = rev8_table[v];
    return ESP_OK;
}

// Reverse bits of an 8-bit value in-place. Caller must pass non-NULL pointer.
void mcp23017_reverse8_inplace(uint8_t *value)
{
    if (!value) return;
    rev8_table_init();
    *value = rev8_table[*value];
}

esp_err_t mcp23017_write_gpio16(void *handle, int dev_idx, uint16_t value)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    uint8_t a = (uint8_t)((value >> 8) & 0xFF);
    uint8_t b = (uint8_t)(value & 0xFF);
    esp_err_t r1 = mcp23017_reg_write8((mcp23017_handle_t)handle, dev_idx, MCP_REG_OLATA, a);
    if (r1 != ESP_OK) return r1;
    return mcp23017_reg_write8((mcp23017_handle_t)handle, dev_idx, MCP_REG_OLATB, b);
}

esp_err_t mcp23017_auto_setup(mcp23017_attached_devices_t *out_devices, bool apply_defaults, const i2c_master_bus_config_t *user_bus_cfg)
{
    if (!out_devices) return ESP_ERR_INVALID_ARG;
    memset(out_devices, 0, sizeof(*out_devices));

    const int AUTO_SETUP_RETRIES = 50;
    const int AUTO_SETUP_DELAY_MS = 200;

    i2c_master_bus_handle_t bus = NULL;
    bool created_bus = false;
    int attempt = 0;

    // If the caller provided a bus config, create the bus directly and skip auto-detection.
    if (user_bus_cfg) {
        esp_err_t rc = i2c_new_master_bus(user_bus_cfg, &bus);
        if (rc != ESP_OK || !bus) {
            ESP_LOGW(TAG, "Failed to create I2C master bus from provided config: 0x%X", rc);
            return rc != ESP_OK ? rc : ESP_ERR_NOT_FOUND;
        }
        created_bus = true;
        ESP_LOGI(TAG, "Created new I2C master bus from provided config (port=%d SDA=%d SCL=%d)", user_bus_cfg->i2c_port, user_bus_cfg->sda_io_num, user_bus_cfg->scl_io_num);
    } else {
        // Auto-detect existing I2C master buses. Prefer I2C_NUM_0 then I2C_NUM_1.
        attempt = 0;
        while (attempt < AUTO_SETUP_RETRIES) {
            if (i2c_master_get_bus_handle(I2C_NUM_0, &bus) == ESP_OK) break;
            if (i2c_master_get_bus_handle(I2C_NUM_1, &bus) == ESP_OK) break;
            attempt++;
            if (attempt >= AUTO_SETUP_RETRIES) break;
            ESP_LOGI(TAG, "mcp23017: waiting for I2C bus (attempt %d/%d)", attempt+1, AUTO_SETUP_RETRIES);
            vTaskDelay(pdMS_TO_TICKS(AUTO_SETUP_DELAY_MS));
        }

        if (!bus) {
            ESP_LOGW(TAG, "No existing I2C master bus found after %d attempt(s); not creating a default bus", AUTO_SETUP_RETRIES);
            return ESP_ERR_NOT_FOUND;
        }
        // bus found via auto-detect; we do not own it
        created_bus = false;
        ESP_LOGI(TAG, "Using existing I2C master bus handle for discovery");
    }

    uint8_t addrs[8];
    i2c_master_dev_handle_t devs[8];
    int found = 0;
    esp_err_t r = ESP_OK;
    attempt = 0;
    while (attempt < AUTO_SETUP_RETRIES) {
        r = discover_devices_on_bus(bus, addrs, devs, &found);
        if (r != ESP_OK) return r;
        if (found > 0) break;
        attempt++;
        if (attempt >= AUTO_SETUP_RETRIES) break;
        ESP_LOGI(TAG, "mcp23017: no devices found, retrying discovery (%d/%d)", attempt+1, AUTO_SETUP_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(AUTO_SETUP_DELAY_MS));
    }
    if (found == 0) {
        ESP_LOGI(TAG, "No MCP23017 devices discovered on bus after %d attempt(s)", AUTO_SETUP_RETRIES);
        return ESP_ERR_NOT_FOUND;
    }

    mcp23017_handle_t h = handle_create_from_devices(bus, addrs, devs, found, false);
    if (!h) {
        // cleanup attached devs
        for (int i = 0; i < found; ++i) if (devs[i]) i2c_master_bus_rm_device(devs[i]);
        return ESP_ERR_NO_MEM;
    }

    out_devices->handle = h;
    out_devices->bus = (void*)bus;
    out_devices->addr_count = found;
    out_devices->owns_bus = created_bus;
    for (int i = 0; i < found; ++i) out_devices->addresses[i] = addrs[i];
    out_devices->defaults_applied = false;

    if (apply_defaults) {
        bool all_ok = true;
        for (int i = 0; i < found; ++i) {
            esp_err_t rc = apply_defaults_to_device(h, i);
            if (rc != ESP_OK) {
                ESP_LOGW(TAG, "Failed to apply defaults to device 0x%02X: %d", addrs[i], rc);
                all_ok = false;
            }
        }
        out_devices->defaults_applied = all_ok;
    }

    // Populate register cache for each attached device so subsequent RMWs
    // can use the local mirror instead of extra I2C transactions.
    for (int i = 0; i < found; ++i) {
        esp_err_t rc = populate_register_cache(h, i);
        if (rc != ESP_OK) {
            ESP_LOGW(TAG, "Failed to populate register cache for device 0x%02X: 0x%X", addrs[i], rc);
            // continue - cache will be invalid and driver will fall back to live reads
        }
    }

    ESP_LOGI(TAG, "discovered %d MCP23017 device(s)", found);
    return ESP_OK;
}

// Interrupt helpers
esp_err_t mcp23017_set_interrupt_mode(mcp23017_handle_t h, int dev_idx, mcp23017_port_t port, uint8_t mask, mcp23017_int_mode_t mode)
{
    if (!h || dev_idx < 0 || dev_idx >= h->addr_count) return ESP_ERR_INVALID_ARG;
    uint8_t reg_gpinten = (port==MCP_PORT_A)?MCP_REG_GPINTENA:MCP_REG_GPINTENB;
    uint8_t reg_intcon  = (port==MCP_PORT_A)?MCP_REG_INTCONA:MCP_REG_INTCONB;
    uint8_t reg_defval  = (port==MCP_PORT_A)?MCP_REG_DEFVALA:MCP_REG_DEFVALB;

    uint8_t cur_gpinten=0, cur_intcon=0, cur_defval=0;
    esp_err_t r;
    r = mcp23017_reg_read8(h, dev_idx, reg_gpinten, &cur_gpinten);
    if (r != ESP_OK) return r;
    r = mcp23017_reg_read8(h, dev_idx, reg_intcon, &cur_intcon);
    if (r != ESP_OK) return r;
    r = mcp23017_reg_read8(h, dev_idx, reg_defval, &cur_defval);
    if (r != ESP_OK) return r;

    switch (mode) {
        case MCP_INT_ANYEDGE:
            // INTCON bits = 0 (compare to previous), enable bits in GPINTEN
            cur_intcon &= (uint8_t)(~mask);
            cur_gpinten |= mask;
            break;
        case MCP_INT_POSEDGE:
            // INTCON bits = 1 (compare to DEFVAL), DEFVAL bits = 0 => rising edge
            cur_intcon |= mask;
            cur_defval &= (uint8_t)(~mask);
            cur_gpinten |= mask;
            break;
        case MCP_INT_NEGEDGE:
            // INTCON bits = 1, DEFVAL bits = 1 => falling edge
            cur_intcon |= mask;
            cur_defval |= mask;
            cur_gpinten |= mask;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    // Write back DEFVAL, INTCON, GPINTEN (order shouldn't matter much)
    r = mcp23017_reg_write8(h, dev_idx, reg_defval, cur_defval);
    if (r != ESP_OK) return r;
    r = mcp23017_reg_write8(h, dev_idx, reg_intcon, cur_intcon);
    if (r != ESP_OK) return r;
    r = mcp23017_reg_write8(h, dev_idx, reg_gpinten, cur_gpinten);
    return r;
}

esp_err_t mcp23017_set_int_polarity(mcp23017_handle_t h, int dev_idx, mcp23017_int_polarity_t polarity)
{
    if (!h || dev_idx < 0 || dev_idx >= h->addr_count) return ESP_ERR_INVALID_ARG;
    uint8_t iocon = 0;
    esp_err_t r = mcp23017_reg_read8(h, dev_idx, MCP_REG_IOCON, &iocon);
    if (r != ESP_OK) return r;
    const uint8_t INTPOL_BIT = 0x02; // IOCON.INTPOL
    const uint8_t ODR_BIT    = 0x04; // IOCON.ODR (open-drain)

    switch (polarity) {
        case MCP_INT_ACTIVE_LOW:
            iocon &= (uint8_t)~INTPOL_BIT;
            iocon &= (uint8_t)~ODR_BIT;
            break;
        case MCP_INT_ACTIVE_HIGH:
            iocon |= INTPOL_BIT;
            iocon &= (uint8_t)~ODR_BIT;
            break;
        case MCP_INT_OPENDRAIN:
            // Open-drain active-low by default
            iocon &= (uint8_t)~INTPOL_BIT;
            iocon |= ODR_BIT;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return mcp23017_reg_write8(h, dev_idx, MCP_REG_IOCON, iocon);
}

// Enable/disable IOCON.MIRROR bit to mirror interrupts between ports
esp_err_t mcp23017_set_int_mirror(mcp23017_handle_t h, int dev_idx, bool enable)
{
    if (!h || dev_idx < 0 || dev_idx >= h->addr_count) return ESP_ERR_INVALID_ARG;
    uint8_t iocon = 0;
    esp_err_t r = mcp23017_reg_read8(h, dev_idx, MCP_REG_IOCON, &iocon);
    if (r != ESP_OK) return r;
    const uint8_t MIRROR_BIT = 0x40; // IOCON.MIRROR (per datasheet)
    if (enable) iocon |= MIRROR_BIT; else iocon &= (uint8_t)(~MIRROR_BIT);
    return mcp23017_reg_write8(h, dev_idx, MCP_REG_IOCON, iocon);
}

// Configure a port (mask) with combined settings: direction, pull-up, interrupt mode/polarity, and initial level for outputs.
// If cfg->flags has MCP_CFG_BATCH_WRITE set, attempt a sequential read of registers 0x00..0x15, modify relevant bytes,
// and write them back in a single sequential write. Falls back to individual register RMW calls.
esp_err_t mcp23017_config_port(mcp23017_handle_t h, int dev_idx, const mcp23017_pin_cfg_t *cfg)
{
    if (!h || !cfg) return ESP_ERR_INVALID_ARG;
    if (dev_idx < 0 || dev_idx >= h->addr_count) return ESP_ERR_INVALID_ARG;

    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)h->dev_handles[dev_idx];

    uint8_t iodir_reg = (cfg->port==MCP_PORT_A)?MCP_REG_IODIRA:MCP_REG_IODIRB;
    uint8_t gppu_reg  = (cfg->port==MCP_PORT_A)?MCP_REG_GPPUA:MCP_REG_GPPUB;
    uint8_t olat_reg  = (cfg->port==MCP_PORT_A)?MCP_REG_OLATA:MCP_REG_OLATB;
    uint8_t intcon_reg= (cfg->port==MCP_PORT_A)?MCP_REG_INTCONA:MCP_REG_INTCONB;
    uint8_t defval_reg= (cfg->port==MCP_PORT_A)?MCP_REG_DEFVALA:MCP_REG_DEFVALB;

    // If batching requested, attempt to perform a block write using the
    // local register cache when available to avoid extra I2C RMWs.
    if (cfg->flags & MCP_CFG_BATCH_WRITE) {
        const uint8_t start_reg = 0x00;
        const size_t n = 0x16; // 0x00..0x15 inclusive

        // If cache valid, modify it in-memory and write sequentially under bus lock.
        if (h->reg_cache_valid[dev_idx]) {
            uint8_t *buf = malloc(n);
            if (!buf) return ESP_ERR_NO_MEM;
            // copy current cache
            memcpy(buf, h->reg_cache[dev_idx], n);

            // Modify IODIR
            uint8_t cur_iodir = buf[iodir_reg];
            if (cfg->pin_mode == MCP_PIN_INPUT) cur_iodir |= cfg->mask; else cur_iodir &= (uint8_t)(~cfg->mask);
            buf[iodir_reg] = cur_iodir;
            // Modify GPPU
            uint8_t cur_gppu = buf[gppu_reg];
            if (cfg->pullup == MCP_PULLUP_ENABLE) cur_gppu |= cfg->mask; else cur_gppu &= (uint8_t)(~cfg->mask);
            buf[gppu_reg] = cur_gppu;
            // Modify OLAT for initial outputs
            uint8_t cur_olat = buf[olat_reg];
            cur_olat = (cur_olat & (uint8_t)(~cfg->mask)) | (cfg->initial_level & cfg->mask);
            buf[olat_reg] = cur_olat;
            // Modify INTCON/DEFVAL if interrupt mode requested
            if (cfg->int_mode != MCP_INT_NONE) {
                uint8_t cur_intcon = buf[intcon_reg];
                uint8_t cur_defval = buf[defval_reg];
                switch (cfg->int_mode) {
                    case MCP_INT_ANYEDGE:
                        cur_intcon &= (uint8_t)(~cfg->mask);
                        break;
                    case MCP_INT_POSEDGE:
                        cur_intcon |= cfg->mask;
                        cur_defval &= (uint8_t)(~cfg->mask);
                        break;
                    case MCP_INT_NEGEDGE:
                        cur_intcon |= cfg->mask;
                        cur_defval |= cfg->mask;
                        break;
                    default:
                        break;
                }
                buf[intcon_reg] = cur_intcon;
                buf[defval_reg] = cur_defval;
                // Also ensure GPINTEN bits enabled for mask
                uint8_t gpinten_reg = (cfg->port==MCP_PORT_A)?MCP_REG_GPINTENA:MCP_REG_GPINTENB;
                buf[gpinten_reg] |= cfg->mask;
            }

            // Write back sequentially under bus lock
            uint8_t *w = malloc(n + 1);
            if (!w) { free(buf); return ESP_ERR_NO_MEM; }
            w[0] = start_reg;
            memcpy(w + 1, buf, n);

            esp_err_t r = bus_lock(h->bus, pdMS_TO_TICKS(500));
            if (r == ESP_OK) {
                r = i2c_master_transmit(dev, w, n + 1, pdMS_TO_TICKS(500));
                if (r == ESP_OK) {
                    // commit cache changes
                    memcpy(h->reg_cache[dev_idx], buf, n);
                    h->reg_cache_valid[dev_idx] = true;
                }
                bus_unlock(h->bus);
            } else {
                r = ESP_ERR_TIMEOUT;
            }
            free(w);
            free(buf);
            if (r == ESP_OK) {
                // configure polarity if requested
                if (cfg->int_polarity != MCP_INT_POL_NONE) {
                    return mcp23017_set_int_polarity(h, dev_idx, cfg->int_polarity);
                }
                return ESP_OK;
            }
            // fall through to per-register path on failure
        } else {
            // Cache not valid: fall back to reading device and performing sequential RMW
            const uint8_t start_reg = 0x00;
            const size_t n = 0x16; // 0x00..0x15 inclusive
            uint8_t *buf = malloc(n);
            if (!buf) return ESP_ERR_NO_MEM;
            esp_err_t r = i2c_master_transmit_receive(dev, &start_reg, 1, buf, n, pdMS_TO_TICKS(300));
            if (r == ESP_OK) {
                // Modify and write back as before
                uint8_t cur_iodir = buf[iodir_reg];
                if (cfg->pin_mode == MCP_PIN_INPUT) cur_iodir |= cfg->mask; else cur_iodir &= (uint8_t)(~cfg->mask);
                buf[iodir_reg] = cur_iodir;
                uint8_t cur_gppu = buf[gppu_reg];
                if (cfg->pullup == MCP_PULLUP_ENABLE) cur_gppu |= cfg->mask; else cur_gppu &= (uint8_t)(~cfg->mask);
                buf[gppu_reg] = cur_gppu;
                uint8_t cur_olat = buf[olat_reg];
                cur_olat = (cur_olat & (uint8_t)(~cfg->mask)) | (cfg->initial_level & cfg->mask);
                buf[olat_reg] = cur_olat;
                if (cfg->int_mode != MCP_INT_NONE) {
                    uint8_t cur_intcon = buf[intcon_reg];
                    uint8_t cur_defval = buf[defval_reg];
                    switch (cfg->int_mode) {
                        case MCP_INT_ANYEDGE:
                            cur_intcon &= (uint8_t)(~cfg->mask);
                            break;
                        case MCP_INT_POSEDGE:
                            cur_intcon |= cfg->mask;
                            cur_defval &= (uint8_t)(~cfg->mask);
                            break;
                        case MCP_INT_NEGEDGE:
                            cur_intcon |= cfg->mask;
                            cur_defval |= cfg->mask;
                            break;
                        default:
                            break;
                    }
                    buf[intcon_reg] = cur_intcon;
                    buf[defval_reg] = cur_defval;
                    uint8_t gpinten_reg = (cfg->port==MCP_PORT_A)?MCP_REG_GPINTENA:MCP_REG_GPINTENB;
                    buf[gpinten_reg] |= cfg->mask;
                }
                uint8_t *w = malloc(n + 1);
                if (!w) { free(buf); return ESP_ERR_NO_MEM; }
                w[0] = start_reg;
                memcpy(w + 1, buf, n);
                r = i2c_master_transmit(dev, w, n + 1, pdMS_TO_TICKS(500));
                free(w);
                if (r == ESP_OK) {
                    // update cache if possible
                    if (h->reg_cache_valid[dev_idx]) memcpy(h->reg_cache[dev_idx], buf, n);
                    free(buf);
                    if (cfg->int_polarity != MCP_INT_POL_NONE) {
                        return mcp23017_set_int_polarity(h, dev_idx, cfg->int_polarity);
                    }
                    return ESP_OK;
                }
                free(buf);
            } else {
                free(buf);
            }
            // fall through to per-register path on failure
        }
    }

    // Fallback: per-register RMW using existing helpers
    esp_err_t r;
    // direction
    uint8_t cur_dir = 0;
    r = mcp23017_reg_read8(h, dev_idx, iodir_reg, &cur_dir);
    if (r != ESP_OK) return r;
    uint8_t next_dir = (cfg->pin_mode == MCP_PIN_INPUT) ? (cur_dir | cfg->mask) : (cur_dir & (uint8_t)(~cfg->mask));
    r = mcp23017_reg_write8(h, dev_idx, iodir_reg, next_dir);
    if (r != ESP_OK) return r;

    // pull-ups
    uint8_t cur_gppu = 0;
    r = mcp23017_reg_read8(h, dev_idx, gppu_reg, &cur_gppu);
    if (r != ESP_OK) return r;
    uint8_t next_gppu = (cfg->pullup == MCP_PULLUP_ENABLE) ? (cur_gppu | cfg->mask) : (cur_gppu & (uint8_t)(~cfg->mask));
    r = mcp23017_reg_write8(h, dev_idx, gppu_reg, next_gppu);
    if (r != ESP_OK) return r;

    // initial outputs (OLAT)
    if (cfg->pin_mode == MCP_PIN_OUTPUT) {
        uint8_t cur_olat = 0;
        r = mcp23017_reg_read8(h, dev_idx, olat_reg, &cur_olat);
        if (r != ESP_OK) return r;
        uint8_t next_olat = (cur_olat & (uint8_t)(~cfg->mask)) | (cfg->initial_level & cfg->mask);
        r = mcp23017_reg_write8(h, dev_idx, olat_reg, next_olat);
        if (r != ESP_OK) return r;
    }

    // interrupts
    if (cfg->int_mode != MCP_INT_NONE) {
        r = mcp23017_set_interrupt_mode(h, dev_idx, cfg->port, cfg->mask, cfg->int_mode);
        if (r != ESP_OK) return r;
    }
    if (cfg->int_polarity != MCP_INT_POL_NONE) {
        r = mcp23017_set_int_polarity(h, dev_idx, cfg->int_polarity);
        if (r != ESP_OK) return r;
    }

    return ESP_OK;
}

// ----------------------- ISR registry (GPIO -> worker tasks) -----------------------
#define MCP_ISR_MAX_GPIO 40
#define MCP_ISR_MAX_WORKERS 8

typedef struct {
    bool handler_added[MCP_ISR_MAX_GPIO];
    TaskHandle_t workers[MCP_ISR_MAX_GPIO][MCP_ISR_MAX_WORKERS];
    uint8_t worker_count[MCP_ISR_MAX_GPIO];
    uint32_t hits[MCP_ISR_MAX_GPIO];
    SemaphoreHandle_t lock;
} isr_registry_t;

static isr_registry_t s_isr_registry = { .lock = NULL };

static void IRAM_ATTR mcp23017_isr_common(void *arg)
{
    int gpio = (int)(intptr_t)arg;
    BaseType_t xHigher = pdFALSE;
    if (gpio < 0 || gpio >= MCP_ISR_MAX_GPIO) return;
    // increment hit counter (race acceptable for diagnostics)
    s_isr_registry.hits[gpio]++;
    uint8_t cnt = s_isr_registry.worker_count[gpio];
    for (uint8_t i = 0; i < cnt; ++i) {
        TaskHandle_t t = s_isr_registry.workers[gpio][i];
        if (t) vTaskNotifyGiveFromISR(t, &xHigher);
    }
    if (xHigher == pdTRUE) portYIELD_FROM_ISR();
}

esp_err_t mcp23017_isr_register(const mcp23017_isr_cfg_t *cfg, TaskHandle_t worker_task)
{
    if (!cfg || worker_task == NULL) return ESP_ERR_INVALID_ARG;
    int g = (int)cfg->int_gpio;
    if (g < 0 || g >= MCP_ISR_MAX_GPIO) return ESP_ERR_INVALID_ARG;

    // lazy-init lock
    if (!s_isr_registry.lock) {
        s_isr_registry.lock = xSemaphoreCreateMutex();
        if (!s_isr_registry.lock) return ESP_ERR_NO_MEM;
    }

    if (xSemaphoreTake(s_isr_registry.lock, pdMS_TO_TICKS(200)) != pdTRUE) return ESP_ERR_TIMEOUT;

    // avoid duplicates
    for (uint8_t i = 0; i < s_isr_registry.worker_count[g]; ++i) {
        if (s_isr_registry.workers[g][i] == worker_task) {
            xSemaphoreGive(s_isr_registry.lock);
            return ESP_OK;
        }
    }

    if (s_isr_registry.worker_count[g] >= MCP_ISR_MAX_WORKERS) {
        xSemaphoreGive(s_isr_registry.lock);
        return ESP_ERR_NO_MEM;
    }

    // Configure GPIO if handler not present
    if (!s_isr_registry.handler_added[g]) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << g),
            .mode = GPIO_MODE_INPUT,
            .intr_type = cfg->intr_type,
            .pull_up_en = (cfg->pull_mode == GPIO_PULLUP_ONLY) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            .pull_down_en = (cfg->pull_mode == GPIO_PULLDOWN_ONLY) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        };
        gpio_config(&io_conf);
        if (cfg->install_isr_service == MCP_ISR_SERVICE_YES) {
            gpio_install_isr_service(0);
        }
        gpio_isr_handler_add((gpio_num_t)g, mcp23017_isr_common, (void*)(intptr_t)g);
        s_isr_registry.handler_added[g] = true;
    }

    // add worker
    s_isr_registry.workers[g][s_isr_registry.worker_count[g]++] = worker_task;

    xSemaphoreGive(s_isr_registry.lock);
    return ESP_OK;
}

esp_err_t mcp23017_isr_unregister(const mcp23017_isr_cfg_t *cfg, TaskHandle_t worker_task)
{
    if (!cfg || worker_task == NULL) return ESP_ERR_INVALID_ARG;
    int g = (int)cfg->int_gpio;
    if (g < 0 || g >= MCP_ISR_MAX_GPIO) return ESP_ERR_INVALID_ARG;
    if (!s_isr_registry.lock) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(s_isr_registry.lock, pdMS_TO_TICKS(200)) != pdTRUE) return ESP_ERR_TIMEOUT;

    bool found = false;
    uint8_t cnt = s_isr_registry.worker_count[g];
    for (uint8_t i = 0; i < cnt; ++i) {
        if (s_isr_registry.workers[g][i] == worker_task) {
            // shift remaining
            for (uint8_t j = i; j + 1 < cnt; ++j) s_isr_registry.workers[g][j] = s_isr_registry.workers[g][j+1];
            s_isr_registry.workers[g][cnt-1] = NULL;
            s_isr_registry.worker_count[g]--;
            found = true;
            break;
        }
    }

    // if no workers left, remove handler
    if (s_isr_registry.worker_count[g] == 0 && s_isr_registry.handler_added[g]) {
        gpio_isr_handler_remove((gpio_num_t)g);
        s_isr_registry.handler_added[g] = false;
    }

    xSemaphoreGive(s_isr_registry.lock);
    return found ? ESP_OK : ESP_ERR_NOT_FOUND;
}

uint32_t mcp23017_isr_get_hits(gpio_num_t int_gpio)
{
    int g = (int)int_gpio;
    if (g < 0 || g >= MCP_ISR_MAX_GPIO) return 0;
    return s_isr_registry.hits[g];
}
