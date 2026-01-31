// MCP23017 - friendly API: enums and register constants
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_types.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Port enums
typedef enum {
    MCP_PORT_A = 0,
    MCP_PORT_B = 1,
} mcp23017_port_t;

// Operation mode (8-bit helpers vs 16-bit helpers)
typedef enum {
    MCP_MODE_8BIT = 0,
    MCP_MODE_16BIT = 1,
} mcp23017_mode_t;

// Pull-up enum
typedef enum {
    MCP_PULLUP_DISABLE = 0,
    MCP_PULLUP_ENABLE = 1,
} mcp23017_pull_t;

// Interrupt/shorthand modes
typedef enum {
    MCP_INT_NONE = -1,
    MCP_INT_ANYEDGE = 0,
    MCP_INT_POSEDGE = 1,
    MCP_INT_NEGEDGE = 2,
} mcp23017_int_mode_t;

// Interrupt polarity / output type
typedef enum {
    MCP_INT_POL_NONE = -1,
    MCP_INT_ACTIVE_LOW = 0,
    MCP_INT_ACTIVE_HIGH = 1,
    MCP_INT_OPENDRAIN = 2,
} mcp23017_int_polarity_t;

// Convenience: operate on full 8-bit port
#define MCP_PORT_ALL ((uint8_t)0xFF)

// Register map (BANK=0 / sequential addressing)
#define MCP_REG_IODIRA   0x00
#define MCP_REG_IODIRB   0x01
#define MCP_REG_IPOLA    0x02
#define MCP_REG_IPOLB    0x03
#define MCP_REG_GPINTENA 0x04
#define MCP_REG_GPINTENB 0x05
#define MCP_REG_DEFVALA  0x06
#define MCP_REG_DEFVALB  0x07
#define MCP_REG_INTCONA  0x08
#define MCP_REG_INTCONB  0x09
#define MCP_REG_IOCON    0x0A
#define MCP_REG_GPPUA    0x0C
#define MCP_REG_GPPUB    0x0D
#define MCP_REG_INTFA    0x0E
#define MCP_REG_INTFB    0x0F
#define MCP_REG_INTCAPA  0x10
#define MCP_REG_INTCAPB  0x11
#define MCP_REG_GPIOA    0x12
#define MCP_REG_GPIOB    0x13
#define MCP_REG_OLATA    0x14
#define MCP_REG_OLATB    0x15

// Version helper
const char *mcp23017_version(void);

// Pin mode (mirror of MCU gpio direction semantics)
typedef enum {
    MCP_PIN_INPUT = 0,
    MCP_PIN_OUTPUT = 1,
} mcp23017_pin_mode_t;

// Pin configuration struct for port-level configuration
typedef struct {
    mcp23017_port_t port;       // MCP_PORT_A or MCP_PORT_B
    uint8_t mask;               // bitmask of pins within port
    mcp23017_pin_mode_t pin_mode; // INPUT or OUTPUT
    mcp23017_pull_t pullup;     // pull-up enable/disable
    mcp23017_int_mode_t int_mode; // interrupt mode or MCP_INT_NONE
    mcp23017_int_polarity_t int_polarity; // desired polarity or MCP_INT_POL_NONE
    uint8_t initial_level;      // levels to apply for outputs (bits valid where mask=1)
    uint32_t flags;             // configuration flags (see MCP_CFG_*)
} mcp23017_pin_cfg_t;

// Flags for mcp23017_pin_cfg_t.flags
#define MCP_CFG_BATCH_WRITE (1u << 0) // perform a batched RMW using sequential access

// High-level 8-bit port helpers
// port: MCP_PORT_A or MCP_PORT_B
esp_err_t mcp23017_port_read(void *handle, int dev_idx, mcp23017_port_t port, uint8_t *value);
esp_err_t mcp23017_port_write(void *handle, int dev_idx, mcp23017_port_t port, uint8_t value);
esp_err_t mcp23017_port_masked_write(void *handle, int dev_idx, mcp23017_port_t port, uint8_t mask, uint8_t value);
esp_err_t mcp23017_port_set_dir(void *handle, int dev_idx, mcp23017_port_t port, uint8_t dir_mask);
esp_err_t mcp23017_port_set_pullup(void *handle, int dev_idx, mcp23017_port_t port, uint8_t mask, mcp23017_pull_t pull);

// 16-bit helpers that simply wrap two 8-bit operations (PORTA as high byte, PORTB as low byte)
esp_err_t mcp23017_read_gpio16(void *handle, int dev_idx, uint16_t *value);
esp_err_t mcp23017_write_gpio16(void *handle, int dev_idx, uint16_t value);

// Opaque handle
typedef struct mcp23017_t* mcp23017_handle_t;

// Collection of attached devices returned by full-auto setup
typedef struct {
    mcp23017_handle_t handle; // handle (NULL on failure)
    void *bus;                   // opaque bus handle (i2c_master_bus_handle_t)
    uint8_t addresses[8];
    int addr_count;
    bool owns_bus;
    bool defaults_applied;
} mcp23017_attached_devices_t;

// Full-auto setup: autodiscover existing I2C bus, autodiscover devices, attach them,
// optionally restore register defaults (apply_defaults=true). Returns a filled bundle on success.
// The only required argument is out_bundle (non-NULL).
//
// The function accepts an optional `i2c_master_bus_config_t *user_bus_cfg`.
// If `user_bus_cfg` is NULL the function uses the built-in defaults when creating
// a new I2C master bus. If non-NULL, the provided `i2c_master_bus_config_t` will
// be used to configure the newly-created master bus (it fully overrides defaults).
esp_err_t mcp23017_auto_setup(mcp23017_attached_devices_t *out_devices, bool apply_defaults, const i2c_master_bus_config_t *user_bus_cfg);

// Low-level reg helpers (operate on handle)
esp_err_t mcp23017_reg_read8(mcp23017_handle_t h, int dev_idx, uint8_t reg_addr, uint8_t *value);
esp_err_t mcp23017_reg_write8(mcp23017_handle_t h, int dev_idx, uint8_t reg_addr, uint8_t value);
// Block read starting at reg_addr (sequential addressing). Performs a single
// I2C transaction reading `len` bytes into `buf`.
esp_err_t mcp23017_reg_read_block(mcp23017_handle_t h, int dev_idx, uint8_t reg_addr, uint8_t *buf, size_t len);

// Lifecycle
esp_err_t mcp23017_delete(mcp23017_handle_t h);

// Interrupt helpers
// mode: MCP_INT_ANYEDGE, MCP_INT_POSEDGE, MCP_INT_NEGEDGE
esp_err_t mcp23017_set_interrupt_mode(mcp23017_handle_t h, int dev_idx, mcp23017_port_t port, uint8_t mask, mcp23017_int_mode_t mode);

// Configure INT output polarity/drive type. For MCP_INT_OPENDRAIN the helper will set the
// IOCON ODR bit and leave INTPOL cleared (open-drain active-low). ACTIVE_HIGH/A_ACTIVE_LOW
// will clear/set INTPOL and clear ODR accordingly.
esp_err_t mcp23017_set_int_polarity(mcp23017_handle_t h, int dev_idx, mcp23017_int_polarity_t polarity);

// Enable or disable INT mirror mode: when enabled the device ORs both
// port interrupt outputs onto the single INT pin(s) per datasheet IOCON.MIRROR bit.
// `enable` true sets the MIRROR bit, false clears it.
esp_err_t mcp23017_set_int_mirror(mcp23017_handle_t h, int dev_idx, bool enable);

// Explicitly re-populate the register cache for the given device index (reads 0x00..0x15).
// Use when external code may have modified device registers and the cache needs refresh.
esp_err_t mcp23017_sync_registers(mcp23017_handle_t h, int dev_idx);

// Invalidate the register cache for the given device index so subsequent calls
// will perform live reads until `mcp23017_sync_registers()` is called.
esp_err_t mcp23017_invalidate_register_cache(mcp23017_handle_t h, int dev_idx);

// Configure a port (mask) with combined settings: direction, pull-up, interrupt mode/polarity, and initial level for outputs.
// If cfg->flags has MCP_CFG_BATCH_WRITE set, attempt a sequential read of registers 0x00..0x15, modify relevant bytes,
// and write them back in a single sequential write. Falls back to individual register RMW calls
esp_err_t mcp23017_config_port(mcp23017_handle_t dev, int addr, const mcp23017_pin_cfg_t *cfg);

// ISR registration API (config-driven, DRY)
typedef enum {
    MCP_ISR_SERVICE_NO = 0,
    MCP_ISR_SERVICE_YES = 1,
} mcp_isr_service_t;

typedef struct {
    gpio_num_t int_gpio;          // MCU GPIO used for MCP INT
    gpio_pull_mode_t pull_mode;   // esp-idf enum for pull configuration
    gpio_int_type_t intr_type;    // esp-idf enum for interrupt type
    mcp_isr_service_t install_isr_service; // whether to install gpio ISR service
} mcp23017_isr_cfg_t;

/**
 * Register a worker task to be notified when the specified MCU GPIO IRQ fires.
 * The ISR will call `vTaskNotifyGiveFromISR()` on the provided worker_task.
 * Multiple workers may be registered for the same GPIO.
 */
esp_err_t mcp23017_isr_register(const mcp23017_isr_cfg_t *cfg, TaskHandle_t worker_task);

/**
 * Unregister a previously-registered worker for the given GPIO.
 */
esp_err_t mcp23017_isr_unregister(const mcp23017_isr_cfg_t *cfg, TaskHandle_t worker_task);

/**
 * Diagnostic: return number of ISR hits observed for the given MCU GPIO.
 */
uint32_t mcp23017_isr_get_hits(gpio_num_t int_gpio);
