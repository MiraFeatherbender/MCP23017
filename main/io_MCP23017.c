// Simple plugin: discover MCP23017 and poll Port B every 2s (no interrupts, no blink)
#include <stdio.h>
#include <string.h>
#include "driver/i2c_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "MCP23017.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

static const char *TAG = "io_MCP23017";

// Shared handle for the discovered MCP23017 so ISR worker can access it
static mcp23017_handle_t s_mcp_dev = NULL;
static mcp23017_attached_devices_t s_mcp_devices = {0};

// Select which MCU GPIO is used for the MCP23017 INT line.
#define MCP_INT_GPIOA GPIO_NUM_1
#define MCP_INT_GPIOB GPIO_NUM_2

// Task-notify based worker handle and minimal diagnostics
static TaskHandle_t s_mcp_gpio_worker_task = NULL;

static void mcp_gpio_isr_worker(void *arg)
{
    (void)arg;
    for (;;) {
        // Wait for a notification from the ISR (coalesces bursts)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!s_mcp_dev) {
            ESP_LOGW(TAG, "ISR worker: MCP23017 device not ready");
            continue;
        }

        // Read INTF flags for both ports to determine which port(s) triggered
        uint8_t intf_a = 0, intf_b = 0;
        esp_err_t rc = mcp23017_reg_read8(s_mcp_dev, 0, MCP_REG_INTFA, &intf_a);
        if (rc != ESP_OK) {
            ESP_LOGW(TAG, "ISR worker: INTFA read failed (0x%X)", rc);
        }
        rc = mcp23017_reg_read8(s_mcp_dev, 0, MCP_REG_INTFB, &intf_b);
        if (rc != ESP_OK) {
            ESP_LOGW(TAG, "ISR worker: INTFB read failed (0x%X)", rc);
        }

        if (intf_a == 0 && intf_b == 0) {
            // No INTF flags set — read GPIOs for diagnostics (no logging)
            uint8_t gpio_a = 0, gpio_b = 0;
            mcp23017_reg_read8(s_mcp_dev, 0, MCP_REG_INTCAPA, &gpio_a);
            mcp23017_reg_read8(s_mcp_dev, 0, MCP_REG_INTCAPB, &gpio_b);
            (void)gpio_a; (void)gpio_b;
            continue;
        }

        if (intf_a) {
            uint8_t intcap_a = 0;
            rc = mcp23017_reg_read8(s_mcp_dev, 0, MCP_REG_INTCAPA, &intcap_a);
            if (rc == ESP_OK) {
                ESP_LOGI(TAG, "MCP23017 Port A INTF=0x%02X INTCAP=0x%02X", intf_a, intcap_a);
            } else {
                ESP_LOGW(TAG, "ISR worker: INTCAPA read failed (0x%X)", rc);
            }
        }

        if (intf_b) {
            uint8_t intcap_b = 0;
            rc = mcp23017_reg_read8(s_mcp_dev, 0, MCP_REG_INTCAPB, &intcap_b);
            if (rc == ESP_OK) {
                ESP_LOGI(TAG, "MCP23017 Port B INTF=0x%02X INTCAP=0x%02X", intf_b, intcap_b);
            } else {
                ESP_LOGW(TAG, "ISR worker: INTCAPB read failed (0x%X)", rc);
            }
        }

    }
}

static void io_mcp23017_task(void *arg)
{
    (void)arg;

        // use auto-setup to probe existing I2C bus and attach devices
        // request that the component apply datasheet defaults to discovered devices
        // allow retries and delay similar to previous behavior in this plugin
    
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = 9,
        .sda_io_num = 8,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags{
            enable_internal_pullup = 1,
        },
    };
    ESP_LOGI(TAG, "Using example I2C bus config: SDA=%d SCL=%d", bus_cfg.sda_io_num, bus_cfg.scl_io_num);

    esp_err_t rc = mcp23017_auto_setup(&s_mcp_devices, true, &bus_cfg);
    if (rc != ESP_OK || s_mcp_devices.handle == NULL) {
        ESP_LOGE(TAG, "Failed to auto-setup MCP23017: 0x%X", rc);
        vTaskDelete(NULL);
        return;
    }

    s_mcp_dev = s_mcp_devices.handle;

    // The auto_setup applied datasheet defaults; now configure Port B with a batched RMW
    mcp23017_pin_cfg_t cfg = {
        .port = MCP_PORT_B,
        .mask = MCP_PORT_ALL,
        .pin_mode = MCP_PIN_INPUT,
        .pullup = MCP_PULLUP_ENABLE,
        .int_mode = MCP_INT_ANYEDGE,
        .int_polarity = MCP_INT_OPENDRAIN,
        .initial_level = 0x00,
        .flags = MCP_CFG_BATCH_WRITE,
    };
    (void)mcp23017_config_port(s_mcp_dev, 0, &cfg);

    // The auto_setup applied datasheet defaults; now configure Port A with a batched RMW
    cfg.port = MCP_PORT_A;
    cfg.mask = 0xAA; // even pins interrupts only
    cfg.pin_mode = MCP_PIN_INPUT;
    cfg.pullup = MCP_PULLUP_ENABLE;
    cfg.int_mode = MCP_INT_ANYEDGE;
    cfg.int_polarity = MCP_INT_OPENDRAIN;
    cfg.initial_level = 0x00;
    cfg.flags = MCP_CFG_BATCH_WRITE;

    (void)mcp23017_config_port(s_mcp_dev, 0, &cfg);

    cfg.port = MCP_PORT_A;
    cfg.mask = 0x55; // odd pins interrupts off
    cfg.pin_mode = MCP_PIN_INPUT;
    cfg.pullup = MCP_PULLUP_ENABLE;
    cfg.int_mode = MCP_INT_NONE;
    cfg.int_polarity = MCP_INT_OPENDRAIN;
    cfg.initial_level = 0x00;
    cfg.flags = MCP_CFG_BATCH_WRITE;
    (void)mcp23017_config_port(s_mcp_dev, 0, &cfg);
    
    // MCP device initialized; ISR worker will report INTF/INTCAP when GPIO ISR fires.
    // Keep this task alive but idle as the ISR-driven worker now handles interrupt reporting.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

void io_MCP23017_start(void)
{
    ESP_LOGI(TAG, "io_MCP23017_start() called");
    
    // Configure MCU INT GPIO and register our worker with the ISR registry
    ESP_LOGI(TAG, "Using MCU INT GPIO %d", (int)MCP_INT_GPIOB);

    mcp23017_isr_cfg_t isr_cfg = {
        .int_gpio = MCP_INT_GPIOB,
        .pull_mode = GPIO_PULLUP_ONLY,
        .intr_type = GPIO_INTR_NEGEDGE,
        .install_isr_service = MCP_ISR_SERVICE_YES,
    };

    BaseType_t created = xTaskCreate(mcp_gpio_isr_worker, "mcp_gpio_worker", 4096, NULL, 6, &s_mcp_gpio_worker_task);
    ESP_LOGI(TAG, "mcp_gpio_worker task create -> %ld", (long)created);
    if (created == pdPASS) {
        esp_err_t reg = mcp23017_isr_register(&isr_cfg, s_mcp_gpio_worker_task);
        ESP_LOGI(TAG, "mcp23017_isr_register() -> 0x%X", reg);
        if (reg != ESP_OK) {
            ESP_LOGW(TAG, "mcp23017_isr_register() failed: 0x%X", reg);
        }
    } else {
        ESP_LOGW(TAG, "Failed to create ISR worker task; ISR disabled (create=%ld)", (long)created);
    }

    // Also register same task for PORTA

    ESP_LOGI(TAG, "Using MCU INT GPIO %d", (int)MCP_INT_GPIOA);

    if (created == pdPASS) {
        isr_cfg.int_gpio = MCP_INT_GPIOA;
        esp_err_t reg = mcp23017_isr_register(&isr_cfg, s_mcp_gpio_worker_task);
        ESP_LOGI(TAG, "mcp23017_isr_register() -> 0x%X", reg);
        if (reg != ESP_OK) {
            ESP_LOGW(TAG, "mcp23017_isr_register() failed: 0x%X", reg);
        }
    } else {
        ESP_LOGW(TAG, "Failed to create ISR worker task; ISR disabled (create=%ld)", (long)created);
    }
    xTaskCreate(io_mcp23017_task, "io_mcp23017", 4096, NULL, 5, NULL);
}
