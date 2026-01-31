#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

extern void io_MCP23017_start(void);

void app_main(void)
{
    ESP_LOGI("main", "Starting MCP23017 example");
    io_MCP23017_start();
    // Keep main alive
    while (1) vTaskDelay(pdMS_TO_TICKS(10000));
}
