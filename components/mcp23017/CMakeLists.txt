idf_component_register(
    SRCS "src/MCP23017.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_driver_gpio
        freertos
)
