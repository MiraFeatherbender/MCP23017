idf_component_register(
    SRCS 
        "main.c"
        "io_MCP23017.C"
        
    INCLUDE_DIRS 
        "."
    
    REQUIRES 
        mcp23017
        esp_driver_i2c
        esp_driver_gpio
        freertos
)
