idf_component_register(
    SRCS 
        "main.c"
        "io_MCP23017.C"
        "src/MCP23017.c"
        
    INCLUDE_DIRS 
        "include"
        "."
    
    REQUIRES 
        i2c_master
)
