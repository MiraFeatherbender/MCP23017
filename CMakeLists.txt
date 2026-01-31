# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# This repository is primarily an ESP-IDF component (sources under `src/` and `include/`)
# and contains an example app under `example/`. When this CMakeLists is included
# from another ESP-IDF project's `components/` folder, we must call
# `idf_component_register()` here. When building from the repository root (top-level
# project), calling `idf_component_register()` directly from the project root causes
# the error "Called idf_component_register from a non-component directory.".
#
# Guard the component registration so the file can be used both as a component and
# as a top-level checkout. To build the example app use `idf.py -C example`.

if(NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_LIST_DIR)
    idf_component_register(
        SRCS "src/MCP23017.c"
        INCLUDE_DIRS "include"
        REQUIRES
            esp_driver_i2c
            esp_driver_gpio
            freertos
    )
else()
    # Top-level project case: include project helper and instruct to build example
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(MCP23017_example)
    message(STATUS "Top-level build: this repo is a component. To build the example run: idf.py -C example build")
endif()
