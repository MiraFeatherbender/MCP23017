cmake_minimum_required(VERSION 3.16)

# Ensure the example can use the component sources stored at the repo root.
# When building the example, tell the IDF build system to search the parent
# directory (repo root) for components by setting `EXTRA_COMPONENT_DIRS`.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(MCP23017_example)
