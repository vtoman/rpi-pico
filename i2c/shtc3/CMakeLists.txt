# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(shtc3 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(shtc3
        main.c
        )

# pull in common dependencies
target_link_libraries(shtc3 pico_stdlib hardware_i2c)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(shtc3)

# add url via pico_set_program_url
pico_enable_stdio_usb(shtc3 1)
pico_enable_stdio_uart(shtc3 0)

# add url via pico_set_program_url
#cmake_minimum_required(VERSION 3.13)
#include(pico_sdk_import.cmake)

#project(i2c_scan)
#pico_sdk_init()

#add_executable(i2c_scan main.c)
#target_link_libraries(i2c_scan pico_stdlib hardware_i2c)
#pico_add_extra_outputs(i2c_scan)
