cmake_minimum_required(VERSION 3.13)

# --- Pico SDK import (must precede project()) ---
# Requires PICO_SDK_PATH environment variable or -DPICO_SDK_PATH=...
set(PICO_BOARD pico2 CACHE STRING "Target board — pico2 (RP2350) only")
include(pico_sdk_import.cmake)

project(lds01_emulator C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

# Initialise Pico SDK (after project())
pico_sdk_init()

add_executable(lds01_emulator
    main.cpp
    usb_descriptors.cpp
)

target_include_directories(lds01_emulator PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}   # picks up tusb_config.h
)

target_link_libraries(lds01_emulator
    pico_stdlib
    hardware_uart
    tinyusb_device
    tinyusb_board
)

# Disable SDK stdio so raw TinyUSB CDC is not polluted by printf/stdin hooks.
pico_enable_stdio_usb(lds01_emulator  0)
pico_enable_stdio_uart(lds01_emulator 0)

# Generate .uf2, .bin, .hex, .map outputs.
pico_add_extra_outputs(lds01_emulator)

# Print memory usage after linking.
add_custom_command(TARGET lds01_emulator POST_BUILD
    COMMAND arm-none-eabi-size $<TARGET_FILE:lds01_emulator>
    VERBATIM
)
