cmake_minimum_required(VERSION 3.13)

# --- Pico SDK import (must be before project()) ---
# Requires PICO_SDK_PATH environment variable OR pass -DPICO_SDK_PATH=...
# Requires PICO_BOARD=pico2 (set below or in environment)
set(PICO_BOARD pico2 CACHE STRING "TurtleBot3 target board")
include(pico_sdk_import.cmake)

project(turtlebot3_pico_fw C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

# Initialise Pico SDK (after project())
pico_sdk_init()

add_executable(turtlebot3_pico_fw
    main.c
    usb_descriptors.c
)

target_include_directories(turtlebot3_pico_fw PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}   # finds tusb_config.h
)

target_link_libraries(turtlebot3_pico_fw
    pico_stdlib
    hardware_pwm
    hardware_adc
    hardware_i2c
    hardware_watchdog
    tinyusb_device
    tinyusb_board
    pico_multicore
)

# ------------- IMU type selection ----------------------------------------
# Pass -DIMU_TYPE=<n> to cmake (or set it in cmake-gui / VS Code CMake Tools)
#   0 = AUTO      try BNO085 first, fall back to BNO055, then simulated
#   1 = BNO085    probe BNO085 only; skip BNO055 init
#   2 = BNO055    probe BNO055 only; skip BNO085 and its long SHTP drain (default)
#   3 = SIMULATED skip all IMU hardware; always use simulated data
set(IMU_TYPE "2" CACHE STRING
    "IMU selection: 0=AUTO 1=BNO085-only 2=BNO055-only 3=SIMULATED-only")
set_property(CACHE IMU_TYPE PROPERTY STRINGS 0 1 2 3)
target_compile_definitions(turtlebot3_pico_fw PRIVATE IMU_TYPE=${IMU_TYPE})
message(STATUS "IMU_TYPE=${IMU_TYPE}  (0=AUTO 1=BNO085 2=BNO055 3=SIMULATED)")

# Disable the SDK's default stdio so our raw TinyUSB CDC is not shared
pico_enable_stdio_usb(turtlebot3_pico_fw  0)
pico_enable_stdio_uart(turtlebot3_pico_fw 0)

# Generate .uf2, .bin, .hex, .map output files
pico_add_extra_outputs(turtlebot3_pico_fw)

# Optional: print memory usage summary after linking
add_custom_command(TARGET turtlebot3_pico_fw POST_BUILD
    COMMAND arm-none-eabi-size $<TARGET_FILE:turtlebot3_pico_fw>
    VERBATIM
)
