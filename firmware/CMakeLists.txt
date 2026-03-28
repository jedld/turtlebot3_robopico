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

# Generate the PIO header from the quadrature encoder PIO program.
pico_generate_pio_header(turtlebot3_pico_fw
    ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio
)

target_link_libraries(turtlebot3_pico_fw
    pico_stdlib
    hardware_pwm
    hardware_adc
    hardware_i2c
    hardware_pio
    hardware_watchdog
    tinyusb_device
    tinyusb_board
    pico_multicore
)

# ------------- IMU type selection ----------------------------------------
# Pass -DIMU_TYPE=<n> to cmake (or set it in cmake-gui / VS Code CMake Tools)
#   0 = AUTO      try BNO08X first, fall back to BNO055, then simulated
#   1 = BNO08X    probe BNO08X (BNO085/BNO086/BNO080) via SHTP; skip BNO055
#   2 = BNO055    probe BNO055 only; skip BNO08X and its long SHTP drain
#   3 = SIMULATED skip all IMU hardware; always use simulated data
set(IMU_TYPE "1" CACHE STRING
    "IMU selection: 0=AUTO 1=BNO08X-only 2=BNO055-only 3=SIMULATED-only")
set_property(CACHE IMU_TYPE PROPERTY STRINGS 0 1 2 3)
target_compile_definitions(turtlebot3_pico_fw PRIVATE IMU_TYPE=${IMU_TYPE})
message(STATUS "IMU_TYPE=${IMU_TYPE}  (0=AUTO 1=BNO08X 2=BNO055 3=SIMULATED)")

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
