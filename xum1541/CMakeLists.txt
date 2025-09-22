cmake_minimum_required(VERSION 3.13)

# Include pico-sdk import as per standard pico-sdk project setup
include(pico_sdk_import.cmake)

# Set project name and version
project(xum1541 VERSION 0.8 LANGUAGES C CXX ASM)

# Initialize the SDK
pico_sdk_init()

# Set C standard
set(CMAKE_C_STANDARD 11)

# Firmware version (matches Makefile)
set(XUMFW_VERSION "08")

# Model selection - can be overridden with -DMODEL=RP2040
if(NOT DEFINED MODEL)
    set(MODEL "RP2040")
endif()

add_compile_definitions(
    "DEBUG=1"
    "MODEL=9"
    "__XUM1541_GIT_REVISION__=\"gitrevxx\""
    "__XUM1541_GCC_VERSION__=\"gccrevxx\""
    "__XUM1541_LIBC_VERSION__=\"newlibxx\""
)

# Optional debug support
option(ENABLE_DEBUG "Enable debug output" OFF)
if(ENABLE_DEBUG)
    add_compile_definitions(
        DEBUG
        DEBUG_LEVEL=1
    )
endif()

# Define core source files (common to all models)
set(CORE_SOURCES
    main.c
    commands.c
    iec.c
    s1.c
    s2.c
    pp.c
    p2.c
    nib.c
)

# Model-specific sources
if(MODEL STREQUAL "RP2040")
    set(MODEL_SOURCES
        board-rp2040.c
        tinyusb_usb_descriptors.c
    )
else()
    message(FATAL_ERROR "Unsupported MODEL: ${MODEL}. Only RP2040 is supported in CMake build.")
endif()

# Create the executable
add_executable(xum1541-${MODEL}-v${XUMFW_VERSION}
    ${CORE_SOURCES}
    ${MODEL_SOURCES}
)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(xum1541-${MODEL}-v${XUMFW_VERSION} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

# Link against pico-sdk libraries
target_link_libraries(xum1541-${MODEL}-v${XUMFW_VERSION}
    pico_stdlib              # Standard library (GPIO, timers, etc.)
    pico_unique_id          # Unique ID functions
    hardware_gpio           # GPIO hardware interface
    hardware_timer          # Timer hardware interface
    hardware_watchdog       # Watchdog timer
    hardware_clocks         # Clock configuration
    pico_bootrom           # Bootloader functions
    tinyusb_device         # TinyUSB device stack
    tinyusb_board          # TinyUSB board support
)

pico_enable_stdio_usb(xum1541-${MODEL}-v${XUMFW_VERSION} 0)
pico_enable_stdio_uart(xum1541-${MODEL}-v${XUMFW_VERSION} 1)

# Create map/bin/hex/uf2 outputs
pico_add_extra_outputs(xum1541-${MODEL}-v${XUMFW_VERSION})

# Set binary name to match original project convention
set_target_properties(xum1541-${MODEL}-v${XUMFW_VERSION} PROPERTIES
    OUTPUT_NAME "xum1541-${MODEL}-v${XUMFW_VERSION}"
)

# Print build information
message(STATUS "Building XUM1541 for ${MODEL}")
message(STATUS "Firmware version: ${XUMFW_VERSION}")
message(STATUS "Debug enabled: ${ENABLE_DEBUG}")

# Custom target to show memory usage (like avr-size)
add_custom_target(size
    COMMAND arm-none-eabi-size $<TARGET_FILE:xum1541-${MODEL}-v${XUMFW_VERSION}>
    DEPENDS xum1541-${MODEL}-v${XUMFW_VERSION}
    COMMENT "Showing memory usage"
)

# Build instructions comment
message(STATUS "")
message(STATUS "Build instructions:")
message(STATUS "  mkdir build && cd build")
message(STATUS "  cmake ..")
message(STATUS "  make")
message(STATUS "")
message(STATUS "Optional build flags:")
message(STATUS "  -DENABLE_DEBUG=ON     Enable debug output")
message(STATUS "")
message(STATUS "Output files will be in build/ directory:")
message(STATUS "  xum1541-${MODEL}-v${XUMFW_VERSION}.uf2   (for drag-and-drop flashing)")
message(STATUS "  xum1541-${MODEL}-v${XUMFW_VERSION}.bin   (binary file)")
message(STATUS "  xum1541-${MODEL}-v${XUMFW_VERSION}.hex   (hex file)")
message(STATUS "")
