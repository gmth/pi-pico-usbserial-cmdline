cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(cmake/pico_sdk_import.cmake)

SET(CMAKE_EXPORT_COMPILE_COMMANDS on)

project(uart_bridge C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_subdirectory(submodules)

add_compile_options(
    -Wall
    -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
    -Wno-unused-function # we have some for the docs that aren't called
    -Wno-maybe-uninitialized
)

add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/uart_bridge.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usb_descriptors.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cli/cli.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cli/alpha_power_modes.c
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${PROJECT_NAME}
    pico_multicore
    pico_stdlib
    tinyusb_device
    hardware_pio
    embedded_cli_lib
)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/src/pio/uart_rx.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/src/pio/uart_tx.pio)

add_custom_target( ${PROJECT_NAME}_install 
    COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.uf2 /run/media/thijs/RPI-RP2/
    DEPENDS ${PROJECT_NAME}
)

# create map/bin/hex file etc.
pico_add_extra_outputs(uart_bridge)


