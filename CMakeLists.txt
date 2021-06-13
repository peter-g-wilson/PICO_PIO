# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/pi/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(PICO_PIO_F007T_WH1080 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(PICO_PIO_F007T_WH1080 PICO_PIO_F007T_WH1080.c )

pico_generate_pio_header(PICO_PIO_F007T_WH1080 ${CMAKE_CURRENT_LIST_DIR}/PICO_PIO_F007T_WH1080.pio)
pico_set_program_name(PICO_PIO_F007T_WH1080 "PICO_PIO_F007T_WH1080")
pico_set_program_version(PICO_PIO_F007T_WH1080 "0.1")

target_sources(PICO_PIO_F007T_WH1080 PRIVATE
        queues_for_msgs_and_bits.c
        output_format.c
        uart_IO.c
        PICO_PIO_WH1080.c
        PICO_PIO_F007T.c
        )

pico_enable_stdio_uart(PICO_PIO_F007T_WH1080 1)
pico_enable_stdio_usb(PICO_PIO_F007T_WH1080 0)

# Add the standard library to the build
target_link_libraries(PICO_PIO_F007T_WH1080
        pico_stdlib
        pico_multicore
        )

# Add any user requested libraries
target_link_libraries(PICO_PIO_F007T_WH1080 
        hardware_pio
        hardware_timer
        )

pico_add_extra_outputs(PICO_PIO_F007T_WH1080)

