# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(system-9-displays C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(system-9-displays system-9-displays.cpp lcd.h lcd.cpp pio_spi.c pio_spi.h)

pico_set_program_name(system-9-displays "system-9-displays")
pico_set_program_version(system-9-displays "0.1")

pico_enable_stdio_usb(system-9-displays ENABLED)

pico_generate_pio_header(system-9-displays ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

# Add the standard library to the build
target_link_libraries(system-9-displays
        pico_stdlib
        hardware_pio)

# Add the standard include files to the build
target_include_directories(system-9-displays PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

pico_add_extra_outputs(system-9-displays)

