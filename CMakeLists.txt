cmake_minimum_required(VERSION 3.13)

# Set PICO_PLATFORM to rp2350 for Pico 2
set(PICO_PLATFORM rp2350)

include(pico_sdk_import.cmake)
include(FreeRTOS_Kernel_import.cmake)

project(debugtarget_project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()  

add_executable(debugtarget
	debugtarget.c
        JLINK_MONITOR.c
        JLINK_MONITOR_ISR_SES.S
        )

# Add include directory for FreeRTOSConfig.h
target_include_directories(debugtarget PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# pull in common dependencies
target_link_libraries(debugtarget
        pico_stdlib
        hardware_gpio
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4)

pico_enable_stdio_usb(debugtarget 0)
pico_enable_stdio_uart(debugtarget 0)
pico_enable_stdio_rtt(debugtarget 1)

pico_set_binary_type(debugtarget copy_to_ram)

# create map/bin/hex file etc.
pico_add_extra_outputs(debugtarget)
