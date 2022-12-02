# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
#set(PICO_SDK_PATH "C:/Users/ravel/Documents/Pico/pico-sdk")
#set(FREERTOS_KERNEL_PATH "C:/Users/ravel/Documents/Pico/FreeRTOS-Kernel")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

include(FreeRTOS_Kernel_import.cmake)

project(RemoteControl C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
if (TARGET tinyusb_device)
        #add_executable(RemoteRemoteControl main.c )
       
        #add_subdirectory(no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI)
        add_executable(RemoteControl
        src/main.cpp
        src/NRF24.cpp
        src/SERVO.cpp
        src/MPU9250.cpp
        #src/tareas.c
        )
        target_include_directories(RemoteControl PUBLIC
        inc
        )
        target_include_directories(RemoteControl PUBLIC ./)
        target_link_libraries(RemoteControl pico_stdlib hardware_spi hardware_adc hardware_pwm hardware_timer hardware_i2c)     

        # Add any user requested libraries
        target_link_libraries(RemoteControl
                FreeRTOS-Kernel
                FreeRTOS-Kernel-Heap4
                pico_stdlib
                pico_multicore
        )

        pico_enable_stdio_uart(RemoteControl 0)
        pico_enable_stdio_usb(RemoteControl 1)
        pico_set_program_name(RemoteControl "RemoteControl")
        pico_set_program_version(RemoteControl "0.1")
        pico_add_extra_outputs(RemoteControl)

elseif(PICO_ON_DEVICE)
        message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()
