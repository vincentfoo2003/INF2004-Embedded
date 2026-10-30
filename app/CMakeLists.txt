cmake_minimum_required(VERSION 3.13)

project(app C CXX ASM)

add_subdirectory(motor_pid_driver)

add_executable(app
    main.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(app
    pico_stdlib
    hardware_pwm
    hardware_gpio
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
    wifi_telemetry
    motor_pid_driver
)

target_compile_definitions(app PRIVATE
    NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
    LWIP_SOCKET=0       # we need the socket API (generally this would be in your lwipopts.h)
)

# printf goes over USB; GP8-GP11 are used by the onboard motor driver
pico_enable_stdio_usb(app 1)
pico_enable_stdio_uart(app 0)

pico_add_extra_outputs(app)

target_compile_options(app PRIVATE -Wall -Wextra -Wno-unused-parameter)