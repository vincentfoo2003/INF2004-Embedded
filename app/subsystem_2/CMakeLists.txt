cmake_minimum_required(VERSION 3.13)

project(motor_pid_driver C CXX ASM)

add_library(motor_pid_driver
    src/motor_pid_driver.c
)

target_include_directories(motor_pid_driver PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_include_directories(motor_pid_driver PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
)

target_link_libraries(motor_pid_driver
    pico_stdlib
    hardware_pwm
    hardware_gpio
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)

add_executable(motor_pid_driver_test
    src/motor_pid_driver_test.c
)

target_include_directories(motor_pid_driver_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/..
)

target_link_libraries(motor_pid_driver_test
    pico_stdlib
    hardware_pwm
    hardware_gpio
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
    motor_pid_driver
)

# printf goes over USB; GP8-GP11 are used by the onboard motor driver
pico_enable_stdio_usb(motor_pid_driver_test 1)
pico_enable_stdio_uart(motor_pid_driver_test 0)

pico_add_extra_outputs(motor_pid_driver_test)

target_compile_options(motor_pid_driver PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_compile_options(motor_pid_driver_test PRIVATE -Wall -Wextra -Wno-unused-parameter)
