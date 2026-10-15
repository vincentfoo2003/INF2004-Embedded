cmake_minimum_required(VERSION 3.13)

project(Robo_Car C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Robo_Car 
    Robo_Car.c 
    motor_pid_driver.c
)

pico_set_program_name(Robo_Car "Robo_Car")
pico_set_program_version(Robo_Car "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Robo_Car 0)
pico_enable_stdio_usb(Robo_Car 1)

# Add the standard library to the build
target_link_libraries(Robo_Car
    pico_stdlib
    hardware_pwm
)

# Add the standard include files to the build
target_include_directories(Robo_Car PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(Robo_Car)

pico_add_extra_outputs(Robo_Car)

