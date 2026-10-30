cmake_minimum_required(VERSION 3.13)

project(obstacle_avoidance C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(obstacle_avoidance
    obstacle_avoidance.c
    obstacle_logic.c
    servo.c
    hcsr04.c
)

pico_set_program_name(obstacle_avoidance "obstacle_avoidance")
pico_set_program_version(obstacle_avoidance "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(obstacle_avoidance 0)
pico_enable_stdio_usb(obstacle_avoidance 1)

# Add the standard library to the build
target_link_libraries(obstacle_avoidance
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_clocks # servo.c uses clock_get_hz(clk_sys) to derive the PWM clkdiv
    pico_time
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap (public header needs FreeRTOS.h/queue.h)
)

# Add the standard include files to the build.
# ${CMAKE_CURRENT_LIST_DIR}    -> this subsystem's headers
# ${CMAKE_CURRENT_LIST_DIR}/.. -> app/ for FreeRTOSConfig.h (required by FreeRTOS-Kernel-Heap4)
target_include_directories(obstacle_avoidance PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
)

pico_add_extra_outputs(obstacle_avoidance)
