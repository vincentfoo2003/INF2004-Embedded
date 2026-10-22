cmake_minimum_required(VERSION 3.13)

project(wifi_telemetry C CXX ASM)

add_library(wifi_telemetry
    src/wifi_telemetry.c
)

target_include_directories(wifi_telemetry PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(wifi_telemetry
    pico_stdlib
    pico_cyw43_arch_lwip_sys_freertos
    pico_lwip_mqtt
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)

target_compile_definitions(wifi_telemetry PRIVATE
    NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
    LWIP_SOCKET=0       # we need the socket API (generally this would be in your lwipopts.h)
)

# printf goes over USB; the link under test is UART1 on GP8/GP9
# pico_enable_stdio_usb(wifi_telemetry 1)
# pico_enable_stdio_uart(wifi_telemetry 0)

# pico_add_extra_outputs(wifi_telemetry)

# --------------------------------------------------------------------
# The Pico SDK does not enable compiler warnings by default. Several of
# the planted defects in this hunt are reported ONLY as warnings, so the
# briefs would be asking you to read messages that are never emitted.
#
#   -Wextra is needed as well as -Wall: the "unsigned value compared
#   against < 0" defect is -Wtype-limits, which -Wall alone does not
#   include.
#
#   -Wno-unused-parameter suppresses noise from SDK callback signatures
#   that must take arguments they do not use. That noise is not a
#   planted defect.
# --------------------------------------------------------------------
target_compile_options(wifi_telemetry PRIVATE -Wall -Wextra -Wno-unused-parameter)