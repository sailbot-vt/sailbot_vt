cmake_minimum_required(VERSION 3.8)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(sensors_transmission C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(/home/sailbot_user/micro_ros_raspberrypi_pico_sdk/libmicroros)

add_executable(sensor_transmission
    src/sensor_transmission.c
    src/pico_uart_transport.c
)
target_link_libraries(sensor_transmission
    pico_stdlib
    microros
)

target_include_directories(sensor_transmission PUBLIC
    # Note for the future -- make this path dynamic
    /home/ws/src/sensors_transmission/src
    /home/sailbot_user/micro_ros_raspberrypi_pico_sdk/libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(sensor_transmission 1)
pico_enable_stdio_uart(sensor_transmission 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(sensor_transmission)