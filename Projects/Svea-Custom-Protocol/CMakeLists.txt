cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zbus_uart_telemetry C)

target_include_directories(app PRIVATE include)

target_sources(app PRIVATE
  src/main.c
  src/channels.c
  src/imu_sensor.c
  src/current_sensor.c
  src/uart_publisher.c
)
