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

# Export HEX to a stable project-local path for external flash tools (e.g. CodeGrip).
set(CODEGRIP_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/output")
set(CODEGRIP_INPUT_HEX "${PROJECT_BINARY_DIR}/zephyr/zephyr.hex")
set(CODEGRIP_OUTPUT_HEX "${CODEGRIP_OUTPUT_DIR}/zephyr-clicker4_stm32f7.hex")

add_custom_command(
  OUTPUT "${CODEGRIP_OUTPUT_HEX}"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CODEGRIP_OUTPUT_DIR}"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CODEGRIP_INPUT_HEX}" "${CODEGRIP_OUTPUT_HEX}"
  DEPENDS "${CODEGRIP_INPUT_HEX}"
  COMMENT "Exporting zephyr.hex for CodeGrip"
  VERBATIM
)

add_custom_target(export_codegrip_hex ALL
  DEPENDS "${CODEGRIP_OUTPUT_HEX}"
)
