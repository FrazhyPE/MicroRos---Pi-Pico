cmake_minimum_required(VERSION 3.13)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_SDK_PATH $ENV{HOME}/micro_ros_ws/src/pico-sdk)
set(MICROROS_DIR  $ENV{HOME}/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk)

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_uros_w5500 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_library(microros STATIC IMPORTED)
set_target_properties(microros PROPERTIES
  IMPORTED_LOCATION ${MICROROS_DIR}/libmicroros/libmicroros.a
)

add_executable(pico_uros_w5500
  src/main.c
  src/wiz_udp_transport.c
  src/clock_gettime_stub.c
)

set(WIZ_IO ${CMAKE_CURRENT_LIST_DIR}/lib/ioLibrary_Driver)
set(WIZ_PORT ${CMAKE_CURRENT_LIST_DIR}/lib/port)

target_sources(pico_uros_w5500 PRIVATE
  ${WIZ_IO}/Ethernet/wizchip_conf.c
  ${WIZ_IO}/Ethernet/socket.c
  ${WIZ_IO}/Ethernet/W5500/w5500.c

  ${WIZ_PORT}/ioLibrary_Driver/src/wizchip_spi.c
  ${WIZ_PORT}/ioLibrary_Driver/src/wizchip_gpio_irq.c

  ${WIZ_PORT}/timer/timer.c
)

target_include_directories(pico_uros_w5500 PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/src

  ${MICROROS_DIR}/libmicroros/include

  ${WIZ_IO}
  ${WIZ_IO}/Ethernet
  ${WIZ_IO}/Ethernet/W5500
  ${WIZ_IO}/Internet


  ${WIZ_PORT}
  ${WIZ_PORT}/ioLibrary_Driver/inc
  ${WIZ_PORT}/timer
)

add_compile_definitions(_WIZCHIP_=W5500)

target_link_libraries(pico_uros_w5500
  pico_stdlib
  hardware_spi
  hardware_dma
  hardware_irq
  microros
  pico_multicore
)

pico_enable_stdio_usb(pico_uros_w5500 1)
pico_enable_stdio_uart(pico_uros_w5500 0)

pico_add_extra_outputs(pico_uros_w5500)
