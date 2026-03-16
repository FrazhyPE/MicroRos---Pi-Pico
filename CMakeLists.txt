cmake_minimum_required(VERSION 3.13)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_PLATFORM rp2350)
set(PICO_SDK_PATH $ENV{HOME}/micro_ros_ws/src/pico-sdk)
set(MICROROS_DIR  $ENV{HOME}/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk)

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_uros_w6100 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_library(microros STATIC IMPORTED)
set_target_properties(microros PROPERTIES
  IMPORTED_LOCATION ${MICROROS_DIR}/libmicroros/libmicroros.a
)

add_executable(pico_uros_w6100
  src/main.cpp
  src/wiz_udp_transport.c
  src/clock_gettime_stub.c
)

set(WIZ_IO ${CMAKE_CURRENT_LIST_DIR}/lib/ioLibrary_Driver)
set(WIZ_PORT ${CMAKE_CURRENT_LIST_DIR}/lib/port)
set(SC_SERVO ${CMAKE_CURRENT_LIST_DIR}/lib/scservo)
set(BNO086 ${CMAKE_CURRENT_LIST_DIR}/lib/BNO08x_Pico_Library)
set(WS2812_LIB ${CMAKE_CURRENT_LIST_DIR}/lib/Pico_WS2812)

pico_generate_pio_header(pico_uros_w6100 ${WS2812_LIB}/WS2812.pio)

add_subdirectory(
	lib/BNO08x_Pico_Library
)

target_sources(pico_uros_w6100 PRIVATE
  ${WIZ_IO}/Ethernet/wizchip_conf.c
  ${WIZ_IO}/Ethernet/socket.c
  ${WIZ_IO}/Ethernet/W6100/w6100.c

  ${WIZ_PORT}/ioLibrary_Driver/src/wizchip_spi.c
  ${WIZ_PORT}/ioLibrary_Driver/src/wizchip_gpio_irq.c

  ${WIZ_PORT}/timer/timer.c

  ${SC_SERVO}/SCS.cpp
  ${SC_SERVO}/SCSCL.cpp
  ${SC_SERVO}/SCSerial.cpp
  ${SC_SERVO}/SMS_STS.cpp

  ${WS2812_LIB}/WS2812.cpp
)

target_include_directories(pico_uros_w6100 PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/src

  ${MICROROS_DIR}/libmicroros/include

  ${WIZ_IO}
  ${WIZ_IO}/Ethernet
  ${WIZ_IO}/Ethernet/W6100
  ${WIZ_IO}/Internet


  ${WIZ_PORT}
  ${WIZ_PORT}/ioLibrary_Driver/inc
  ${WIZ_PORT}/timer
  ${SC_SERVO}
  ${WS2812_LIB}
)

add_compile_definitions(
  _WIZCHIP_=W6100
  DEVICE_BOARD_NAME=W6100_EVB_PICO2
)

target_link_libraries(pico_uros_w6100
  pico_stdlib
  hardware_spi
  hardware_dma
  hardware_irq
  hardware_uart
  microros
  pico_multicore
  hardware_i2c
  hardware_gpio
  BNO08x_Pico_Library
  hardware_adc
  hardware_pio
)

pico_enable_stdio_usb(pico_uros_w6100 1)
pico_enable_stdio_uart(pico_uros_w6100 0)
pico_add_extra_outputs(pico_uros_w6100)
