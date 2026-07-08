# PICO_UROS_W6100

Firmware embebido para **Raspberry Pi Pico 2 + W6100-EVB-Pico2** con integración **micro-ROS (ROS 2)** por UDP.

El proyecto publica datos de **IMU** y **GNSS**, y expone control remoto para **servo** y salidas digitales (COP, LIGHT, CAM) mediante tópicos ROS 2.

## Tabla de contenido

1. [Características](#características)
2. [Arquitectura](#arquitectura)
3. [Requisitos](#requisitos)
4. [Estructura del repositorio](#estructura-del-repositorio)
5. [Configuración de red y agente micro-ROS](#configuración-de-red-y-agente-micro-ros)
6. [Compilación](#compilación)
7. [Flasheo](#flasheo)
8. [Interfaces ROS 2](#interfaces-ros-2)
9. [Mapa de pines](#mapa-de-pines)
10. [Comportamiento en ejecución](#comportamiento-en-ejecución)
11. [Pruebas rápidas](#pruebas-rápidas)

## Características

- Conectividad micro-ROS sobre **UDP/IP** usando transporte custom para W6100.
- Publicación de sensores:
  - `sensor_msgs/msg/Imu` (`imu`)
  - `sensor_msgs/msg/NavSatFix` (`gnss/fix`)
  - `sensor_msgs/msg/Imu` para heading GNSS (`gnss/heading`)
  - métricas de calidad GNSS y heading (`std_msgs/msg/UInt8`)
- Control de actuadores y salidas por suscripción:
  - Servo serial (`servo/req`)
  - LEDs/salidas COP, LIGHT y CAM
- Ejecución **multinúcleo**:
  - Core 1: adquisición de IMU/GNSS + ejecución de servo
  - Core 0: micro-ROS, publicaciones, callbacks y supervisión del agente
- Mecanismo de robustez:
  - Ping periódico al agente micro-ROS
  - Reboot por watchdog si se pierde conectividad sostenida

## Arquitectura

- MCU objetivo: RP2350 (Pico 2)
- Ethernet: W6100 (ioLibrary + porting layer)
- Middleware: `rcl`, `rclc`, `rmw_microxrcedds`
- Transporte micro-ROS: `src/wiz_udp_transport.c`
- Sensores/actuadores integrados:
  - IMU BNO08x por I2C
  - GNSS por UART (tramas NMEA GGA + mensajes `#HEADINGA/#UNIHEADINGA`)
  - Servo serial SCServo (UART)
  - Salidas GPIO COP/LIGHT/CAM + LED WS2812

## Requisitos

### Hardware

- W6100-EVB-Pico2 (o plataforma compatible con la configuración actual)
- IMU BNO08x
- Receptor GNSS con salida NMEA GGA y heading (`#HEADINGA` o `#UNIHEADINGA`)
- Servo SCServo compatible
- Red Ethernet en la misma subred que el agente micro-ROS

### Software

- CMake >= 3.13
- Toolchain ARM embebida (`arm-none-eabi-*`)
- Pico SDK disponible en:
  - `$HOME/micro_ros_ws/src/pico-sdk`
- micro-ROS Raspberry Pi Pico SDK disponible en:
  - `$HOME/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk`
- Librería estática micro-ROS compilada en:
  - `$HOME/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk/libmicroros/libmicroros.a`

Nota: estos paths están fijados en `CMakeLists.txt` mediante `PICO_SDK_PATH` y `MICROROS_DIR`.

## Estructura del repositorio

```text
.
├── CMakeLists.txt
├── src/
│   ├── main.cpp
│   ├── wiz_udp_transport.c
│   ├── wiz_udp_transport.h
│   └── clock_gettime_stub.c
├── lib/
│   ├── ioLibrary_Driver/
│   ├── port/
│   ├── BNO08x_Pico_Library/
│   ├── scservo/
│   └── Pico_WS2812/
└── README.md
```

## Configuración de red y agente micro-ROS

La configuración actual en `src/main.cpp` usa IP estática:

- Pi Pico 2:
  - IP: `192.168.123.2`
  - Máscara: `255.255.255.0`
  - Gateway: `0.0.0.0`
- Agente micro-ROS:
  - IP: `192.168.123.18`
  - Puerto agente: `8888`
  - Puerto local: `9999`

Ajusta estos valores antes de compilar según tu red.

## Compilación

Desde la raíz del proyecto:

```bash
cd build
rm -rf *
cmake ..
make
```

Artefactos esperados:

- `build/pico_uros_w6100.uf2`
- `build/pico_uros_w6100.elf`
- `build/pico_uros_w6100.bin`

## Flasheo

1. Conecta la placa en modo BOOTSEL.
2. Copia `build/pico_uros_w6100.uf2` como unidad USB masiva.
3. Reinicia la placa.

## Interfaces ROS 2

Node:

- Nombre: `node`
- Namespace: `pico`

### Publishers

- `imu` (`sensor_msgs/msg/Imu`)
  - `frame_id`: `imu_link`
- `gnss/fix` (`sensor_msgs/msg/NavSatFix`)
  - `frame_id`: `gnss`
- `gnss/fix_quality` (`std_msgs/msg/UInt8`)
- `gnss/heading` (`sensor_msgs/msg/Imu`)
  - `frame_id`: `gnss_heading`
  - usa quaternion derivado de yaw GNSS
- `gnss/heading_quality` (`std_msgs/msg/UInt8`)
- `servo/resp` (`std_msgs/msg/Bool`)
- `led_cop/resp` (`std_msgs/msg/Bool`)
- `led_light/resp` (`std_msgs/msg/Bool`)
- `led_cam/resp` (`std_msgs/msg/Bool`)

### Subscribers

- `servo/req` (`std_msgs/msg/UInt8`)
  - rango válido: `0..180`
- `led_cop/req` (`std_msgs/msg/UInt8`)
  - `0`: OFF
  - `1`: ON
  - `2`: pulso de patrón (solo si COP está ON)
- `led_light/req` (`std_msgs/msg/UInt8`)
  - `0`: OFF, `1`: ON
- `led_cam/req` (`std_msgs/msg/UInt8`)
  - `0`: OFF, `1`: ON

## Mapa de pines

### Periféricos de aplicación (`src/main.cpp`)

- GNSS UART (`uart1`):
  - TX: GPIO 8
  - RX: GPIO 9
- Servo UART (`uart0`, 1 Mbps):
  - TX: GPIO 12
  - RX: GPIO 13
- IMU BNO08x:
  - RESET: GPIO 1
  - INT: GPIO 4
- Salidas:
  - COP LED: GPIO 0
  - COP PATTERN: GPIO 5
  - LIGHT: GPIO 14
  - CAM: GPIO 15
  - LED RGB WS2812: GPIO 22
- Entradas:
  - PGOOD: GPIO 10
  - CURRENT (ADC): GPIO 28 (ADC2)


## Comportamiento en ejecución

- Arranque:
  - Inicializa red W6100 y transporte micro-ROS.
  - Hace ping al agente hasta conectar.
  - LED RGB indica estado de conexión (espera/conectado).
- Operación normal:
  - Core 1 adquiere IMU + GNSS y procesa comandos de servo.
  - Core 0 publica tópicos ROS 2 y atiende suscripciones.
- Recuperación ante fallos:
  - Ping cada 1 s.
  - Si falla repetidamente, reinicia por watchdog.

## Pruebas rápidas

### 1) Levantar agente micro-ROS

En el host (ajusta interfaz/red según tu entorno):

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

### 2) Ver tópicos

```bash
ros2 topic list
```

### 3) Ver IMU

```bash
ros2 topic echo /pico/imu
```

### 4) Mover servo a 90°

```bash
ros2 topic pub /pico/servo/req std_msgs/msg/UInt8 "{data: 90}"
```

### 5) Encender LIGHT

```bash
ros2 topic pub /pico/led_light/req std_msgs/msg/UInt8 "{data: 1}"
```
