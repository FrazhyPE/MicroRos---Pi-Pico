#include <stdio.h>
#include <unistd.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "wizchip_conf.h"
#include "wizchip_spi.h"
#include "wiz_udp_transport.h"

#ifdef __cplusplus
}
#endif

#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#include <std_msgs/msg/char.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/nav_sat_status.h>
#include <sensor_msgs/msg/imu.h>
#include <my_robot_msgs/msg/servo_cmd.h>
#include <rosidl_runtime_c/string_functions.h>

#include "hardware/uart.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/adc.h"
#include <SCServo.h>

#include "bno08x.h"
#include "utils.h"

#include "pico/stdlib.h"
#include "WS2812.hpp"

#define RCABORT(fn) do {rcl_ret_t rc = fn; if(rc != RCL_RET_OK){ printf("Error en la línea %d: %d. Abortando...\n",__LINE__,(int)rc); sleep_ms(1000); watchdog_reboot(0,0,0);}} while (0)
#define RCCONTINUE(fn) do {rcl_ret_t rc = fn; if(rc != RCL_RET_OK){ printf("Error en la línea %d: %d. Continuando...\n",__LINE__,(int)rc);}} while (0)

#define IMU_QUAT 10
#define IMU_GYRO 20
#define IMU_LINACC 20

#define GNSS_UART uart1
#define GNSS_BAUD 115200
#define GNSS_TX_PIN 8
#define GNSS_RX_PIN 9

#define SERVO_UART uart0
#define SERVO_BAUD 1000000
#define SERVO_TX_PIN 12
#define SERVO_RX_PIN 13 

#define SERVO_SPEED 1500
#define SERVO_ACC 50

#define COP_PULSE_MS 75

#define LED_LENGTH 1

/*********************************/
/*          Estructuras          */
/*********************************/

typedef struct {
    float qx;
    float qy;
    float qz;
    float qw;
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
} imu_t;

typedef struct {
    double lat_deg;
    double lon_deg;
    double alt_m;
    uint8_t fix_q;
    bool valid;
} gnss_fix_t;

typedef struct {
    float qx;
    float qy;
    float qz;
    float qw;
    float yaw_var;
    bool valid;
    uint8_t quality;
} gnss_heading_t;

typedef struct {
    uint8_t id;
    uint8_t angle;
} servo_t;

/*********************************/
/*      Declaración de Pines     */
/*********************************/

const uint BNO_RST = 1;
const uint BNO_INT = 4;

const uint COP_LED = 0;
const uint COP_PATHERN = 5;

const uint LIGHT = 14;

const uint CAM1 = 15;
const uint CAM2 = 26;

const uint LED_RGB = 22;

const uint PGOOD = 10;

const uint CURRENT = 28;

/*********************************/
/*       Variables Globales      */
/*********************************/

static queue_t q_imu;
static queue_t q_gnss;
static queue_t q_gnss_heading;
static queue_t q_servo;

// IMU
imu_t imu_old;
imu_t imu_latest;

static BNO08x IMU;

// GNSS
gnss_fix_t gnss_old;
gnss_fix_t gnss_latest;

gnss_heading_t gnss_heading_old;
gnss_heading_t gnss_heading_latest;

// SERVO
servo_t servo_old;
servo_t servo_latest;

SMS_STS st;

/*********************************/
/*    Estructuras Micro - ROS    */
/*********************************/

// Nodo
static rcl_node_t node;

// Agente ROS
static absolute_time_t next_ping;
static uint8_t ping_fail = 0;
static absolute_time_t next_sync;

// IMU
static rcl_publisher_t pub_imu;
static sensor_msgs__msg__Imu imu_msg;

// GNSS
static rcl_publisher_t pub_gnss;
static sensor_msgs__msg__NavSatFix gnss_msg;

static rcl_publisher_t pub_gnss_quality;
static std_msgs__msg__UInt8 gnss_quality_msg;

static rcl_publisher_t pub_gnss_heading;
static sensor_msgs__msg__Imu gnss_heading_msg;

static rcl_publisher_t pub_gnss_heading_quality;
static std_msgs__msg__UInt8 gnss_heading_quality_msg;

// Servo
static rcl_subscription_t sub_SERVO;
static rcl_publisher_t pub_SERVO;

static my_robot_msgs__msg__ServoCmd SERVO_req_msg;
static std_msgs__msg__Bool SERVO_resp_msg;

// Led COP
static rcl_subscription_t sub_COP;
static rcl_publisher_t pub_COP;

static std_msgs__msg__UInt8 COP_req_msg;
static std_msgs__msg__Bool COP_resp_msg;

static volatile alarm_id_t cop_pattern_alarm = -1;

// Led Light
static rcl_subscription_t sub_LIGHT;
static rcl_publisher_t pub_LIGHT;

static std_msgs__msg__UInt8 LIGHT_req_msg;
static std_msgs__msg__Bool LIGHT_resp_msg;

// led CAM
static rcl_subscription_t sub_CAM1;
static rcl_publisher_t pub_CAM1;

static std_msgs__msg__UInt8 CAM1_req_msg;
static std_msgs__msg__Bool CAM1_resp_msg;

static rcl_subscription_t sub_CAM2;
static rcl_publisher_t pub_CAM2;

static std_msgs__msg__UInt8 CAM2_req_msg;
static std_msgs__msg__Bool CAM2_resp_msg;

// led RGB
static absolute_time_t next_led_update;
static absolute_time_t next_led;
uint8_t brillo = 0;

/*********************************/
/*         Conexión UDP          */
/*********************************/

static wiz_NetInfo netinfo = {
    .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56},
    .ip  = {192, 168, 123, 2},
    .sn  = {255, 255, 255, 0},
    .gw  = {0, 0, 0, 0},

    .lla = {0},
    .gua = {0},
    .sn6 = {0},
    .gw6 = {0},

    .dns = {8, 8, 8, 8},
    .dns6 = {0},

    .ipmode = NETINFO_STATIC_ALL,
    .dhcp = NETINFO_STATIC
};


static wiz_uros_udp_params_t uros_params = {
    .agent_ip   = {192, 168, 123, 18}, //18: unitree, 222: pc
    .agent_port = 8888,
    .local_port = 9999
};

static void wiznet_init(void)
{
    wizchip_spi_initialize();
    wizchip_cris_initialize();
    wizchip_reset();
    wizchip_initialize();
    wizchip_check();
    network_initialize(netinfo);
    sleep_ms(200);
}

/*********************************/
/*     Inicialización Pines      */
/*********************************/

void gpio_init_custom(void)
{
    gpio_init(BNO_RST);
    gpio_set_dir(BNO_RST, GPIO_OUT);
    gpio_put(BNO_RST, 1);

    gpio_init(BNO_INT);
    gpio_set_dir(BNO_INT, GPIO_IN);
    gpio_pull_up(BNO_INT);

    gpio_init(COP_LED);
    gpio_set_dir(COP_LED, GPIO_OUT);
    gpio_put(COP_LED, 0);

    gpio_init(COP_PATHERN);
    gpio_set_dir(COP_PATHERN, GPIO_OUT);
    gpio_put(COP_PATHERN, 0);

    gpio_init(LIGHT);
    gpio_set_dir(LIGHT, GPIO_OUT);
    gpio_put(LIGHT, 0);

    gpio_init(CAM1);
    gpio_set_dir(CAM1, GPIO_OUT);
    gpio_put(CAM1, 0);

    gpio_init(CAM2);
    gpio_set_dir(CAM2, GPIO_OUT);
    gpio_put(CAM2, 0);

    gpio_init(PGOOD);
    gpio_set_dir(PGOOD, GPIO_IN);

    adc_init();
    adc_gpio_init(CURRENT);
    adc_select_input(2);
}

/*********************************/
/*     Funciones Envio Cores     */
/*********************************/

// Extracción y Liberación de la Cola : Lectura
static inline bool queue_try_get_latest(queue_t *q, void *msg)
{
    bool has = false;
    while (queue_try_remove(q, msg)) {
        has = true;
    }
    return has;
}

// Envio y Limpieza de la cola : Envio
static inline void queue_push_latest(queue_t *q, const void *elem, void *scratch_old)
{
    if (!queue_try_add(q, elem)) {
        (void)queue_try_remove(q, scratch_old);
        (void)queue_try_add(q, elem);
    }
}

// GNSS
static inline double nmea_degmin_to_deg(double degmin)
{
    int deg = (int)(degmin / 100.0);
    double minutes = degmin - (double)deg * 100.0;
    return (double)deg + minutes / 60.0;
}

static bool parse_gga_line(const char *line, gnss_fix_t *msg)
{
    // $GNGGA,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,...
    if (strncmp(line, "$GNGGA,", 7) != 0 && strncmp(line, "$GPGGA,", 7) != 0) return false;

    char buf[200];
    strncpy(buf, line, sizeof(buf)-1);
    buf[sizeof(buf)-1] = 0; 

    char *save = NULL;
    char *tok = strtok_r(buf, ",", &save);
    int field = 0;

    double lat_dm = 0, lon_dm = 0, alt = 0, hdop = 0;
    char ns = 0, ew = 0;
    int fix = 0, sats = 0;

    while (tok) {
        if (field == 2 && tok[0]) lat_dm = atof(tok);
        if (field == 3 && tok[0]) ns = tok[0];
        if (field == 4 && tok[0]) lon_dm = atof(tok);
        if (field == 5 && tok[0]) ew = tok[0];
        if (field == 6 && tok[0]) fix = atoi(tok);
        if (field == 7 && tok[0]) sats = atoi(tok);
        if (field == 8 && tok[0]) hdop = atof(tok);
        if (field == 9 && tok[0]) alt = atof(tok);

        tok = strtok_r(NULL, ",", &save);
        field++;
    }

    msg->fix_q = (uint8_t)fix;
    msg->alt_m = alt;

    if (fix <= 0 || lat_dm == 0 || lon_dm == 0 || ns == 0 || ew == 0) {
        msg->valid = false;
        return true;
    }

    double lat = nmea_degmin_to_deg(lat_dm);
    double lon = nmea_degmin_to_deg(lon_dm);
    if (ns == 'S') lat = -lat;
    if (ew == 'W') lon = -lon;

    msg->lat_deg = lat;
    msg->lon_deg = lon;
    msg->valid = true;
    return true;
}

static inline float deg2radf(float deg)
{
    return deg * 0.017453292519943295f;
}

static inline void quat_from_yaw(float yaw_rad, float *qx, float *qy, float *qz, float *qw)
{
    float half = 0.5f * yaw_rad;
    *qx = 0.0f;
    *qy = 0.0f;
    *qz = sinf(half);
    *qw = cosf(half);
}

static uint8_t heading_quality_from_sol(const char *sol)
{
    if (!sol) return 0;
    if (strcmp(sol, "NARROW_INT") == 0)   return 4; // mejor
    if (strcmp(sol, "NARROW_FLOAT") == 0) return 3;
    if (strcmp(sol, "FLOAT") == 0 || strcmp(sol, "L1_FLOAT") == 0) return 2;
    if (strcmp(sol, "SINGLE") == 0)       return 1;
    if (strcmp(sol, "NONE") == 0)         return 0;
    return 0;
}

static bool parse_headingga_line(const char *line, gnss_heading_t *msg)
{
    if (strncmp(line, "#UNIHEADINGA,", 12) != 0 && strncmp(line, "#HEADINGA,", 9) != 0) {
        return false;
    }

    const char *semi = strchr(line, ';');
    if (!semi || !semi[1]) return true;

    char buf[220];
    strncpy(buf, semi + 1, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;

    char *save = NULL;
    char *tok = strtok_r(buf, ",", &save);

    // Esperado:
    // tok0 = SOL_COMPUTED  (o INSUFFICIENT_OBS, etc.)
    // tok1 = NARROW_FLOAT / NARROW_INT / NONE / ...
    // tok2 = baseline_m
    // tok3 = heading_deg
    // tok4 = pitch_deg
    // tok5 = reserved (a veces 0.0000)
    // tok6 = heading_std
    // tok7 = pitch_std
    const char *status = NULL;
    const char *sol = NULL;

    float baseline = 0, heading = 0, pitch = 0, hstd = 0, pstd = 0;

    int field = 0;
    while (tok) {
        // tok puede venir con \r al final
        size_t n = strlen(tok);
        if (n && tok[n-1] == '\r') tok[n-1] = 0;

        if (field == 0) status = tok;
        else if (field == 1) sol = tok;
        else if (field == 2 && tok[0]) baseline = (float)atof(tok);
        else if (field == 3 && tok[0]) heading  = (float)atof(tok);
        else if (field == 4 && tok[0]) pitch    = (float)atof(tok);
        else if (field == 6 && tok[0]) hstd     = (float)atof(tok);
        else if (field == 7 && tok[0]) pstd     = (float)atof(tok);

        tok = strtok_r(NULL, ",", &save);
        field++;
    }

    while (heading < 0.0f) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;

    float half = 0.5f * heading * 0.017453292519943295f;

    float yaw_rad = deg2radf(heading);
    quat_from_yaw(yaw_rad, &msg->qx, &msg->qy, &msg->qz, &msg->qw);

    float yaw_std_rad = deg2radf(hstd);
    msg->yaw_var = yaw_std_rad * yaw_std_rad;

    msg->valid = true;
    msg->quality = heading_quality_from_sol(sol);

    if (status && (strncmp(status, "INSUFFICIENT_OBS", 16) == 0)) msg->valid = false;
    if (sol && strcmp(sol, "NONE") == 0) msg->valid = false;

    if (!msg->valid) {
    msg->yaw_var = 99999.0f;
    msg->qx = 0.0f; msg->qy = 0.0f; msg->qz = 0.0f; msg->qw = 1.0f;
    }

    return true;
}

static int8_t navsat_status_from_fixq(uint8_t fixq)
{
    if (fixq == 0) return -1;
    if (fixq == 4 || fixq == 5) return 2;
    return 0;
}

static inline bool stamp_from_ros_time(builtin_interfaces__msg__Time *t)
{
    if (!rmw_uros_epoch_synchronized()) {
        return false;
    }

    int64_t ns = rmw_uros_epoch_nanos();
    t->sec = (int32_t)(ns / 1000000000LL);
    t->nanosec = (uint32_t)(ns % 1000000000LL);
    return true;
}

/*********************************/
/*    Callbacks Timer y Subs     */
/*********************************/

//Callback Servo
void subscription_callback_servo(const void * msgin)
{
  const my_robot_msgs__msg__ServoCmd * msg = (const my_robot_msgs__msg__ServoCmd *)msgin;
  
  servo_t servo;

  SERVO_resp_msg.data = false;

  servo.id = msg->id;
  servo.angle = msg->angle;

  if (servo.angle <= 180) {
    queue_push_latest(&q_servo, &servo, &servo_old);
    SERVO_resp_msg.data = true;
  }

  RCCONTINUE(rcl_publish(&pub_SERVO, &SERVO_resp_msg, NULL));
}

// Callback Led COP
static int64_t cop_pattern_off_cb(alarm_id_t id, void *user_data)
{
    (void)id; (void)user_data;
    gpio_put(COP_PATHERN, 0);
    cop_pattern_alarm = -1;
    return 0;
}

static inline void cop_trigger_pattern_pulse(uint32_t ms)
{
    if (cop_pattern_alarm >= 0) {
        cancel_alarm(cop_pattern_alarm);
        cop_pattern_alarm = -1;
    }

    gpio_put(COP_PATHERN, 1);
    cop_pattern_alarm = add_alarm_in_ms((int32_t)ms, cop_pattern_off_cb, NULL, true);
}

void subscription_callback_cop(const void * msgin)
{
    const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
    bool ok = true;

    switch (msg->data) {
        case 0:
            gpio_put(COP_LED, 0);
            break;

        case 1: 
            gpio_put(COP_LED, 1);
            break;

        case 2:
            if (gpio_get(COP_LED)) {
                cop_trigger_pattern_pulse(COP_PULSE_MS);
            } else {
                ok = false;
            }
            break;

        default:
            ok = false;
            break;
    }

    COP_resp_msg.data = ok;
    RCCONTINUE(rcl_publish(&pub_COP, &COP_resp_msg, NULL));
}

// Callback Light
void subscription_callback_light(const void * msgin)
{
  const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
  
  LIGHT_resp_msg.data = false;

  if (msg->data) {
    gpio_put(LIGHT, 1);
    LIGHT_resp_msg.data = true;
  } else {
    gpio_put(LIGHT, 0);
    LIGHT_resp_msg.data = true;
  }
  RCCONTINUE(rcl_publish(&pub_LIGHT, &LIGHT_resp_msg, NULL));
}

// Callback CAM
void subscription_callback_cam1(const void * msgin)
{
  const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
  
  CAM1_resp_msg.data = false;

  if (msg->data) {
    gpio_put(CAM1, 1);
    CAM1_resp_msg.data = true;
  } else {
    gpio_put(CAM1, 0);
    CAM1_resp_msg.data = true;
  }
  RCCONTINUE(rcl_publish(&pub_CAM1, &CAM1_resp_msg, NULL));
}

void subscription_callback_cam2(const void * msgin)
{
  const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
  
  CAM2_resp_msg.data = false;

  if (msg->data) {
    gpio_put(CAM2, 1);
    CAM2_resp_msg.data = true;
  } else {
    gpio_put(CAM2, 0);
    CAM2_resp_msg.data = true;
  }
  RCCONTINUE(rcl_publish(&pub_CAM2, &CAM2_resp_msg, NULL));
}

// Función Led RGB
void change_led_rgb(WS2812* ledStrip, uint8_t r, uint8_t g, uint8_t b)
{
    ledStrip->fill(WS2812::RGB(r, g, b));
    ledStrip->show();
}

/*********************************/
/*        Nucleo Secundario      */
/*********************************/

void core1_entry(void)
{   
    // Inicialización I2C IMU
    i2c_inst_t* i2c_port = i2c1;
    initI2C(i2c_port, false);

    gpio_put(BNO_RST, 0);
    sleep_ms(10);
    gpio_put(BNO_RST, 1);
    sleep_ms(100);

    while (!IMU.begin(0x4B, i2c_port)) {
        sleep_ms(50);
    }

    IMU.enableRotationVector(IMU_QUAT);
    IMU.enableGyro(IMU_GYRO);
    IMU.enableLinearAccelerometer(IMU_LINACC);

    imu_t imu = {0};

    // Inicialización GNSS
    gnss_fix_t gnss = {0};
    gnss_heading_t gnss_heading = {0};

    // Inicialización Servo
    uint8_t sid;

    char line[220];
    int idx = 0;

    while (true)
    {   
        // GNSS
        while (uart_is_readable(GNSS_UART)) {
            char c = (char)uart_getc(GNSS_UART);

            if (c == '\r') continue;

            if (c == '\n') {
                line[idx] = 0;
                idx = 0;

                if (line[0] == '$') {
                    if (parse_gga_line(line, &gnss)) {
                        queue_push_latest(&q_gnss, &gnss, &gnss_old);
                    }
                } else if (line[0] == '#') {
                    if (parse_headingga_line(line, &gnss_heading)) {
                        queue_push_latest(&q_gnss_heading, &gnss_heading, &gnss_heading_old);
                    }
                }
            } else {
                if (idx < (int)sizeof(line) - 1) line[idx++] = c;
                else idx = 0;
            }
        }

        // IMU
        bool got_rv = false;
        while (IMU.getSensorEvent()) {
            sid = IMU.getSensorEventID();

            if (sid == SENSOR_REPORTID_ROTATION_VECTOR) {
                imu.qx = IMU.getQuatI();
                imu.qy = IMU.getQuatJ();
                imu.qz = IMU.getQuatK();
                imu.qw = IMU.getQuatReal();
                got_rv = true;
            }
            else if (sid == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
                imu.gx = IMU.getGyroX();
                imu.gy = IMU.getGyroY();
                imu.gz = IMU.getGyroZ();
            }
            else if (sid == SENSOR_REPORTID_LINEAR_ACCELERATION) {
            imu.ax = IMU.getLinAccelX();
            imu.ay = IMU.getLinAccelY();
            imu.az = IMU.getLinAccelZ();
            }

            if (IMU.wasReset()) {
                IMU.enableRotationVector(IMU_QUAT);
                IMU.enableGyro(IMU_GYRO);
                IMU.enableLinearAccelerometer(IMU_LINACC);
            }
        }
        if (got_rv) queue_push_latest(&q_imu, &imu, &imu_old);

        // SERVO
        if (queue_try_get_latest(&q_servo, &servo_latest)) {
            uint8_t id = servo_latest.id;
            uint8_t angle = servo_latest.angle;

            uint16_t pos = (uint16_t)(angle*512.0/45.0+1024.0);

            st.WritePosEx(id, pos, SERVO_SPEED, SERVO_ACC);
        }
        tight_loop_contents();
    }
}

/*********************************/
/*        Nucelo Principal       */
/*********************************/

int main(void)
{
    // Inicialización de la placa y conexión de red
    stdio_init_all();
    sleep_ms(1500);
    wiznet_init();
    rmw_uros_set_custom_transport(
        false,
        &uros_params,
        wiz_uros_udp_open,
        wiz_uros_udp_close,
        wiz_uros_udp_write,
        wiz_uros_udp_read
    );

    WS2812 ledStrip(
        LED_RGB,           
        LED_LENGTH,        
        pio0,              
        0,                 
        WS2812::FORMAT_GRB 
    );
    
    next_ping = make_timeout_time_ms(500);
    next_sync = make_timeout_time_ms(30000);
    next_led  = make_timeout_time_ms(300);

    while (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {

        if (absolute_time_diff_us(get_absolute_time(), next_ping) <= 0) {
            next_ping = make_timeout_time_ms(500);

            rmw_uros_ping_agent(100, 1);
        }

        if (absolute_time_diff_us(get_absolute_time(), next_led) <= 0) {
            next_led = make_timeout_time_ms(300);

            brillo = !brillo;

            change_led_rgb(&ledStrip, brillo * 250, 0, 0);
        }
    }

    change_led_rgb(&ledStrip, 255, 255, 0);
    sleep_ms(1500);

    // Configuración Pines
    gpio_init_custom();

    // Configuración UART1 para GNSS

    uart_init(GNSS_UART, GNSS_BAUD);
    gpio_set_function(GNSS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GNSS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(GNSS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GNSS_UART, true);

    // Configuración UART para Servo
    uart_init(SERVO_UART, SERVO_BAUD);
    gpio_set_function(SERVO_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SERVO_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(SERVO_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(SERVO_UART, true);
    st.setUart(SERVO_UART); 

    // Colas Core
    queue_init(&q_imu,  sizeof(imu_t),  8);
    queue_init(&q_gnss, sizeof(gnss_fix_t),  8);
    queue_init(&q_gnss_heading, sizeof(gnss_heading_t), 8);
    queue_init(&q_servo, sizeof(servo_t), 8);

    multicore_launch_core1(core1_entry);

    // Configuración Nodo
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    RCABORT(rclc_support_init(&support, 0, NULL, &allocator));
    RCABORT(rclc_node_init_default(&node, "node", "pico", &support));

    bool time_synced = false;

    while (!time_synced) {
        if (rmw_uros_sync_session(200) == RMW_RET_OK) {
            time_synced = true;
            printf("Time sync: OK\n");
        } else {
            printf("Time sync: FAILED, reintentando...\n");
            sleep_ms(200);
        }
    }

    // Configuración IMU
    const rosidl_message_type_support_t * type_support_pub_imu = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);
    RCABORT(rclc_publisher_init_best_effort(&pub_imu, &node, type_support_pub_imu, "imu"));

    // Configuración GNSS
    const rosidl_message_type_support_t * type_support_pub_gnss = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix);
    RCABORT(rclc_publisher_init_default(&pub_gnss, &node, type_support_pub_gnss, "gnss/fix"));

    const rosidl_message_type_support_t * type_support_pub_gnss_quality = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8);
    RCABORT(rclc_publisher_init_default(&pub_gnss_quality, &node, type_support_pub_gnss_quality, "gnss/fix_quality"));

    const rosidl_message_type_support_t * type_support_pub_gnss_heading = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);
    RCABORT(rclc_publisher_init_default(&pub_gnss_heading, &node, type_support_pub_gnss_heading, "gnss/heading"));

    const rosidl_message_type_support_t * type_support_pub_gnss_heading_quality = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8);
    RCABORT(rclc_publisher_init_default(&pub_gnss_heading_quality, &node, type_support_pub_gnss_heading_quality, "gnss/heading_quality"));

    // Configuración SERVO
    const rosidl_message_type_support_t * type_support_sub_servo = ROSIDL_GET_MSG_TYPE_SUPPORT(my_robot_msgs, msg, ServoCmd);
    RCABORT(rclc_subscription_init_default(&sub_SERVO, &node, type_support_sub_servo, "servo/req"));

    const rosidl_message_type_support_t * type_support_pub_servo = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);
    RCABORT(rclc_publisher_init_default(&pub_SERVO, &node, type_support_pub_servo, "servo/resp"));

    // Configuración LED COP
    const rosidl_message_type_support_t * type_support_sub_cop = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8);
    RCABORT(rclc_subscription_init_default(&sub_COP, &node, type_support_sub_cop, "led_cop/req"));

    const rosidl_message_type_support_t * type_support_pub_cop = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);
    RCABORT(rclc_publisher_init_default(&pub_COP, &node, type_support_pub_cop, "led_cop/resp"));

    // Configuración LIGHT
    const rosidl_message_type_support_t * type_support_sub_light = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8);
    RCABORT(rclc_subscription_init_default(&sub_LIGHT, &node, type_support_sub_light, "led_light/req"));

    const rosidl_message_type_support_t * type_support_pub_light = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);
    RCABORT(rclc_publisher_init_default(&pub_LIGHT, &node, type_support_pub_light, "led_light/resp"));

    // Configuración CAM
    const rosidl_message_type_support_t * type_support_sub_cam1 = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8);
    RCABORT(rclc_subscription_init_default(&sub_CAM1, &node, type_support_sub_cam1, "led_cam1/req"));

    const rosidl_message_type_support_t * type_support_pub_cam1 = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);
    RCABORT(rclc_publisher_init_default(&pub_CAM1, &node, type_support_pub_cam1, "led_cam1/resp"));

    const rosidl_message_type_support_t * type_support_sub_cam2 = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8);
    RCABORT(rclc_subscription_init_default(&sub_CAM2, &node, type_support_sub_cam2, "led_cam2/req"));

    const rosidl_message_type_support_t * type_support_pub_cam2 = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);
    RCABORT(rclc_publisher_init_default(&pub_CAM2, &node, type_support_pub_cam2, "led_cam2/resp"));

    // Incialización mensajes
    sensor_msgs__msg__Imu__init(&imu_msg);
    sensor_msgs__msg__NavSatFix__init(&gnss_msg);
    std_msgs__msg__UInt8__init(&gnss_quality_msg);
    sensor_msgs__msg__Imu__init(&gnss_heading_msg);    
    std_msgs__msg__UInt8__init(&gnss_heading_quality_msg);
    my_robot_msgs__msg__ServoCmd__init(&SERVO_req_msg);
    std_msgs__msg__Bool__init(&SERVO_resp_msg);
    std_msgs__msg__UInt8__init(&COP_req_msg);
    std_msgs__msg__Bool__init(&COP_resp_msg);
    std_msgs__msg__UInt8__init(&LIGHT_req_msg);  
    std_msgs__msg__Bool__init(&LIGHT_resp_msg);
    std_msgs__msg__UInt8__init(&CAM1_req_msg);
    std_msgs__msg__Bool__init(&CAM1_resp_msg);
    std_msgs__msg__UInt8__init(&CAM2_req_msg);
    std_msgs__msg__Bool__init(&CAM2_resp_msg);

    // Configuración campos constantes mensajes

    // IMU
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
    imu_msg.orientation_covariance[0] = -1;
    imu_msg.angular_velocity_covariance[0] = -1;
    imu_msg.linear_acceleration_covariance[0] = -1;

    // GNSS
    rosidl_runtime_c__String__assign(&gnss_msg.header.frame_id, "gnss_link");
    gnss_msg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;

    rosidl_runtime_c__String__assign(&gnss_heading_msg.header.frame_id, "gnss_heading");
    gnss_heading_msg.angular_velocity_covariance[0] = -1;
    gnss_heading_msg.linear_acceleration_covariance[0] = -1;  

    // Configuración Executor
    rclc_executor_t executor;
    executor = rclc_executor_get_zero_initialized_executor();
    unsigned int num_handles = 5;
    RCABORT(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
    RCABORT(rclc_executor_add_subscription(&executor, &sub_COP, &COP_req_msg, &subscription_callback_cop, ON_NEW_DATA));
    RCABORT(rclc_executor_add_subscription(&executor, &sub_LIGHT, &LIGHT_req_msg, &subscription_callback_light, ON_NEW_DATA));
    RCABORT(rclc_executor_add_subscription(&executor, &sub_CAM1, &CAM1_req_msg, &subscription_callback_cam1, ON_NEW_DATA));
    RCABORT(rclc_executor_add_subscription(&executor, &sub_CAM2, &CAM2_req_msg, &subscription_callback_cam2, ON_NEW_DATA));
    RCABORT(rclc_executor_add_subscription(&executor, &sub_SERVO, &SERVO_req_msg, &subscription_callback_servo, ON_NEW_DATA));
    rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(5));

    next_ping = make_timeout_time_ms(1000);
    next_led_update = make_timeout_time_ms(500);
    ping_fail = 0;
    
    // Publicación Topics
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

        if (queue_try_get_latest(&q_imu, &imu_latest)) {
            if (stamp_from_ros_time(&imu_msg.header.stamp)) {
                imu_msg.orientation.x = imu_latest.qx;
                imu_msg.orientation.y = imu_latest.qy;
                imu_msg.orientation.z = imu_latest.qz;
                imu_msg.orientation.w = imu_latest.qw;

                imu_msg.angular_velocity.x = imu_latest.gx;
                imu_msg.angular_velocity.y = imu_latest.gy;
                imu_msg.angular_velocity.z = imu_latest.gz;

                imu_msg.linear_acceleration.x = imu_latest.ax;
                imu_msg.linear_acceleration.y = imu_latest.ay;
                imu_msg.linear_acceleration.z = imu_latest.az;

                RCCONTINUE(rcl_publish(&pub_imu, &imu_msg, NULL));
            }
        }

        if (queue_try_get_latest(&q_gnss, &gnss_latest)) {
            if (stamp_from_ros_time(&gnss_msg.header.stamp)) {
                gnss_msg.latitude  = gnss_latest.lat_deg;
                gnss_msg.longitude = gnss_latest.lon_deg;
                gnss_msg.altitude  = gnss_latest.alt_m;
            
                gnss_quality_msg.data = gnss_latest.fix_q;
                gnss_msg.status.status  = navsat_status_from_fixq(gnss_latest.fix_q);
                gnss_msg.status.service = 1 | 2 | 4 | 8;

                if (!gnss_latest.valid) {
                gnss_msg.status.status = -1;
                }

                RCCONTINUE(rcl_publish(&pub_gnss, &gnss_msg, NULL));
                RCCONTINUE(rcl_publish(&pub_gnss_quality, &gnss_quality_msg, NULL));
            }
        }

        if (queue_try_get_latest(&q_gnss_heading, &gnss_heading_latest)) {
            if (stamp_from_ros_time(&gnss_heading_msg.header.stamp)) {

                gnss_heading_msg.orientation.x = gnss_heading_latest.qx;
                gnss_heading_msg.orientation.y = gnss_heading_latest.qy;
                gnss_heading_msg.orientation.z = gnss_heading_latest.qz;
                gnss_heading_msg.orientation.w = gnss_heading_latest.qw;

                gnss_heading_msg.orientation_covariance[0] = 99999.0;
                gnss_heading_msg.orientation_covariance[4] = 99999.0;
                gnss_heading_msg.orientation_covariance[8] = gnss_heading_latest.yaw_var;
                
                gnss_heading_quality_msg.data = gnss_heading_latest.quality;

                RCCONTINUE(rcl_publish(&pub_gnss_heading, &gnss_heading_msg, NULL));
                RCCONTINUE(rcl_publish(&pub_gnss_heading_quality, &gnss_heading_quality_msg, NULL));
            }
        }
        
        if (absolute_time_diff_us(get_absolute_time(), next_ping) <= 0){
            next_ping = make_timeout_time_ms(1000);

            bool ok = false;
            for (int i = 0; i < 10; i++) {
                if (rmw_uros_ping_agent(2, 1) == RCL_RET_OK) { ok = true; break; }
            }

            if (!ok) {
                ping_fail++;
                printf("PING FAIL (%u)\n", ping_fail);
            } else {
                ping_fail = 0;
            }

            if (ping_fail >= 3) {
                printf("Agent caído -> reboot\n");
                
                change_led_rgb(&ledStrip, 0, 0, 250);
                sleep_ms(20);
                watchdog_reboot(0, 0, 0);
            }
        }

        if (absolute_time_diff_us(get_absolute_time(), next_sync) <= 0){
            next_sync = make_timeout_time_ms(30000);

            if (rmw_uros_sync_session(100) == RMW_RET_OK) {
                printf("Time re-sync OK\n");
            } else {
                printf("Time re-sync FAIL\n");
            }
        }

        if (absolute_time_diff_us(get_absolute_time(), next_led_update) <= 0) {
            next_led_update = make_timeout_time_ms(800);

            brillo = !brillo;

            change_led_rgb(&ledStrip, 0, brillo * 250, 0);
        
        }

    }

    // Finalización de recursos
    rclc_executor_fini(&executor);

    rcl_publisher_fini(&pub_imu, &node);
    rcl_publisher_fini(&pub_gnss, &node);
    rcl_publisher_fini(&pub_gnss_quality, &node);
    rcl_publisher_fini(&pub_gnss_heading, &node);
    rcl_publisher_fini(&pub_gnss_heading_quality, &node);

    rcl_publisher_fini(&pub_SERVO, &node);
    rcl_subscription_fini(&sub_SERVO, &node);

    rcl_publisher_fini(&pub_COP, &node);
    rcl_subscription_fini(&sub_COP, &node);

    rcl_publisher_fini(&pub_LIGHT, &node);
    rcl_subscription_fini(&sub_LIGHT, &node);

    rcl_publisher_fini(&pub_CAM1, &node);
    rcl_subscription_fini(&sub_CAM1, &node);

    rcl_publisher_fini(&pub_CAM2, &node);
    rcl_subscription_fini(&sub_CAM2, &node);

    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return 0;
}
