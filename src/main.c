#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>

#include "wizchip_conf.h"
#include "wizchip_spi.h"
#include "wiz_udp_transport.h"

#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#include <std_msgs/msg/char.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/nav_sat_status.h>
#include <sensor_msgs/msg/imu.h>
#include <rosidl_runtime_c/string_functions.h>

#include "hardware/uart.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"

#define RCABORT(fn) do {rcl_ret_t rc = fn; if(rc != RCL_RET_OK){ printf("Error en la línea %d: %d. Abortando...\n",__LINE__,(int)rc); sleep_ms(1000); watchdog_reboot(0,0,0);}} while (0)
#define RCCONTINUE(fn) do {rcl_ret_t rc = fn; if(rc != RCL_RET_OK){ printf("Error en la línea %d: %d. Continuando...\n",__LINE__,(int)rc);}} while (0)
#define COP_PULSE_MS 75
#define TIMER_IMU_MS 10
#define GNSS_UART uart1
#define GNSS_BAUD 115200
#define GNSS_TX_PIN 8
#define GNSS_RX_PIN 9

/*********************************/
/*          Estructuras          */
/*********************************/

typedef struct {
    double lat_deg;
    double lon_deg;
    double alt_m;
    uint8_t fix_q;
    uint8_t sats;
    float hdop;
    bool valid;
} gnss_fix_t;

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
    uint8_t calib_status;
    bool valid;
} imu_t;

/*********************************/
/*      Declaración de Pines     */
/*********************************/

const uint COP_LED = 0;
const uint COP_PATHERN = 5;
const uint LIGHT = 14;
const uint CAM = 15;

/*********************************/
/*       Variables Globales      */
/*********************************/

static queue_t q_gnss;
static queue_t q_gnss_heading;
static queue_t q_imu;
static queue_t q_servo;

uint8_t imu_old;
uint8_t imu_latest;

gnss_fix_t gnss_old;
gnss_fix_t gnss_latest;

imu_t gnss_heading_old;
imu_t gnss_heading_latest;

uint8_t servo_old;
uint8_t servo_latest;

/*********************************/
/*    Estructuras Micro - ROS    */
/*********************************/

// Nodo
static rcl_node_t node;

// Agente ROS
static absolute_time_t next_ping;
static uint8_t ping_fail = 0;

// GNSS
static rcl_publisher_t pub_gnss;
static sensor_msgs__msg__NavSatFix gnss_msg;

static rcl_publisher_t pub_gnss_quality;
static std_msgs__msg__UInt8 gnss_quality_msg;

static rcl_publisher_t pub_gnss_heading;
static sensor_msgs__msg__Imu gnss_heading_msg;

static rcl_publisher_t pub_gnss_heading_quality;
static std_msgs__msg__Float32 gnss_heading_quality_msg;

// IMU
static rcl_publisher_t pub_imu;
static std_msgs__msg__UInt8 imu_msg;

// Led COP
static rcl_subscription_t sub_COP;
static rcl_publisher_t pub_COP;

static std_msgs__msg__UInt8 COP_req_msg;
static std_msgs__msg__Bool COP_resp_msg;

static volatile alarm_id_t cop_pattern_alarm = -1;

// Led Light
static rcl_subscription_t sub_LIGHT;
static rcl_publisher_t pub_LIGHT;

static std_msgs__msg__Bool LIGHT_req_msg;
static std_msgs__msg__Bool LIGHT_resp_msg;

// led CAM
static rcl_subscription_t sub_CAM;
static rcl_publisher_t pub_CAM;

static std_msgs__msg__Bool CAM_req_msg;
static std_msgs__msg__Bool CAM_resp_msg;

// Servo
static rcl_subscription_t sub_SERVO;
static rcl_publisher_t pub_SERVO;

static std_msgs__msg__UInt8 SERVO_req_msg;
static std_msgs__msg__Bool SERVO_resp_msg;

/*********************************/
/*         Conexión UDP          */
/*********************************/

static wiz_NetInfo netinfo = {
    .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56},
    .ip  = {192, 168, 123, 2},
    .sn  = {255, 255, 255, 0},
    .gw  = {0, 0, 0, 0},
    .dns = {8, 8, 8, 8},
    .dhcp = NETINFO_STATIC
};

static wiz_uros_udp_params_t uros_params = {
    .agent_ip   = {192, 168, 123, 222}, //18
    .agent_port = 8888,
    .local_port = 9999
};

static void wiznet_init(void)
{
    wizchip_spi_initialize();

    wizchip_cris_initialize();

    wizchip_initialize();

    wizchip_setnetinfo(&netinfo);

    sleep_ms(200);
}

/*********************************/
/*     Inicialización Pines      */
/*********************************/

void gpio_init_custom(void)
{
    gpio_init(COP_LED);
    gpio_set_dir(COP_LED, GPIO_OUT);
    gpio_put(COP_LED, 0);

    gpio_init(COP_PATHERN);
    gpio_set_dir(COP_PATHERN, GPIO_OUT);
    gpio_put(COP_PATHERN, 0);

    gpio_init(LIGHT);
    gpio_set_dir(LIGHT, GPIO_OUT);
    gpio_put(LIGHT, 0);

    gpio_init(CAM);
    gpio_set_dir(CAM, GPIO_OUT);
    gpio_put(CAM, 0);
}

/*********************************/
/*     Funciones Envio Cores     */
/*********************************/

// Extracción y Liberación de la Cola : Lectura
static inline bool queue_try_get_latest(queue_t *q, void *out)
{
    bool has = false;
    while (queue_try_remove(q, out)) {
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
static inline void stamp_from_us(builtin_interfaces__msg__Time *t, uint64_t us)
{
    uint64_t ns = us * 1000ULL;
    t->sec = (int32_t)(ns / 1000000000ULL);
    t->nanosec = (uint32_t)(ns % 1000000000ULL);
}

static inline double nmea_degmin_to_deg(double degmin)
{
    int deg = (int)(degmin / 100.0);
    double minutes = degmin - (double)deg * 100.0;
    return (double)deg + minutes / 60.0;
}

static bool parse_gga_line(const char *line, gnss_fix_t *out)
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

    out->fix_q = (uint8_t)fix;
    out->sats  = (uint8_t)sats;
    out->hdop  = (float)hdop;
    out->alt_m = alt;

    if (fix <= 0 || lat_dm == 0 || lon_dm == 0 || ns == 0 || ew == 0) {
        out->valid = false;
        return true;
    }

    double lat = nmea_degmin_to_deg(lat_dm);
    double lon = nmea_degmin_to_deg(lon_dm);
    if (ns == 'S') lat = -lat;
    if (ew == 'W') lon = -lon;

    out->lat_deg = lat;
    out->lon_deg = lon;
    out->valid = true;
    return true;
}

static bool parse_headingga_line(char *line, *gnss_heading){

}

static int8_t navsat_status_from_fixq(uint8_t fixq)
{
    if (fixq == 0) return -1;
    if (fixq == 4 || fixq == 5) return 2;
    return 0;
}

/*********************************/
/*    Callbacks Timer y Subs     */
/*********************************/

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
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  
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
void subscription_callback_cam(const void * msgin)
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  
  CAM_resp_msg.data = false;

  if (msg->data) {
    gpio_put(CAM, 1);
    CAM_resp_msg.data = true;
  } else {
    gpio_put(CAM, 0);
    CAM_resp_msg.data = true;
  }
  RCCONTINUE(rcl_publish(&pub_CAM, &CAM_resp_msg, NULL));
}

//Callback Servo
void subscription_callback_servo(const void * msgin)
{
  const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
  
  SERVO_resp_msg.data = false;
  uint8_t a = msg->data;

  if (a <= 180) {
    queue_push_latest(&q_servo, &a, &servo_old);
    SERVO_resp_msg.data = true;
  }

  RCCONTINUE(rcl_publish(&pub_SERVO, &SERVO_resp_msg, NULL));
}

/*********************************/
/*       Función Secundaria      */
/*********************************/

void core1_entry(void)
{
    uint8_t imu  = 0;
    gnss_fix_t gnss;
    
    absolute_time_t next_imu  = get_absolute_time();

    const int imu_period_us  = TIMER_IMU_MS  * 1000;

    char line[220];
    int idx = 0;

    while (true)
    {
        absolute_time_t now = get_absolute_time();

        if (absolute_time_diff_us(now, next_imu) <= 0) {
            next_imu = delayed_by_us(next_imu, imu_period_us);
            imu++;
            queue_push_latest(&q_imu, &imu, &imu_old);
        }

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

        if (queue_try_get_latest(&q_servo, &servo_latest)) {
            uint8_t current_angle = servo_latest;

            printf("Angulo servo %d\n",current_angle);
        }

        tight_loop_contents();
    }
}

/*********************************/
/*       Función Principal       */
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
    while (rmw_uros_ping_agent(1000, 1) != RCL_RET_OK) {
        sleep_ms(500);
    }

    // Configuración Pines
    gpio_init_custom();

    // Configuración UART1

    uart_init(GNSS_UART, GNSS_BAUD);
    gpio_set_function(GNSS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GNSS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(GNSS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GNSS_UART, true);

    // Colas Core
    queue_init(&q_imu,  sizeof(uint8_t),  16);
    queue_init(&q_gnss, sizeof(gnss_fix_t),  8);
    queue_init(&q_servo, sizeof(uint8_t), 8);
    queue_init(&q_gnss_heading, sizeof(float), 8);

    multicore_launch_core1(core1_entry);

    // Configuración Nodo
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    RCABORT(rclc_support_init(&support, 0, NULL, &allocator));
    RCABORT(rclc_node_init_default(&node, "node", "pico", &support));

    // Configuración LED COP
    const rosidl_message_type_support_t * type_support_sub_cop = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8);
    RCABORT(rclc_subscription_init_default(&sub_COP, &node, type_support_sub_cop, "led_cop/req"));

    const rosidl_message_type_support_t * type_support_pub_cop = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);
    RCABORT(rclc_publisher_init_default(&pub_COP, &node, type_support_pub_cop, "led_cop/resp"));

    // Configuración LIGHT
    const rosidl_message_type_support_t * type_support_sub_light = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);
    RCABORT(rclc_subscription_init_default(&sub_LIGHT, &node, type_support_sub_light, "led_light/req"));

    const rosidl_message_type_support_t * type_support_pub_light = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);
    RCABORT(rclc_publisher_init_default(&pub_LIGHT, &node, type_support_pub_light, "led_light/resp"));

    // Configuración CAM
    const rosidl_message_type_support_t * type_support_sub_cam = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);
    RCABORT(rclc_subscription_init_default(&sub_CAM, &node, type_support_sub_cam, "led_cam/req"));

    const rosidl_message_type_support_t * type_support_pub_cam = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);
    RCABORT(rclc_publisher_init_default(&pub_CAM, &node, type_support_pub_cam, "led_cam/resp"));

    // Configuración SERVO
    const rosidl_message_type_support_t * type_support_sub_servo = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8);
    RCABORT(rclc_subscription_init_default(&sub_SERVO, &node, type_support_sub_servo, "servo/req"));

    const rosidl_message_type_support_t * type_support_pub_servo = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);
    RCABORT(rclc_publisher_init_default(&pub_SERVO, &node, type_support_pub_servo, "servo/resp"));

    // Configuración GNSS
    const rosidl_message_type_support_t * type_support_pub_gnss = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix);
    RCABORT(rclc_publisher_init_default(&pub_gnss, &node, type_support_pub_gnss, "gnss/fix"));

    const rosidl_message_type_support_t * type_support_pub_gnss_quality = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8);
    RCABORT(rclc_publisher_init_default(&pub_gnss_quality, &node, type_support_pub_gnss_quality, "gnss/fix_quality"));

    const rosidl_message_type_support_t * type_support_pub_gnss_heading = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Imu);
    RCABORT(rclc_publisher_init_default(&pub_gnss_heading, &node, type_support_pub_gnss_heading, "gnss/heading"));

    const rosidl_message_type_support_t * type_support_pub_gnss_heading_quality = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8);
    RCABORT(rclc_publisher_init_default(&pub_gnss_heading, &node, type_support_pub_gnss_heading_quality, "gnss/heading_quality"));

    // Configuración IMU
    const rosidl_message_type_support_t * type_support_pub_imu = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8);
    RCABORT(rclc_publisher_init_best_effort(&pub_imu, &node, type_support_pub_imu, "imu"));

    // Incialización mensajes
    std_msgs__msg__UInt8__init(&COP_req_msg);
    std_msgs__msg__Bool__init(&COP_resp_msg);
    std_msgs__msg__Bool__init(&LIGHT_req_msg);  
    std_msgs__msg__Bool__init(&LIGHT_resp_msg);
    std_msgs__msg__Bool__init(&CAM_req_msg);
    std_msgs__msg__Bool__init(&CAM_resp_msg);
    std_msgs__msg__UInt8__init(&SERVO_req_msg);
    std_msgs__msg__Bool__init(&SERVO_resp_msg);
    sensor_msgs__msg__NavSatFix__init(&gnss_msg);
    std_msgs__msg__UInt8__init(&gnss_quality_msg);
    sensor_msgs__msg__Imu__init(&gnss_heading_msg);    
    std_msgs__msg__UInt8__init(&gnss_heading_quality_msg);
    std_msgs__msg__UInt8__init(&imu_msg);

    rosidl_runtime_c__String__assign(&gnss_msg.header.frame_id, "gnss");
    gnss_msg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;

    rosidl_runtime_c__String__assign(&gnss_heading_msg.header.frame_id, "gnss_heading");
    gnss_heading_msg.angular_velocity_covariance[0] = -1;
    gnss_heading_msg.linear_acceleration_covariance[0] = -1;
    float var = 0.0001f;

    gnss_heading_msg.orientation_covariance[0] = var;
    gnss_heading_msg.orientation_covariance[4] = var;
    gnss_heading_msg.orientation_covariance[8] = var;   

    // Configuración Executor
    rclc_executor_t executor;
    executor = rclc_executor_get_zero_initialized_executor();
    unsigned int num_handles = 4;
    RCABORT(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
    RCABORT(rclc_executor_add_subscription(&executor, &sub_COP, &COP_req_msg, &subscription_callback_cop, ON_NEW_DATA));
    RCABORT(rclc_executor_add_subscription(&executor, &sub_LIGHT, &LIGHT_req_msg, &subscription_callback_light, ON_NEW_DATA));
    RCABORT(rclc_executor_add_subscription(&executor, &sub_CAM, &CAM_req_msg, &subscription_callback_cam, ON_NEW_DATA));
    RCABORT(rclc_executor_add_subscription(&executor, &sub_SERVO, &SERVO_req_msg, &subscription_callback_servo, ON_NEW_DATA));

    rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(5));

    next_ping = make_timeout_time_ms(1000);
    ping_fail = 0;
    
    uint64_t now_us;
    
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

        if (queue_try_get_latest(&q_imu, &imu_latest)) {
            imu_msg.data = imu_latest;
            RCCONTINUE(rcl_publish(&pub_imu, &imu_msg, NULL));
        }

        if (queue_try_get_latest(&q_gnss, &gnss_latest)) {

            now_us = time_us_64();
            stamp_from_us(&gnss_msg.header.stamp, now_us);

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

        if (queue_try_get_latest(&q_gnss_heading, &gnss_heading_latest)) {
            
            now_us = time_us_64();
            stamp_from_us(&gnss_heading_msg.header.stamp, now_us);

            gnss_heading_quality_msg.data = ;

            RCCONTINUE(rcl_publish(&pub_gnss_heading, &gnss_heading_msg, NULL));
            RCCONTINUE(rcl_publish(&pub_gnss_heading_quality, &gnss_heading_quality_msg, NULL));
        }
        
        if (absolute_time_diff_us(get_absolute_time(), next_ping) <= 0)
        {
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
                sleep_ms(20);
                watchdog_reboot(0, 0, 0);
            }
        }
    }

    rclc_executor_fini(&executor);
    rcl_publisher_fini(&pub_COP, &node);
    rcl_publisher_fini(&pub_LIGHT, &node);
    rcl_publisher_fini(&pub_CAM, &node);
    rcl_publisher_fini(&pub_SERVO, &node);
    rcl_subscription_fini(&sub_COP, &node);
    rcl_subscription_fini(&sub_LIGHT, &node);
    rcl_subscription_fini(&sub_CAM, &node);
    rcl_subscription_fini(&sub_SERVO, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return 0;
}
