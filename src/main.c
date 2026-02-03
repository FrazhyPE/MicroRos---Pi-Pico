#include <stdio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include "pico/stdlib.h"

#include "wizchip_conf.h"
#include "wizchip_spi.h"

#include "wiz_udp_transport.h"

static wiz_NetInfo netinfo = {
    .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56},
    .ip  = {192, 168, 123, 2},
    .sn  = {255, 255, 255, 0},
    .gw  = {0, 0, 0, 0},
    .dns = {8, 8, 8, 8},
    .dhcp = NETINFO_STATIC
};

static wiz_uros_udp_params_t uros_params = {
    .agent_ip   = {192, 168, 123, 222},
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


int main(void)
{
    stdio_init_all();
    sleep_ms(1500);

    wiznet_init();

    rmw_uros_set_custom_transport(
        false,               // UDP => false
        &uros_params,
        wiz_uros_udp_open,
        wiz_uros_udp_close,
        wiz_uros_udp_write,
        wiz_uros_udp_read
    );

    while (rmw_uros_ping_agent(1000, 1) != RCL_RET_OK) {
        sleep_ms(500);
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_publisher_t pub;
    std_msgs__msg__Int32 msg;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_w5500_node", "", &support);

    rclc_publisher_init_default(
        &pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_int"
    );

    msg.data = 0;

    while (true) {
        msg.data++;
        (void)rcl_publish(&pub, &msg, NULL);
        sleep_ms(1000);
    }
}
