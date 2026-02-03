#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <uxr/client/profile/transport/custom/custom_transport.h>

typedef struct
{
    uint8_t agent_ip[4];
    uint16_t agent_port;
    uint16_t local_port;
} wiz_uros_udp_params_t;

bool   wiz_uros_udp_open(struct uxrCustomTransport * transport);
bool   wiz_uros_udp_close(struct uxrCustomTransport * transport);
size_t wiz_uros_udp_write(struct uxrCustomTransport* transport,
                          const uint8_t * buf, size_t len, uint8_t * err);
size_t wiz_uros_udp_read(struct uxrCustomTransport* transport,
                         uint8_t* buf, size_t len, int timeout, uint8_t* err);
