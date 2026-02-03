#include "wiz_udp_transport.h"

#include "pico/stdlib.h"
#include "wizchip_conf.h"
#include "socket.h"

static int8_t g_sock = -1;

static wiz_uros_udp_params_t* params(struct uxrCustomTransport* t)
{
    return (wiz_uros_udp_params_t*) t->args;
}

bool wiz_uros_udp_open(struct uxrCustomTransport * transport)
{
    wiz_uros_udp_params_t* p = params(transport);

    if (g_sock >= 0) {
        close(g_sock);
        g_sock = -1;
    }

    g_sock = socket(0, Sn_MR_UDP, p->local_port, 0);
    return (g_sock >= 0);
}

bool wiz_uros_udp_close(struct uxrCustomTransport * transport)
{
    (void)transport;
    if (g_sock >= 0) {
        close(g_sock);
        g_sock = -1;
    }
    return true;
}

size_t wiz_uros_udp_write(struct uxrCustomTransport* transport,
                          const uint8_t * buf,
                          size_t len,
                          uint8_t * err)
{
    wiz_uros_udp_params_t* p = params(transport);

    if (g_sock < 0) { *err = 1; return 0; }

    int32_t r = sendto(g_sock, (uint8_t*)buf, (uint16_t)len, p->agent_ip, p->agent_port);
    if (r < 0) { *err = 1; return 0; }

    *err = 0;
    return (size_t)r;
}

size_t wiz_uros_udp_read(struct uxrCustomTransport* transport,
                         uint8_t* buf,
                         size_t len,
                         int timeout,
                         uint8_t* err)
{
    (void)transport;

    if (g_sock < 0) { *err = 1; return 0; }

    absolute_time_t t_end = make_timeout_time_ms(timeout);

    while (absolute_time_diff_us(get_absolute_time(), t_end) > 0) {

        if (getSn_RX_RSR(g_sock) > 0) {
            uint8_t rip[4];
            uint16_t rport;

            int32_t r = recvfrom(g_sock, buf, (uint16_t)len, rip, &rport);
            if (r > 0) {
                *err = 0;
                return (size_t)r;
            }
        }

        sleep_ms(1);
    }

    *err = 0;  
    return 0;
}
