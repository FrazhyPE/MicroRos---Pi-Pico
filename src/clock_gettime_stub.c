#include <time.h>
#include "pico/time.h"

int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    (void)clk_id;
    uint64_t us = time_us_64();
    tp->tv_sec  = (time_t)(us / 1000000ULL);
    tp->tv_nsec = (long)((us % 1000000ULL) * 1000ULL);
    return 0;
}
