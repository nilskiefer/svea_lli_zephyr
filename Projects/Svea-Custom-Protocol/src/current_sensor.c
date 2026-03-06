#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "channels.h"

LOG_MODULE_REGISTER(current_sensor, LOG_LEVEL_INF);

#define CURRENT_PERIOD_MS 50
#define CURRENT_STACK_SIZE 1024
#define CURRENT_THREAD_PRIO 4

static K_THREAD_STACK_DEFINE(current_stack, CURRENT_STACK_SIZE);
static struct k_thread current_thread;

static void current_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t seq = 0;

    while (1) {
        struct current_msg msg;
        int32_t ripple = (int32_t)(seq % 80U) - 40;

        msg.t_ms = (uint32_t)k_uptime_get();
        msg.seq = seq;

        msg.bus_mv = 24000 + ripple * 5;
        msg.current_ma = 1500 + ripple * 12;
        msg.power_mw = (msg.bus_mv * msg.current_ma) / 1000;

        int err = zbus_chan_pub(&current_chan, &msg, K_MSEC(5));
        if (err != 0) {
            LOG_WRN("current publish failed: %d", err);
        }

        seq++;
        k_sleep(K_MSEC(CURRENT_PERIOD_MS));
    }
}

void current_sensor_start(void)
{
    k_thread_create(&current_thread,
                    current_stack,
                    K_THREAD_STACK_SIZEOF(current_stack),
                    current_thread_fn,
                    NULL,
                    NULL,
                    NULL,
                    CURRENT_THREAD_PRIO,
                    0,
                    K_NO_WAIT);

    k_thread_name_set(&current_thread, "current_sensor");
    LOG_INF("Current stub sensor started");
}
