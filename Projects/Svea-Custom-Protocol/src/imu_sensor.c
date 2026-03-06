#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "channels.h"

LOG_MODULE_REGISTER(imu_sensor, LOG_LEVEL_INF);

#define IMU_PERIOD_MS 20
#define IMU_STACK_SIZE 1024
#define IMU_THREAD_PRIO 4

static K_THREAD_STACK_DEFINE(imu_stack, IMU_STACK_SIZE);
static struct k_thread imu_thread;

static void imu_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t seq = 0;

    while (1) {
        struct imu_msg msg;
        int32_t phase = (int32_t)(seq % 400U) - 200;

        msg.t_ms = (uint32_t)k_uptime_get();
        msg.seq = seq;

        msg.ax_mg = phase * 4;
        msg.ay_mg = -phase * 3;
        msg.az_mg = 1000 + (phase / 4);

        msg.gx_mdps = phase * 20;
        msg.gy_mdps = (200 - phase) * 10;
        msg.gz_mdps = -phase * 12;

        int err = zbus_chan_pub(&imu_chan, &msg, K_MSEC(5));
        if (err != 0) {
            LOG_WRN("imu publish failed: %d", err);
        }

        seq++;
        k_sleep(K_MSEC(IMU_PERIOD_MS));
    }
}

void imu_sensor_start(void)
{
    k_thread_create(&imu_thread,
                    imu_stack,
                    K_THREAD_STACK_SIZEOF(imu_stack),
                    imu_thread_fn,
                    NULL,
                    NULL,
                    NULL,
                    IMU_THREAD_PRIO,
                    0,
                    K_NO_WAIT);

    k_thread_name_set(&imu_thread, "imu_sensor");
    LOG_INF("IMU stub sensor started");
}
