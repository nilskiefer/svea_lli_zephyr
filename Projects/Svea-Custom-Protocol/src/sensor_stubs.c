#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "channels.h"

LOG_MODULE_REGISTER(sensor_stubs, LOG_LEVEL_INF);

#define LSM6DSOX_PERIOD_MS 5
#define ADS1115_PERIOD_MS 10
#define INA3221_PERIOD_MS 25
#define BQ76942_PERIOD_MS 100
#define INA226_PERIOD_MS 20

#define STUB_STACK_SIZE 1024
#define STUB_THREAD_PRIO 4

static K_THREAD_STACK_DEFINE(lsm6dsox_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(ads1115_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(ina3221_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(bq76942_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(ina226_stack, STUB_STACK_SIZE);

static struct k_thread lsm6dsox_thread;
static struct k_thread ads1115_thread;
static struct k_thread ina3221_thread;
static struct k_thread bq76942_thread;
static struct k_thread ina226_thread;

static void lsm6dsox_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t seq = 0;

    while (1) {
        struct lsm6dsox_msg msg;
        int32_t phase = (int32_t)(seq % 400U) - 200;

        msg.t_ms = (uint32_t)k_uptime_get();
        msg.seq = seq;
        msg.ax_mg = phase * 4;
        msg.ay_mg = -phase * 3;
        msg.az_mg = 1000 + (phase / 4);
        msg.gx_mdps = phase * 20;
        msg.gy_mdps = (200 - phase) * 10;
        msg.gz_mdps = -phase * 12;

        (void)zbus_chan_pub(&lsm6dsox_chan, &msg, K_MSEC(5));
        seq++;
        k_sleep(K_MSEC(LSM6DSOX_PERIOD_MS));
    }
}

static void ads1115_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t seq = 0;

    while (1) {
        struct ads1115_msg msg;
        int32_t phase = (int32_t)(seq % 200U) - 100;

        msg.t_ms = (uint32_t)k_uptime_get();
        msg.seq = seq;
        msg.ain0_mv = 1650 + (phase * 8);

        (void)zbus_chan_pub(&ads1115_chan, &msg, K_MSEC(5));
        seq++;
        k_sleep(K_MSEC(ADS1115_PERIOD_MS));
    }
}

static void ina3221_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t seq = 0;

    while (1) {
        int32_t ripple = (int32_t)(seq % 120U) - 60;
        struct ina3221_msg a_msg;
        struct ina3221_msg b_msg;

        a_msg.t_ms = (uint32_t)k_uptime_get();
        a_msg.seq = seq;
        a_msg.bus_mv = 24000 + ripple * 4;
        a_msg.current_ma = 1400 + ripple * 6;
        a_msg.power_mw = (a_msg.bus_mv * a_msg.current_ma) / 1000;

        b_msg.t_ms = a_msg.t_ms;
        b_msg.seq = seq;
        b_msg.bus_mv = 12000 + ripple * 3;
        b_msg.current_ma = 700 + ripple * 4;
        b_msg.power_mw = (b_msg.bus_mv * b_msg.current_ma) / 1000;

        (void)zbus_chan_pub(&ina3221_a_chan, &a_msg, K_MSEC(5));
        (void)zbus_chan_pub(&ina3221_b_chan, &b_msg, K_MSEC(5));

        seq++;
        k_sleep(K_MSEC(INA3221_PERIOD_MS));
    }
}

static void bq76942_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t seq = 0;

    while (1) {
        struct bq76942_msg msg;
        int32_t phase = (int32_t)(seq % 100U) - 50;

        msg.t_ms = (uint32_t)k_uptime_get();
        msg.seq = seq;
        msg.pack_mv = 24000 + phase * 6;
        msg.pack_ma = 1200 + phase * 5;
        msg.soc_deci_pct = 820 - (int32_t)(seq % 20U);
        msg.temp_cdeg = 2550 + phase * 2;

        (void)zbus_chan_pub(&bq76942_chan, &msg, K_MSEC(5));
        seq++;
        k_sleep(K_MSEC(BQ76942_PERIOD_MS));
    }
}

static void ina226_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t seq = 0;

    while (1) {
        int32_t ripple = (int32_t)(seq % 80U) - 40;
        struct ina226_msg a_msg;
        struct ina226_msg b_msg;

        a_msg.t_ms = (uint32_t)k_uptime_get();
        a_msg.seq = seq;
        a_msg.bus_mv = 5000 + ripple * 3;
        a_msg.current_ma = 1800 + ripple * 8;
        a_msg.power_mw = (a_msg.bus_mv * a_msg.current_ma) / 1000;

        b_msg.t_ms = a_msg.t_ms;
        b_msg.seq = seq;
        b_msg.bus_mv = 3300 + ripple * 2;
        b_msg.current_ma = 900 + ripple * 5;
        b_msg.power_mw = (b_msg.bus_mv * b_msg.current_ma) / 1000;

        (void)zbus_chan_pub(&ina226_a_chan, &a_msg, K_MSEC(5));
        (void)zbus_chan_pub(&ina226_b_chan, &b_msg, K_MSEC(5));

        seq++;
        k_sleep(K_MSEC(INA226_PERIOD_MS));
    }
}

void lsm6dsox_stub_start(void) {
    k_thread_create(&lsm6dsox_thread,
                    lsm6dsox_stack,
                    K_THREAD_STACK_SIZEOF(lsm6dsox_stack),
                    lsm6dsox_thread_fn,
                    NULL,
                    NULL,
                    NULL,
                    STUB_THREAD_PRIO,
                    0,
                    K_NO_WAIT);
    k_thread_name_set(&lsm6dsox_thread, "lsm6dsox_stub");
    LOG_INF("LSM6DSOX stub started (%d Hz)", 1000 / LSM6DSOX_PERIOD_MS);
}

void ads1115_stub_start(void) {
    k_thread_create(&ads1115_thread,
                    ads1115_stack,
                    K_THREAD_STACK_SIZEOF(ads1115_stack),
                    ads1115_thread_fn,
                    NULL,
                    NULL,
                    NULL,
                    STUB_THREAD_PRIO,
                    0,
                    K_NO_WAIT);
    k_thread_name_set(&ads1115_thread, "ads1115_stub");
    LOG_INF("ADS1115 stub started (%d Hz)", 1000 / ADS1115_PERIOD_MS);
}

void ina3221_stub_start(void) {
    k_thread_create(&ina3221_thread,
                    ina3221_stack,
                    K_THREAD_STACK_SIZEOF(ina3221_stack),
                    ina3221_thread_fn,
                    NULL,
                    NULL,
                    NULL,
                    STUB_THREAD_PRIO,
                    0,
                    K_NO_WAIT);
    k_thread_name_set(&ina3221_thread, "ina3221_stub");
    LOG_INF("INA3221 stubs started (%d Hz each)", 1000 / INA3221_PERIOD_MS);
}

void bq76942_stub_start(void) {
    k_thread_create(&bq76942_thread,
                    bq76942_stack,
                    K_THREAD_STACK_SIZEOF(bq76942_stack),
                    bq76942_thread_fn,
                    NULL,
                    NULL,
                    NULL,
                    STUB_THREAD_PRIO,
                    0,
                    K_NO_WAIT);
    k_thread_name_set(&bq76942_thread, "bq76942_stub");
    LOG_INF("BQ76942 stub started (%d Hz)", 1000 / BQ76942_PERIOD_MS);
}

void ina226_stub_start(void) {
    k_thread_create(&ina226_thread,
                    ina226_stack,
                    K_THREAD_STACK_SIZEOF(ina226_stack),
                    ina226_thread_fn,
                    NULL,
                    NULL,
                    NULL,
                    STUB_THREAD_PRIO,
                    0,
                    K_NO_WAIT);
    k_thread_name_set(&ina226_thread, "ina226_stub");
    LOG_INF("INA226 stubs started (%d Hz each)", 1000 / INA226_PERIOD_MS);
}
