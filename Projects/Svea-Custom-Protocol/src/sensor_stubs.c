#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "channels.h"

LOG_MODULE_REGISTER(sensor_stubs, LOG_LEVEL_INF);

#define LSM6DSOX_PERIOD_MS 1000 / 200
#define ADS1115_PERIOD_MS 10
#define INA3221_PERIOD_MS 25
#define BQ76942_PERIOD_MS 100
#define INA226_PERIOD_MS 20
#define HEARTBEAT_PERIOD_MS 100
#define RC_COMMAND_PERIOD_MS 10

#define STUB_STACK_SIZE 1024
#define STUB_THREAD_PRIO 4

static K_THREAD_STACK_DEFINE(lsm6dsox_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(ads1115_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(ina3221_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(bq76942_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(ina226_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(heartbeat_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(rc_command_stack, STUB_STACK_SIZE);

static struct k_thread lsm6dsox_thread;
static struct k_thread ads1115_thread;
static struct k_thread ina3221_thread;
static struct k_thread bq76942_thread;
static struct k_thread ina226_thread;
static struct k_thread heartbeat_thread;
static struct k_thread rc_command_thread;

/*
 * Periodic pacing with bounded catch-up:
 * if a thread falls behind by >1 period, resync to now+period instead of
 * trying to replay missed periods and hogging CPU.
 */
static void sleep_until_next_period(int64_t *next_deadline_ms, int32_t period_ms) {
    int64_t now = k_uptime_get();
    int64_t next = *next_deadline_ms + period_ms;

    if (now > (next + period_ms)) {
        next = now + period_ms;
    }

    *next_deadline_ms = next;
    k_sleep(K_TIMEOUT_ABS_MS(next));
}

static void lsm6dsox_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t seq = 0;
    int64_t next_deadline_ms = k_uptime_get();

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
        msg.temp_cdeg = 2550 + (phase / 8);

        (void)zbus_chan_pub(&lsm6dsox_chan, &msg, K_MSEC(5));
        seq++;
        sleep_until_next_period(&next_deadline_ms, LSM6DSOX_PERIOD_MS);
    }
}

static void ads1115_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t seq = 0;
    int64_t next_deadline_ms = k_uptime_get();

    while (1) {
        struct ads1115_msg msg;
        int32_t phase = (int32_t)(seq % 200U) - 100;

        msg.t_ms = (uint32_t)k_uptime_get();
        msg.seq = seq;
        msg.ain0_mv = 1650 + (phase * 8);
        msg.ain1_mv = 1200 + (phase * 5);
        msg.ain2_mv = 2400 - (phase * 4);
        msg.ain3_mv = 3300 - (phase * 2);

        (void)zbus_chan_pub(&ads1115_chan, &msg, K_MSEC(5));
        seq++;
        sleep_until_next_period(&next_deadline_ms, ADS1115_PERIOD_MS);
    }
}

static void ina3221_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t seq = 0;
    int64_t next_deadline_ms = k_uptime_get();

    while (1) {
        int32_t ripple = (int32_t)(seq % 120U) - 60;
        struct ina3221_msg a_msg;
        struct ina3221_msg b_msg;

        a_msg.t_ms = (uint32_t)k_uptime_get();
        a_msg.seq = seq;
        a_msg.ch1_bus_mv = 24000 + ripple * 4;
        a_msg.ch1_current_ma = 1400 + ripple * 6;
        a_msg.ch1_power_mw = (a_msg.ch1_bus_mv * a_msg.ch1_current_ma) / 1000;
        a_msg.ch2_bus_mv = 12000 + ripple * 3;
        a_msg.ch2_current_ma = 700 + ripple * 4;
        a_msg.ch2_power_mw = (a_msg.ch2_bus_mv * a_msg.ch2_current_ma) / 1000;
        a_msg.ch3_bus_mv = 5000 + ripple * 2;
        a_msg.ch3_current_ma = 380 + ripple * 2;
        a_msg.ch3_power_mw = (a_msg.ch3_bus_mv * a_msg.ch3_current_ma) / 1000;

        b_msg.t_ms = a_msg.t_ms;
        b_msg.seq = seq;
        b_msg.ch1_bus_mv = 24000 + ripple * 5;
        b_msg.ch1_current_ma = 900 + ripple * 3;
        b_msg.ch1_power_mw = (b_msg.ch1_bus_mv * b_msg.ch1_current_ma) / 1000;
        b_msg.ch2_bus_mv = 12000 + ripple * 2;
        b_msg.ch2_current_ma = 620 + ripple * 3;
        b_msg.ch2_power_mw = (b_msg.ch2_bus_mv * b_msg.ch2_current_ma) / 1000;
        b_msg.ch3_bus_mv = 5000 + ripple;
        b_msg.ch3_current_ma = 310 + ripple * 2;
        b_msg.ch3_power_mw = (b_msg.ch3_bus_mv * b_msg.ch3_current_ma) / 1000;

        (void)zbus_chan_pub(&ina3221_a_chan, &a_msg, K_MSEC(5));
        (void)zbus_chan_pub(&ina3221_b_chan, &b_msg, K_MSEC(5));

        seq++;
        sleep_until_next_period(&next_deadline_ms, INA3221_PERIOD_MS);
    }
}

static void bq76942_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t seq = 0;
    int64_t next_deadline_ms = k_uptime_get();

    while (1) {
        struct bq76942_msg msg;
        int32_t phase = (int32_t)(seq % 100U) - 50;

        msg.t_ms = (uint32_t)k_uptime_get();
        msg.seq = seq;
        msg.pack_mv = 24000 + phase * 6;
        msg.pack_ma = 1200 + phase * 5;
        msg.soc_deci_pct = 820 - (int32_t)(seq % 20U);
        msg.temp_cdeg = 2550 + phase * 2;
        msg.cell_min_mv = 3950 + phase;
        msg.cell_avg_mv = 4010 + phase;
        msg.cell_max_mv = 4080 + phase;
        msg.error_flags = (seq % 300U == 0U) ? 0x00000004 : 0;

        (void)zbus_chan_pub(&bq76942_chan, &msg, K_MSEC(5));
        seq++;
        sleep_until_next_period(&next_deadline_ms, BQ76942_PERIOD_MS);
    }
}

static void ina226_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t seq = 0;
    int64_t next_deadline_ms = k_uptime_get();

    while (1) {
        int32_t ripple = (int32_t)(seq % 80U) - 40;
        struct ina226_msg a_msg;
        struct ina226_msg b_msg;

        a_msg.t_ms = (uint32_t)k_uptime_get();
        a_msg.seq = seq;
        a_msg.bus_mv = 5000 + ripple * 3;
        a_msg.shunt_uv = 10000 + ripple * 40;
        a_msg.current_ma = 1800 + ripple * 8;
        a_msg.power_mw = (a_msg.bus_mv * a_msg.current_ma) / 1000;

        b_msg.t_ms = a_msg.t_ms;
        b_msg.seq = seq;
        b_msg.bus_mv = 3300 + ripple * 2;
        b_msg.shunt_uv = 6500 + ripple * 35;
        b_msg.current_ma = 900 + ripple * 5;
        b_msg.power_mw = (b_msg.bus_mv * b_msg.current_ma) / 1000;

        (void)zbus_chan_pub(&ina226_a_chan, &a_msg, K_MSEC(5));
        (void)zbus_chan_pub(&ina226_b_chan, &b_msg, K_MSEC(5));

        seq++;
        sleep_until_next_period(&next_deadline_ms, INA226_PERIOD_MS);
    }
}

static void heartbeat_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t seq = 0;
    int64_t next_deadline_ms = k_uptime_get();

    while (1) {
        struct heartbeat_msg msg;
        msg.t_ms = (uint32_t)k_uptime_get();
        msg.seq = seq++;
        (void)zbus_chan_pub(&heartbeat_chan, &msg, K_MSEC(5));
        sleep_until_next_period(&next_deadline_ms, HEARTBEAT_PERIOD_MS);
    }
}

static void rc_command_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t seq = 0;
    int64_t next_deadline_ms = k_uptime_get();

    while (1) {
        struct rc_command_msg msg;
        int32_t phase = (int32_t)(seq % 256U) - 128;

        msg.t_ms = (uint32_t)k_uptime_get();
        msg.seq = seq;
        msg.steering = phase;
        msg.throttle = (int32_t)((seq % 200U) - 100);
        msg.high_gear = (seq / 200U) % 2U;
        msg.diff_lock = (seq / 150U) % 2U;
        msg.override_mode = (seq / 300U) % 3U;      /* 0=ROS, 1=MUTE, 2=REMOTE */
        msg.connected = ((seq / 500U) % 10U) != 9U; /* drop link briefly */

        (void)zbus_chan_pub(&rc_command_chan, &msg, K_MSEC(5));
        seq++;
        sleep_until_next_period(&next_deadline_ms, RC_COMMAND_PERIOD_MS);
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

void heartbeat_stub_start(void) {
    k_thread_create(&heartbeat_thread,
                    heartbeat_stack,
                    K_THREAD_STACK_SIZEOF(heartbeat_stack),
                    heartbeat_thread_fn,
                    NULL,
                    NULL,
                    NULL,
                    STUB_THREAD_PRIO,
                    0,
                    K_NO_WAIT);
    k_thread_name_set(&heartbeat_thread, "heartbeat_stub");
    LOG_INF("Heartbeat stub started (%d Hz)", 1000 / HEARTBEAT_PERIOD_MS);
}

void rc_command_stub_start(void) {
    k_thread_create(&rc_command_thread,
                    rc_command_stack,
                    K_THREAD_STACK_SIZEOF(rc_command_stack),
                    rc_command_thread_fn,
                    NULL,
                    NULL,
                    NULL,
                    STUB_THREAD_PRIO,
                    0,
                    K_NO_WAIT);
    k_thread_name_set(&rc_command_thread, "rc_command_stub");
    LOG_INF("RC command stub started (%d Hz)", 1000 / RC_COMMAND_PERIOD_MS);
}
