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
#define HOST_CMD_STATS_PERIOD_MS 1000

#define STUB_STACK_SIZE 1024
#define STUB_THREAD_PRIO 4

static K_THREAD_STACK_DEFINE(lsm6dsox_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(ads1115_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(ina3221_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(bq76942_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(ina226_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(heartbeat_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(rc_command_stack, STUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(host_command_stack, STUB_STACK_SIZE);

static struct k_thread lsm6dsox_thread;
static struct k_thread ads1115_thread;
static struct k_thread ina3221_thread;
static struct k_thread bq76942_thread;
static struct k_thread ina226_thread;
static struct k_thread heartbeat_thread;
static struct k_thread rc_command_thread;
static struct k_thread host_command_thread;

static uint32_t hz_x10(uint32_t count_delta, int64_t dt_ms) {
    if (dt_ms <= 0) {
        return 0;
    }
    return (uint32_t)(((uint64_t)count_delta * 10000ULL) / (uint64_t)dt_ms);
}

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
        msg.ax_mg = (float)(phase * 4);
        msg.ay_mg = (float)(-phase * 3);
        msg.az_mg = (float)(1000 + (phase / 4));
        msg.gx_mdps = (float)(phase * 20);
        msg.gy_mdps = (float)((200 - phase) * 10);
        msg.gz_mdps = (float)(-phase * 12);
        msg.temp_cdeg = (float)(2550 + (phase / 8));

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
        msg.ain0_mv = (float)(1650 + (phase * 8));
        msg.ain1_mv = (float)(1200 + (phase * 5));
        msg.ain2_mv = (float)(2400 - (phase * 4));
        msg.ain3_mv = (float)(3300 - (phase * 2));

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
        a_msg.ch1_bus_mv = (float)(24000 + ripple * 4);
        a_msg.ch1_current_ma = (float)(1400 + ripple * 6);
        a_msg.ch1_power_mw = (a_msg.ch1_bus_mv * a_msg.ch1_current_ma) / 1000.0f;
        a_msg.ch2_bus_mv = (float)(12000 + ripple * 3);
        a_msg.ch2_current_ma = (float)(700 + ripple * 4);
        a_msg.ch2_power_mw = (a_msg.ch2_bus_mv * a_msg.ch2_current_ma) / 1000.0f;
        a_msg.ch3_bus_mv = (float)(5000 + ripple * 2);
        a_msg.ch3_current_ma = (float)(380 + ripple * 2);
        a_msg.ch3_power_mw = (a_msg.ch3_bus_mv * a_msg.ch3_current_ma) / 1000.0f;

        b_msg.t_ms = a_msg.t_ms;
        b_msg.seq = seq;
        b_msg.ch1_bus_mv = (float)(24000 + ripple * 5);
        b_msg.ch1_current_ma = (float)(900 + ripple * 3);
        b_msg.ch1_power_mw = (b_msg.ch1_bus_mv * b_msg.ch1_current_ma) / 1000.0f;
        b_msg.ch2_bus_mv = (float)(12000 + ripple * 2);
        b_msg.ch2_current_ma = (float)(620 + ripple * 3);
        b_msg.ch2_power_mw = (b_msg.ch2_bus_mv * b_msg.ch2_current_ma) / 1000.0f;
        b_msg.ch3_bus_mv = (float)(5000 + ripple);
        b_msg.ch3_current_ma = (float)(310 + ripple * 2);
        b_msg.ch3_power_mw = (b_msg.ch3_bus_mv * b_msg.ch3_current_ma) / 1000.0f;

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
        msg.pack_mv = (float)(24000 + phase * 6);
        msg.pack_ma = (float)(1200 + phase * 5);
        msg.soc_pct = 82.0f - (float)(seq % 20U) * 0.1f;
        msg.temp_cdeg = (float)(2550 + phase * 2);
        msg.cell_min_mv = (float)(3950 + phase);
        msg.cell_avg_mv = (float)(4010 + phase);
        msg.cell_max_mv = (float)(4080 + phase);
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
        a_msg.bus_mv = (float)(5000 + ripple * 3);
        a_msg.shunt_uv = (float)(10000 + ripple * 40);
        a_msg.current_ma = (float)(1800 + ripple * 8);
        a_msg.power_mw = (a_msg.bus_mv * a_msg.current_ma) / 1000.0f;

        b_msg.t_ms = a_msg.t_ms;
        b_msg.seq = seq;
        b_msg.bus_mv = (float)(3300 + ripple * 2);
        b_msg.shunt_uv = (float)(6500 + ripple * 35);
        b_msg.current_ma = (float)(900 + ripple * 5);
        b_msg.power_mw = (b_msg.bus_mv * b_msg.current_ma) / 1000.0f;

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
        msg.steering = (int8_t)phase;
        msg.throttle = (int8_t)((seq % 200U) - 100);
        msg.high_gear = ((seq / 200U) % 2U) != 0U;
        msg.diff_lock = ((seq / 150U) % 2U) != 0U;
        msg.override_mode = ((seq / 300U) % 2U) != 0U;
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

static void host_command_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    struct servo_control_msg servo = {0};
    struct host_heartbeat_msg hb = {0};
    uint32_t last_servo_seq = UINT32_MAX;
    uint32_t last_hb_seq = UINT32_MAX;
    uint32_t servo_count = 0;
    uint32_t hb_count = 0;
    uint32_t prev_servo_count = 0;
    uint32_t prev_hb_count = 0;
    int64_t start_ms = k_uptime_get();
    int64_t last_stats_ms = start_ms;

    while (1) {
        if (zbus_chan_read(&servo_control_chan, &servo, K_NO_WAIT) == 0 &&
            servo.seq != last_servo_seq) {
            last_servo_seq = servo.seq;
            servo_count++;
        }

        if (zbus_chan_read(&host_heartbeat_chan, &hb, K_NO_WAIT) == 0 &&
            hb.seq != last_hb_seq) {
            last_hb_seq = hb.seq;
            hb_count++;
        }

        int64_t now_ms = k_uptime_get();
        int64_t dt_ms = now_ms - last_stats_ms;
        if (dt_ms >= HOST_CMD_STATS_PERIOD_MS) {
            int64_t elapsed_ms = now_ms - start_ms;
            uint32_t total_count = servo_count + hb_count;
            uint32_t prev_total_count = prev_servo_count + prev_hb_count;

            uint32_t servo_hz1_x10 = hz_x10(servo_count - prev_servo_count, dt_ms);
            uint32_t hb_hz1_x10 = hz_x10(hb_count - prev_hb_count, dt_ms);
            uint32_t total_hz1_x10 = hz_x10(total_count - prev_total_count, dt_ms);

            uint32_t servo_hz_avg_x10 = hz_x10(servo_count, elapsed_ms);
            uint32_t hb_hz_avg_x10 = hz_x10(hb_count, elapsed_ms);
            uint32_t total_hz_avg_x10 = hz_x10(total_count, elapsed_ms);

            LOG_INF("Host RX Stats (%lld.%01llds)",
                    elapsed_ms / 1000,
                    (elapsed_ms % 1000) / 100);
            LOG_INF("topic             count  hz(1s)  hz(avg)");
            LOG_INF("servo_control %8u  %3u.%01u    %3u.%01u",
                    servo_count,
                    servo_hz1_x10 / 10, servo_hz1_x10 % 10,
                    servo_hz_avg_x10 / 10, servo_hz_avg_x10 % 10);
            LOG_INF("host_heartbeat %8u  %3u.%01u    %3u.%01u",
                    hb_count,
                    hb_hz1_x10 / 10, hb_hz1_x10 % 10,
                    hb_hz_avg_x10 / 10, hb_hz_avg_x10 % 10);
            LOG_INF("TOTAL          %8u  %3u.%01u    %3u.%01u",
                    total_count,
                    total_hz1_x10 / 10, total_hz1_x10 % 10,
                    total_hz_avg_x10 / 10, total_hz_avg_x10 % 10);

            prev_servo_count = servo_count;
            prev_hb_count = hb_count;
            last_stats_ms = now_ms;
        }

        k_sleep(K_MSEC(5));
    }
}

void host_command_stub_start(void) {
    k_thread_create(&host_command_thread,
                    host_command_stack,
                    K_THREAD_STACK_SIZEOF(host_command_stack),
                    host_command_thread_fn,
                    NULL,
                    NULL,
                    NULL,
                    STUB_THREAD_PRIO,
                    0,
                    K_NO_WAIT);
    k_thread_name_set(&host_command_thread, "host_cmd_stub");
    LOG_INF("Host command rate stub started (%d ms stats)", HOST_CMD_STATS_PERIOD_MS);
}
