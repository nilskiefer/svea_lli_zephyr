#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <errno.h>

#include "channels.h"

LOG_MODULE_REGISTER(uart_publisher, LOG_LEVEL_INF);

#if !DT_NODE_HAS_STATUS(DT_ALIAS(telemetry_uart), okay)
#error "telemetry-uart alias is missing or disabled"
#endif

#define UART_PUB_STACK_SIZE 1536
#define UART_PUB_THREAD_PRIO 3
#define HEARTBEAT_STACK_SIZE 768
#define HEARTBEAT_THREAD_PRIO 1
#define HEARTBEAT_PERIOD_MS 100
#define UART_DEBUG_EVERY_N_MSGS 50U

static K_THREAD_STACK_DEFINE(uart_pub_stack, UART_PUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(heartbeat_stack, HEARTBEAT_STACK_SIZE);
static struct k_thread uart_pub_thread;
static struct k_thread heartbeat_thread;

static const struct device *telemetry_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
static struct k_mutex uart_tx_lock;
static uint32_t dbg_lsm6dsox_sent;
static uint32_t dbg_ads1115_sent;
static uint32_t dbg_ina3221_a_sent;
static uint32_t dbg_ina3221_b_sent;
static uint32_t dbg_bq76942_sent;
static uint32_t dbg_ina226_a_sent;
static uint32_t dbg_ina226_b_sent;
static uint32_t heartbeat_seq;

static void telemetry_wait_for_host(void)
{
    uint32_t dtr = 0;
    int ret;
    int attempts = 50; /* 5 seconds max */

    /* Try to honor DTR when available, but never block forever on UART links. */
    while (attempts-- > 0) {
        ret = uart_line_ctrl_get(telemetry_uart, UART_LINE_CTRL_DTR, &dtr);
        if (ret != 0) {
            LOG_INF("No modem-control support on %s (ret=%d), continuing",
                    telemetry_uart->name, ret);
            return;
        }

        if (dtr != 0U) {
            (void)uart_line_ctrl_set(telemetry_uart, UART_LINE_CTRL_DCD, 1);
            (void)uart_line_ctrl_set(telemetry_uart, UART_LINE_CTRL_DSR, 1);
            LOG_INF("Host connected to telemetry serial");
            return;
        }

        k_sleep(K_MSEC(100));
    }

    LOG_INF("No DTR detected on %s, continuing without host handshake",
            telemetry_uart->name);
}

static void uart_write_str(const char *str)
{
    k_mutex_lock(&uart_tx_lock, K_FOREVER);
    for (size_t i = 0; str[i] != '\0'; i++) {
        uart_poll_out(telemetry_uart, str[i]);
    }
    k_mutex_unlock(&uart_tx_lock);
}

static void uart_write_debug(const char *event, int32_t value)
{
    char line[160];

    snprintk(line,
             sizeof(line),
             "{\"topic\":\"debug\",\"event\":\"%s\",\"uart\":\"%s\",\"v\":%d}\n",
             event,
             telemetry_uart->name,
             value);
    uart_write_str(line);
}

static void uart_pub_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    const struct zbus_channel *chan;
    char line[192];

    uart_write_debug("uart_pub_thread_start", 0);
    uart_write_str("{\"topic\":\"status\",\"msg\":\"telemetry_online\"}\n");

    while (1) {
        if (zbus_sub_wait(&uart_pub_sub, &chan, K_FOREVER) != 0) {
            continue;
        }

        if (chan == &lsm6dsox_chan) {
            struct lsm6dsox_msg msg;
            if (zbus_chan_read(&lsm6dsox_chan, &msg, K_MSEC(5)) == 0) {
                snprintk(line,
                         sizeof(line),
                         "{\"topic\":\"lsm6dsox\",\"t_ms\":%u,\"seq\":%u,\"ax_mg\":%d,\"ay_mg\":%d,\"az_mg\":%d,\"gx_mdps\":%d,\"gy_mdps\":%d,\"gz_mdps\":%d}\n",
                         msg.t_ms,
                         msg.seq,
                         msg.ax_mg,
                         msg.ay_mg,
                         msg.az_mg,
                         msg.gx_mdps,
                         msg.gy_mdps,
                         msg.gz_mdps);
                uart_write_str(line);
                dbg_lsm6dsox_sent++;
                if ((dbg_lsm6dsox_sent % UART_DEBUG_EVERY_N_MSGS) == 0U) {
                    uart_write_debug("lsm6dsox_sent", (int32_t)dbg_lsm6dsox_sent);
                }
            }
        } else if (chan == &ads1115_chan) {
            struct ads1115_msg msg;
            if (zbus_chan_read(&ads1115_chan, &msg, K_MSEC(5)) == 0) {
                snprintk(line,
                         sizeof(line),
                         "{\"topic\":\"ads1115\",\"t_ms\":%u,\"seq\":%u,\"ain0_mv\":%d}\n",
                         msg.t_ms,
                         msg.seq,
                         msg.ain0_mv);
                uart_write_str(line);
                dbg_ads1115_sent++;
                if ((dbg_ads1115_sent % UART_DEBUG_EVERY_N_MSGS) == 0U) {
                    uart_write_debug("ads1115_sent", (int32_t)dbg_ads1115_sent);
                }
            }
        } else if (chan == &ina3221_a_chan) {
            struct ina3221_msg msg;
            if (zbus_chan_read(&ina3221_a_chan, &msg, K_MSEC(5)) == 0) {
                snprintk(line,
                         sizeof(line),
                         "{\"topic\":\"ina3221_a\",\"t_ms\":%u,\"seq\":%u,\"bus_mv\":%d,\"current_ma\":%d,\"power_mw\":%d}\n",
                         msg.t_ms,
                         msg.seq,
                         msg.bus_mv,
                         msg.current_ma,
                         msg.power_mw);
                uart_write_str(line);
                dbg_ina3221_a_sent++;
                if ((dbg_ina3221_a_sent % UART_DEBUG_EVERY_N_MSGS) == 0U) {
                    uart_write_debug("ina3221_a_sent", (int32_t)dbg_ina3221_a_sent);
                }
            }
        } else if (chan == &ina3221_b_chan) {
            struct ina3221_msg msg;
            if (zbus_chan_read(&ina3221_b_chan, &msg, K_MSEC(5)) == 0) {
                snprintk(line,
                         sizeof(line),
                         "{\"topic\":\"ina3221_b\",\"t_ms\":%u,\"seq\":%u,\"bus_mv\":%d,\"current_ma\":%d,\"power_mw\":%d}\n",
                         msg.t_ms,
                         msg.seq,
                         msg.bus_mv,
                         msg.current_ma,
                         msg.power_mw);
                uart_write_str(line);
                dbg_ina3221_b_sent++;
                if ((dbg_ina3221_b_sent % UART_DEBUG_EVERY_N_MSGS) == 0U) {
                    uart_write_debug("ina3221_b_sent", (int32_t)dbg_ina3221_b_sent);
                }
            }
        } else if (chan == &bq76942_chan) {
            struct bq76942_msg msg;
            if (zbus_chan_read(&bq76942_chan, &msg, K_MSEC(5)) == 0) {
                snprintk(line,
                         sizeof(line),
                         "{\"topic\":\"bq76942\",\"t_ms\":%u,\"seq\":%u,\"pack_mv\":%d,\"pack_ma\":%d,\"soc_deci_pct\":%d,\"temp_cdeg\":%d}\n",
                         msg.t_ms,
                         msg.seq,
                         msg.pack_mv,
                         msg.pack_ma,
                         msg.soc_deci_pct,
                         msg.temp_cdeg);
                uart_write_str(line);
                dbg_bq76942_sent++;
                if ((dbg_bq76942_sent % UART_DEBUG_EVERY_N_MSGS) == 0U) {
                    uart_write_debug("bq76942_sent", (int32_t)dbg_bq76942_sent);
                }
            }
        } else if (chan == &ina226_a_chan) {
            struct ina226_msg msg;
            if (zbus_chan_read(&ina226_a_chan, &msg, K_MSEC(5)) == 0) {
                snprintk(line,
                         sizeof(line),
                         "{\"topic\":\"ina226_a\",\"t_ms\":%u,\"seq\":%u,\"bus_mv\":%d,\"current_ma\":%d,\"power_mw\":%d}\n",
                         msg.t_ms,
                         msg.seq,
                         msg.bus_mv,
                         msg.current_ma,
                         msg.power_mw);
                uart_write_str(line);
                dbg_ina226_a_sent++;
                if ((dbg_ina226_a_sent % UART_DEBUG_EVERY_N_MSGS) == 0U) {
                    uart_write_debug("ina226_a_sent", (int32_t)dbg_ina226_a_sent);
                }
            }
        } else if (chan == &ina226_b_chan) {
            struct ina226_msg msg;
            if (zbus_chan_read(&ina226_b_chan, &msg, K_MSEC(5)) == 0) {
                snprintk(line,
                         sizeof(line),
                         "{\"topic\":\"ina226_b\",\"t_ms\":%u,\"seq\":%u,\"bus_mv\":%d,\"current_ma\":%d,\"power_mw\":%d}\n",
                         msg.t_ms,
                         msg.seq,
                         msg.bus_mv,
                         msg.current_ma,
                         msg.power_mw);
                uart_write_str(line);
                dbg_ina226_b_sent++;
                if ((dbg_ina226_b_sent % UART_DEBUG_EVERY_N_MSGS) == 0U) {
                    uart_write_debug("ina226_b_sent", (int32_t)dbg_ina226_b_sent);
                }
            }
        } else {
            uart_write_debug("unknown_channel", -1);
        }
    }
}

static void heartbeat_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    char line[128];

    while (1) {
        heartbeat_seq++;
        snprintk(line,
                 sizeof(line),
                 "{\"topic\":\"heartbeat\",\"t_ms\":%u,\"seq\":%u}\n",
                 (uint32_t)k_uptime_get(),
                 heartbeat_seq);
        uart_write_str(line);
        k_sleep(K_MSEC(HEARTBEAT_PERIOD_MS));
    }
}

void uart_publisher_start(void)
{
    if (!device_is_ready(telemetry_uart)) {
        LOG_ERR("Telemetry UART not ready");
        return;
    }

    k_mutex_init(&uart_tx_lock);

    uart_write_debug("telemetry_uart_ready", 1);

    telemetry_wait_for_host();

    k_thread_create(&uart_pub_thread,
                    uart_pub_stack,
                    K_THREAD_STACK_SIZEOF(uart_pub_stack),
                    uart_pub_thread_fn,
                    NULL,
                    NULL,
                    NULL,
                    UART_PUB_THREAD_PRIO,
                    0,
                    K_NO_WAIT);

    k_thread_name_set(&uart_pub_thread, "uart_publisher");

    k_thread_create(&heartbeat_thread,
                    heartbeat_stack,
                    K_THREAD_STACK_SIZEOF(heartbeat_stack),
                    heartbeat_thread_fn,
                    NULL,
                    NULL,
                    NULL,
                    HEARTBEAT_THREAD_PRIO,
                    0,
                    K_NO_WAIT);
    k_thread_name_set(&heartbeat_thread, "uart_heartbeat");

    LOG_INF("UART publisher started on %s", telemetry_uart->name);
}
