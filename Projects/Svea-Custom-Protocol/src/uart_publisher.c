#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/logging/log.h>

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

#define TELEMETRY_FRAME_MAGIC 0xA5
#define TELEMETRY_MAX_CBOR_PAYLOAD 192

static K_THREAD_STACK_DEFINE(uart_pub_stack, UART_PUB_STACK_SIZE);
static K_THREAD_STACK_DEFINE(heartbeat_stack, HEARTBEAT_STACK_SIZE);
static struct k_thread uart_pub_thread;
static struct k_thread heartbeat_thread;

static const struct device *telemetry_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
static struct k_mutex uart_tx_lock;
static struct k_sem uart_tx_done_sem;
static bool uart_async_ready;
static __nocache uint8_t uart_tx_frame[3 + TELEMETRY_MAX_CBOR_PAYLOAD];
static uint32_t heartbeat_seq;

struct cbor_buf {
    uint8_t *buf;
    size_t len;
    size_t cap;
};

static bool cbor_put_byte(struct cbor_buf *b, uint8_t v)
{
    if (b->len >= b->cap) {
        return false;
    }
    b->buf[b->len++] = v;
    return true;
}

static bool cbor_put_type_val(struct cbor_buf *b, uint8_t major, uint32_t val)
{
    uint8_t prefix = (uint8_t)(major << 5);

    if (val <= 23U) {
        return cbor_put_byte(b, (uint8_t)(prefix | val));
    }

    if (val <= 0xFFU) {
        return cbor_put_byte(b, (uint8_t)(prefix | 24U)) &&
               cbor_put_byte(b, (uint8_t)val);
    }

    if (val <= 0xFFFFU) {
        return cbor_put_byte(b, (uint8_t)(prefix | 25U)) &&
               cbor_put_byte(b, (uint8_t)(val >> 8)) &&
               cbor_put_byte(b, (uint8_t)val);
    }

    return cbor_put_byte(b, (uint8_t)(prefix | 26U)) &&
           cbor_put_byte(b, (uint8_t)(val >> 24)) &&
           cbor_put_byte(b, (uint8_t)(val >> 16)) &&
           cbor_put_byte(b, (uint8_t)(val >> 8)) &&
           cbor_put_byte(b, (uint8_t)val);
}

static bool cbor_put_uint32(struct cbor_buf *b, uint32_t v)
{
    return cbor_put_type_val(b, 0U, v);
}

static bool cbor_put_int32(struct cbor_buf *b, int32_t v)
{
    if (v >= 0) {
        return cbor_put_type_val(b, 0U, (uint32_t)v);
    }

    return cbor_put_type_val(b, 1U, (uint32_t)(-1 - v));
}

static bool cbor_put_tstr(struct cbor_buf *b, const char *s)
{
    size_t n = strlen(s);
    if (n > UINT16_MAX) {
        return false;
    }

    if (!cbor_put_type_val(b, 3U, (uint32_t)n)) {
        return false;
    }

    for (size_t i = 0; i < n; i++) {
        if (!cbor_put_byte(b, (uint8_t)s[i])) {
            return false;
        }
    }

    return true;
}

static bool cbor_put_map_start(struct cbor_buf *b, uint32_t pairs)
{
    return cbor_put_type_val(b, 5U, pairs);
}

static void telemetry_wait_for_host(void)
{
    uint32_t dtr = 0;
    int ret;
    int attempts = 50; /* 5 seconds max */

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

static void uart_event_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    if (evt->type == UART_TX_DONE || evt->type == UART_TX_ABORTED) {
        k_sem_give(&uart_tx_done_sem);
    }
}

static void uart_send_cbor_payload(const uint8_t *payload, size_t len)
{
    if (len == 0 || len > UINT16_MAX) {
        return;
    }

    if (!uart_async_ready) {
        return;
    }

    k_mutex_lock(&uart_tx_lock, K_FOREVER);

    uart_tx_frame[0] = TELEMETRY_FRAME_MAGIC;
    uart_tx_frame[1] = (uint8_t)((len >> 8) & 0xFF);
    uart_tx_frame[2] = (uint8_t)(len & 0xFF);
    (void)memcpy(&uart_tx_frame[3], payload, len);

    k_sem_reset(&uart_tx_done_sem);
    int ret = uart_tx(telemetry_uart, uart_tx_frame, len + 3U, SYS_FOREVER_US);
    if (ret == 0) {
        (void)k_sem_take(&uart_tx_done_sem, K_FOREVER);
    } else {
        LOG_ERR("uart_tx failed: %d", ret);
    }

    k_mutex_unlock(&uart_tx_lock);
}

static void cbor_send_status_online(void)
{
    uint8_t payload[96];
    struct cbor_buf c = {.buf = payload, .len = 0, .cap = sizeof(payload)};

    bool ok = cbor_put_map_start(&c, 2U) &&
              cbor_put_tstr(&c, "topic") && cbor_put_tstr(&c, "status") &&
              cbor_put_tstr(&c, "msg") && cbor_put_tstr(&c, "telemetry_online");

    if (ok) {
        uart_send_cbor_payload(payload, c.len);
    }
}

static void cbor_send_heartbeat(uint32_t t_ms, uint32_t seq)
{
    uint8_t payload[96];
    struct cbor_buf c = {.buf = payload, .len = 0, .cap = sizeof(payload)};

    bool ok = cbor_put_map_start(&c, 3U) &&
              cbor_put_tstr(&c, "topic") && cbor_put_tstr(&c, "heartbeat") &&
              cbor_put_tstr(&c, "t_ms") && cbor_put_uint32(&c, t_ms) &&
              cbor_put_tstr(&c, "seq") && cbor_put_uint32(&c, seq);

    if (ok) {
        uart_send_cbor_payload(payload, c.len);
    }
}

static void uart_pub_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    const struct zbus_channel *chan;

    cbor_send_status_online();

    while (1) {
        if (zbus_sub_wait(&uart_pub_sub, &chan, K_FOREVER) != 0) {
            continue;
        }

        if (chan == &lsm6dsox_chan) {
            struct lsm6dsox_msg msg;
            if (zbus_chan_read(&lsm6dsox_chan, &msg, K_MSEC(5)) == 0) {
                uint8_t payload[TELEMETRY_MAX_CBOR_PAYLOAD];
                struct cbor_buf b = {.buf = payload, .len = 0, .cap = sizeof(payload)};

                bool ok = cbor_put_map_start(&b, 9U) &&
                          cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "lsm6dsox") &&
                          cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, msg.t_ms) &&
                          cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, msg.seq) &&
                          cbor_put_tstr(&b, "ax_mg") && cbor_put_int32(&b, msg.ax_mg) &&
                          cbor_put_tstr(&b, "ay_mg") && cbor_put_int32(&b, msg.ay_mg) &&
                          cbor_put_tstr(&b, "az_mg") && cbor_put_int32(&b, msg.az_mg) &&
                          cbor_put_tstr(&b, "gx_mdps") && cbor_put_int32(&b, msg.gx_mdps) &&
                          cbor_put_tstr(&b, "gy_mdps") && cbor_put_int32(&b, msg.gy_mdps) &&
                          cbor_put_tstr(&b, "gz_mdps") && cbor_put_int32(&b, msg.gz_mdps);

                if (ok) {
                    uart_send_cbor_payload(payload, b.len);
                }
            }
        } else if (chan == &ads1115_chan) {
            struct ads1115_msg msg;
            if (zbus_chan_read(&ads1115_chan, &msg, K_MSEC(5)) == 0) {
                uint8_t payload[TELEMETRY_MAX_CBOR_PAYLOAD];
                struct cbor_buf b = {.buf = payload, .len = 0, .cap = sizeof(payload)};

                bool ok = cbor_put_map_start(&b, 4U) &&
                          cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "ads1115") &&
                          cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, msg.t_ms) &&
                          cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, msg.seq) &&
                          cbor_put_tstr(&b, "ain0_mv") && cbor_put_int32(&b, msg.ain0_mv);

                if (ok) {
                    uart_send_cbor_payload(payload, b.len);
                }
            }
        } else if (chan == &ina3221_a_chan) {
            struct ina3221_msg msg;
            if (zbus_chan_read(&ina3221_a_chan, &msg, K_MSEC(5)) == 0) {
                uint8_t payload[TELEMETRY_MAX_CBOR_PAYLOAD];
                struct cbor_buf b = {.buf = payload, .len = 0, .cap = sizeof(payload)};

                bool ok = cbor_put_map_start(&b, 6U) &&
                          cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "ina3221_a") &&
                          cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, msg.t_ms) &&
                          cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, msg.seq) &&
                          cbor_put_tstr(&b, "bus_mv") && cbor_put_int32(&b, msg.bus_mv) &&
                          cbor_put_tstr(&b, "current_ma") && cbor_put_int32(&b, msg.current_ma) &&
                          cbor_put_tstr(&b, "power_mw") && cbor_put_int32(&b, msg.power_mw);

                if (ok) {
                    uart_send_cbor_payload(payload, b.len);
                }
            }
        } else if (chan == &ina3221_b_chan) {
            struct ina3221_msg msg;
            if (zbus_chan_read(&ina3221_b_chan, &msg, K_MSEC(5)) == 0) {
                uint8_t payload[TELEMETRY_MAX_CBOR_PAYLOAD];
                struct cbor_buf b = {.buf = payload, .len = 0, .cap = sizeof(payload)};

                bool ok = cbor_put_map_start(&b, 6U) &&
                          cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "ina3221_b") &&
                          cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, msg.t_ms) &&
                          cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, msg.seq) &&
                          cbor_put_tstr(&b, "bus_mv") && cbor_put_int32(&b, msg.bus_mv) &&
                          cbor_put_tstr(&b, "current_ma") && cbor_put_int32(&b, msg.current_ma) &&
                          cbor_put_tstr(&b, "power_mw") && cbor_put_int32(&b, msg.power_mw);

                if (ok) {
                    uart_send_cbor_payload(payload, b.len);
                }
            }
        } else if (chan == &bq76942_chan) {
            struct bq76942_msg msg;
            if (zbus_chan_read(&bq76942_chan, &msg, K_MSEC(5)) == 0) {
                uint8_t payload[TELEMETRY_MAX_CBOR_PAYLOAD];
                struct cbor_buf b = {.buf = payload, .len = 0, .cap = sizeof(payload)};

                bool ok = cbor_put_map_start(&b, 7U) &&
                          cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "bq76942") &&
                          cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, msg.t_ms) &&
                          cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, msg.seq) &&
                          cbor_put_tstr(&b, "pack_mv") && cbor_put_int32(&b, msg.pack_mv) &&
                          cbor_put_tstr(&b, "pack_ma") && cbor_put_int32(&b, msg.pack_ma) &&
                          cbor_put_tstr(&b, "soc_deci_pct") && cbor_put_int32(&b, msg.soc_deci_pct) &&
                          cbor_put_tstr(&b, "temp_cdeg") && cbor_put_int32(&b, msg.temp_cdeg);

                if (ok) {
                    uart_send_cbor_payload(payload, b.len);
                }
            }
        } else if (chan == &ina226_a_chan) {
            struct ina226_msg msg;
            if (zbus_chan_read(&ina226_a_chan, &msg, K_MSEC(5)) == 0) {
                uint8_t payload[TELEMETRY_MAX_CBOR_PAYLOAD];
                struct cbor_buf b = {.buf = payload, .len = 0, .cap = sizeof(payload)};

                bool ok = cbor_put_map_start(&b, 6U) &&
                          cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "ina226_a") &&
                          cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, msg.t_ms) &&
                          cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, msg.seq) &&
                          cbor_put_tstr(&b, "bus_mv") && cbor_put_int32(&b, msg.bus_mv) &&
                          cbor_put_tstr(&b, "current_ma") && cbor_put_int32(&b, msg.current_ma) &&
                          cbor_put_tstr(&b, "power_mw") && cbor_put_int32(&b, msg.power_mw);

                if (ok) {
                    uart_send_cbor_payload(payload, b.len);
                }
            }
        } else if (chan == &ina226_b_chan) {
            struct ina226_msg msg;
            if (zbus_chan_read(&ina226_b_chan, &msg, K_MSEC(5)) == 0) {
                uint8_t payload[TELEMETRY_MAX_CBOR_PAYLOAD];
                struct cbor_buf b = {.buf = payload, .len = 0, .cap = sizeof(payload)};

                bool ok = cbor_put_map_start(&b, 6U) &&
                          cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "ina226_b") &&
                          cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, msg.t_ms) &&
                          cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, msg.seq) &&
                          cbor_put_tstr(&b, "bus_mv") && cbor_put_int32(&b, msg.bus_mv) &&
                          cbor_put_tstr(&b, "current_ma") && cbor_put_int32(&b, msg.current_ma) &&
                          cbor_put_tstr(&b, "power_mw") && cbor_put_int32(&b, msg.power_mw);

                if (ok) {
                    uart_send_cbor_payload(payload, b.len);
                }
            }
        }
    }
}

static void heartbeat_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    while (1) {
        heartbeat_seq++;
        cbor_send_heartbeat((uint32_t)k_uptime_get(), heartbeat_seq);
        k_sleep(K_MSEC(HEARTBEAT_PERIOD_MS));
    }
}

int uart_publisher_start(void)
{
    if (!device_is_ready(telemetry_uart)) {
        LOG_ERR("Telemetry UART not ready");
        return -ENODEV;
    }

    k_mutex_init(&uart_tx_lock);
    k_sem_init(&uart_tx_done_sem, 0, 1);

    int cb_ret = uart_callback_set(telemetry_uart, uart_event_cb, NULL);
    if (cb_ret == 0) {
        uart_async_ready = true;
        LOG_INF("Telemetry UART async TX enabled");
    } else {
        uart_async_ready = false;
        LOG_ERR("Telemetry UART async TX unavailable (%d)", cb_ret);
        return cb_ret;
    }

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
    return 0;
}
