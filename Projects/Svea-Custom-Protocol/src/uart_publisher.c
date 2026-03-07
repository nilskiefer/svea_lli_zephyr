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

#define TELEMETRY_FRAME_MAGIC 0xA5
#define TELEMETRY_MAX_CBOR_PAYLOAD 320

static K_THREAD_STACK_DEFINE(uart_pub_stack, UART_PUB_STACK_SIZE);
static struct k_thread uart_pub_thread;

static const struct device *telemetry_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
static struct k_mutex uart_tx_lock;
static struct k_sem uart_tx_done_sem;
static bool uart_async_ready;
static __nocache uint8_t uart_tx_frame[3 + TELEMETRY_MAX_CBOR_PAYLOAD];

enum topic_slot {
    TOPIC_SLOT_HEARTBEAT = 0,
    TOPIC_SLOT_LSM6DSOX,
    TOPIC_SLOT_ADS1115,
    TOPIC_SLOT_INA3221_A,
    TOPIC_SLOT_INA3221_B,
    TOPIC_SLOT_BQ76942,
    TOPIC_SLOT_INA226_A,
    TOPIC_SLOT_INA226_B,
    TOPIC_SLOT_COUNT
};

/*
 * Higher value = higher publish priority.
 * Tune these to change arbitration under load.
 */
#define TOPIC_PRIO_BQ76942 100
#define TOPIC_PRIO_INA226_A 70
#define TOPIC_PRIO_INA226_B 70
#define TOPIC_PRIO_INA3221_A 60
#define TOPIC_PRIO_INA3221_B 60
#define TOPIC_PRIO_ADS1115 50
#define TOPIC_PRIO_LSM6DSOX 40
#define TOPIC_PRIO_HEARTBEAT 10

static uint8_t topic_priority[TOPIC_SLOT_COUNT] = {
    [TOPIC_SLOT_HEARTBEAT] = TOPIC_PRIO_HEARTBEAT,
    [TOPIC_SLOT_LSM6DSOX] = TOPIC_PRIO_LSM6DSOX,
    [TOPIC_SLOT_ADS1115] = TOPIC_PRIO_ADS1115,
    [TOPIC_SLOT_INA3221_A] = TOPIC_PRIO_INA3221_A,
    [TOPIC_SLOT_INA3221_B] = TOPIC_PRIO_INA3221_B,
    [TOPIC_SLOT_BQ76942] = TOPIC_PRIO_BQ76942,
    [TOPIC_SLOT_INA226_A] = TOPIC_PRIO_INA226_A,
    [TOPIC_SLOT_INA226_B] = TOPIC_PRIO_INA226_B,
};

static bool topic_pending[TOPIC_SLOT_COUNT];
static struct heartbeat_msg heartbeat_latest;
static struct lsm6dsox_msg lsm6dsox_latest;
static struct ads1115_msg ads1115_latest;
static struct ina3221_msg ina3221_a_latest;
static struct ina3221_msg ina3221_b_latest;
static struct bq76942_msg bq76942_latest;
static struct ina226_msg ina226_a_latest;
static struct ina226_msg ina226_b_latest;

struct cbor_buf {
    uint8_t *buf;
    size_t len;
    size_t cap;
};

static bool cbor_put_byte(struct cbor_buf *b, uint8_t v) {
    if (b->len >= b->cap) {
        return false;
    }
    b->buf[b->len++] = v;
    return true;
}

static bool cbor_put_type_val(struct cbor_buf *b, uint8_t major, uint32_t val) {
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

static bool cbor_put_uint32(struct cbor_buf *b, uint32_t v) {
    return cbor_put_type_val(b, 0U, v);
}

static bool cbor_put_int32(struct cbor_buf *b, int32_t v) {
    if (v >= 0) {
        return cbor_put_type_val(b, 0U, (uint32_t)v);
    }

    return cbor_put_type_val(b, 1U, (uint32_t)(-1 - v));
}

static bool cbor_put_tstr(struct cbor_buf *b, const char *s) {
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

static bool cbor_put_map_start(struct cbor_buf *b, uint32_t pairs) {
    return cbor_put_type_val(b, 5U, pairs);
}

static void telemetry_wait_for_host(void) {
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

static void uart_event_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    if (evt->type == UART_TX_DONE || evt->type == UART_TX_ABORTED) {
        k_sem_give(&uart_tx_done_sem);
    }
}

static void uart_send_cbor_payload(const uint8_t *payload, size_t len) {
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

static void cbor_send_status_online(void) {
    uint8_t payload[96];
    struct cbor_buf c = {.buf = payload, .len = 0, .cap = sizeof(payload)};

    bool ok = cbor_put_map_start(&c, 3U) &&
              cbor_put_tstr(&c, "topic") && cbor_put_tstr(&c, "status") &&
              cbor_put_tstr(&c, "t_ms") && cbor_put_uint32(&c, (uint32_t)k_uptime_get()) &&
              cbor_put_tstr(&c, "msg") && cbor_put_tstr(&c, "telemetry_online");

    if (ok) {
        uart_send_cbor_payload(payload, c.len);
    }
}

static enum topic_slot channel_to_slot(const struct zbus_channel *chan) {
    if (chan == &heartbeat_chan) {
        return TOPIC_SLOT_HEARTBEAT;
    }
    if (chan == &lsm6dsox_chan) {
        return TOPIC_SLOT_LSM6DSOX;
    }
    if (chan == &ads1115_chan) {
        return TOPIC_SLOT_ADS1115;
    }
    if (chan == &ina3221_a_chan) {
        return TOPIC_SLOT_INA3221_A;
    }
    if (chan == &ina3221_b_chan) {
        return TOPIC_SLOT_INA3221_B;
    }
    if (chan == &bq76942_chan) {
        return TOPIC_SLOT_BQ76942;
    }
    if (chan == &ina226_a_chan) {
        return TOPIC_SLOT_INA226_A;
    }
    if (chan == &ina226_b_chan) {
        return TOPIC_SLOT_INA226_B;
    }
    return TOPIC_SLOT_COUNT;
}

static void capture_topic_update(enum topic_slot slot) {
    switch (slot) {
    case TOPIC_SLOT_HEARTBEAT:
        if (zbus_chan_read(&heartbeat_chan, &heartbeat_latest, K_MSEC(1)) == 0) {
            topic_pending[slot] = true;
        }
        break;
    case TOPIC_SLOT_LSM6DSOX:
        if (zbus_chan_read(&lsm6dsox_chan, &lsm6dsox_latest, K_MSEC(1)) == 0) {
            topic_pending[slot] = true;
        }
        break;
    case TOPIC_SLOT_ADS1115:
        if (zbus_chan_read(&ads1115_chan, &ads1115_latest, K_MSEC(1)) == 0) {
            topic_pending[slot] = true;
        }
        break;
    case TOPIC_SLOT_INA3221_A:
        if (zbus_chan_read(&ina3221_a_chan, &ina3221_a_latest, K_MSEC(1)) == 0) {
            topic_pending[slot] = true;
        }
        break;
    case TOPIC_SLOT_INA3221_B:
        if (zbus_chan_read(&ina3221_b_chan, &ina3221_b_latest, K_MSEC(1)) == 0) {
            topic_pending[slot] = true;
        }
        break;
    case TOPIC_SLOT_BQ76942:
        if (zbus_chan_read(&bq76942_chan, &bq76942_latest, K_MSEC(1)) == 0) {
            topic_pending[slot] = true;
        }
        break;
    case TOPIC_SLOT_INA226_A:
        if (zbus_chan_read(&ina226_a_chan, &ina226_a_latest, K_MSEC(1)) == 0) {
            topic_pending[slot] = true;
        }
        break;
    case TOPIC_SLOT_INA226_B:
        if (zbus_chan_read(&ina226_b_chan, &ina226_b_latest, K_MSEC(1)) == 0) {
            topic_pending[slot] = true;
        }
        break;
    case TOPIC_SLOT_COUNT:
    default:
        break;
    }
}

static enum topic_slot pick_next_pending_slot(void) {
    enum topic_slot best = TOPIC_SLOT_COUNT;
    uint8_t best_prio = 0;

    for (int i = 0; i < TOPIC_SLOT_COUNT; i++) {
        if (!topic_pending[i]) {
            continue;
        }
        if (best == TOPIC_SLOT_COUNT || topic_priority[i] > best_prio) {
            best = (enum topic_slot)i;
            best_prio = topic_priority[i];
        }
    }

    return best;
}

static void send_pending_slot(enum topic_slot slot) {
    uint8_t payload[TELEMETRY_MAX_CBOR_PAYLOAD];
    struct cbor_buf b = {.buf = payload, .len = 0, .cap = sizeof(payload)};
    bool ok = false;

    switch (slot) {
    case TOPIC_SLOT_HEARTBEAT:
        ok = cbor_put_map_start(&b, 3U) &&
             cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "heartbeat") &&
             cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, heartbeat_latest.t_ms) &&
             cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, heartbeat_latest.seq);
        break;
    case TOPIC_SLOT_LSM6DSOX:
        ok = cbor_put_map_start(&b, 10U) &&
             cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "lsm6dsox") &&
             cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, lsm6dsox_latest.t_ms) &&
             cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, lsm6dsox_latest.seq) &&
             cbor_put_tstr(&b, "ax_mg") && cbor_put_int32(&b, lsm6dsox_latest.ax_mg) &&
             cbor_put_tstr(&b, "ay_mg") && cbor_put_int32(&b, lsm6dsox_latest.ay_mg) &&
             cbor_put_tstr(&b, "az_mg") && cbor_put_int32(&b, lsm6dsox_latest.az_mg) &&
             cbor_put_tstr(&b, "gx_mdps") && cbor_put_int32(&b, lsm6dsox_latest.gx_mdps) &&
             cbor_put_tstr(&b, "gy_mdps") && cbor_put_int32(&b, lsm6dsox_latest.gy_mdps) &&
             cbor_put_tstr(&b, "gz_mdps") && cbor_put_int32(&b, lsm6dsox_latest.gz_mdps) &&
             cbor_put_tstr(&b, "temp_cdeg") && cbor_put_int32(&b, lsm6dsox_latest.temp_cdeg);
        break;
    case TOPIC_SLOT_ADS1115:
        ok = cbor_put_map_start(&b, 7U) &&
             cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "ads1115") &&
             cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, ads1115_latest.t_ms) &&
             cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, ads1115_latest.seq) &&
             cbor_put_tstr(&b, "ain0_mv") && cbor_put_int32(&b, ads1115_latest.ain0_mv) &&
             cbor_put_tstr(&b, "ain1_mv") && cbor_put_int32(&b, ads1115_latest.ain1_mv) &&
             cbor_put_tstr(&b, "ain2_mv") && cbor_put_int32(&b, ads1115_latest.ain2_mv) &&
             cbor_put_tstr(&b, "ain3_mv") && cbor_put_int32(&b, ads1115_latest.ain3_mv);
        break;
    case TOPIC_SLOT_INA3221_A:
        ok = cbor_put_map_start(&b, 12U) &&
             cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "ina3221_a") &&
             cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, ina3221_a_latest.t_ms) &&
             cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, ina3221_a_latest.seq) &&
             cbor_put_tstr(&b, "esc_bus_mv") && cbor_put_int32(&b, ina3221_a_latest.esc_bus_mv) &&
             cbor_put_tstr(&b, "esc_current_ma") && cbor_put_int32(&b, ina3221_a_latest.esc_current_ma) &&
             cbor_put_tstr(&b, "esc_power_mw") && cbor_put_int32(&b, ina3221_a_latest.esc_power_mw) &&
             cbor_put_tstr(&b, "v12_bus_mv") && cbor_put_int32(&b, ina3221_a_latest.v12_bus_mv) &&
             cbor_put_tstr(&b, "v12_current_ma") && cbor_put_int32(&b, ina3221_a_latest.v12_current_ma) &&
             cbor_put_tstr(&b, "v12_power_mw") && cbor_put_int32(&b, ina3221_a_latest.v12_power_mw) &&
             cbor_put_tstr(&b, "v5_bus_mv") && cbor_put_int32(&b, ina3221_a_latest.v5_bus_mv) &&
             cbor_put_tstr(&b, "v5_current_ma") && cbor_put_int32(&b, ina3221_a_latest.v5_current_ma) &&
             cbor_put_tstr(&b, "v5_power_mw") && cbor_put_int32(&b, ina3221_a_latest.v5_power_mw);
        break;
    case TOPIC_SLOT_INA3221_B:
        ok = cbor_put_map_start(&b, 12U) &&
             cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "ina3221_b") &&
             cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, ina3221_b_latest.t_ms) &&
             cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, ina3221_b_latest.seq) &&
             cbor_put_tstr(&b, "esc_bus_mv") && cbor_put_int32(&b, ina3221_b_latest.esc_bus_mv) &&
             cbor_put_tstr(&b, "esc_current_ma") && cbor_put_int32(&b, ina3221_b_latest.esc_current_ma) &&
             cbor_put_tstr(&b, "esc_power_mw") && cbor_put_int32(&b, ina3221_b_latest.esc_power_mw) &&
             cbor_put_tstr(&b, "v12_bus_mv") && cbor_put_int32(&b, ina3221_b_latest.v12_bus_mv) &&
             cbor_put_tstr(&b, "v12_current_ma") && cbor_put_int32(&b, ina3221_b_latest.v12_current_ma) &&
             cbor_put_tstr(&b, "v12_power_mw") && cbor_put_int32(&b, ina3221_b_latest.v12_power_mw) &&
             cbor_put_tstr(&b, "v5_bus_mv") && cbor_put_int32(&b, ina3221_b_latest.v5_bus_mv) &&
             cbor_put_tstr(&b, "v5_current_ma") && cbor_put_int32(&b, ina3221_b_latest.v5_current_ma) &&
             cbor_put_tstr(&b, "v5_power_mw") && cbor_put_int32(&b, ina3221_b_latest.v5_power_mw);
        break;
    case TOPIC_SLOT_BQ76942:
        ok = cbor_put_map_start(&b, 11U) &&
             cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "bq76942") &&
             cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, bq76942_latest.t_ms) &&
             cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, bq76942_latest.seq) &&
             cbor_put_tstr(&b, "pack_mv") && cbor_put_int32(&b, bq76942_latest.pack_mv) &&
             cbor_put_tstr(&b, "pack_ma") && cbor_put_int32(&b, bq76942_latest.pack_ma) &&
             cbor_put_tstr(&b, "soc_deci_pct") && cbor_put_int32(&b, bq76942_latest.soc_deci_pct) &&
             cbor_put_tstr(&b, "temp_cdeg") && cbor_put_int32(&b, bq76942_latest.temp_cdeg) &&
             cbor_put_tstr(&b, "cell_min_mv") && cbor_put_int32(&b, bq76942_latest.cell_min_mv) &&
             cbor_put_tstr(&b, "cell_avg_mv") && cbor_put_int32(&b, bq76942_latest.cell_avg_mv) &&
             cbor_put_tstr(&b, "cell_max_mv") && cbor_put_int32(&b, bq76942_latest.cell_max_mv) &&
             cbor_put_tstr(&b, "error_flags") && cbor_put_uint32(&b, (uint32_t)bq76942_latest.error_flags);
        break;
    case TOPIC_SLOT_INA226_A:
        ok = cbor_put_map_start(&b, 7U) &&
             cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "ina226_a") &&
             cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, ina226_a_latest.t_ms) &&
             cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, ina226_a_latest.seq) &&
             cbor_put_tstr(&b, "bus_mv") && cbor_put_int32(&b, ina226_a_latest.bus_mv) &&
             cbor_put_tstr(&b, "shunt_uv") && cbor_put_int32(&b, ina226_a_latest.shunt_uv) &&
             cbor_put_tstr(&b, "current_ma") && cbor_put_int32(&b, ina226_a_latest.current_ma) &&
             cbor_put_tstr(&b, "power_mw") && cbor_put_int32(&b, ina226_a_latest.power_mw);
        break;
    case TOPIC_SLOT_INA226_B:
        ok = cbor_put_map_start(&b, 7U) &&
             cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "ina226_b") &&
             cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, ina226_b_latest.t_ms) &&
             cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, ina226_b_latest.seq) &&
             cbor_put_tstr(&b, "bus_mv") && cbor_put_int32(&b, ina226_b_latest.bus_mv) &&
             cbor_put_tstr(&b, "shunt_uv") && cbor_put_int32(&b, ina226_b_latest.shunt_uv) &&
             cbor_put_tstr(&b, "current_ma") && cbor_put_int32(&b, ina226_b_latest.current_ma) &&
             cbor_put_tstr(&b, "power_mw") && cbor_put_int32(&b, ina226_b_latest.power_mw);
        break;
    case TOPIC_SLOT_COUNT:
    default:
        break;
    }

    if (ok) {
        uart_send_cbor_payload(payload, b.len);
    }
}

static void uart_pub_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    const struct zbus_channel *chan;
    enum topic_slot slot;

    cbor_send_status_online();

    while (1) {
        if (pick_next_pending_slot() == TOPIC_SLOT_COUNT) {
            if (zbus_sub_wait(&uart_pub_sub, &chan, K_FOREVER) == 0) {
                slot = channel_to_slot(chan);
                if (slot != TOPIC_SLOT_COUNT) {
                    capture_topic_update(slot);
                }
            }
        }

        /* Coalesce bursts before selecting the next frame to transmit. */
        while (zbus_sub_wait(&uart_pub_sub, &chan, K_NO_WAIT) == 0) {
            slot = channel_to_slot(chan);
            if (slot != TOPIC_SLOT_COUNT) {
                capture_topic_update(slot);
            }
        }

        slot = pick_next_pending_slot();
        if (slot != TOPIC_SLOT_COUNT) {
            topic_pending[slot] = false;
            send_pending_slot(slot);
        }
    }
}

int uart_publisher_start(void) {
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

    LOG_INF("UART publisher started on %s", telemetry_uart->name);
    return 0;
}
