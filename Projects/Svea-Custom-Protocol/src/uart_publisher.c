#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/logging/log.h>

#include "channels.h"
#include "topic_registry.h"

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

static bool cbor_put_uint64(struct cbor_buf *b, uint64_t v) {
    if (v <= UINT32_MAX) {
        return cbor_put_type_val(b, 0U, (uint32_t)v);
    }

    return cbor_put_byte(b, (uint8_t)((0U << 5) | 27U)) &&
           cbor_put_byte(b, (uint8_t)(v >> 56)) &&
           cbor_put_byte(b, (uint8_t)(v >> 48)) &&
           cbor_put_byte(b, (uint8_t)(v >> 40)) &&
           cbor_put_byte(b, (uint8_t)(v >> 32)) &&
           cbor_put_byte(b, (uint8_t)(v >> 24)) &&
           cbor_put_byte(b, (uint8_t)(v >> 16)) &&
           cbor_put_byte(b, (uint8_t)(v >> 8)) &&
           cbor_put_byte(b, (uint8_t)v);
}

static bool cbor_put_int32(struct cbor_buf *b, int32_t v) {
    if (v >= 0) {
        return cbor_put_type_val(b, 0U, (uint32_t)v);
    }

    return cbor_put_type_val(b, 1U, (uint32_t)(-1 - v));
}

static __attribute__((unused)) bool cbor_put_int64(struct cbor_buf *b, int64_t v) {
    if (v >= 0) {
        return cbor_put_uint64(b, (uint64_t)v);
    }

    uint64_t n = (uint64_t)(-1 - v);
    if (n <= UINT32_MAX) {
        return cbor_put_type_val(b, 1U, (uint32_t)n);
    }

    return cbor_put_byte(b, (uint8_t)((1U << 5) | 27U)) &&
           cbor_put_byte(b, (uint8_t)(n >> 56)) &&
           cbor_put_byte(b, (uint8_t)(n >> 48)) &&
           cbor_put_byte(b, (uint8_t)(n >> 40)) &&
           cbor_put_byte(b, (uint8_t)(n >> 32)) &&
           cbor_put_byte(b, (uint8_t)(n >> 24)) &&
           cbor_put_byte(b, (uint8_t)(n >> 16)) &&
           cbor_put_byte(b, (uint8_t)(n >> 8)) &&
           cbor_put_byte(b, (uint8_t)n);
}

static __attribute__((unused)) bool cbor_put_bool(struct cbor_buf *b, bool v) {
    return cbor_put_byte(b, (uint8_t)((7U << 5) | (v ? 21U : 20U)));
}

static bool cbor_put_float32(struct cbor_buf *b, float v) {
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));

    return cbor_put_byte(b, (uint8_t)((7U << 5) | 26U)) &&
           cbor_put_byte(b, (uint8_t)(bits >> 24)) &&
           cbor_put_byte(b, (uint8_t)(bits >> 16)) &&
           cbor_put_byte(b, (uint8_t)(bits >> 8)) &&
           cbor_put_byte(b, (uint8_t)bits);
}

static __attribute__((unused)) bool cbor_put_float64(struct cbor_buf *b, double v) {
    uint64_t bits;
    memcpy(&bits, &v, sizeof(bits));

    return cbor_put_byte(b, (uint8_t)((7U << 5) | 27U)) &&
           cbor_put_byte(b, (uint8_t)(bits >> 56)) &&
           cbor_put_byte(b, (uint8_t)(bits >> 48)) &&
           cbor_put_byte(b, (uint8_t)(bits >> 40)) &&
           cbor_put_byte(b, (uint8_t)(bits >> 32)) &&
           cbor_put_byte(b, (uint8_t)(bits >> 24)) &&
           cbor_put_byte(b, (uint8_t)(bits >> 16)) &&
           cbor_put_byte(b, (uint8_t)(bits >> 8)) &&
           cbor_put_byte(b, (uint8_t)bits);
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

#define TOPIC_SLOT_ENTRY(id, topic_name, chan_name, type_name, priority) TOPIC_SLOT_##id,
enum topic_slot {
    TELEMETRY_TOPIC_LIST(TOPIC_SLOT_ENTRY)
    TOPIC_SLOT_COUNT
};
#undef TOPIC_SLOT_ENTRY

#define TOPIC_LATEST_ENTRY(id, topic_name, chan_name, type_name, priority) \
    static struct type_name latest_##topic_name;
TELEMETRY_TOPIC_LIST(TOPIC_LATEST_ENTRY)
#undef TOPIC_LATEST_ENTRY

#define TOPIC_PENDING_ENTRY(id, topic_name, chan_name, type_name, priority) false,
static bool topic_pending[TOPIC_SLOT_COUNT] = {
    TELEMETRY_TOPIC_LIST(TOPIC_PENDING_ENTRY)
};
#undef TOPIC_PENDING_ENTRY

#define TOPIC_PRIORITY_ENTRY(id, topic_name, chan_name, type_name, priority) \
    [TOPIC_SLOT_##id] = (priority),
static uint8_t topic_priority[TOPIC_SLOT_COUNT] = {
    TELEMETRY_TOPIC_LIST(TOPIC_PRIORITY_ENTRY)
};
#undef TOPIC_PRIORITY_ENTRY

static bool encode_heartbeat_msg(struct cbor_buf *b, const char *topic, const void *msg);
static bool encode_rc_command_msg(struct cbor_buf *b, const char *topic, const void *msg);
static bool encode_lsm6dsox_msg(struct cbor_buf *b, const char *topic, const void *msg);
static bool encode_ads1115_msg(struct cbor_buf *b, const char *topic, const void *msg);
static bool encode_ina3221_msg(struct cbor_buf *b, const char *topic, const void *msg);
static bool encode_bq76942_msg(struct cbor_buf *b, const char *topic, const void *msg);
static bool encode_ina226_msg(struct cbor_buf *b, const char *topic, const void *msg);

struct topic_descriptor {
    const struct zbus_channel *chan;
    const char *topic_label;
    void *latest;
    bool (*encode)(struct cbor_buf *b, const char *topic, const void *msg);
};

#define TOPIC_DESC_ENTRY(id, topic_name, chan_name, type_name, priority)    \
    [TOPIC_SLOT_##id] = {                                                    \
        .chan = &chan_name,                                                  \
        .topic_label = #topic_name,                                          \
        .latest = &latest_##topic_name,                                      \
        .encode = encode_##type_name,                                        \
    },
static const struct topic_descriptor topic_desc[TOPIC_SLOT_COUNT] = {
    TELEMETRY_TOPIC_LIST(TOPIC_DESC_ENTRY)
};
#undef TOPIC_DESC_ENTRY

static enum topic_slot channel_to_slot(const struct zbus_channel *chan) {
    for (int i = 0; i < TOPIC_SLOT_COUNT; i++) {
        if (topic_desc[i].chan == chan) {
            return (enum topic_slot)i;
        }
    }
    return TOPIC_SLOT_COUNT;
}

static void capture_topic_update(enum topic_slot slot) {
    const struct topic_descriptor *desc = &topic_desc[slot];

    if (zbus_chan_read(desc->chan, desc->latest, K_MSEC(1)) == 0) {
        topic_pending[slot] = true;
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
    const struct topic_descriptor *desc = &topic_desc[slot];

    if (desc->encode(&b, desc->topic_label, desc->latest)) {
        uart_send_cbor_payload(payload, b.len);
    }
}

static bool encode_heartbeat_msg(struct cbor_buf *b, const char *topic, const void *msg) {
    const struct heartbeat_msg *m = msg;

    return cbor_put_map_start(b, 3U) &&
           cbor_put_tstr(b, "topic") && cbor_put_tstr(b, topic) &&
           cbor_put_tstr(b, "t_ms") && cbor_put_uint32(b, m->t_ms) &&
           cbor_put_tstr(b, "seq") && cbor_put_uint32(b, m->seq);
}

static bool encode_rc_command_msg(struct cbor_buf *b, const char *topic, const void *msg) {
    const struct rc_command_msg *m = msg;

    return cbor_put_map_start(b, 9U) &&
           cbor_put_tstr(b, "topic") && cbor_put_tstr(b, topic) &&
           cbor_put_tstr(b, "t_ms") && cbor_put_uint32(b, m->t_ms) &&
           cbor_put_tstr(b, "seq") && cbor_put_uint32(b, m->seq) &&
           cbor_put_tstr(b, "steering") && cbor_put_int32(b, (int32_t)m->steering) &&
           cbor_put_tstr(b, "throttle") && cbor_put_int32(b, (int32_t)m->throttle) &&
           cbor_put_tstr(b, "high_gear") && cbor_put_bool(b, m->high_gear) &&
           cbor_put_tstr(b, "diff_lock") && cbor_put_bool(b, m->diff_lock) &&
           cbor_put_tstr(b, "override_mode") && cbor_put_bool(b, m->override_mode) &&
           cbor_put_tstr(b, "connected") && cbor_put_bool(b, m->connected);
}

static bool encode_lsm6dsox_msg(struct cbor_buf *b, const char *topic, const void *msg) {
    const struct lsm6dsox_msg *m = msg;

    return cbor_put_map_start(b, 10U) &&
           cbor_put_tstr(b, "topic") && cbor_put_tstr(b, topic) &&
           cbor_put_tstr(b, "t_ms") && cbor_put_uint32(b, m->t_ms) &&
           cbor_put_tstr(b, "seq") && cbor_put_uint32(b, m->seq) &&
           cbor_put_tstr(b, "ax_mg") && cbor_put_float32(b, m->ax_mg) &&
           cbor_put_tstr(b, "ay_mg") && cbor_put_float32(b, m->ay_mg) &&
           cbor_put_tstr(b, "az_mg") && cbor_put_float32(b, m->az_mg) &&
           cbor_put_tstr(b, "gx_mdps") && cbor_put_float32(b, m->gx_mdps) &&
           cbor_put_tstr(b, "gy_mdps") && cbor_put_float32(b, m->gy_mdps) &&
           cbor_put_tstr(b, "gz_mdps") && cbor_put_float32(b, m->gz_mdps) &&
           cbor_put_tstr(b, "temp_cdeg") && cbor_put_float32(b, m->temp_cdeg);
}

static bool encode_ads1115_msg(struct cbor_buf *b, const char *topic, const void *msg) {
    const struct ads1115_msg *m = msg;

    return cbor_put_map_start(b, 7U) &&
           cbor_put_tstr(b, "topic") && cbor_put_tstr(b, topic) &&
           cbor_put_tstr(b, "t_ms") && cbor_put_uint32(b, m->t_ms) &&
           cbor_put_tstr(b, "seq") && cbor_put_uint32(b, m->seq) &&
           cbor_put_tstr(b, "ain0_mv") && cbor_put_float32(b, m->ain0_mv) &&
           cbor_put_tstr(b, "ain1_mv") && cbor_put_float32(b, m->ain1_mv) &&
           cbor_put_tstr(b, "ain2_mv") && cbor_put_float32(b, m->ain2_mv) &&
           cbor_put_tstr(b, "ain3_mv") && cbor_put_float32(b, m->ain3_mv);
}

static bool encode_ina3221_msg(struct cbor_buf *b, const char *topic, const void *msg) {
    const struct ina3221_msg *m = msg;

    return cbor_put_map_start(b, 12U) &&
           cbor_put_tstr(b, "topic") && cbor_put_tstr(b, topic) &&
           cbor_put_tstr(b, "t_ms") && cbor_put_uint32(b, m->t_ms) &&
           cbor_put_tstr(b, "seq") && cbor_put_uint32(b, m->seq) &&
           cbor_put_tstr(b, "ch1_bus_mv") && cbor_put_float32(b, m->ch1_bus_mv) &&
           cbor_put_tstr(b, "ch1_current_ma") && cbor_put_float32(b, m->ch1_current_ma) &&
           cbor_put_tstr(b, "ch1_power_mw") && cbor_put_float32(b, m->ch1_power_mw) &&
           cbor_put_tstr(b, "ch2_bus_mv") && cbor_put_float32(b, m->ch2_bus_mv) &&
           cbor_put_tstr(b, "ch2_current_ma") && cbor_put_float32(b, m->ch2_current_ma) &&
           cbor_put_tstr(b, "ch2_power_mw") && cbor_put_float32(b, m->ch2_power_mw) &&
           cbor_put_tstr(b, "ch3_bus_mv") && cbor_put_float32(b, m->ch3_bus_mv) &&
           cbor_put_tstr(b, "ch3_current_ma") && cbor_put_float32(b, m->ch3_current_ma) &&
           cbor_put_tstr(b, "ch3_power_mw") && cbor_put_float32(b, m->ch3_power_mw);
}

static bool encode_bq76942_msg(struct cbor_buf *b, const char *topic, const void *msg) {
    const struct bq76942_msg *m = msg;

    return cbor_put_map_start(b, 11U) &&
           cbor_put_tstr(b, "topic") && cbor_put_tstr(b, topic) &&
           cbor_put_tstr(b, "t_ms") && cbor_put_uint32(b, m->t_ms) &&
           cbor_put_tstr(b, "seq") && cbor_put_uint32(b, m->seq) &&
           cbor_put_tstr(b, "pack_mv") && cbor_put_float32(b, m->pack_mv) &&
           cbor_put_tstr(b, "pack_ma") && cbor_put_float32(b, m->pack_ma) &&
           cbor_put_tstr(b, "soc_pct") && cbor_put_float32(b, m->soc_pct) &&
           cbor_put_tstr(b, "temp_cdeg") && cbor_put_float32(b, m->temp_cdeg) &&
           cbor_put_tstr(b, "cell_min_mv") && cbor_put_float32(b, m->cell_min_mv) &&
           cbor_put_tstr(b, "cell_avg_mv") && cbor_put_float32(b, m->cell_avg_mv) &&
           cbor_put_tstr(b, "cell_max_mv") && cbor_put_float32(b, m->cell_max_mv) &&
           cbor_put_tstr(b, "error_flags") && cbor_put_uint32(b, (uint32_t)m->error_flags);
}

static bool encode_ina226_msg(struct cbor_buf *b, const char *topic, const void *msg) {
    const struct ina226_msg *m = msg;

    return cbor_put_map_start(b, 7U) &&
           cbor_put_tstr(b, "topic") && cbor_put_tstr(b, topic) &&
           cbor_put_tstr(b, "t_ms") && cbor_put_uint32(b, m->t_ms) &&
           cbor_put_tstr(b, "seq") && cbor_put_uint32(b, m->seq) &&
           cbor_put_tstr(b, "bus_mv") && cbor_put_float32(b, m->bus_mv) &&
           cbor_put_tstr(b, "shunt_uv") && cbor_put_float32(b, m->shunt_uv) &&
           cbor_put_tstr(b, "current_ma") && cbor_put_float32(b, m->current_ma) &&
           cbor_put_tstr(b, "power_mw") && cbor_put_float32(b, m->power_mw);
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
