#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/logging/log.h>

#include "telemetry_encode.h"
#include "telemetry_channels.h"
#include "topic_registry.h"
#include "uart_shared.h"

LOG_MODULE_REGISTER(uart_publisher, LOG_LEVEL_INF);

#define UART_PUB_STACK_SIZE 1536
#define UART_PUB_THREAD_PRIO 3

#define TELEMETRY_FRAME_MAGIC 0xA5
#define TELEMETRY_MAX_CBOR_PAYLOAD 320

static K_THREAD_STACK_DEFINE(uart_pub_stack, UART_PUB_STACK_SIZE);
static struct k_thread uart_pub_thread;

static struct k_mutex uart_tx_lock;
static struct k_sem uart_tx_done_sem;
static bool uart_async_ready;
static __nocache uint8_t uart_tx_frame[3 + TELEMETRY_MAX_CBOR_PAYLOAD];

static void telemetry_wait_for_host(void) {
    const struct device *telemetry_uart = uart_shared_device();
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
    const struct device *telemetry_uart = uart_shared_device();
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
    size_t payload_len = 0;

    if (telemetry_encode_status_online(payload, sizeof(payload), &payload_len, (uint32_t)k_uptime_get())) {
        uart_send_cbor_payload(payload, payload_len);
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

struct topic_descriptor {
    const struct zbus_channel *chan;
    const char *topic_label;
    void *latest;
    telemetry_topic_encode_fn encode;
};

#define TOPIC_DESC_ENTRY(id, topic_name, chan_name, type_name, priority)    \
    [TOPIC_SLOT_##id] = {                                                    \
        .chan = &chan_name,                                                  \
        .topic_label = #topic_name,                                          \
        .latest = &latest_##topic_name,                                      \
        .encode = telemetry_encode_##type_name,                              \
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
    size_t payload_len = 0;
    const struct topic_descriptor *desc = &topic_desc[slot];

    if (desc->encode(payload, sizeof(payload), &payload_len, desc->topic_label, desc->latest)) {
        uart_send_cbor_payload(payload, payload_len);
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
    const struct device *telemetry_uart = uart_shared_device();
    if (!device_is_ready(telemetry_uart)) {
        LOG_ERR("Telemetry UART not ready");
        return -ENODEV;
    }

    k_mutex_init(&uart_tx_lock);
    k_sem_init(&uart_tx_done_sem, 0, 1);

    int cb_ret = uart_shared_register_handler(uart_event_cb, NULL);
    if (cb_ret != 0) {
        uart_async_ready = false;
        LOG_ERR("Telemetry UART handler register failed (%d)", cb_ret);
        return cb_ret;
    }
    uart_async_ready = true;
    LOG_INF("Telemetry UART async TX enabled");

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
