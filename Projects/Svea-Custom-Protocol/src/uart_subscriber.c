#include <errno.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/logging/log.h>

#include "host_command_dispatch.h"
#include "uart_shared.h"

LOG_MODULE_REGISTER(uart_subscriber, LOG_LEVEL_ERR);

#define UART_SUB_STACK_SIZE 1536
#define UART_SUB_THREAD_PRIO 3
#define UART_SUB_DMA_BUF_SIZE 256
#define UART_SUB_RING_SIZE 2048

static K_THREAD_STACK_DEFINE(uart_sub_stack, UART_SUB_STACK_SIZE);
static struct k_thread uart_sub_thread;
static struct k_sem uart_sub_data_sem;

static __nocache uint8_t uart_dma_buf_a[UART_SUB_DMA_BUF_SIZE];
static __nocache uint8_t uart_dma_buf_b[UART_SUB_DMA_BUF_SIZE];
static uint8_t uart_rx_ring[UART_SUB_RING_SIZE];
static volatile uint32_t uart_rx_read_idx;
static volatile uint32_t uart_rx_write_idx;
static volatile uint32_t uart_rx_drop_count;
static bool uart_dma_buf_a_free;
static bool uart_dma_buf_b_free;
static bool uart_subscriber_started;

static void uart_subscriber_on_uart_event(const struct device *dev,
                                          struct uart_event *evt,
                                          void *user_data);

static void uart_rx_push_bytes(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint32_t used = uart_rx_write_idx - uart_rx_read_idx;
        if (used >= UART_SUB_RING_SIZE) {
            uart_rx_drop_count++;
            continue;
        }
        uart_rx_ring[uart_rx_write_idx % UART_SUB_RING_SIZE] = data[i];
        uart_rx_write_idx++;
    }
}

static size_t uart_rx_pop_bytes(uint8_t *dst, size_t cap) {
    unsigned int key = irq_lock();
    uint32_t used = uart_rx_write_idx - uart_rx_read_idx;
    size_t count = used < cap ? (size_t)used : cap;

    for (size_t i = 0; i < count; i++) {
        dst[i] = uart_rx_ring[uart_rx_read_idx % UART_SUB_RING_SIZE];
        uart_rx_read_idx++;
    }
    irq_unlock(key);
    return count;
}

static void uart_rx_start(void) {
    const struct device *telemetry_uart = uart_shared_device();
    int ret;

    uart_dma_buf_a_free = false;
    uart_dma_buf_b_free = true;

    ret = uart_rx_enable(telemetry_uart, uart_dma_buf_a, sizeof(uart_dma_buf_a), 50);
    if (ret != 0) {
        LOG_ERR("uart_rx_enable failed: %d", ret);
    }
}

static void uart_sub_thread_fn(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    char line[128];
    size_t line_len = 0;

    while (1) {
        uint8_t chunk[64];
        size_t count;

        (void)k_sem_take(&uart_sub_data_sem, K_MSEC(100));

        while ((count = uart_rx_pop_bytes(chunk, sizeof(chunk))) > 0U) {
            for (size_t i = 0; i < count; i++) {
                unsigned char ch = chunk[i];

                if (ch == '\r') {
                    continue;
                }

                if (ch == '\n') {
                    line[line_len] = '\0';
                    if (line_len > 0U) {
                        (void)host_command_dispatch_line(line);
                    }
                    line_len = 0;
                    continue;
                }

                if (line_len + 1U < sizeof(line)) {
                    line[line_len++] = (char)ch;
                } else {
                    line_len = 0;
                }
            }
        }

        if (uart_rx_drop_count > 0U) {
            static uint32_t last_drop_reported;
            if (uart_rx_drop_count != last_drop_reported) {
                uint32_t dropped = uart_rx_drop_count - last_drop_reported;
                last_drop_reported = uart_rx_drop_count;
                LOG_ERR("UART subscriber dropped %u RX bytes (ring full)", dropped);
            }
        }
    }
}

int uart_subscriber_start(void) {
    const struct device *telemetry_uart = uart_shared_device();
    if (!device_is_ready(telemetry_uart)) {
        LOG_ERR("Telemetry UART not ready");
        return -ENODEV;
    }

    k_sem_init(&uart_sub_data_sem, 0, UINT_MAX);
    uart_rx_read_idx = 0;
    uart_rx_write_idx = 0;
    uart_rx_drop_count = 0;

    k_thread_create(&uart_sub_thread,
                    uart_sub_stack,
                    K_THREAD_STACK_SIZEOF(uart_sub_stack),
                    uart_sub_thread_fn,
                    NULL,
                    NULL,
                    NULL,
                    UART_SUB_THREAD_PRIO,
                    0,
                    K_NO_WAIT);
    k_thread_name_set(&uart_sub_thread, "uart_subscriber");

    int cb_ret = uart_shared_register_handler(uart_subscriber_on_uart_event, NULL);
    if (cb_ret != 0) {
        LOG_ERR("Telemetry UART handler register failed (%d)", cb_ret);
        return cb_ret;
    }

    uart_subscriber_started = true;
    uart_rx_start();

    return 0;
}

static void uart_subscriber_on_uart_event(const struct device *dev,
                                          struct uart_event *evt,
                                          void *user_data) {
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);
    const struct device *telemetry_uart = uart_shared_device();

    if (!uart_subscriber_started || evt == NULL) {
        return;
    }

    switch (evt->type) {
    case UART_RX_RDY: {
        const uint8_t *data = evt->data.rx.buf + evt->data.rx.offset;
        uart_rx_push_bytes(data, evt->data.rx.len);
        k_sem_give(&uart_sub_data_sem);
        break;
    }
    case UART_RX_BUF_REQUEST:
        if (uart_dma_buf_a_free) {
            uart_dma_buf_a_free = false;
            (void)uart_rx_buf_rsp(telemetry_uart, uart_dma_buf_a, sizeof(uart_dma_buf_a));
        } else if (uart_dma_buf_b_free) {
            uart_dma_buf_b_free = false;
            (void)uart_rx_buf_rsp(telemetry_uart, uart_dma_buf_b, sizeof(uart_dma_buf_b));
        }
        break;
    case UART_RX_BUF_RELEASED:
        if (evt->data.rx_buf.buf == uart_dma_buf_a) {
            uart_dma_buf_a_free = true;
        } else if (evt->data.rx_buf.buf == uart_dma_buf_b) {
            uart_dma_buf_b_free = true;
        }
        break;
    case UART_RX_DISABLED:
        uart_rx_start();
        break;
    case UART_RX_STOPPED:
        LOG_ERR("UART RX stopped: reason=%d", evt->data.rx_stop.reason);
        uart_rx_start();
        break;
    default:
        break;
    }
}
