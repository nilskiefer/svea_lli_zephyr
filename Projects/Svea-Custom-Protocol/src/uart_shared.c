#include <errno.h>
#include <stddef.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include "uart_shared.h"

LOG_MODULE_REGISTER(uart_shared, LOG_LEVEL_ERR);

#if !DT_NODE_HAS_STATUS(DT_ALIAS(telemetry_uart), okay)
#error "telemetry-uart alias is missing or disabled"
#endif

#define UART_SHARED_MAX_HANDLERS 4

struct handler_entry {
    uart_shared_event_handler_t fn;
    void *user_data;
};

static const struct device *telemetry_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
static struct handler_entry handlers[UART_SHARED_MAX_HANDLERS];
static size_t handler_count;
static bool uart_shared_started;

static void uart_shared_event_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
    ARG_UNUSED(user_data);

    for (size_t i = 0; i < handler_count; i++) {
        handlers[i].fn(dev, evt, handlers[i].user_data);
    }
}

int uart_shared_start(void) {
    if (uart_shared_started) {
        return 0;
    }

    if (!device_is_ready(telemetry_uart)) {
        LOG_ERR("Telemetry UART not ready");
        return -ENODEV;
    }

    int ret = uart_callback_set(telemetry_uart, uart_shared_event_cb, NULL);
    if (ret != 0) {
        LOG_ERR("Telemetry UART callback setup failed (%d)", ret);
        return ret;
    }

    uart_shared_started = true;
    return 0;
}

const struct device *uart_shared_device(void) {
    return telemetry_uart;
}

int uart_shared_register_handler(uart_shared_event_handler_t handler, void *user_data) {
    if (!uart_shared_started) {
        return -EACCES;
    }
    if (handler == NULL) {
        return -EINVAL;
    }
    if (handler_count >= UART_SHARED_MAX_HANDLERS) {
        return -ENOMEM;
    }

    handlers[handler_count].fn = handler;
    handlers[handler_count].user_data = user_data;
    handler_count++;
    return 0;
}
