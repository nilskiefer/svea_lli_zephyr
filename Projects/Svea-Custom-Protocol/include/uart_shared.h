#ifndef PROJECT_UART_SHARED_H_
#define PROJECT_UART_SHARED_H_

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

/*
 * Shared UART transport ownership:
 * - one callback is installed on the telemetry UART
 * - publisher/subscriber modules register their event handlers here
 */

typedef void (*uart_shared_event_handler_t)(const struct device *dev,
                                            struct uart_event *evt,
                                            void *user_data);

int uart_shared_start(void);
const struct device *uart_shared_device(void);
int uart_shared_register_handler(uart_shared_event_handler_t handler, void *user_data);

#endif
