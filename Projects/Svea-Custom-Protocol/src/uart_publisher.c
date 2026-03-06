#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "channels.h"

LOG_MODULE_REGISTER(uart_publisher, LOG_LEVEL_INF);

#if !DT_NODE_HAS_STATUS(DT_ALIAS(telemetry_uart), okay)
#error "telemetry-uart alias is missing or disabled"
#endif

#define UART_PUB_STACK_SIZE 1536
#define UART_PUB_THREAD_PRIO 3

static K_THREAD_STACK_DEFINE(uart_pub_stack, UART_PUB_STACK_SIZE);
static struct k_thread uart_pub_thread;

static const struct device *telemetry_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));

static void uart_write_str(const char *str)
{
    for (size_t i = 0; str[i] != '\0'; i++) {
        uart_poll_out(telemetry_uart, str[i]);
    }
}

static void uart_pub_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    const struct zbus_channel *chan;
    char line[192];

    uart_write_str("{\"topic\":\"status\",\"msg\":\"telemetry_online\"}\n");

    while (1) {
        if (zbus_sub_wait(&uart_pub_sub, &chan, K_FOREVER) != 0) {
            continue;
        }

        if (chan == &imu_chan) {
            struct imu_msg msg;
            if (zbus_chan_read(&imu_chan, &msg, K_MSEC(5)) == 0) {
                snprintk(line,
                         sizeof(line),
                         "{\"topic\":\"imu\",\"t_ms\":%u,\"seq\":%u,\"ax_mg\":%d,\"ay_mg\":%d,\"az_mg\":%d,\"gx_mdps\":%d,\"gy_mdps\":%d,\"gz_mdps\":%d}\n",
                         msg.t_ms,
                         msg.seq,
                         msg.ax_mg,
                         msg.ay_mg,
                         msg.az_mg,
                         msg.gx_mdps,
                         msg.gy_mdps,
                         msg.gz_mdps);
                uart_write_str(line);
            }
        } else if (chan == &current_chan) {
            struct current_msg msg;
            if (zbus_chan_read(&current_chan, &msg, K_MSEC(5)) == 0) {
                snprintk(line,
                         sizeof(line),
                         "{\"topic\":\"current\",\"t_ms\":%u,\"seq\":%u,\"bus_mv\":%d,\"current_ma\":%d,\"power_mw\":%d}\n",
                         msg.t_ms,
                         msg.seq,
                         msg.bus_mv,
                         msg.current_ma,
                         msg.power_mw);
                uart_write_str(line);
            }
        }
    }
}

void uart_publisher_start(void)
{
    if (!device_is_ready(telemetry_uart)) {
        LOG_ERR("Telemetry UART not ready");
        return;
    }

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
}
