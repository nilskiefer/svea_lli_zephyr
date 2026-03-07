#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app_threads.h"

LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

int main(void)
{
    LOG_INF("zbus telemetry app starting");

    int ret = uart_shared_start();
    if (ret != 0) {
        LOG_ERR("UART shared transport failed to start (%d); stubs not started", ret);
        while (1) {
            k_sleep(K_SECONDS(1));
        }
    }

    ret = uart_publisher_start();
    if (ret != 0) {
        LOG_ERR("UART publisher failed to start (%d); stubs not started", ret);
        while (1) {
            k_sleep(K_SECONDS(1));
        }
    }

    ret = uart_subscriber_start();
    if (ret != 0) {
        LOG_ERR("UART subscriber failed to start (%d); stubs not started", ret);
        while (1) {
            k_sleep(K_SECONDS(1));
        }
    }

    lsm6dsox_stub_start();
    ads1115_stub_start();
    ina3221_stub_start();
    bq76942_stub_start();
    ina226_stub_start();
    heartbeat_stub_start();
    rc_command_stub_start();
    host_command_stub_start();

    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
