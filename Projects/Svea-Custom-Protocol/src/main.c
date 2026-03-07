#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app_threads.h"

LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

int main(void)
{
    LOG_INF("zbus telemetry app starting");

    uart_publisher_start();
    lsm6dsox_stub_start();
    ads1115_stub_start();
    ina3221_stub_start();
    bq76942_stub_start();
    ina226_stub_start();

    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
