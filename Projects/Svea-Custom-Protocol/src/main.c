#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app_threads.h"

LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

int main(void)
{
    LOG_INF("zbus UART telemetry app starting");

    uart_publisher_start();
    imu_sensor_start();
    current_sensor_start();

    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
