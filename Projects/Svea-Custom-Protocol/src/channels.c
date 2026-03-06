#include <zephyr/zbus/zbus.h>

#include "channels.h"

ZBUS_SUBSCRIBER_DEFINE(uart_pub_sub, 16);

ZBUS_CHAN_DEFINE(imu_chan,
                 struct imu_msg,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(uart_pub_sub),
                 ZBUS_MSG_INIT(.t_ms = 0,
                               .seq = 0,
                               .ax_mg = 0,
                               .ay_mg = 0,
                               .az_mg = 0,
                               .gx_mdps = 0,
                               .gy_mdps = 0,
                               .gz_mdps = 0));

ZBUS_CHAN_DEFINE(current_chan,
                 struct current_msg,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(uart_pub_sub),
                 ZBUS_MSG_INIT(.t_ms = 0,
                               .seq = 0,
                               .bus_mv = 0,
                               .current_ma = 0,
                               .power_mw = 0));
