#include <zephyr/zbus/zbus.h>

#include "channels.h"

ZBUS_SUBSCRIBER_DEFINE(uart_pub_sub, 16);

ZBUS_CHAN_DEFINE(lsm6dsox_chan,
                 struct lsm6dsox_msg,
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

ZBUS_CHAN_DEFINE(ads1115_chan,
                 struct ads1115_msg,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(uart_pub_sub),
                 ZBUS_MSG_INIT(.t_ms = 0,
                               .seq = 0,
                               .ain0_mv = 0));

ZBUS_CHAN_DEFINE(ina3221_a_chan,
                 struct ina3221_msg,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(uart_pub_sub),
                 ZBUS_MSG_INIT(.t_ms = 0,
                               .seq = 0,
                               .bus_mv = 0,
                               .current_ma = 0,
                               .power_mw = 0));

ZBUS_CHAN_DEFINE(ina3221_b_chan,
                 struct ina3221_msg,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(uart_pub_sub),
                 ZBUS_MSG_INIT(.t_ms = 0,
                               .seq = 0,
                               .bus_mv = 0,
                               .current_ma = 0,
                               .power_mw = 0));

ZBUS_CHAN_DEFINE(bq76942_chan,
                 struct bq76942_msg,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(uart_pub_sub),
                 ZBUS_MSG_INIT(.t_ms = 0,
                               .seq = 0,
                               .pack_mv = 0,
                               .pack_ma = 0,
                               .soc_deci_pct = 0,
                               .temp_cdeg = 0));

ZBUS_CHAN_DEFINE(ina226_a_chan,
                 struct ina226_msg,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(uart_pub_sub),
                 ZBUS_MSG_INIT(.t_ms = 0,
                               .seq = 0,
                               .bus_mv = 0,
                               .current_ma = 0,
                               .power_mw = 0));

ZBUS_CHAN_DEFINE(ina226_b_chan,
                 struct ina226_msg,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS(uart_pub_sub),
                 ZBUS_MSG_INIT(.t_ms = 0,
                               .seq = 0,
                               .bus_mv = 0,
                               .current_ma = 0,
                               .power_mw = 0));
