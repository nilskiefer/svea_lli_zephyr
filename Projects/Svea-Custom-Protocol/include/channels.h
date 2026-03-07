#ifndef PROJECT_CHANNELS_H_
#define PROJECT_CHANNELS_H_

#include <zephyr/zbus/zbus.h>

#include "messages.h"

ZBUS_OBS_DECLARE(uart_pub_sub);

ZBUS_CHAN_DECLARE(lsm6dsox_chan);
ZBUS_CHAN_DECLARE(ads1115_chan);
ZBUS_CHAN_DECLARE(ina3221_a_chan);
ZBUS_CHAN_DECLARE(ina3221_b_chan);
ZBUS_CHAN_DECLARE(bq76942_chan);
ZBUS_CHAN_DECLARE(ina226_a_chan);
ZBUS_CHAN_DECLARE(ina226_b_chan);

#endif
