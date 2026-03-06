#ifndef PROJECT_CHANNELS_H_
#define PROJECT_CHANNELS_H_

#include <zephyr/zbus/zbus.h>

#include "messages.h"

ZBUS_SUBSCRIBER_DECLARE(uart_pub_sub);

ZBUS_CHAN_DECLARE(imu_chan);
ZBUS_CHAN_DECLARE(current_chan);

#endif
