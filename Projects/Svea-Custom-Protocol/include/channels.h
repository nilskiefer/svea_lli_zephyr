#ifndef PROJECT_CHANNELS_H_
#define PROJECT_CHANNELS_H_

#include <zephyr/zbus/zbus.h>

#include "messages.h"
#include "topic_registry.h"

ZBUS_OBS_DECLARE(uart_pub_sub);

#define CHANNEL_DECLARE_ENTRY(id, topic_name, chan_name, type_name, priority) \
    ZBUS_CHAN_DECLARE(chan_name);
TELEMETRY_TOPIC_LIST(CHANNEL_DECLARE_ENTRY)
#undef CHANNEL_DECLARE_ENTRY

#endif
