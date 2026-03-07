#ifndef PROJECT_TELEMETRY_CHANNELS_H_
#define PROJECT_TELEMETRY_CHANNELS_H_

#include <zephyr/zbus/zbus.h>

#include "messages.h"
#include "topic_registry.h"

/* Subscriber used by the telemetry UART publisher thread. */
ZBUS_OBS_DECLARE(uart_pub_sub);

/* Topic channels are declared from the central registry. */
#define CHANNEL_DECLARE_ENTRY(id, topic_name, chan_name, type_name, priority) \
    ZBUS_CHAN_DECLARE(chan_name);
TELEMETRY_TOPIC_LIST(CHANNEL_DECLARE_ENTRY)
#undef CHANNEL_DECLARE_ENTRY

#endif
