#ifndef PROJECT_HOST_CHANNELS_H_
#define PROJECT_HOST_CHANNELS_H_

#include <zephyr/zbus/zbus.h>

#include "messages.h"

/*
 * Host-to-MCU command channels (not forwarded over telemetry UART).
 * New host command message types should add a dedicated channel here.
 */
ZBUS_CHAN_DECLARE(servo_control_chan);
ZBUS_CHAN_DECLARE(host_heartbeat_chan);

#endif
