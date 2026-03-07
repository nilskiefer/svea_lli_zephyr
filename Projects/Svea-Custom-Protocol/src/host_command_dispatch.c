#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "host_channels.h"

static int8_t clamp_i8(int v) {
    if (v > 127) {
        return 127;
    }
    if (v < -128) {
        return -128;
    }
    return (int8_t)v;
}

static bool dispatch_servo(const char *line) {
    unsigned long seq_ul = 0;
    int steering = 0;
    int throttle = 0;
    int high_gear = 0;
    int front_diff = 0;
    int rear_diff = 0;
    int extra1 = 0;
    int extra2 = 0;

    if (sscanf(line, "SERVO %lu %d %d %d %d %d %d %d",
               &seq_ul,
               &steering,
               &throttle,
               &high_gear,
               &front_diff,
               &rear_diff,
               &extra1,
               &extra2) != 8) {
        return false;
    }

    struct servo_control_msg msg = {
        .t_ms = (uint32_t)k_uptime_get(),
        .seq = (uint32_t)seq_ul,
        .steering = clamp_i8(steering),
        .throttle = clamp_i8(throttle),
        .high_gear = high_gear != 0,
        .front_diff = front_diff != 0,
        .rear_diff = rear_diff != 0,
        .extra1 = clamp_i8(extra1),
        .extra2 = clamp_i8(extra2),
    };
    (void)zbus_chan_pub(&servo_control_chan, &msg, K_NO_WAIT);
    return true;
}

static bool dispatch_hb(const char *line) {
    unsigned long seq_ul = 0;

    if (sscanf(line, "HB %lu", &seq_ul) != 1) {
        return false;
    }

    struct host_heartbeat_msg msg = {
        .t_ms = (uint32_t)k_uptime_get(),
        .seq = (uint32_t)seq_ul,
    };
    (void)zbus_chan_pub(&host_heartbeat_chan, &msg, K_NO_WAIT);
    return true;
}

struct host_cmd_dispatch_entry {
    bool (*dispatch)(const char *line);
};

/*
 * To add a new command:
 * 1) Add a dispatch_<name>() function.
 * 2) Append it to this table.
 */
static const struct host_cmd_dispatch_entry dispatch_table[] = {
    {.dispatch = dispatch_servo},
    {.dispatch = dispatch_hb},
};

bool host_command_dispatch_line(const char *line) {
    for (size_t i = 0; i < ARRAY_SIZE(dispatch_table); i++) {
        if (dispatch_table[i].dispatch(line)) {
            return true;
        }
    }
    return false;
}
