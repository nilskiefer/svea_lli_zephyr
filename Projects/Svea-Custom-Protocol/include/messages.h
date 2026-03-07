#ifndef PROJECT_MESSAGES_H_
#define PROJECT_MESSAGES_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Message schema definitions (payload only).
 *
 * Extension flow:
 * 1) Add/modify a struct here (keep `t_ms` + `seq` for telemetry messages).
 * 2) Bind topic->message in topic_registry.h.
 * 3) Add channel default in src/channels.c.
 * 4) Add encoder in telemetry_encode.c.
 * 5) Publish data from a producer thread/stub.
 */

struct heartbeat_msg {
    uint32_t t_ms;
    uint32_t seq;
};

struct rc_command_msg {
    uint32_t t_ms;
    uint32_t seq;
    int8_t steering;
    int8_t throttle;
    bool high_gear;
    bool diff_lock;
    bool override_mode;
    bool connected;
};

struct servo_control_msg {
    uint32_t t_ms;
    uint32_t seq;
    int8_t steering;
    int8_t throttle;
    bool high_gear;
    bool front_diff;
    bool rear_diff;
    int8_t extra1;
    int8_t extra2;
};

struct host_heartbeat_msg {
    uint32_t t_ms;
    uint32_t seq;
};

struct lsm6dsox_msg {
    uint32_t t_ms;
    uint32_t seq;
    float ax_mg;
    float ay_mg;
    float az_mg;
    float gx_mdps;
    float gy_mdps;
    float gz_mdps;
    float temp_cdeg;
};

struct ads1115_msg {
    uint32_t t_ms;
    uint32_t seq;
    float ain0_mv;
    float ain1_mv;
    float ain2_mv;
    float ain3_mv;
};

struct ina3221_msg {
    uint32_t t_ms;
    uint32_t seq;
    float ch1_bus_mv;
    float ch1_current_ma;
    float ch1_power_mw;
    float ch2_bus_mv;
    float ch2_current_ma;
    float ch2_power_mw;
    float ch3_bus_mv;
    float ch3_current_ma;
    float ch3_power_mw;
};

struct bq76942_msg {
    uint32_t t_ms;
    uint32_t seq;
    float pack_mv;
    float pack_ma;
    float soc_pct;
    float temp_cdeg;
    float cell_min_mv;
    float cell_avg_mv;
    float cell_max_mv;
    uint32_t error_flags;
};

struct ina226_msg {
    uint32_t t_ms;
    uint32_t seq;
    float bus_mv;
    float shunt_uv;
    float current_ma;
    float power_mw;
};

#endif
