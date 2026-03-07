#ifndef PROJECT_MESSAGES_H_
#define PROJECT_MESSAGES_H_

#include <stdint.h>

struct heartbeat_msg {
    uint32_t t_ms;
    uint32_t seq;
};

struct rc_command_msg {
    uint32_t t_ms;
    uint32_t seq;
    int32_t steering;
    int32_t throttle;
    uint32_t high_gear;
    uint32_t diff_lock;
    uint32_t override_mode;
    uint32_t connected;
};

struct lsm6dsox_msg {
    uint32_t t_ms;
    uint32_t seq;
    int32_t ax_mg;
    int32_t ay_mg;
    int32_t az_mg;
    int32_t gx_mdps;
    int32_t gy_mdps;
    int32_t gz_mdps;
    int32_t temp_cdeg;
};

struct ads1115_msg {
    uint32_t t_ms;
    uint32_t seq;
    int32_t ain0_mv;
    int32_t ain1_mv;
    int32_t ain2_mv;
    int32_t ain3_mv;
};

struct ina3221_msg {
    uint32_t t_ms;
    uint32_t seq;
    int32_t ch1_bus_mv;
    int32_t ch1_current_ma;
    int32_t ch1_power_mw;
    int32_t ch2_bus_mv;
    int32_t ch2_current_ma;
    int32_t ch2_power_mw;
    int32_t ch3_bus_mv;
    int32_t ch3_current_ma;
    int32_t ch3_power_mw;
};

struct bq76942_msg {
    uint32_t t_ms;
    uint32_t seq;
    int32_t pack_mv;
    int32_t pack_ma;
    int32_t soc_deci_pct;
    int32_t temp_cdeg;
    int32_t cell_min_mv;
    int32_t cell_avg_mv;
    int32_t cell_max_mv;
    int32_t error_flags;
};

struct ina226_msg {
    uint32_t t_ms;
    uint32_t seq;
    int32_t bus_mv;
    int32_t shunt_uv;
    int32_t current_ma;
    int32_t power_mw;
};

#endif
