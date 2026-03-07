#ifndef PROJECT_MESSAGES_H_
#define PROJECT_MESSAGES_H_

#include <stdbool.h>
#include <stdint.h>

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
