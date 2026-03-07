#ifndef PROJECT_MESSAGES_H_
#define PROJECT_MESSAGES_H_

#include <stdint.h>

enum sensor_topic_id {
    TOPIC_HEARTBEAT = 0,
    TOPIC_LSM6DSOX = 1,
    TOPIC_ADS1115 = 2,
    TOPIC_INA3221_A = 3,
    TOPIC_INA3221_B = 4,
    TOPIC_BQ76942 = 5,
    TOPIC_INA226_A = 6,
    TOPIC_INA226_B = 7,
};

struct heartbeat_msg {
    uint32_t t_ms;
    uint32_t seq;
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
    int32_t esc_bus_mv;
    int32_t esc_current_ma;
    int32_t esc_power_mw;
    int32_t v12_bus_mv;
    int32_t v12_current_ma;
    int32_t v12_power_mw;
    int32_t v5_bus_mv;
    int32_t v5_current_ma;
    int32_t v5_power_mw;
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
