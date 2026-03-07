#ifndef PROJECT_MESSAGES_H_
#define PROJECT_MESSAGES_H_

#include <stdint.h>

enum sensor_topic_id {
    TOPIC_LSM6DSOX = 1,
    TOPIC_ADS1115 = 2,
    TOPIC_INA3221_A = 3,
    TOPIC_INA3221_B = 4,
    TOPIC_BQ76942 = 5,
    TOPIC_INA226_A = 6,
    TOPIC_INA226_B = 7,
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
};

struct ads1115_msg {
    uint32_t t_ms;
    uint32_t seq;
    int32_t ain0_mv;
};

struct ina3221_msg {
    uint32_t t_ms;
    uint32_t seq;
    int32_t bus_mv;
    int32_t current_ma;
    int32_t power_mw;
};

struct bq76942_msg {
    uint32_t t_ms;
    uint32_t seq;
    int32_t pack_mv;
    int32_t pack_ma;
    int32_t soc_deci_pct;
    int32_t temp_cdeg;
};

struct ina226_msg {
    uint32_t t_ms;
    uint32_t seq;
    int32_t bus_mv;
    int32_t current_ma;
    int32_t power_mw;
};

#endif
