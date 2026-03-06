#ifndef PROJECT_MESSAGES_H_
#define PROJECT_MESSAGES_H_

#include <stdint.h>

enum sensor_topic_id {
    TOPIC_IMU = 1,
    TOPIC_CURRENT = 2,
};

struct imu_msg {
    uint32_t t_ms;
    uint32_t seq;
    int32_t ax_mg;
    int32_t ay_mg;
    int32_t az_mg;
    int32_t gx_mdps;
    int32_t gy_mdps;
    int32_t gz_mdps;
};

struct current_msg {
    uint32_t t_ms;
    uint32_t seq;
    int32_t bus_mv;
    int32_t current_ma;
    int32_t power_mw;
};

#endif
