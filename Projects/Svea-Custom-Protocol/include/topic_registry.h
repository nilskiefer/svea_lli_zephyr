#ifndef PROJECT_TOPIC_REGISTRY_H_
#define PROJECT_TOPIC_REGISTRY_H_

/*
 * Central telemetry topic registry.
 * Add new entries here so channel declarations, topic enums, and publisher
 * routing stay aligned.
 */

// always have heartbeat at lowest priority to make it obvious when publish frequency is too high for other stuff

#define TELEMETRY_TOPIC_LIST(X)                                    \
    X(HEARTBEAT, heartbeat, heartbeat_chan, heartbeat_msg, 10)     \
    X(RC_COMMAND, rc_command, rc_command_chan, rc_command_msg, 95) \
    X(LSM6DSOX, lsm6dsox, lsm6dsox_chan, lsm6dsox_msg, 40)         \
    X(ADS1115, ads1115, ads1115_chan, ads1115_msg, 50)             \
    X(INA3221_A, ina3221_a, ina3221_a_chan, ina3221_msg, 60)       \
    X(INA3221_B, ina3221_b, ina3221_b_chan, ina3221_msg, 60)       \
    X(BQ76942, bq76942, bq76942_chan, bq76942_msg, 100)            \
    X(INA226_A, ina226_a, ina226_a_chan, ina226_msg, 70)           \
    X(INA226_B, ina226_b, ina226_b_chan, ina226_msg, 70)

enum sensor_topic_id {
#define TOPIC_ENUM_ENTRY(id, topic_name, chan_name, type_name, priority) TOPIC_##id,
    TELEMETRY_TOPIC_LIST(TOPIC_ENUM_ENTRY)
#undef TOPIC_ENUM_ENTRY
        TOPIC_COUNT
};

#endif
