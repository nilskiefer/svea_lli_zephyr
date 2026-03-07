#ifndef PROJECT_TELEMETRY_ENCODE_H_
#define PROJECT_TELEMETRY_ENCODE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "topic_registry.h"

/*
 * Encoders convert message structs into one framed payload body.
 * uart_publisher owns framing/transport; this module owns schema encoding.
 */

typedef bool (*telemetry_topic_encode_fn)(uint8_t *payload,
                                          size_t payload_cap,
                                          size_t *payload_len,
                                          const char *topic,
                                          const void *msg);

bool telemetry_encode_status_online(uint8_t *payload,
                                    size_t payload_cap,
                                    size_t *payload_len,
                                    uint32_t t_ms);

/* Encoder declarations stay aligned with TELEMETRY_TOPIC_LIST entries. */
#define TELEMETRY_ENCODER_DECLARE(id, topic_name, chan_name, type_name, priority) \
    bool telemetry_encode_##type_name(uint8_t *payload,                            \
                                      size_t payload_cap,                           \
                                      size_t *payload_len,                          \
                                      const char *topic,                            \
                                      const void *msg);
TELEMETRY_TOPIC_LIST(TELEMETRY_ENCODER_DECLARE)
#undef TELEMETRY_ENCODER_DECLARE

#endif
