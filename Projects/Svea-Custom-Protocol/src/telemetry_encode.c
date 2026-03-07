#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "messages.h"
#include "telemetry_encode.h"

struct cbor_buf {
    uint8_t *buf;
    size_t len;
    size_t cap;
};

static bool cbor_put_byte(struct cbor_buf *b, uint8_t v) {
    if (b->len >= b->cap) {
        return false;
    }
    b->buf[b->len++] = v;
    return true;
}

static bool cbor_put_type_val(struct cbor_buf *b, uint8_t major, uint32_t val) {
    uint8_t prefix = (uint8_t)(major << 5);

    if (val <= 23U) {
        return cbor_put_byte(b, (uint8_t)(prefix | val));
    }

    if (val <= 0xFFU) {
        return cbor_put_byte(b, (uint8_t)(prefix | 24U)) &&
               cbor_put_byte(b, (uint8_t)val);
    }

    if (val <= 0xFFFFU) {
        return cbor_put_byte(b, (uint8_t)(prefix | 25U)) &&
               cbor_put_byte(b, (uint8_t)(val >> 8)) &&
               cbor_put_byte(b, (uint8_t)val);
    }

    return cbor_put_byte(b, (uint8_t)(prefix | 26U)) &&
           cbor_put_byte(b, (uint8_t)(val >> 24)) &&
           cbor_put_byte(b, (uint8_t)(val >> 16)) &&
           cbor_put_byte(b, (uint8_t)(val >> 8)) &&
           cbor_put_byte(b, (uint8_t)val);
}

static bool cbor_put_uint32(struct cbor_buf *b, uint32_t v) {
    return cbor_put_type_val(b, 0U, v);
}

static bool cbor_put_int32(struct cbor_buf *b, int32_t v) {
    if (v >= 0) {
        return cbor_put_type_val(b, 0U, (uint32_t)v);
    }

    return cbor_put_type_val(b, 1U, (uint32_t)(-1 - v));
}

static bool cbor_put_bool(struct cbor_buf *b, bool v) {
    return cbor_put_byte(b, (uint8_t)((7U << 5) | (v ? 21U : 20U)));
}

static bool cbor_put_float32(struct cbor_buf *b, float v) {
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));

    return cbor_put_byte(b, (uint8_t)((7U << 5) | 26U)) &&
           cbor_put_byte(b, (uint8_t)(bits >> 24)) &&
           cbor_put_byte(b, (uint8_t)(bits >> 16)) &&
           cbor_put_byte(b, (uint8_t)(bits >> 8)) &&
           cbor_put_byte(b, (uint8_t)bits);
}

static bool cbor_put_tstr(struct cbor_buf *b, const char *s) {
    size_t n = strlen(s);
    if (n > UINT16_MAX) {
        return false;
    }

    if (!cbor_put_type_val(b, 3U, (uint32_t)n)) {
        return false;
    }

    for (size_t i = 0; i < n; i++) {
        if (!cbor_put_byte(b, (uint8_t)s[i])) {
            return false;
        }
    }

    return true;
}

static bool cbor_put_map_start(struct cbor_buf *b, uint32_t pairs) {
    return cbor_put_type_val(b, 5U, pairs);
}

bool telemetry_encode_status_online(uint8_t *payload,
                                    size_t payload_cap,
                                    size_t *payload_len,
                                    uint32_t t_ms) {
    struct cbor_buf b = {.buf = payload, .len = 0, .cap = payload_cap};

    bool ok = cbor_put_map_start(&b, 3U) &&
              cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, "status") &&
              cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, t_ms) &&
              cbor_put_tstr(&b, "msg") && cbor_put_tstr(&b, "telemetry_online");
    if (!ok) {
        return false;
    }

    *payload_len = b.len;
    return true;
}

bool telemetry_encode_heartbeat_msg(uint8_t *payload,
                                    size_t payload_cap,
                                    size_t *payload_len,
                                    const char *topic,
                                    const void *msg) {
    const struct heartbeat_msg *m = msg;
    struct cbor_buf b = {.buf = payload, .len = 0, .cap = payload_cap};

    bool ok = cbor_put_map_start(&b, 3U) &&
              cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, topic) &&
              cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, m->t_ms) &&
              cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, m->seq);
    if (!ok) {
        return false;
    }
    *payload_len = b.len;
    return true;
}

bool telemetry_encode_rc_command_msg(uint8_t *payload,
                                     size_t payload_cap,
                                     size_t *payload_len,
                                     const char *topic,
                                     const void *msg) {
    const struct rc_command_msg *m = msg;
    struct cbor_buf b = {.buf = payload, .len = 0, .cap = payload_cap};

    bool ok = cbor_put_map_start(&b, 9U) &&
              cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, topic) &&
              cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, m->t_ms) &&
              cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, m->seq) &&
              cbor_put_tstr(&b, "steering") && cbor_put_int32(&b, (int32_t)m->steering) &&
              cbor_put_tstr(&b, "throttle") && cbor_put_int32(&b, (int32_t)m->throttle) &&
              cbor_put_tstr(&b, "high_gear") && cbor_put_bool(&b, m->high_gear) &&
              cbor_put_tstr(&b, "diff_lock") && cbor_put_bool(&b, m->diff_lock) &&
              cbor_put_tstr(&b, "override_mode") && cbor_put_bool(&b, m->override_mode) &&
              cbor_put_tstr(&b, "connected") && cbor_put_bool(&b, m->connected);
    if (!ok) {
        return false;
    }
    *payload_len = b.len;
    return true;
}

bool telemetry_encode_lsm6dsox_msg(uint8_t *payload,
                                   size_t payload_cap,
                                   size_t *payload_len,
                                   const char *topic,
                                   const void *msg) {
    const struct lsm6dsox_msg *m = msg;
    struct cbor_buf b = {.buf = payload, .len = 0, .cap = payload_cap};

    bool ok = cbor_put_map_start(&b, 10U) &&
              cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, topic) &&
              cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, m->t_ms) &&
              cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, m->seq) &&
              cbor_put_tstr(&b, "ax_mg") && cbor_put_float32(&b, m->ax_mg) &&
              cbor_put_tstr(&b, "ay_mg") && cbor_put_float32(&b, m->ay_mg) &&
              cbor_put_tstr(&b, "az_mg") && cbor_put_float32(&b, m->az_mg) &&
              cbor_put_tstr(&b, "gx_mdps") && cbor_put_float32(&b, m->gx_mdps) &&
              cbor_put_tstr(&b, "gy_mdps") && cbor_put_float32(&b, m->gy_mdps) &&
              cbor_put_tstr(&b, "gz_mdps") && cbor_put_float32(&b, m->gz_mdps) &&
              cbor_put_tstr(&b, "temp_cdeg") && cbor_put_float32(&b, m->temp_cdeg);
    if (!ok) {
        return false;
    }
    *payload_len = b.len;
    return true;
}

bool telemetry_encode_ads1115_msg(uint8_t *payload,
                                  size_t payload_cap,
                                  size_t *payload_len,
                                  const char *topic,
                                  const void *msg) {
    const struct ads1115_msg *m = msg;
    struct cbor_buf b = {.buf = payload, .len = 0, .cap = payload_cap};

    bool ok = cbor_put_map_start(&b, 7U) &&
              cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, topic) &&
              cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, m->t_ms) &&
              cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, m->seq) &&
              cbor_put_tstr(&b, "ain0_mv") && cbor_put_float32(&b, m->ain0_mv) &&
              cbor_put_tstr(&b, "ain1_mv") && cbor_put_float32(&b, m->ain1_mv) &&
              cbor_put_tstr(&b, "ain2_mv") && cbor_put_float32(&b, m->ain2_mv) &&
              cbor_put_tstr(&b, "ain3_mv") && cbor_put_float32(&b, m->ain3_mv);
    if (!ok) {
        return false;
    }
    *payload_len = b.len;
    return true;
}

bool telemetry_encode_ina3221_msg(uint8_t *payload,
                                  size_t payload_cap,
                                  size_t *payload_len,
                                  const char *topic,
                                  const void *msg) {
    const struct ina3221_msg *m = msg;
    struct cbor_buf b = {.buf = payload, .len = 0, .cap = payload_cap};

    bool ok = cbor_put_map_start(&b, 12U) &&
              cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, topic) &&
              cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, m->t_ms) &&
              cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, m->seq) &&
              cbor_put_tstr(&b, "ch1_bus_mv") && cbor_put_float32(&b, m->ch1_bus_mv) &&
              cbor_put_tstr(&b, "ch1_current_ma") && cbor_put_float32(&b, m->ch1_current_ma) &&
              cbor_put_tstr(&b, "ch1_power_mw") && cbor_put_float32(&b, m->ch1_power_mw) &&
              cbor_put_tstr(&b, "ch2_bus_mv") && cbor_put_float32(&b, m->ch2_bus_mv) &&
              cbor_put_tstr(&b, "ch2_current_ma") && cbor_put_float32(&b, m->ch2_current_ma) &&
              cbor_put_tstr(&b, "ch2_power_mw") && cbor_put_float32(&b, m->ch2_power_mw) &&
              cbor_put_tstr(&b, "ch3_bus_mv") && cbor_put_float32(&b, m->ch3_bus_mv) &&
              cbor_put_tstr(&b, "ch3_current_ma") && cbor_put_float32(&b, m->ch3_current_ma) &&
              cbor_put_tstr(&b, "ch3_power_mw") && cbor_put_float32(&b, m->ch3_power_mw);
    if (!ok) {
        return false;
    }
    *payload_len = b.len;
    return true;
}

bool telemetry_encode_bq76942_msg(uint8_t *payload,
                                  size_t payload_cap,
                                  size_t *payload_len,
                                  const char *topic,
                                  const void *msg) {
    const struct bq76942_msg *m = msg;
    struct cbor_buf b = {.buf = payload, .len = 0, .cap = payload_cap};

    bool ok = cbor_put_map_start(&b, 11U) &&
              cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, topic) &&
              cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, m->t_ms) &&
              cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, m->seq) &&
              cbor_put_tstr(&b, "pack_mv") && cbor_put_float32(&b, m->pack_mv) &&
              cbor_put_tstr(&b, "pack_ma") && cbor_put_float32(&b, m->pack_ma) &&
              cbor_put_tstr(&b, "soc_pct") && cbor_put_float32(&b, m->soc_pct) &&
              cbor_put_tstr(&b, "temp_cdeg") && cbor_put_float32(&b, m->temp_cdeg) &&
              cbor_put_tstr(&b, "cell_min_mv") && cbor_put_float32(&b, m->cell_min_mv) &&
              cbor_put_tstr(&b, "cell_avg_mv") && cbor_put_float32(&b, m->cell_avg_mv) &&
              cbor_put_tstr(&b, "cell_max_mv") && cbor_put_float32(&b, m->cell_max_mv) &&
              cbor_put_tstr(&b, "error_flags") && cbor_put_uint32(&b, m->error_flags);
    if (!ok) {
        return false;
    }
    *payload_len = b.len;
    return true;
}

bool telemetry_encode_ina226_msg(uint8_t *payload,
                                 size_t payload_cap,
                                 size_t *payload_len,
                                 const char *topic,
                                 const void *msg) {
    const struct ina226_msg *m = msg;
    struct cbor_buf b = {.buf = payload, .len = 0, .cap = payload_cap};

    bool ok = cbor_put_map_start(&b, 7U) &&
              cbor_put_tstr(&b, "topic") && cbor_put_tstr(&b, topic) &&
              cbor_put_tstr(&b, "t_ms") && cbor_put_uint32(&b, m->t_ms) &&
              cbor_put_tstr(&b, "seq") && cbor_put_uint32(&b, m->seq) &&
              cbor_put_tstr(&b, "bus_mv") && cbor_put_float32(&b, m->bus_mv) &&
              cbor_put_tstr(&b, "shunt_uv") && cbor_put_float32(&b, m->shunt_uv) &&
              cbor_put_tstr(&b, "current_ma") && cbor_put_float32(&b, m->current_ma) &&
              cbor_put_tstr(&b, "power_mw") && cbor_put_float32(&b, m->power_mw);
    if (!ok) {
        return false;
    }
    *payload_len = b.len;
    return true;
}
