# Svea Custom Protocol (Zephyr)

Custom MCU-to-host telemetry pipeline for Clicker4 STM32F7 using:

- `zbus` internal pub/sub
- framed `CBOR` over UART (`telemetry-uart`)
- Python host decoder (`tools/telemetry_host.py`)

This schema is intentionally aligned with the older `Projects/svea-lli` telemetry density:

- IMU includes accel + gyro vectors (covariance omitted by design)
- INA3221 publishes all 3 rails per sample
- BMS publishes pack + cell summary style fields

## Wire Format

Each UART frame:

- `magic` (1 byte): `0xA5`
- `len` (2 bytes, big-endian)
- `payload` (`len` bytes, CBOR map)

## Current Topics

## Priority Scheduling

The UART publisher now does priority-based arbitration across topic updates.

- Per-topic updates are coalesced as "latest pending sample".
- On each TX slot, the highest-priority pending topic is sent first.
- Heartbeat is a regular zbus topic (`heartbeat_chan`) and uses the same path.
- `bq76942` is configured highest by default; heartbeat is low priority by default.

Tune priorities in [uart_publisher.c](/workdir/Projects/Svea-Custom-Protocol/src/uart_publisher.c) by editing:

- `TOPIC_PRIO_HEARTBEAT`
- `TOPIC_PRIO_BQ76942`
- `TOPIC_PRIO_INA226_A`, `TOPIC_PRIO_INA226_B`
- `TOPIC_PRIO_INA3221_A`, `TOPIC_PRIO_INA3221_B`
- `TOPIC_PRIO_ADS1115`
- `TOPIC_PRIO_LSM6DSOX`

### `lsm6dsox`

Modeled after `sensor_msgs/Imu` core vector content from `svea-lli` (without covariance arrays).

| Field | Type | Unit | Notes |
|---|---|---|---|
| `topic` | string | - | `"lsm6dsox"` |
| `t_ms` | u32 | ms | Uptime timestamp |
| `seq` | u32 | - | Per-topic sequence |
| `ax_mg` | i32 | mg | Accel X |
| `ay_mg` | i32 | mg | Accel Y |
| `az_mg` | i32 | mg | Accel Z |
| `gx_mdps` | i32 | mdps | Gyro X |
| `gy_mdps` | i32 | mdps | Gyro Y |
| `gz_mdps` | i32 | mdps | Gyro Z |
| `temp_cdeg` | i32 | cdegC | IMU temp proxy |

### `ads1115`

Single sample includes all ADC channels for density.

| Field | Type | Unit | Notes |
|---|---|---|---|
| `topic` | string | - | `"ads1115"` |
| `t_ms` | u32 | ms | Uptime timestamp |
| `seq` | u32 | - | Per-topic sequence |
| `ain0_mv` | i32 | mV | ADC channel 0 |
| `ain1_mv` | i32 | mV | ADC channel 1 |
| `ain2_mv` | i32 | mV | ADC channel 2 |
| `ain3_mv` | i32 | mV | ADC channel 3 |

### `ina3221_a` / `ina3221_b`

Dense 3-rail payload, directly mirroring old INA3221 layout (`ESC`, `12V`, `5V`) from `svea-lli`.

| Field | Type | Unit | Notes |
|---|---|---|---|
| `topic` | string | - | `"ina3221_a"` or `"ina3221_b"` |
| `t_ms` | u32 | ms | Uptime timestamp |
| `seq` | u32 | - | Per-topic sequence |
| `ch1_bus_mv` | i32 | mV | INA3221 channel 1 bus voltage |
| `ch1_current_ma` | i32 | mA | INA3221 channel 1 current |
| `ch1_power_mw` | i32 | mW | INA3221 channel 1 power |
| `ch2_bus_mv` | i32 | mV | INA3221 channel 2 bus voltage |
| `ch2_current_ma` | i32 | mA | INA3221 channel 2 current |
| `ch2_power_mw` | i32 | mW | INA3221 channel 2 power |
| `ch3_bus_mv` | i32 | mV | INA3221 channel 3 bus voltage |
| `ch3_current_ma` | i32 | mA | INA3221 channel 3 current |
| `ch3_power_mw` | i32 | mW | INA3221 channel 3 power |

### `bq76942`

Battery-state style summary (compact form of old `BatteryState` output).

| Field | Type | Unit | Notes |
|---|---|---|---|
| `topic` | string | - | `"bq76942"` |
| `t_ms` | u32 | ms | Uptime timestamp |
| `seq` | u32 | - | Per-topic sequence |
| `pack_mv` | i32 | mV | Pack voltage |
| `pack_ma` | i32 | mA | Pack current |
| `soc_deci_pct` | i32 | 0.1% | SoC in deci-percent |
| `temp_cdeg` | i32 | cdegC | Pack temp proxy |
| `cell_min_mv` | i32 | mV | Min cell voltage |
| `cell_avg_mv` | i32 | mV | Avg cell voltage |
| `cell_max_mv` | i32 | mV | Max cell voltage |
| `error_flags` | u32 | bitfield | Fault/status flags |

### `ina226_a` / `ina226_b`

| Field | Type | Unit | Notes |
|---|---|---|---|
| `topic` | string | - | `"ina226_a"` or `"ina226_b"` |
| `t_ms` | u32 | ms | Uptime timestamp |
| `seq` | u32 | - | Per-topic sequence |
| `bus_mv` | i32 | mV | Bus voltage |
| `shunt_uv` | i32 | uV | Shunt voltage |
| `current_ma` | i32 | mA | Current |
| `power_mw` | i32 | mW | Power |

### `heartbeat`

| Field | Type | Unit | Notes |
|---|---|---|---|
| `topic` | string | - | `"heartbeat"` |
| `t_ms` | u32 | ms | Uptime timestamp |
| `seq` | u32 | - | Heartbeat sequence |

### `status`

| Field | Type | Unit | Notes |
|---|---|---|---|
| `topic` | string | - | `"status"` |
| `t_ms` | u32 | ms | Uptime timestamp |
| `msg` | string | - | currently `"telemetry_online"` |

## Build and Flash

```bash
west build -p always -b clicker4_stm32f7 /workdir/Projects/Svea-Custom-Protocol
west flash
```

## Host Script

Install dependencies:

```bash
python3 -m pip install pyserial cbor2 rich
```

Run:

```bash
python3 /workdir/Projects/Svea-Custom-Protocol/tools/telemetry_host.py \
  --port /dev/ttyUSB0 \
  --baud 1000000 \
  --encoding cbor \
  --summary-only \
  --stats-interval 1 \
  --pretty
```
