# zbus UART Telemetry (STM32F7)

Minimal Zephyr project for custom MCU-to-host communication without micro-ROS.

## Architecture

- `imu_sensor.c`: stub IMU producer thread
- `current_sensor.c`: stub current sensor producer thread
- `channels.c`: zbus channels and subscriber wiring
- `uart_publisher.c`: dedicated thread that subscribes to zbus and writes UART frames

Flow:

1. Sensor threads publish typed structs to zbus channels.
2. UART publisher wakes on zbus updates.
3. UART publisher emits one JSON line per message.

Example line:

```json
{"topic":"imu","t_ms":1234,"seq":56,"ax_mg":-120,"ay_mg":90,"az_mg":1005,"gx_mdps":-1500,"gy_mdps":420,"gz_mdps":800}
```

## Build

```bash
west build -s /workdir/Projects/Svea-Custom-Protocol -b clicker4_stm32f7
```

## Flash

```bash
west flash
```

## UART notes

- Devicetree alias `telemetry-uart` points to `&usart3` in `boards/clicker4_stm32f7.overlay`.
- Current speed is set to `1000000` baud.
- Format is newline-delimited JSON for easy host-side Python parsing and ROS republish.

## Next iterations

- Replace stub sensor values with real drivers while keeping the same message structs.
- Add one command channel (host -> MCU) for config/setpoints.
- Add checksum or COBS framing if you move from text to binary.
