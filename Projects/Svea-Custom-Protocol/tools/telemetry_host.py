#!/usr/bin/env python3
"""
Host-side reader for Svea telemetry stream.

Supports framed CBOR (default) and newline-delimited JSON.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any

try:
    from rich.console import Console
    from rich.table import Table
except ModuleNotFoundError:
    Console = None
    Table = None

try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError:
    print("Missing dependency: pyserial")
    print("Install with: python3 -m pip install pyserial")
    sys.exit(1)

try:
    import cbor2
except ModuleNotFoundError:
    cbor2 = None

CBOR_MAGIC = 0xA5
MAX_FRAME = 2048


def list_candidate_ports() -> list[Any]:
    ports = []
    for p in list_ports.comports():
        dev = p.device.lower()
        if (
            "ttyacm" in dev
            or "ttyusb" in dev
            or "usbmodem" in dev
            or dev.startswith("com")
        ):
            ports.append(p)
    return ports


def auto_detect_port() -> str | None:
    candidates = list_candidate_ports()
    if not candidates:
        return None

    if len(candidates) > 1:
        return None

    for p in candidates:
        text = f"{p.device} {getattr(p, 'description', '')} {getattr(p, 'manufacturer', '')}".lower()
        if "espressif" in text or "jtag" in text:
            return p.device

    return candidates[0].device


def print_ports() -> None:
    ports = list_candidate_ports()
    if not ports:
        print("No candidate serial ports found.")
        return

    print("Candidate serial ports:")
    for p in ports:
        desc = getattr(p, "description", "") or "unknown"
        manu = getattr(p, "manufacturer", "") or "unknown"
        print(f"  {p.device:16} desc={desc} manufacturer={manu}")


def open_serial(port: str, baud: int, timeout: float) -> serial.Serial:
    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
    except serial.SerialException as exc:
        raise SystemExit(f"Failed to open {port}: {exc}") from exc

    ser.dtr = True
    return ser


def format_msg(msg: dict[str, Any]) -> str:
    topic = msg.get("topic", "unknown")
    t_ms = msg.get("t_ms", "-")
    seq = msg.get("seq", "-")

    if topic == "lsm6dsox":
        return (
            f"lsm6  t={t_ms:>8} seq={seq:>6} "
            f"a=({msg.get('ax_mg', '-'):>5},{msg.get('ay_mg', '-'):>5},{msg.get('az_mg', '-'):>5}) mg "
            f"g=({msg.get('gx_mdps', '-'):>6},{msg.get('gy_mdps', '-'):>6},{msg.get('gz_mdps', '-'):>6}) mdps "
            f"T={msg.get('temp_cdeg', '-'):>5} cdeg"
        )
    if topic == "ads1115":
        return (
            f"ads   t={t_ms:>8} seq={seq:>6} "
            f"AIN=[{msg.get('ain0_mv', '-'):>5},{msg.get('ain1_mv', '-'):>5},"
            f"{msg.get('ain2_mv', '-'):>5},{msg.get('ain3_mv', '-'):>5}] mV"
        )
    if topic == "rc_command":
        return (
            f"rc    t={t_ms:>8} seq={seq:>6} "
            f"steer={msg.get('steering', '-'):>4} thr={msg.get('throttle', '-'):>4} "
            f"gear={msg.get('high_gear', '-')} diff={msg.get('diff_lock', '-')} "
            f"ovr={msg.get('override_mode', '-')} link={msg.get('connected', '-')}"
        )
    if topic.startswith("ina3221"):
        return (
            f"{topic:8} t={t_ms:>8} seq={seq:>6} "
            f"CH1=({msg.get('ch1_bus_mv', '-'):>5}mV,{msg.get('ch1_current_ma', '-'):>5}mA,{msg.get('ch1_power_mw', '-'):>6}mW) "
            f"CH2=({msg.get('ch2_bus_mv', '-'):>5}mV,{msg.get('ch2_current_ma', '-'):>5}mA,{msg.get('ch2_power_mw', '-'):>6}mW) "
            f"CH3=({msg.get('ch3_bus_mv', '-'):>5}mV,{msg.get('ch3_current_ma', '-'):>5}mA,{msg.get('ch3_power_mw', '-'):>6}mW)"
        )
    if topic.startswith("ina226"):
        return (
            f"{topic:8} t={t_ms:>8} seq={seq:>6} "
            f"Vbus={msg.get('bus_mv', '-'):>6} mV "
            f"Vshunt={msg.get('shunt_uv', '-'):>6} uV "
            f"I={msg.get('current_ma', '-'):>6} mA "
            f"P={msg.get('power_mw', '-'):>7} mW"
        )
    if topic == "bq76942":
        return (
            f"bq    t={t_ms:>8} seq={seq:>6} "
            f"Vpack={msg.get('pack_mv', '-'):>6} mV "
            f"Ipack={msg.get('pack_ma', '-'):>6} mA "
            f"SoC={msg.get('soc_deci_pct', '-'):>4} d% "
            f"T={msg.get('temp_cdeg', '-'):>5} cdeg "
            f"cell(min/avg/max)=({msg.get('cell_min_mv', '-'):>4}/"
            f"{msg.get('cell_avg_mv', '-'):>4}/"
            f"{msg.get('cell_max_mv', '-'):>4})mV "
            f"flags=0x{int(msg.get('error_flags', 0)) & 0xFFFFFFFF:08X}"
        )
    return json.dumps(msg, separators=(",", ":"))


def print_stats_table(
    total: int,
    elapsed_s: float,
    per_topic: dict[str, int],
    prev_total: int,
    prev_per_topic: dict[str, int],
    dt: float,
    pretty: bool,
) -> None:
    if dt <= 0:
        dt = 1.0

    total_hz = (total - prev_total) / dt
    avg_total_hz = total / elapsed_s if elapsed_s > 0 else 0.0

    rows: list[tuple[str, int, float, float]] = []
    for topic in sorted(per_topic):
        count = per_topic[topic]
        window_hz = (count - prev_per_topic.get(topic, 0)) / dt
        avg_hz = count / elapsed_s if elapsed_s > 0 else 0.0
        rows.append((topic, count, window_hz, avg_hz))

    if pretty and Console is not None and Table is not None:
        console = Console()
        table = Table(title=f"Telemetry Stats ({elapsed_s:.1f}s)")
        table.add_column("Topic")
        table.add_column("Count", justify="right")
        table.add_column("Hz (1s)", justify="right")
        table.add_column("Hz (avg)", justify="right")
        for topic, count, window_hz, avg_hz in rows:
            table.add_row(topic, str(count), f"{window_hz:.1f}", f"{avg_hz:.1f}")
        table.add_row("TOTAL", str(total), f"{total_hz:.1f}", f"{avg_total_hz:.1f}")
        console.print(table)
        return

    print(f"\n[stats] elapsed={elapsed_s:.1f}s total={total} hz(1s)={total_hz:.1f} hz(avg)={avg_total_hz:.1f}")
    print(f"{'topic':16} {'count':>8} {'hz(1s)':>8} {'hz(avg)':>8}")
    print("-" * 46)
    for topic, count, window_hz, avg_hz in rows:
        print(f"{topic:16} {count:8d} {window_hz:8.1f} {avg_hz:8.1f}")
    print(f"{'TOTAL':16} {total:8d} {total_hz:8.1f} {avg_total_hz:8.1f}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Read Svea telemetry")
    parser.add_argument("--port", help="Serial port (example: /dev/ttyUSB0 or COM5)")
    parser.add_argument("--baud", type=int, default=1000000, help="Baud rate (default: 1000000)")
    parser.add_argument("--encoding", choices=["cbor", "json"], default="cbor", help="Wire encoding")
    parser.add_argument("--raw", action="store_true", help="Print raw frames/lines")
    parser.add_argument("--timeout", type=float, default=0.2, help="Read timeout in seconds")
    parser.add_argument("--stats-interval", type=float, default=1.0, help="Stats print interval in seconds")
    parser.add_argument("--summary-only", action="store_true", help="Only print periodic frequency table")
    parser.add_argument("--pretty", action="store_true", help="Use rich table output if rich is installed")
    parser.add_argument("--list-ports", action="store_true", help="List likely serial ports and exit")
    args = parser.parse_args()

    if args.encoding == "cbor" and cbor2 is None:
        print("Missing dependency: cbor2")
        print("Install with: python3 -m pip install cbor2")
        return 2

    if args.list_ports:
        print_ports()
        return 0

    if not args.port:
        candidates = list_candidate_ports()
        if len(candidates) > 1:
            print("Multiple candidate serial ports found. Pass --port explicitly.")
            print_ports()
            return 2

    port = args.port or auto_detect_port()
    if not port:
        print("No serial port auto-detected. Pass --port explicitly.")
        return 2

    print(f"Opening {port} at {args.baud} baud ({args.encoding})...")
    ser = open_serial(port, args.baud, args.timeout)
    print("Connected. Press Ctrl+C to stop.")
    if args.pretty and Console is None:
        print("[info] rich not installed; using plain text table")

    total = 0
    per_topic: dict[str, int] = {}
    prev_total = 0
    prev_per_topic: dict[str, int] = {}
    start_ts = time.monotonic()
    last_stat = start_ts

    def maybe_print_stats(now: float) -> None:
        nonlocal prev_total, prev_per_topic, last_stat
        if now - last_stat >= args.stats_interval:
            print_stats_table(
                total=total,
                elapsed_s=now - start_ts,
                per_topic=per_topic,
                prev_total=prev_total,
                prev_per_topic=prev_per_topic,
                dt=now - last_stat,
                pretty=args.pretty,
            )
            prev_total = total
            prev_per_topic = per_topic.copy()
            last_stat = now

    def handle_msg(msg: dict[str, Any]) -> None:
        nonlocal total
        topic = str(msg.get("topic", "unknown"))
        total += 1
        per_topic[topic] = per_topic.get(topic, 0) + 1
        if args.summary_only:
            return
        if args.raw:
            print(msg)
        else:
            print(format_msg(msg))

    try:
        if args.encoding == "json":
            while True:
                line = ser.readline()
                now = time.monotonic()
                maybe_print_stats(now)

                if not line:
                    continue

                text = line.decode("utf-8", errors="replace").strip()
                if not text:
                    continue

                try:
                    msg = json.loads(text)
                except json.JSONDecodeError:
                    if args.raw and not args.summary_only:
                        print(f"[raw] {text}")
                    continue

                if isinstance(msg, dict):
                    handle_msg(msg)
            
        rx_buf = bytearray()
        while True:
            chunk = ser.read(256)
            now = time.monotonic()
            maybe_print_stats(now)

            if not chunk:
                continue

            rx_buf.extend(chunk)

            while True:
                if len(rx_buf) < 3:
                    break

                sync_idx = rx_buf.find(bytes([CBOR_MAGIC]))
                if sync_idx < 0:
                    rx_buf.clear()
                    break

                if sync_idx > 0:
                    del rx_buf[:sync_idx]

                if len(rx_buf) < 3:
                    break

                frame_len = (rx_buf[1] << 8) | rx_buf[2]
                if frame_len == 0 or frame_len > MAX_FRAME:
                    del rx_buf[0]
                    continue

                if len(rx_buf) < 3 + frame_len:
                    break

                payload = bytes(rx_buf[3:3 + frame_len])
                del rx_buf[:3 + frame_len]

                try:
                    msg = cbor2.loads(payload)
                except Exception:
                    if args.raw and not args.summary_only:
                        print(f"[raw] invalid_cbor len={frame_len}")
                    continue

                if isinstance(msg, dict):
                    handle_msg(msg)

    except KeyboardInterrupt:
        print("\nStopping.")
        print(f"Final stats: total={total} per_topic={dict(sorted(per_topic.items()))}")
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
