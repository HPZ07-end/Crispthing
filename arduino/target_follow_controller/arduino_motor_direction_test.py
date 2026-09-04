#!/usr/bin/env python3
"""Arduino 履带方向自动架空测试。"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit(
        "未安装 pyserial。请先运行：python -m pip install pyserial"
    ) from exc


MOVE_SECONDS = 0.8
STOP_SECONDS = 0.7

TESTS = (
    ("前进参考", "u", "左右履带都应沿小车前进方向转动"),
    ("后退参考", "b", "左右履带都应沿小车后退方向转动"),
    ("原地左转", "l", "左履带后转，右履带前转"),
    ("原地右转", "r", "左履带前转，右履带后转"),
)


def available_ports() -> list[str]:
    return [port.device for port in list_ports.comports()]


def send_command(ser: serial.Serial, command: str) -> None:
    ser.write((command + "\n").encode("ascii"))
    ser.flush()


def read_output(ser: serial.Serial, duration: float) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if line:
            print(f"  ← {line}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Arduino 履带方向自动架空测试"
    )
    parser.add_argument("--port", help="串口名称，例如 COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--list-ports", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    ports = available_ports()

    if args.list_ports:
        print("可用串口：")
        for port in ports:
            print(f"  {port}")
        return 0

    port = args.port
    if not port:
        if len(ports) == 1:
            port = ports[0]
        else:
            print("请用 --port 指定串口。当前检测到：")
            for item in ports:
                print(f"  {item}")
            return 2

    print("安全检查：")
    print("1. config.h 中 MOTOR_ENABLED 必须为 1；")
    print("2. 两条履带必须可靠架空，禁止落地运行；")
    print("3. 手、导线和衣物必须远离履带；")
    print("4. 准备物理断电，并关闭 Arduino 串口监视器。")
    confirmation = input("确认以上条件后输入 SAFE：").strip()

    if confirmation != "SAFE":
        print("已取消测试。")
        return 1

    ser: serial.Serial | None = None

    try:
        ser = serial.Serial(
            port=port,
            baudrate=args.baud,
            timeout=0.05,
            write_timeout=1.0,
        )

        print(f"\n已连接 {port} @ {args.baud}")
        print("等待 Arduino 复位……")
        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print("\n3秒后开始自动测试，请观察或录像。")
        for remaining in (3, 2, 1):
            print(remaining)
            time.sleep(1.0)

        for index, (name, command, expected) in enumerate(TESTS, start=1):
            print(f"\n=== M{index:02d} {name} ===")
            print(f"预期：{expected}")
            print(f"自动发送：{command}")
            send_command(ser, command)
            read_output(ser, MOVE_SECONDS)

            print("自动停车")
            send_command(ser, "s")
            read_output(ser, STOP_SECONDS)

        print("\n测试结束，已发送停车指令。")
        print("请记录每个阶段左、右履带的实际转动方向。")
        return 0

    except serial.SerialException as exc:
        print(f"串口错误：{exc}")
        return 4
    except KeyboardInterrupt:
        print("\n测试已被用户中止。")
        return 130
    finally:
        if ser is not None and ser.is_open:
            try:
                send_command(ser, "s")
                time.sleep(0.1)
            except serial.SerialException:
                pass
            ser.close()


if __name__ == "__main__":
    sys.exit(main())
