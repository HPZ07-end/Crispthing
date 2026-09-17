#!/usr/bin/env python3
"""Arduino 原地对准逻辑自动回放测试。"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Iterable

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit(
        "未安装 pyserial。请先运行：python -m pip install pyserial"
    ) from exc


@dataclass(frozen=True)
class TestStep:
    name: str
    packets: tuple[str, ...]
    expected: tuple[str, ...]
    wait_after_each: float = 0.25
    settle_time: float = 0.40


def available_ports() -> list[str]:
    return [port.device for port in list_ports.comports()]


def read_lines(ser: serial.Serial, duration: float) -> list[str]:
    deadline = time.monotonic() + duration
    lines: list[str] = []

    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        if line:
            print(f"  ← {line}")
            lines.append(line)

    return lines


def send_packet(ser: serial.Serial, packet: str) -> None:
    ser.write((packet + "\n").encode("utf-8"))
    ser.flush()
    print(f"  → {packet}")


def contains_all(
    lines: Iterable[str], expected: Iterable[str]
) -> tuple[bool, list[str]]:
    combined = "\n".join(lines)
    missing = [text for text in expected if text not in combined]
    return not missing, missing


def run_step(ser: serial.Serial, step: TestStep) -> bool:
    print(f"\n=== {step.name} ===")
    captured: list[str] = []

    for packet in step.packets:
        send_packet(ser, packet)
        captured.extend(read_lines(ser, step.wait_after_each))

    captured.extend(read_lines(ser, step.settle_time))
    passed, missing = contains_all(captured, step.expected)

    if passed:
        print(f"[PASS] {step.name}")
        return True

    print(f"[FAIL] {step.name}")
    for text in missing:
        print(f"       未找到关键输出：{text}")
    return False


def build_tests() -> tuple[TestStep, ...]:
    return (
        TestStep(
            name="T01 进入自动模式并等待目标",
            packets=("CMD,100,AUTO",),
            expected=("action=AUTO", "waiting for target"),
        ),
        TestStep(
            name="T02 安全距离内居中保持停车",
            packets=("TARGET,101,1,0.000,1.100,0.900",),
            expected=(
                "reason=target centered in safe distance",
                "targetL=0, targetR=0",
            ),
        ),
        TestStep(
            name="T03 连续两帧启动向右原地对准",
            packets=(
                "TARGET,102,1,0.300,1.100,0.900",
                "TARGET,103,1,0.300,1.100,0.900",
            ),
            expected=(
                "Alignment confirmation: 1/2, direction=right",
                "Alignment confirmation: 2/2, direction=right",
                "reason=aligned in place",
                "targetL=-80, targetR=80",
            ),
            settle_time=0.50,
        ),
        TestStep(
            name="T04 距离1.14时保持正在进行的原地对准",
            packets=("TARGET,104,1,0.300,1.140,0.900",),
            expected=(
                "TARGET: seq=104",
            ),
        ),
        TestStep(
            name="T05 连续两帧确认后才反向",
            packets=(
                "TARGET,105,1,-0.300,1.140,0.900",
                "TARGET,106,1,-0.300,1.140,0.900",
            ),
            expected=(
                "Alignment reversal confirmation: 1/2, direction=left",
                "reason=confirming alignment reversal",
                "Alignment reversal confirmation: 2/2, direction=left",
                "targetL=80, targetR=-80",
            ),
            settle_time=0.50,
        ),
        TestStep(
            name="T06 进入停止阈值后完成校准",
            packets=("TARGET,107,1,-0.050,1.140,0.900",),
            expected=(
                "reason=in-place alignment completed",
                "targetL=0, targetR=0",
                "L=0, R=0",
            ),
        ),
        TestStep(
            name="T07 未在校准时滞回区不得重新启动",
            packets=("TARGET,108,1,0.300,1.140,0.900",),
            expected=(
                "TARGET: seq=108",
            ),
        ),
        TestStep(
            name="T08 退出到手动停车",
            packets=("CMD,109,MANUAL",),
            expected=("action=MANUAL", "reason=remote stop"),
        ),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Arduino 原地对准逻辑自动回放测试"
    )
    parser.add_argument("--port", help="串口名称，例如 COM6")
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
    print("1. config.h 中必须设置 MOTOR_ENABLED = 0；")
    print("2. 必须关闭 Arduino IDE 的 Serial Monitor；")
    confirmation = input("确认后输入 SAFE：").strip()

    if confirmation != "SAFE":
        print("已取消测试。")
        return 1

    try:
        with serial.Serial(
            port=port,
            baudrate=args.baud,
            timeout=0.05,
            write_timeout=1.0,
        ) as ser:
            print(f"\n已连接 {port} @ {args.baud}")
            print("等待 Arduino 复位……")
            time.sleep(2.0)
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            tests = build_tests()
            passed = sum(run_step(ser, step) for step in tests)

            print("\n==============================")
            print(f"测试结果：{passed}/{len(tests)} 通过")
            print("==============================")
            return 0 if passed == len(tests) else 3

    except serial.SerialException as exc:
        print(f"串口错误：{exc}")
        print("请确认串口号正确，并关闭串口监视器。")
        return 4
    except KeyboardInterrupt:
        print("\n用户中止测试。")
        return 130


if __name__ == "__main__":
    sys.exit(main())
