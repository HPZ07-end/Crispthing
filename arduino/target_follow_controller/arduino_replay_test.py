#!/usr/bin/env python3
"""
Arduino 自主跟随协议自动回放测试

用途：
1. 自动向 Arduino 发送 CMD / TARGET 测试序列；
2. 读取 Arduino 串口输出；
3. 根据关键日志判断主要安全逻辑是否通过。

运行前：
- 强烈建议 config.h 中设置 MOTOR_ENABLED 0；
- 或者将履带可靠架空，并准备物理断电；
- 关闭 Arduino IDE 的 Serial Monitor，否则串口会被占用。
"""

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
    wait_after_each: float = 0.15
    settle_time: float = 0.35


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
    payload = (packet.rstrip("\r\n") + "\n").encode("utf-8")
    ser.write(payload)
    ser.flush()
    print(f"  → {packet}")


def contains_all(lines: Iterable[str], expected: Iterable[str]) -> tuple[bool, list[str]]:
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
    for item in missing:
        print(f"       未找到关键输出：{item}")
    return False


def build_tests() -> tuple[TestStep, ...]:
    return (
        TestStep(
            name="T01 AUTO 后等待新目标",
            packets=("CMD,100,AUTO",),
            expected=("action=AUTO", "waiting for target"),
        ),
        TestStep(
            name="T02 连续两条不同远目标后启动",
            packets=(
                "TARGET,101,1,0.000,1.500,0.818",
                "TARGET,102,1,0.000,1.500,0.818",
            ),
            expected=(
                "Far target confirmation: 1/2",
                "Far target confirmation: 2/2",
                "reason=auto follow",
            ),
            wait_after_each=0.10,
            settle_time=0.55,
        ),
        TestStep(
            name="T03 近距离立即停车",
            packets=("TARGET,104,1,0.000,1.050,0.818",),
            expected=("target centered in safe distance", "L=0", "R=0"),
        ),
        TestStep(
            name="T04 重复序号不得重复累计",
            packets=(
                "TARGET,105,1,0.000,1.500,0.818",
                "TARGET,105,1,0.000,1.500,0.818",
                "TARGET,105,1,0.000,1.500,0.818",
            ),
            expected=("Far target confirmation: 1/2",),
            wait_after_each=0.10,
            settle_time=0.25,
        ),
        TestStep(
            name="T05 目标丢失立即停车",
            packets=("TARGET,106,0,0.000,-1.000,0.000",),
            expected=("target invalid", "L=0", "R=0"),
        ),
        TestStep(
            name="T06 距离无效立即停车",
            packets=("TARGET,107,1,0.000,-1.000,0.818",),
            expected=("invalid relative distance", "L=0", "R=0"),
        ),
        TestStep(
            name="T07 相似度不足立即停车",
            packets=("TARGET,108,1,0.000,1.500,0.300",),
            expected=("low target similarity", "L=0", "R=0"),
        ),
        TestStep(
            name="T08 滞回区内不得重新启动",
            packets=(
                "CMD,109,AUTO",
                "TARGET,110,1,0.000,1.150,0.818",
                "TARGET,111,1,0.000,1.150,0.818",
            ),
            expected=("waiting for restart distance", "L=0", "R=0"),
        ),
        TestStep(
            name="T09 达到重启阈值并连续两帧后启动",
            packets=(
                "TARGET,112,1,0.000,1.170,0.818",
                "TARGET,113,1,0.000,1.170,0.818",
            ),
            expected=(
                "Far target confirmation: 1/2",
                "Far target confirmation: 2/2",
                "reason=auto follow",
            ),
        ),
        TestStep(
            name="T10 运行中进入滞回区仍继续运行",
            packets=("TARGET,114,1,0.300,1.150,0.818",),
            expected=("reason=auto follow",),
        ),
        TestStep(
            name="T11 到达停止阈值立即停车",
            packets=("TARGET,115,1,0.000,1.100,0.818",),
            expected=("target centered in safe distance", "L=0", "R=0"),
        ),
        TestStep(
            name="T12 MANUAL 退出自动模式",
            packets=("CMD,116,MANUAL",),
            expected=("action=MANUAL", "remote stop"),
        ),

        TestStep(
            name="T13 重新进入AUTO并启动直行",
            packets=(
                "CMD,200,AUTO",
                "TARGET,201,1,0.000,1.500,0.900",
                "TARGET,202,1,0.000,1.500,0.900",
            ),
            expected=(
                "action=AUTO",
                "Far target confirmation: 1/2",
                "Far target confirmation: 2/2",
                "targetL=80, targetR=80",
            ),
            wait_after_each=0.10,
        ),

        TestStep(
            name="T14 偏差低于启动阈值不得转向",
            packets=("TARGET,203,1,0.110,1.500,0.900",),
            expected=(
                "TARGET: seq=203",
            ),
        ),

        TestStep(
            name="T15 大偏差启动非线性转向",
            packets=("TARGET,204,1,0.300,1.500,0.900",),
            expected=(
                "targetL=72, targetR=80",
            ),
        ),

        TestStep(
            name="T16 回到滞回区仍保持转向",
            packets=("TARGET,205,1,0.110,1.500,0.900",),
            expected=(
                "targetL=79, targetR=80",
            ),
        ),

        TestStep(
            name="T17 进入停止阈值后停止转向",
            packets=("TARGET,206,1,0.050,1.500,0.900",),
            expected=(
                "targetL=80, targetR=80",
            ),
        ),

        TestStep(
            name="T18 反向偏差低于启动阈值不得反转",
            packets=("TARGET,207,1,-0.110,1.500,0.900",),
            expected=(
                "TARGET: seq=207",
            ),
        ),

        TestStep(
            name="T19 反向大偏差启动反向转向",
            packets=("TARGET,208,1,-0.300,1.500,0.900",),
            expected=(
                "targetL=80, targetR=72",
            ),
        ),

        TestStep(
            name="T20 返回中心后停止反向转向",
            packets=("TARGET,209,1,-0.050,1.500,0.900",),
            expected=(
                "targetL=80, targetR=80",
            ),
        ),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Arduino 自主跟随协议自动回放测试"
    )
    parser.add_argument(
        "--port",
        help="串口名称，例如 COM5；省略时自动选择唯一串口",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="波特率，默认 115200",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="列出当前可用串口后退出",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    ports = available_ports()

    if args.list_ports:
        if ports:
            print("可用串口：")
            for port in ports:
                print(f"  {port}")
        else:
            print("未发现串口。")
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
    print("1. 推荐 Arduino 中 MOTOR_ENABLED = 0；")
    print("2. 若电机已启用，必须将履带可靠架空并准备物理断电；")
    print("3. 必须关闭 Arduino IDE 的 Serial Monitor。")
    confirmation = input("确认安全后输入 SAFE：").strip()

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
            passed_count = 0

            for step in tests:
                if run_step(ser, step):
                    passed_count += 1

            print("\n==============================")
            print(f"测试结果：{passed_count}/{len(tests)} 通过")
            print("==============================")

            return 0 if passed_count == len(tests) else 3

    except serial.SerialException as exc:
        print(f"串口错误：{exc}")
        print("请确认串口号正确，并关闭 Arduino IDE 串口监视器。")
        return 4
    except KeyboardInterrupt:
        print("\n用户中止测试。")
        return 130


if __name__ == "__main__":
    sys.exit(main())
