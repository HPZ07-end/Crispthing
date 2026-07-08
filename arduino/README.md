# Arduino Target Follow Protocol

## 通信协议

OpenBot sends target data to Arduino through serial.

Format:
TARGET,visible,x,size,quality

Example:
TARGET,1,-0.25,0.12,0.85

Fields:
visible:
  1 = target visible
  0 = target lost

x:
  normalized horizontal offset
  range: -1.0 to 1.0
  x < 0 means target is on the left
  x > 0 means target is on the right

size:
  target size ratio, used for rough distance estimation

quality:
  detection quality, range 0.0 to 1.

## Arduino Code for Crispthing

本目录存放自主跟随小车 Arduino UNO 侧代码。

Arduino 侧主要负责：

1. 接收 OpenBot 输出的目标状态数据；
2. 解析 `TARGET,visible,x,size,quality`；
3. 根据目标位置、目标大小和识别质量计算左右轮速度；
4. 预留距离传感器安全停车逻辑；
5. 后续根据电机驱动板接线控制底盘运动。



### Directory Structure

\```text
arduino/
├── tests/
│   ├── test_00_blink/
│   ├── test_01_serial_print/
│   ├── test_02_serial_command/
│   ├── test_03_target_parser_debug/
│   ├── test_04_motor_driver_test/
│   └── test_05_distance_sensor_test/
├── target_follow_controller/
└── docs/
