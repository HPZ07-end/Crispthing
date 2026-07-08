# Target Follow Controller

This is the main Arduino-side controller for the autonomous following robot.

## Role

The Arduino UNO receives target information from OpenBot and computes motor speed commands.

OpenBot handles:

- Camera input
- Purple vest detection
- Target visibility
- Target horizontal position
- Target size
- Detection quality

Arduino handles:

- Serial message parsing
- Safety checks
- Simple following decision
- Left/right motor speed calculation
- Motor driver output
- Future distance sensor integration

## Serial Protocol

OpenBot sends one line per frame:

```c++
TARGET,visible,x,size,quality
```

## The process

loop()
  ↓
readSerialLines()
  ↓
收到一整行 TARGET 数据
  ↓
handleLine()
  ↓
parseTargetMessage()
  ↓
得到 visible / x / size / quality
  ↓
computeMotionCommand()
  ↓
判断：
  - 目标是否可见
  - quality 是否足够
  - 前方距离是否安全
  - size 是否太大
  ↓
计算：
  - forwardSpeed
  - turnSpeed
  - leftSpeed
  - rightSpeed
  ↓
applyMotionCommand()
  ↓
调试模式：打印左右轮速度
电机模式：真正控制电机


