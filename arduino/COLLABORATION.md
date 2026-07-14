# 三人协同建议

## 1. 推荐分工

| 成员 | 主要文件 | 主要任务 |
|---|---|---|
| 组员 A | `SerialProtocol.*`、OpenBot 端 | 串口协议、TARGET 数据、通信联调 |
| 组员 B | `SensorManager.*`、`SafetyManager.*` | 超声波、ToF、急停和避障安全规则 |
| 组员 C | `FollowController.*`、`MotorDriver.*` | 状态机、速度计算、履带方向和实车驱动 |
| 全员共同 | `Config.h`、`RobotTypes.h`、主 `.ino` | 接口评审、集成测试；修改前先沟通 |

## 2. Git 使用方式

保持 `main` 分支始终可验证，每项功能从独立分支开发：

```text
feature/serial-protocol
feature/sensors-safety
feature/motor-control
feature/bypass
fix/track-direction
```

一次提交只解决一个问题，例如：

```text
feat(serial): parse TARGET message
feat(sensor): add front ultrasonic reading
fix(motor): reverse right track direction
safety: stop on target timeout
```

## 3. 接口约定

1. `SerialProtocol` 只产生 `TargetData`，不直接决定电机速度。
2. `SensorManager` 只产生 `DistanceData`，无效值统一为 `-1`。
3. `SafetyManager` 只回答是否触发安全条件。
4. `FollowController` 只输出 `MotionCommand`，不直接操作引脚。
5. `MotorDriver` 只执行左右履带速度，不参与识别与决策。
6. 所有公共结构只放在 `RobotTypes.h`，所有参数只放在 `Config.h`。

## 4. 合并前检查

- Arduino IDE“验证”通过。
- `MOTOR_ENABLED` 不应被意外改为 `true` 后提交。
- 协议、接线或参数变化时同步修改 `docs/`。
- 不在多个模块中复制同一个引脚或阈值。
- 实车代码合并前先完成架空低速测试。
