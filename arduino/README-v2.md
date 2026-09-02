
# 自主跟随机器人平台

> 当前阶段：V2 自主跟随实车优化版  
> 开发分支：`v2-openbot-follow`

本项目面向低成本、可扩展的自主跟随机器人平台开发。系统采用 Android 手机完成行人检测、指定人员 ReID、人体姿态估计和相对距离计算，Arduino UNO 负责串口协议解析、安全状态管理、差速运动控制和电机输出。

当前已经完成 V1 手动遥控原型，并打通 V2 的手机感知、CH340 串口通信、Arduino 控制与双履带电机闭环。已完成手机带电机架空跟随，验证了近停远启滞回、两级目标确认、PWM 平滑启动、异常立即停车和转向方向修正。当前工作重点由“打通链路”转为运动控制、主动绕障、电源稳定性和多人场景鲁棒性优化。

---

## 1. 当前版本概览

### V1：手动遥控版

已完成：

- Arduino UNO、双履带底盘和 TB6612 电机驱动集成
- USB 无线手柄 + I2C 转接板遥控
- 前进、后退、原地左转、原地右转和停车
- X/Y 键调速
- 急停检测
- 手柄断连与通信超时停车
- 急停释放后回中重置
- 左右履带方向与电机通道校准

### V2：自主跟随联调版

#### 已完成：

- 手机端指定人员检测、ReID 与 Pose 距离估计
- 手机通过 UDP 输出调试数据
- 手机通过 USB 串口发送 `TARGET` 和 `CMD`
- Arduino 解析 `TARGET` 和 `CMD`
- 相对距离控制
- 相似度阈值判断
- 自动/手动模式切换
- 目标超时停车
- 软件 STOP、ESTOP 状态预留
- OpenBot 手机端连续 3 帧确认目标
- Arduino 端连续 2 条不同 TARGET 确认后才启动
- 中央转向死区
- 差速转向控制
- 电机 PWM 平滑斜坡
- 停车命令立即归零
- 电脑串口模拟手机完成架空电机测试
- CH340 将手机通信与整车供电分离
- 手机 + CH340 + 电池 + 电机架空闭环测试
- `1.12` 停止、`1.16` 重启的距离滞回实机验证
- 固定低速 PWM `70` 条件下的跟随测试
- 自主转向方向修正并实机确认：`TURN_SIGN=-1`
- 电池充电后运行卡顿明显减轻，初步定位为电量/压降相关

#### 待完成：

- 近距离原地对准控制
- 左右转向灵敏度独立标定
- 四路超声波 + 前部 ToF/激光传感器接入
- 主动避障与绕行状态机
- 电池电压监测、低电压告警与保护
- 多人检测框重合时的目标连续性与切换滞回
- 低速落地跟随测试
- I2C 手柄与自主跟随共存的抗干扰处理
- 参数实车标定与长期稳定性测试

---

## 2. 系统架构

```text
Android 手机
├── 摄像头图像
├── Person Detection
├── 指定人员 ReID
├── Pose 肩髋尺度
├── 相对距离与水平偏差计算
└── TARGET / CMD 文本协议
        │
        ├── UDP → 电脑调试窗口
└── USB 串口 → Arduino UNO
                         │
                         ├── 协议解析
                         ├── 模式与安全状态机
                         ├── 连续帧启动确认
                         ├── 主动避障仲裁（规划）
                         ├── 近距离原地对准（规划）
                         ├── 远距离跟随与差速转向
                         ├── PWM 输出斜坡
                         └── 双履带差速驱动

本地传感器（待接入）
├── 前部 ToF/激光：前方精确测距
├── 前超声波：前方交叉确认
├── 左/右超声波：选边与绕行侧距
└── 后超声波：后退脱困安全判断
```

---

## 3. 工程结构

```text
target_follow_controller/
├── target_follow_controller.ino   # 主程序、全局状态与控制循环
├── config.h                       # 引脚、阈值、速度和功能开关
├── remote_control.ino             # 串口协议、CMD 与手动遥控
├── safety_redundancy.ino          # 急停、超时、安全仲裁和电机输出
├── obstacle_avoidance.ino         # 超声波/ToF 避障接口
└── auto_follow.ino                # TARGET 解析和自主跟随控制
```

Arduino 会自动编译同一工程文件夹内的全部 `.ino` 文件。不要在主文件中手动 `#include` 其他 `.ino` 文件。

---

## 4. Arduino 引脚分配

| 功能 | Arduino UNO 引脚 |
|---|---:|
| M2 左履带 PWM | D6 |
| M2 左履带方向 1 | D12 |
| M2 左履带方向 2 | D13 |
| M1 右履带 PWM | D5 |
| M1 右履带方向 1 | D7 |
| M1 右履带方向 2 | D8 |
| I2C 遥控板 SDA | A4 |
| I2C 遥控板 SCL | A5 |
| 急停输入 | A3 |

履带方向参数：

```cpp
#define LEFT_TRACK_DIR   1
#define RIGHT_TRACK_DIR -1
```

---

## 5. 手机—Arduino 通信协议

通信方式：USB 串口文本通信  
建议波特率：`115200 baud`  
行结束符：`\n`  
手机端 `TARGET` 发送频率：约 `5 Hz`

### 5.1 TARGET：目标跟随数据

格式：

```text
TARGET,seq,valid,xError,relativeDistance,similarity
```

示例：

```text
TARGET,175,1,-0.084,1.644,0.868
```

| 字段 | 含义 |
|---|---|
| `seq` | 目标数据序号 |
| `valid` | `1` 表示目标有效，`0` 表示目标丢失 |
| `xError` | 带符号的水平偏差；物理转向方向由实车标定后的 `TURN_SIGN` 统一修正 |
| `relativeDistance` | 注册尺度 / 当前尺度 |
| `similarity` | 当前人员与注册人员的 ReID 相似度 |

相对距离定义：

```text
relativeDistance < 1   → 人比注册位置近
relativeDistance ≈ 1   → 人处于注册位置附近
relativeDistance > 1   → 人比注册位置远
relativeDistance = -1  → 当前距离不可用
```

目标丢失示例：

```text
TARGET,183,0,0.00,-1.0,0.00
```

### 5.2 CMD：模式与安全命令

格式：

```text
CMD,seq,AUTO|MANUAL|STOP|ESTOP
```

| 命令 | Arduino 行为 |
|---|---|
| `AUTO` | 进入自动模式，清除旧目标并等待新 TARGET |
| `MANUAL` | 退出自动跟随，进入手动停车状态 |
| `STOP` | 普通停车并保持锁定 |
| `ESTOP` | 软件急停并保持锁定 |

当前手机界面通过 `Auto Mode` 开关发送：

```text
打开 Auto Mode  → CMD,...,AUTO
关闭 Auto Mode  → CMD,...,MANUAL
```

Arduino 已支持 `STOP` 和 `ESTOP`，但当前 App 尚未提供独立按钮。

### 5.3 OpenBot 后台消息

OpenBot 原有代码还可能发送：

```text
f
h750
c0,0
s100
v250
w500
```

当前 Arduino 仅使用自定义 `TARGET` 和 `CMD` 协议，上述后台消息应被忽略，不能直接控制电机。

---

## 6. 自主跟随控制逻辑

### 6.1 身份确认

```cpp
const float SIMILARITY_MIN = 0.50f;
```

低于阈值时立即停车。

### 6.2 距离控制

```cpp
const float FOLLOW_STOP_RELATIVE_DISTANCE = 1.12f;
const float FOLLOW_RESTART_RELATIVE_DISTANCE = 1.16f;
const float FOLLOW_FULL_SPEED_RELATIVE_DISTANCE = 1.80f;
```

控制规则：

```text
relativeDistance <= 1.12  → 立即停止继续向前
停止后 1.12 ～ 1.16       → 保持停车，不累计启动确认帧
relativeDistance >= 1.16  → 开始累计 Arduino 端 2 帧确认
运行中 1.12 ～ 1.80       → 继续运行并线性计算前进 PWM
relativeDistance >= 1.80  → 使用最大跟随 PWM
relativeDistance <= 0     → 数据无效，立即停车
```

其中 `1.12～1.16` 为距离滞回区间：小车运行时到 `1.12` 才停车；停车后必须重新远到 `1.16`，并连续收到 2 条不同的合格 TARGET，才允许重新启动。这样可以减少距离估计在停止阈值附近波动造成的反复启停。

第一版只向前跟随。目标过近时停车，不主动倒车。

### 6.3 手机端 3 帧 + Arduino 端 2 帧确认

```cpp
const uint8_t FAR_TARGET_CONFIRM_FRAMES = 2;
```

OpenBot 手机端先完成连续 3 帧目标确认；Arduino 随后必须连续收到 2 条序号不同、目标有效、相似度合格且距离超过阈值的 `TARGET`，才允许开始运动。

```text
手机端：连续 3 帧确认目标
Arduino 端：连续 2 条不同 TARGET 再确认
停车迅速：任意异常帧立即停车
```

重复相同 `seq` 的数据不会重复累计确认帧。

### 6.4 转向死区

```cpp
const float TARGET_CENTER_X_THRESHOLD = 0.10f;
```

```text
|xError| <= 0.10  → 认为目标位于中央，不转向
|xError| > 0.10   → 根据偏差大小差速修正
```

死区外的误差会重新映射，使转向量从 0 平滑增加。

自主转向方向已经根据手机画面与实车运动关系完成修正：

```cpp
#define TURN_SIGN -1
```

该参数只修正自主模式下 `xError` 与差速转向方向的对应关系，不改变已经校准正确的左右履带前进方向。

### 6.5 前进与转向

```cpp
const int MIN_SPEED = 70;
const int MAX_SPEED = 70;  // 当前实车标定阶段临时固定低速
const float K_TURN = 70.0f;
```

差速控制形式：

```text
leftSpeed  = forwardSpeed + turnSpeed
rightSpeed = forwardSpeed - turnSpeed
```

远距离跟随阶段继续保留这套“前进 + 差速修正”逻辑，不因后续增加原地对准而改变。

### 6.6 近距离原地对准（已完成设计，待实现）

当前控制在目标较近时优先停车，因此人靠近小车但偏离画面中心时，小车不会继续调整朝向。后续只在近距离增加独立原地对准；远距离跟随逻辑保持不变。

建议初始参数：

```cpp
const float TURN_IN_PLACE_START_X = 0.30f;
const float TURN_IN_PLACE_STOP_X = 0.15f;
const int TURN_IN_PLACE_SPEED = 60;
const uint8_t TURN_IN_PLACE_CONFIRM_FRAMES = 2;
```

目标行为：

| 条件 | 行为 |
|---|---|
| 无障碍，`relativeDistance > 1.16` | 保持现有前进 + 差速跟随 |
| 无障碍，目标较近且 `|xError| >= 0.30` 连续 2 帧同方向 | 左右履带反向，低速原地对准 |
| 原地对准中且 `|xError| > 0.15` | 保持原地转向 |
| `|xError| <= 0.15` | 立即退出原地对准，按距离决定停车或恢复跟随 |
| 目标无效、身份不足、距离无效或数据超时 | 立即停车 |
| 发现障碍或避障仍在进行 | 主动绕障接管，不执行对准或普通跟随 |

使用进入阈值 `0.30` 和退出阈值 `0.15` 构成转向滞回，并要求两帧同方向确认，防止人体框抖动或两人短暂重合触发突然旋转。

---

## 7. 电机输出斜坡

```cpp
#define MOTOR_RAMP_ENABLED 1
const int MOTOR_RAMP_STEP = 5;
const unsigned long CONTROL_INTERVAL_MS = 20;
```

自动跟随算法计算的是目标 PWM，安全输出模块保存当前实际 PWM，并在每个控制周期最多变化 `5`。

例如目标为 `81`：

```text
0 → 5 → 10 → 15 → ... → 80 → 81
```

从 0 到 81 大约需要：

```text
17 × 20 ms ≈ 340 ms
```

规则：

```text
正常起步、调速和转向变化 → 使用斜坡
停车、目标丢失、超时、距离无效、急停 → 立即归零
```

这里的 `L/R` 是实际输出的 PWM，不是编码器测得的履带真实速度。

---

## 8. 安全策略

### 电机驱动板重新上电保护

`MOTOR_ENABLED=0` 现在表示“主动安全关闭”，而不是停止管理电机引脚：

- M1/M2 的 PWM 和方向脚始终配置为输出
- 方向脚主动保持 `LOW`，PWM 主动保持 `0`
- 电机安全初始化是 `setup()` 的第一项动作，早于串口启动和 `delay(300)`
- 启动日志应出现 `MOTOR OUTPUTS DISABLED - pins held LOW`

软件无法控制 Arduino 复位和 Bootloader 执行期间的引脚状态。若要覆盖这一短暂窗口，还应在硬件上增加输入下拉，或使用默认拉低的驱动板 `STBY/EN` 使能控制。电机调试时不要带电反复开关扩展板电源。

当前已实现的安全停车优先级从高到低：

1. 硬件急停输入
2. 软件 ESTOP
3. CMD STOP
4. 手动模式安全约束或自主模式仲裁
5. 前方障碍停车接口
6. TARGET 超时
7. TARGET 有效性、相似度与距离判断
8. 正常自主跟随

目标控制架构的优先级为：

1. 硬件急停、软件 ESTOP、CMD STOP 和通信失联等安全停车
2. 主动避障与完整绕行状态机
3. 近距离原地对准
4. 远距离正常跟随或手动运动请求
5. 待机停车

“避障优先于所有运动”表示检测到障碍后，避障控制器不是简单禁止前进，而是暂时接管履带，主动生成停车、选边、必要时后退、转向和绕行动作；只有完整确认绕过障碍后才交还控制权。急停和失联停车仍高于避障。

自动模式下以下情况立即停车：

- `valid = 0`
- `similarity < SIMILARITY_MIN`
- `relativeDistance <= 0`
- `relativeDistance <= FOLLOW_STOP_RELATIVE_DISTANCE`
- 超过 `TARGET_TIMEOUT_MS` 未收到新目标
- 切换到 `MANUAL`
- 收到 `STOP` 或 `ESTOP`
- 硬件急停触发

当前目标超时参数：

```cpp
const unsigned long TARGET_TIMEOUT_MS = 1000;
```

---

## 9. 主动避障与绕行设计（待实现）

当前 `obstacle_avoidance.ino` 只实现“前方有障碍则停车”，尚未实现主动绕行。下一阶段将其升级为持续状态机，进入避障后不能因为小车刚转向、前传感器暂时看不到障碍就提前退出。

计划状态：

```cpp
enum AvoidanceState {
  AVOID_IDLE,
  AVOID_BRAKE,
  AVOID_SELECT_SIDE,
  AVOID_REVERSE,
  AVOID_TURN,
  AVOID_BYPASS,
  AVOID_CLEAR_CONFIRM
};
```

完整流程：

```text
发现障碍
→ 立即制动
→ 比较左右空间并选择绕行侧
→ 左右都不足时，在后方安全条件下先后退
→ 向空旷侧低速转向
→ 沿障碍侧面前进并保持侧向距离
→ 连续确认前方和绕行方向安全
→ 结束绕行并清除旧目标
→ 重新识别、对准目标
→ 恢复正常跟随
```

传感器计划采用“前部 ToF/激光 + 四路超声波”：

| 位置 | 安装与作用 |
|---|---|
| 前部 ToF/激光 | 朝正前方，提供较精确的前方距离 |
| 前超声波 | 朝正前方，与 ToF 交叉确认障碍 |
| 左/右超声波 | 对称安装在车体纵向中部或略靠前，水平朝外；用于选边和绕行侧距 |
| 后超声波 | 朝正后方，判断能否安全后退脱困 |

左右传感器位于车身中部可以满足第一版绕行，但对原地转向时车头两角扫过的区域存在盲区。初次测试需采用低速转向，并同时检查前方和准备转向一侧的距离；若频繁出现前角接近障碍，再将左右传感器前移或向前外侧偏转约 `15°～25°`。

关键实现要求：

```cpp
return avoidanceActive || isFrontBlocked(distance);
```

即只要绕行尚未完整结束，避障就持续拥有运动控制权。绕行结束后不直接使用绕行前的旧 TARGET，而是等待新的有效目标并重新完成启动确认。

---

## 10. I2C 遥控板现状

自主跟随调试阶段建议：

```cpp
#define I2C_REMOTE_ENABLED 0
```

原因：实测开启 I2C 遥控板后，电机运行时可能出现主循环无法继续处理串口 STOP 的现象；关闭 I2C 后停车恢复正常。

当前推测为电机干扰导致 I2C 通信阻塞或异常。后续需要进一步处理：

- I2C 超时与非阻塞读取
- 电机端抗干扰
- 供电与接地检查
- 遥控板和自动模式共存测试

---

## 11. CH340 独立通信方案（已验证）

商家板的 USB 线同时承担数据和供电。电脑 USB 能驱动架空电机，但手机 OTG 不应承担整车供电，因此使用 CH340 USB-TTL 将手机通信和小车供电分离。

该方案已完成手机带电机架空闭环测试。CH340 只连接 TXD、RXD 和共地，不连接 5V/3.3V，整车由电池独立供电。

### 接线

全部断电后连接：

```text
CH340 TXD → 小车板串口 RXD
CH340 RXD → 小车板串口 TXD
CH340 GND → 小车板串口 GND
```

以下接口不接：

```text
CH340 5V / 3.3V
小车板 VCC
RTS / CTS
RS232 / RS485
```

运行结构：

```text
手机 Type-C
→ OTG 转接头
→ CH340 USB-TTL
→ TXD / RXD / GND
→ 小车板串口

当前 12 V、2200 mAh 电池
→ 小车板电源接口
→ Arduino、驱动板和电机
```

上传程序时拔掉 CH340，使用商家蓝色 USB 线连接电脑；实际运行时拔掉电脑 USB，再连接手机、CH340 和电池。

---

## 12. 编译与上传

### 环境

- Arduino IDE 2.x
- 开发板：Arduino Uno
- 串口监视器：`115200 baud`
- 行尾：`New Line`
- `Wire` 为 Arduino 内置库

### 上传步骤

1. 关闭外部电池。
2. 拔掉 CH340 和外接串口线。
3. 用商家蓝色 USB 数据线连接电脑。
4. 打开 `target_follow_controller.ino`。
5. 选择 `Arduino Uno` 和正确 COM 端口。
6. 点击“验证”。
7. 点击“上传”。
8. 上传完成后关闭串口监视器并拔掉电脑 USB。
9. 履带架空后再连接运行所需供电和通信。

---

## 13. 推荐配置

### 纯软件与协议测试

```cpp
#define MOTOR_ENABLED 0
#define SENSOR_ENABLED 0
#define TOF_ENABLED 0
#define EMERGENCY_STOP_ENABLED 1
#define I2C_REMOTE_ENABLED 0
#define DEBUG_PRINT 1
```

连接电机电源前，先在串口或手机 Logcat 中确认启动日志包含：

```text
MOTOR OUTPUTS DISABLED - pins held LOW
```

### CH340 架空闭环测试

```cpp
#define MOTOR_ENABLED 1
#define SENSOR_ENABLED 0
#define TOF_ENABLED 0
#define EMERGENCY_STOP_ENABLED 1
#define I2C_REMOTE_ENABLED 0
#define DEBUG_PRINT 1
```

当前实车优化阶段固定低速：

```cpp
const int MIN_SPEED = 70;
const int MAX_SPEED = 70;
```

完成左右标定、避障和落地测试后，再逐步提高：

```cpp
const int MAX_SPEED = 90;
```

---

## 14. 测试记录

### V1 手动遥控

- [x] 左右电机通道可独立正反转
- [x] 前进、后退、左转、右转逻辑正确
- [x] 松开方向键停车
- [x] A 键停车
- [x] 手柄断开后停车
- [x] 急停按下停车
- [x] 急停释放后不会自动恢复旧指令
- [x] 7.4 V 电池供电架空测试

### V2 协议与控制

- [x] UDP 连续接收 TARGET
- [x] 手机 Auto Mode 发送 AUTO/MANUAL
- [x] Arduino 解析 TARGET
- [x] Arduino 解析 CMD
- [x] 相对距离方向正确
- [x] 目标丢失数据正确
- [x] 相似度阈值停车
- [x] 距离无效停车
- [x] 目标超时停车
- [x] STOP 锁定
- [x] ESTOP 锁定
- [x] 手机端连续 3 帧确认
- [x] Arduino 端连续 2 条不同 TARGET 后启动
- [x] 重复序号不重复计数
- [x] 中央死区不转向
- [x] 左右偏差产生反向差速
- [x] PWM 平滑起步
- [x] 停车立即归零
- [x] `MOTOR_ENABLED=0` 时 PWM/方向脚仍主动拉低
- [x] 电脑串口模拟手机完成架空电机测试
- [x] CH340 独立串口通信
- [x] 手机 + CH340 + 电池 + 电机架空闭环
- [x] `1.12/1.16` 距离滞回实机测试
- [x] 自主转向符号修正并实机确认
- [x] 电池充电前后对照测试，卡顿与电量/压降相关性较高
- [ ] 近距离原地对准
- [ ] 左右转向灵敏度独立标定
- [ ] 低速落地自主跟随
- [ ] I2C 手柄与自动模式共存
- [ ] 多传感器接入与主动绕障
- [ ] 多人重合场景目标保持

---

## 15. 回归测试用例

| 编号 | 输入条件 | 预期结果 |
|---|---|---|
| T01 | `AUTO` 后无 TARGET | 等待目标并停车 |
| T02 | 仅收到 1 条远目标 | 继续停车 |
| T03 | 连续 2 条不同远目标 | 开始平滑加速 |
| T04 | 重复相同 `seq` | 不重复累计确认帧 |
| T05 | `relativeDistance <= 1.12` | 立即停车 |
| T06 | `valid = 0` | 立即停车 |
| T07 | `relativeDistance = -1` | 立即停车 |
| T08 | `similarity < 0.50` | 立即停车 |
| T09 | 1 秒无新 TARGET | 立即停车 |
| T10 | `xError = 0.05` | 左右 PWM 相同 |
| T11 | `xError = 0.20` | 产生一个方向的差速输出 |
| T12 | `xError = -0.20` | 输出与 T11 镜像，实车均转向目标所在侧 |
| T13 | 关闭 Auto Mode | 切换 MANUAL 并停车 |
| T14 | `STOP` 后继续发送 TARGET | 保持停车 |
| T15 | `ESTOP` 后直接 AUTO | 拒绝恢复 |
| T16 | 远目标启动后发送近目标 | PWM 立即归零 |
| T17 | 停车后距离处于 `1.12～1.16` | 保持停车，不累计确认帧 |
| T18 | 运行中距离处于 `1.12～1.16` | 继续运行，不重新确认 |
| T19 | 停车后连续 2 帧距离不小于 `1.16` | 重新启动 |

---

## 16. Git 工作流

稳定的 V1 保留在 `main`，V2 在独立分支开发：

```bash
git switch v2-openbot-follow
git status
git diff --stat
git add README.md arduino/target_follow_controller
git diff --cached --stat
git commit -m "docs: update follow control and avoidance roadmap"
git push origin v2-openbot-follow
```

提交前必须先查看 `git status` 和暂存区统计，确认没有把日志、编译产物或无关文件带入提交。V2 完成低速落地跟随和第一版主动避障前，暂不合并回 `main`。

---

## 17. 后续计划

后续坚持“一次只修改一个问题、先回放再架空实测”的顺序：

1. **近距离原地对准**：只在目标较近时启用；远距离前进 + 差速跟随保持不变；加入进入/退出滞回和两帧同方向确认。
2. **左右转向不对称**：对比相同幅值的正负 `xError` 与目标 PWM；区分手机偏差、驱动/履带差异，再决定采用 `K_TURN_LEFT/K_TURN_RIGHT` 或左右电机补偿。
3. **电源稳定性**：接入电池电压采样和滤波，记录卡顿前后的电压与 `rx_gap_ms`；设置低电压告警、限速或停车保护。
4. **多人重合干扰**：手机端融合 ReID 相似度、上一帧检测框 IoU、中心点位移和连续帧切换确认，避免一帧重合就切换跟随者。
5. **主动绕障**：先逐个验证前、左、右、后超声波和前部 ToF，再实现制动、选边、后退脱困、转向、侧向绕行、清障确认和目标重捕获状态机。
6. **低速落地与长期测试**：依次验证直行、普通差速转向、近距离对准、绕障和恢复跟随；最后再逐步提高 `MAX_SPEED`。
7. **长期闭环升级**：条件允许时增加左右履带编码器和双路速度 PID，以补偿电机差异、电池电压变化和履带摩擦。

---

## 18. 许可证

课程项目可根据团队要求选择许可证。计划开源时可使用 MIT License；暂不确定时，可以先不添加 License 文件。
