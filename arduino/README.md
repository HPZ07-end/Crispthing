# Arduino 五模块精简版

主程序仅拆成用户指定的五个功能模块，另保留一个入口文件：

```text
target_follow_controller/
├── target_follow_controller.ino   # setup / loop，总调度
├── config.h                       # 开关、引脚、参数、公共数据类型
├── remote_control.ino             # 遥控与串口接收
├── obstacle_avoidance.ino         # 超声波、ToF、避障判断
├── auto_follow.ino                # TARGET 解析与自主跟随算法
└── safety_redundancy.ino          # 急停、超时、优先级、最终电机输出
```

## 当前串口输入

自动跟随：

```text
TARGET,visible,x,size,quality
```

遥控模式（串口监视器选择 Newline）：

```text
f  前进
b  后退
l  左转
r  右转
s  停止
a  返回自动跟随模式
```

## 首次验证

`config.h` 中保持：

```cpp
#define MOTOR_ENABLED 0
#define SENSOR_ENABLED 0
#define TOF_ENABLED 0
#define DEBUG_PRINT 1
```

先验证编译、串口解析和左右履带指令打印，再逐步启用传感器与电机。
