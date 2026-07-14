#ifndef TARGET_FOLLOW_CONFIG_H
#define TARGET_FOLLOW_CONFIG_H

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// ============================================================
// 1. 功能开关
// ============================================================
#define MOTOR_ENABLED 0
#define SENSOR_ENABLED 0
#define TOF_ENABLED 0
#define EMERGENCY_STOP_ENABLED 1
#define DEBUG_PRINT 1

// ============================================================
// 2. 电机与履带参数
// ============================================================
const int LEFT_PWM  = 5;
const int LEFT_IN1  = 7;
const int LEFT_IN2  = 8;
const int RIGHT_PWM = 6;
const int RIGHT_IN1 = 9;
const int RIGHT_IN2 = 10;

#define LEFT_TRACK_DIR 1
#define RIGHT_TRACK_DIR 1
#define TURN_SIGN 1

// ============================================================
// 3. 距离传感器引脚
// ============================================================
const int ULTRA_LEFT_TRIG  = 2;
const int ULTRA_LEFT_ECHO  = 3;
const int ULTRA_FRONT_TRIG = 4;
const int ULTRA_FRONT_ECHO = 11;
const int ULTRA_RIGHT_TRIG = 12;
const int ULTRA_RIGHT_ECHO = A0;
const int ULTRA_REAR_TRIG  = A1;
const int ULTRA_REAR_ECHO  = A2;

// ToF 默认使用 I2C：SDA=A4，SCL=A5

// ============================================================
// 4. 安全输入
// ============================================================
const int EMERGENCY_STOP_PIN = 13;

// ============================================================
// 5. 自主跟随参数
// ============================================================
const float QUALITY_MIN = 0.50f;
const float SIZE_FAR  = 0.08f;
const float SIZE_NEAR = 0.30f;
const int MIN_SPEED = 70;
const int MAX_SPEED = 130;
const float K_TURN = 70.0f;
const float TARGET_CENTER_X_THRESHOLD = 0.25f;

// ============================================================
// 6. 避障参数
// ============================================================
const float ULTRA_FRONT_SAFE_CM = 45.0f;
const float ULTRA_SIDE_SAFE_CM  = 35.0f;
const float ULTRA_REAR_SAFE_CM  = 35.0f;
const float TOF_TOO_CLOSE_CM = 80.0f;
const float TOF_FAR_CM       = 180.0f;

// ============================================================
// 7. 超时与控制周期
// ============================================================
const unsigned long TARGET_TIMEOUT_MS = 500;
const unsigned long REMOTE_TIMEOUT_MS = 500;
const unsigned long CONTROL_INTERVAL_MS = 50;

// ============================================================
// 8. 公共数据类型
// ============================================================
struct TargetData {
  bool visible;
  float x;
  float size;
  float quality;
  unsigned long receivedAt;
};

struct DistanceData {
  float ultraLeftCm;
  float ultraFrontCm;
  float ultraRightCm;
  float ultraRearCm;
  float tofFrontCm;
};

struct MotionCommand {
  int leftSpeed;
  int rightSpeed;
  const char* reason;
};

enum RobotState {
  STATE_IDLE,
  STATE_REMOTE,
  STATE_AVOID,
  STATE_AUTO,
  STATE_EMERGENCY
};

enum RemoteAction {
  REMOTE_STOP,
  REMOTE_FORWARD,
  REMOTE_BACKWARD,
  REMOTE_LEFT,
  REMOTE_RIGHT
};

// ============================================================
// 9. 全局数据（在主 .ino 中定义）
// ============================================================
extern TargetData latestTarget;
extern DistanceData latestDistance;
extern RobotState currentState;
extern bool hasReceivedTarget;
extern bool manualModeActive;
extern RemoteAction remoteAction;
extern int remoteSpeed;
extern unsigned long lastRemoteCommandTime;

// ============================================================
// 10. 模块接口声明
// ============================================================
// 遥控功能
void setupRemoteControl();
void readSerialCommunication();
MotionCommand computeRemoteCommand();
bool isRemoteCommandTimedOut(unsigned long now);

// 避障（传感器）
void setupObstacleSensors();
DistanceData makeInvalidDistanceData();
DistanceData readDistanceSensors();
bool isFrontBlocked(const DistanceData& d);
bool isLeftBlocked(const DistanceData& d);
bool isRightBlocked(const DistanceData& d);
bool isRearBlocked(const DistanceData& d);
bool obstacleOverrideRequired(const DistanceData& d);
MotionCommand computeObstacleCommand(const DistanceData& d);

// 自主跟随
void setupAutoFollow();
bool parseTargetMessage(char* line, TargetData& target);
MotionCommand computeAutoFollowCommand(const TargetData& target,
                                       const DistanceData& distance);

// 安全冗余
void setupSafetyRedundancy();
MotionCommand chooseSafeCommand(unsigned long now);
MotionCommand makeStopCommand(const char* reason);
void applySafeMotionCommand(const MotionCommand& cmd);
void stopCar();

#endif
