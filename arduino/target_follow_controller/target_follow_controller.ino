#include "config.h"

// ============================================================
// 全局运行数据
// ============================================================
TargetData latestTarget = {false, 0.0f, 0.0f, 0.0f, 0};
DistanceData latestDistance;
RobotState currentState = STATE_IDLE;

bool hasReceivedTarget = false;
bool manualModeActive = false;
RemoteAction remoteAction = REMOTE_STOP;
int remoteSpeed = 80;
unsigned long lastRemoteCommandTime = 0;

unsigned long lastControlTime = 0;

void setup() {
  Serial.begin(9600);

  setupRemoteControl();
  setupObstacleSensors();
  setupAutoFollow();
  setupSafetyRedundancy();

  latestDistance = makeInvalidDistanceData();
  stopCar();

  Serial.println("Target follow controller ready.");
  Serial.println("AUTO: TARGET,visible,x,size,quality");
  Serial.println("REMOTE: f / b / l / r / s, a returns to auto");
}

void loop() {
  // 串口同时接收自主跟随 TARGET 数据与遥控指令
  readSerialCommunication();

  const unsigned long now = millis();
  if (now - lastControlTime < CONTROL_INTERVAL_MS) {
    return;
  }
  lastControlTime = now;

  // 读取传感器
  latestDistance = readDistanceSensors();

  // 安全冗余模块按优先级选择最终命令
  const MotionCommand command = chooseSafeCommand(now);

  // 所有电机输出都必须经过安全输出接口
  applySafeMotionCommand(command);
}
