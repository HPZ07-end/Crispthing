#include "config.h"

namespace {
bool isEmergencyStopActive() {
#if EMERGENCY_STOP_ENABLED
  return digitalRead(EMERGENCY_STOP_PIN) == HIGH;
#else
  return false;
#endif
}

bool isTargetTimedOut(unsigned long now) {
  return !hasReceivedTarget || now - latestTarget.receivedAt > TARGET_TIMEOUT_MS;
}

void setMotor(int pwmPin, int in1Pin, int in2Pin, int speed) {
#if MOTOR_ENABLED
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);
  }
#else
  (void)pwmPin;
  (void)in1Pin;
  (void)in2Pin;
  (void)speed;
#endif
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
#if MOTOR_ENABLED
  setMotor(LEFT_PWM, LEFT_IN1, LEFT_IN2, leftSpeed * LEFT_TRACK_DIR);
  setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, rightSpeed * RIGHT_TRACK_DIR);
#else
  (void)leftSpeed;
  (void)rightSpeed;
#endif
}

void printRobotState(RobotState state) {
#if DEBUG_PRINT
  if (state == STATE_IDLE) Serial.print("IDLE");
  else if (state == STATE_REMOTE) Serial.print("REMOTE");
  else if (state == STATE_AVOID) Serial.print("AVOID");
  else if (state == STATE_AUTO) Serial.print("AUTO");
  else if (state == STATE_EMERGENCY) Serial.print("EMERGENCY");
  else Serial.print("UNKNOWN");
#else
  (void)state;
#endif
}
}  // namespace

void setupSafetyRedundancy() {
#if EMERGENCY_STOP_ENABLED
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
#endif

#if MOTOR_ENABLED
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  // 上电后第一时间关闭全部电机输出。
  setMotorSpeed(0, 0);
#endif
}

MotionCommand makeStopCommand(const char* reason) {
  MotionCommand cmd;
  cmd.leftSpeed = 0;
  cmd.rightSpeed = 0;
  cmd.reason = reason;
  return cmd;
}

MotionCommand chooseSafeCommand(unsigned long now) {
  // 优先级 1：硬件急停
  if (isEmergencyStopActive()) {
    currentState = STATE_EMERGENCY;
    return makeStopCommand("emergency stop");
  }

  // 优先级 2：遥控模式，但遥控命令也有看门狗超时。
  if (manualModeActive) {
    if (isRemoteCommandTimedOut(now)) {
      currentState = STATE_IDLE;
      return makeStopCommand("remote command timeout");
    }

    MotionCommand remote = computeRemoteCommand();

    // 遥控同样不能绕过本地硬安全约束。
    if (remote.leftSpeed > 0 && remote.rightSpeed > 0 &&
        isFrontBlocked(latestDistance)) {
      currentState = STATE_AVOID;
      return makeStopCommand("front blocked in remote mode");
    }
    if (remote.leftSpeed < 0 && remote.rightSpeed < 0 &&
        isRearBlocked(latestDistance)) {
      currentState = STATE_AVOID;
      return makeStopCommand("rear blocked in remote mode");
    }

    currentState = STATE_REMOTE;
    return remote;
  }

  // 优先级 3：自主模式中的本地避障覆盖。
  if (obstacleOverrideRequired(latestDistance)) {
    currentState = STATE_AVOID;
    return computeObstacleCommand(latestDistance);
  }

  // 优先级 4：目标数据有效性与超时看门狗。
  if (isTargetTimedOut(now)) {
    currentState = STATE_IDLE;
    return makeStopCommand(hasReceivedTarget ? "target timeout" : "waiting for target");
  }

  // 优先级 5：正常自主跟随。
  currentState = STATE_AUTO;
  return computeAutoFollowCommand(latestTarget, latestDistance);
}

void applySafeMotionCommand(
    const MotionCommand& cmd) {

  const int targetLeft =
    constrain(
        cmd.leftSpeed,
        -MOTOR_PWM_LIMIT,
        MOTOR_PWM_LIMIT);

  const int targetRight =
      constrain(
          cmd.rightSpeed,
          -MOTOR_PWM_LIMIT,
          MOTOR_PWM_LIMIT);


  /*
   * 只要控制命令要求左右履带都停车，
   * 就立即归零，不使用减速斜坡。
   *
   * 因此以下情况都能立即停车：
   * - 目标丢失
   * - 距离数据无效
   * - 到达跟随距离
   * - TARGET 超时
   * - MANUAL / STOP / ESTOP
   * - 硬件急停
   */
  if (targetLeft == 0 &&
      targetRight == 0) {

    appliedLeftSpeed = 0;
    appliedRightSpeed = 0;
  }
  else {

#if MOTOR_RAMP_ENABLED

    /*
     * 如果任意一侧履带准备反向：
     *
     * 不让 appliedSpeed 通过斜坡慢慢跨过零点，
     * 而是先让左右履带同时立即停车。
     *
     * 下一控制周期如果反向命令仍然存在，
     * 再从 0 开始向新的方向逐步加速。
     */
    const bool leftDirectionReversing =
        (appliedLeftSpeed > 0 && targetLeft < 0) ||
        (appliedLeftSpeed < 0 && targetLeft > 0);

    const bool rightDirectionReversing =
        (appliedRightSpeed > 0 && targetRight < 0) ||
        (appliedRightSpeed < 0 && targetRight > 0);

    if (leftDirectionReversing ||
        rightDirectionReversing) {

      appliedLeftSpeed = 0;
      appliedRightSpeed = 0;
    }
    else {
      appliedLeftSpeed =
          approachSpeed(
              appliedLeftSpeed,
              targetLeft,
              MOTOR_RAMP_STEP);

      appliedRightSpeed =
          approachSpeed(
              appliedRightSpeed,
              targetRight,
              MOTOR_RAMP_STEP);
    }

#else
    appliedLeftSpeed = targetLeft;
    appliedRightSpeed = targetRight;
#endif
  }


#if DEBUG_PRINT
  // 只在状态或速度变化时打印，避免串口输出阻塞控制循环。
  static int lastPrintedLeft = 32767;
  static int lastPrintedRight = 32767;
  static int lastPrintedState = -1;

  if (safeLeft != lastPrintedLeft ||
      safeRight != lastPrintedRight ||
      (int)currentState != lastPrintedState) {

    Serial.print(F("State="));
    printRobotState(currentState);
    Serial.print(F(", reason="));
    Serial.print(cmd.reason);
    Serial.print(F(", L="));
    Serial.print(safeLeft);
    Serial.print(F(", R="));
    Serial.println(safeRight);

    lastPrintedLeft = safeLeft;
    lastPrintedRight = safeRight;
    lastPrintedState = (int)currentState;
  }
#endif

  setMotorSpeed(safeLeft, safeRight);
}

void stopCar() {
  setMotorSpeed(0, 0);
}