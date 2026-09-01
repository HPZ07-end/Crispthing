#include "config.h"

namespace {

// 当前真正施加到左右履带的速度
int appliedLeftSpeed = 0;
int appliedRightSpeed = 0;


// 让 current 每次最多向 target 靠近 step
int approachSpeed(
    int current,
    int target,
    int step) {

  if (current < target) {
    current += step;

    if (current > target) {
      current = target;
    }
  }
  else if (current > target) {
    current -= step;

    if (current < target) {
      current = target;
    }
  }

  return current;
}

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

// 无论是否允许电机运动，都可以调用该函数将驱动输入锁定为安全电平。
void forceMotorOutputsOff() {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);

  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
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
  (void)speed;

  // 禁止运动命令时仍主动保持该电机通道关闭。
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  analogWrite(pwmPin, 0);
#endif
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
#if MOTOR_ENABLED
  setMotor(LEFT_PWM, LEFT_IN1, LEFT_IN2, leftSpeed * LEFT_TRACK_DIR);
  setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, rightSpeed * RIGHT_TRACK_DIR);
#else
  (void)leftSpeed;
  (void)rightSpeed;

  // MOTOR_ENABLED=0 不是“不管引脚”，而是主动安全关闭。
  forceMotorOutputsOff();
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

  /*
   * 无论 MOTOR_ENABLED 是 0 还是 1，都必须主动控制电机引脚。
   * 先将输出锁存器预置为 LOW，再切换为 OUTPUT，减少切换瞬间毛刺，
   * 避免驱动板重新上电时 PWM/方向输入悬空。
   */
  digitalWrite(LEFT_PWM, LOW);
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_PWM, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  forceMotorOutputsOff();
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

    // 优先级 2：手机端软件急停锁定
  if (softwareEmergencyActive) {
    currentState = STATE_EMERGENCY;

    return makeStopCommand(
        "software emergency stop");
  }

  // 优先级 3：手机端普通停车锁定
  if (commandStopActive) {
    currentState = STATE_IDLE;

    return makeStopCommand(
        "command stop");
  }

  // 遥控模式，但遥控命令也有看门狗超时。
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
    resetAutoFollowConfirmation();

    currentState = STATE_IDLE;

    return makeStopCommand(
        hasReceivedTarget
            ? "target timeout"
            : "waiting for target");
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
          -MAX_SPEED,
          MAX_SPEED);

  const int targetRight =
      constrain(
          cmd.rightSpeed,
          -MAX_SPEED,
          MAX_SPEED);


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
#else
    appliedLeftSpeed = targetLeft;
    appliedRightSpeed = targetRight;
#endif
  }


#if DEBUG_PRINT
  // 打印实际施加的速度，而不是尚未达到的目标速度
  static int lastPrintedLeft = 32767;
  static int lastPrintedRight = 32767;
  static int lastPrintedState = -1;
  static const char* lastPrintedReason = NULL;

  const bool reasonChanged =
      lastPrintedReason == NULL ||
      strcmp(cmd.reason, lastPrintedReason) != 0;

  if (appliedLeftSpeed != lastPrintedLeft ||
      appliedRightSpeed != lastPrintedRight ||
      (int)currentState != lastPrintedState ||
      reasonChanged) {

    Serial.print(F("t_ms="));
    Serial.print(millis());
    Serial.print(F(", State="));
    printRobotState(currentState);

    Serial.print(F(", reason="));
    Serial.print(cmd.reason);

    Serial.print(F(", target_age_ms="));

    if (hasReceivedTarget) {
      const unsigned long targetAgeMs =
          millis() - latestTarget.receivedAt;

      Serial.print(targetAgeMs);
    } else {
      Serial.print(F("NA"));
    }

    Serial.print(F(", targetL="));
    Serial.print(targetLeft);

    Serial.print(F(", targetR="));
    Serial.print(targetRight);

    Serial.print(F(", L="));
    Serial.print(appliedLeftSpeed);

    Serial.print(F(", R="));
    Serial.println(appliedRightSpeed);

    lastPrintedLeft =
        appliedLeftSpeed;

    lastPrintedRight =
        appliedRightSpeed;

    lastPrintedState =
        (int)currentState;

    lastPrintedReason = cmd.reason;
  }
#endif


  setMotorSpeed(
      appliedLeftSpeed,
      appliedRightSpeed);
}

void stopCar() {
  appliedLeftSpeed = 0;
  appliedRightSpeed = 0;

  setMotorSpeed(0, 0);
}
