#include "config.h"

namespace {

uint8_t farTargetFrameCount = 0;

// 记录上一条参与确认的TARGET序号，避免同一帧重复计数
unsigned long lastCountedTargetSequence = 0;
bool hasCountedTargetSequence = false;

// 根据目标水平偏差计算转向速度
int computeTurnSpeed(float xError) {
  xError = constrain(xError, -1.0f, 1.0f);

  return (int)(
      K_TURN
      * xError
      * TURN_SIGN);
}


// 根据相对距离比例计算前进速度
int computeForwardSpeedByRelativeDistance(
    float relativeDistance) {

  // -1 或其他非正数都视为无效
  if (relativeDistance <= 0.0f) {
    return 0;
  }

  // 已经到达注册距离附近，停止继续向前
  if (relativeDistance <=
      FOLLOW_STOP_RELATIVE_DISTANCE) {
    return 0;
  }

  // 目标明显较远，使用最大速度
  if (relativeDistance >=
      FOLLOW_FULL_SPEED_RELATIVE_DISTANCE) {
    return MAX_SPEED;
  }

  /*
   * 在停止阈值与全速阈值之间线性调速：
   *
   * relativeDistance 越大
   * → 目标越远
   * → 前进速度越高
   */
  const float ratio =
      (relativeDistance
       - FOLLOW_STOP_RELATIVE_DISTANCE)
      /
      (FOLLOW_FULL_SPEED_RELATIVE_DISTANCE
       - FOLLOW_STOP_RELATIVE_DISTANCE);

  const int speed =
      MIN_SPEED
      + (int)(
          ratio
          * (MAX_SPEED - MIN_SPEED));

  return constrain(
      speed,
      MIN_SPEED,
      MAX_SPEED);
}

}  // namespace

void resetAutoFollowConfirmation() {
  farTargetFrameCount = 0;
  lastCountedTargetSequence = 0;
  hasCountedTargetSequence = false;
}

void setupAutoFollow() {
  // 当前自主跟随模块不需要额外硬件初始化
}


// 解析协议：
// TARGET,序号,目标有效,x偏差,相对距离比例,相似度
bool parseTargetMessage(
    char* line,
    TargetData& target) {

  char* token = strtok(line, ",");

  // 消息类型必须是 TARGET
  if (token == NULL ||
      strcmp(token, "TARGET") != 0) {
    return false;
  }


  // 1. 序号
  token = strtok(NULL, ",");

  if (token == NULL) {
    return false;
  }

  target.sequence =
      strtoul(token, NULL, 10);


  // 2. 目标是否有效
  token = strtok(NULL, ",");

  if (token == NULL) {
    return false;
  }

  const int validValue = atoi(token);

  if (validValue != 0 &&
      validValue != 1) {
    return false;
  }

  target.valid =
      (validValue == 1);


  // 3. 目标水平偏差
  token = strtok(NULL, ",");

  if (token == NULL) {
    return false;
  }

  target.xError = atof(token);


  // 4. 相对距离比例
  token = strtok(NULL, ",");

  if (token == NULL) {
    return false;
  }

  target.relativeDistance =
      atof(token);


  // 5. 身份相似度
  token = strtok(NULL, ",");

  if (token == NULL) {
    return false;
  }

  target.similarity =
      atof(token);


  // 不允许出现额外字段
  if (strtok(NULL, ",") != NULL) {
    return false;
  }


  // x偏差范围检查
  if (target.xError < -1.0f ||
      target.xError > 1.0f) {
    return false;
  }


  // 相似度范围检查
  if (target.similarity < 0.0f ||
      target.similarity > 1.0f) {
    return false;
  }


  /*
   * relativeDistance：
   * -1   表示无效
   * > 0  表示有效
   *
   * 0 或其他负数不符合协议。
   */
  if (target.relativeDistance != -1.0f &&
      target.relativeDistance <= 0.0f) {
    return false;
  }


  target.receivedAt = millis();


#if DEBUG_PRINT
  Serial.print(F("TARGET: seq="));
  Serial.print(target.sequence);

  Serial.print(F(", valid="));
  Serial.print(target.valid);

  Serial.print(F(", xError="));
  Serial.print(target.xError, 3);

  Serial.print(F(", relativeDistance="));
  Serial.print(target.relativeDistance, 3);

  Serial.print(F(", similarity="));
  Serial.println(target.similarity, 3);
#endif

  return true;
}


MotionCommand computeAutoFollowCommand(
    const TargetData& target,
    const DistanceData& distance) {

  // 没有检测到有效目标
  if (!target.valid) {
    resetAutoFollowConfirmation();

    return makeStopCommand(
        "target invalid");
  }


  // 当前人员与注册人员相似度不足
  if (target.similarity < SIMILARITY_MIN) {
    resetAutoFollowConfirmation();

    return makeStopCommand(
        "low target similarity");
  }


  // 相对距离数据无效
  if (target.relativeDistance <= 0.0f) {
    resetAutoFollowConfirmation();

    return makeStopCommand(
        "invalid relative distance");
  }


  // 人已到达注册距离附近或比注册位置更近
  if (target.relativeDistance <=
      FOLLOW_STOP_RELATIVE_DISTANCE) {

    resetAutoFollowConfirmation();

    return makeStopCommand(
        "target distance reached");
  }

  /*
  * 主循环约50 Hz，而手机TARGET约5 Hz。
  * 只有收到不同sequence的新TARGET时才计数，
  * 防止同一条TARGET在多个控制周期内重复累计。
  */
  const bool isNewTargetFrame =
      !hasCountedTargetSequence ||
      target.sequence != lastCountedTargetSequence;

  if (isNewTargetFrame) {
    lastCountedTargetSequence = target.sequence;
    hasCountedTargetSequence = true;

    if (farTargetFrameCount <
        FAR_TARGET_CONFIRM_FRAMES) {

      farTargetFrameCount++;
    }

  #if DEBUG_PRINT
    Serial.print(F("Far target confirmation: "));
    Serial.print(farTargetFrameCount);
    Serial.print(F("/"));
    Serial.println(FAR_TARGET_CONFIRM_FRAMES);
  #endif
  }

  if (farTargetFrameCount <
      FAR_TARGET_CONFIRM_FRAMES) {

    return makeStopCommand(
        "confirming far target");
  }

  // 根据相对距离计算前进速度
  const int forwardSpeed =
      computeForwardSpeedByRelativeDistance(
          target.relativeDistance);


  // 根据水平偏差计算转向速度
  int turnSpeed =
      computeTurnSpeed(target.xError);


  // 左侧存在障碍时，禁止继续向左修正
  if (target.xError < 0.0f &&
      isLeftBlocked(distance)) {
    turnSpeed = 0;
  }


  // 右侧存在障碍时，禁止继续向右修正
  if (target.xError > 0.0f &&
      isRightBlocked(distance)) {
    turnSpeed = 0;
  }


  MotionCommand cmd;

  cmd.leftSpeed = constrain(
      forwardSpeed + turnSpeed,
      -MAX_SPEED,
      MAX_SPEED);

  cmd.rightSpeed = constrain(
      forwardSpeed - turnSpeed,
      -MAX_SPEED,
      MAX_SPEED);

  cmd.reason = "auto follow";

  return cmd;
}