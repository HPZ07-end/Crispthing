#include "config.h"

namespace {

uint8_t farTargetFrameCount = 0;

// 记录上一条参与确认的TARGET序号，避免同一帧重复计数
unsigned long lastCountedTargetSequence = 0;
bool hasCountedTargetSequence = false;

// 记录当前是否正在执行安全距离内的原地对准
bool aligningInPlace = false;

// 当前原地对准的方向：1 表示向右，-1 表示向左，0 表示未对准
int8_t activeAlignDirection = 0;

// 原地对准启动确认：必须由不同序号、同一方向的 TARGET 连续确认
uint8_t alignConfirmFrameCount = 0;
int8_t pendingAlignDirection = 0;
unsigned long lastCountedAlignSequence = 0;
bool hasCountedAlignSequence = false;

// 根据目标水平偏差计算转向速度
int computeTurnSpeed(float xError) {
  xError = constrain(xError, -1.0f, 1.0f);

  const float absoluteError = fabs(xError);

  /*
   * 目标处于画面中央死区内时，不进行转向，
   * 避免关键点轻微抖动导致左右履带频繁调整。
   */
  if (absoluteError <= TARGET_CENTER_X_THRESHOLD) {
    return 0;
  }

  /*
   * 去掉死区后重新映射到 0～1。
   *
   * 这样刚超过死区时，转向速度会从0平滑增加，
   * 不会在0.10附近突然产生较大的转向量。
   */
  const float normalizedError =
      (absoluteError - TARGET_CENTER_X_THRESHOLD)
      /
      (1.0f - TARGET_CENTER_X_THRESHOLD);

  const float signedError =
      (xError > 0.0f)
          ? normalizedError
          : -normalizedError;

  const int turnSpeed =
      (int)(
          K_TURN
          * signedError
          * TURN_SIGN);

  return turnSpeed;
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

void resetFarTargetConfirmationState() {
  farTargetFrameCount = 0;
  lastCountedTargetSequence = 0;
  hasCountedTargetSequence = false;
}

void resetAlignConfirmationState() {
  alignConfirmFrameCount = 0;
  pendingAlignDirection = 0;
  lastCountedAlignSequence = 0;
  hasCountedAlignSequence = false;
}

void resetAlignmentState() {
  aligningInPlace = false;
  activeAlignDirection = 0;
  resetAlignConfirmationState();
}

}  // namespace

void resetAutoFollowConfirmation() {
  resetFarTargetConfirmationState();
  resetAlignmentState();
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

  static bool hasPreviousTargetRx = false;
  static unsigned long previousTargetRxMs = 0;

  Serial.print(F(", rx_ms="));
  Serial.print(target.receivedAt);

  Serial.print(F(", rx_gap_ms="));

  if (hasPreviousTargetRx) {
    Serial.print(target.receivedAt - previousTargetRxMs);
  } else {
    Serial.print(F("NA"));
  }

  previousTargetRxMs = target.receivedAt;
  hasPreviousTargetRx = true;

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


  /*
  * 原地对准的距离滞回：
  *
  * 尚未开始原地对准时，只有距离 <= 1.12 才允许进入；
  * 已经开始原地对准后，只要距离仍 < 1.16，就继续对准；
  * 距离达到 1.16 后，退出原地对准。
  */
  const bool targetInsideStopDistance =
      target.relativeDistance <=
          FOLLOW_STOP_RELATIVE_DISTANCE;

  const bool keepCurrentAlignment =
      aligningInPlace &&
      target.relativeDistance <
          FOLLOW_RESTART_RELATIVE_DISTANCE;

  if (targetInsideStopDistance ||
      keepCurrentAlignment) {
    
    // 安全距离内只清除远目标确认，不清除正在进行的原地对准状态。
    resetFarTargetConfirmationState();

    const float absoluteXError = 
        fabs(target.xError);

        /*
     * 当前尚未原地转动：
     * 只有偏差达到启动阈值 0.15，才开始转动。
     */
    if (!aligningInPlace) {

      if (absoluteXError <
          ALIGN_START_X_THRESHOLD) {

        resetAlignConfirmationState();

        return makeStopCommand(
            "target centered in safe distance");
      }

      const int8_t requestedDirection =
          target.xError > 0.0f ? 1 : -1;

      const bool isNewAlignFrame =
          !hasCountedAlignSequence ||
          target.sequence != lastCountedAlignSequence;

      if (isNewAlignFrame) {
        lastCountedAlignSequence = target.sequence;
        hasCountedAlignSequence = true;

        if (requestedDirection == pendingAlignDirection) {
          if (alignConfirmFrameCount < ALIGN_CONFIRM_FRAMES) {
            alignConfirmFrameCount++;
          }
        } else {
          // 方向改变后，上一方向的确认作废，并从当前帧重新计数。
          pendingAlignDirection = requestedDirection;
          alignConfirmFrameCount = 1;
        }

      #if DEBUG_PRINT
        Serial.print(F("Alignment confirmation: "));
        Serial.print(alignConfirmFrameCount);
        Serial.print(F("/"));
        Serial.print(ALIGN_CONFIRM_FRAMES);
        Serial.print(F(", direction="));
        Serial.println(pendingAlignDirection > 0 ? F("right") : F("left"));
      #endif
      }

      if (alignConfirmFrameCount < ALIGN_CONFIRM_FRAMES) {
        return makeStopCommand(
            "confirming in-place alignment");
      }

      aligningInPlace = true;

      // 保存刚刚经过连续两帧确认的启动方向
      activeAlignDirection = pendingAlignDirection;

      resetAlignConfirmationState();
    }

    /*
     * 当前已经在原地转动：
     * 只有偏差减小到停止阈值 0.08，才停止转动。
     */
    if (aligningInPlace &&
        absoluteXError <=
            ALIGN_STOP_X_THRESHOLD) {

        resetAlignmentState();

        return makeStopCommand(
            "in-place alignment completed");
    }

    /*
    * 原地对准过程中的换向确认：
    *
    * 如果目标突然出现在当前转向方向的另一侧，
    * 先立即停车，并等待连续两条不同序号的 TARGET
    * 都确认方向已经改变，才真正反向转动。
    */
    const int8_t requestedDirection =
        target.xError > 0.0f ? 1 : -1;

    if (requestedDirection !=
        activeAlignDirection) {

      const bool isNewReverseFrame =
          !hasCountedAlignSequence ||
          target.sequence != lastCountedAlignSequence;

      if (isNewReverseFrame) {
        lastCountedAlignSequence =
            target.sequence;

        hasCountedAlignSequence = true;

        if (requestedDirection ==
            pendingAlignDirection) {

          if (alignConfirmFrameCount <
              ALIGN_CONFIRM_FRAMES) {
            alignConfirmFrameCount++;
          }
        }
        else {
          pendingAlignDirection =
              requestedDirection;

          alignConfirmFrameCount = 1;
        }

    #if DEBUG_PRINT
        Serial.print(
            F("Alignment reversal confirmation: "));

        Serial.print(alignConfirmFrameCount);
        Serial.print(F("/"));
        Serial.print(ALIGN_CONFIRM_FRAMES);

        Serial.print(F(", direction="));

        Serial.println(
            pendingAlignDirection > 0
                ? F("right")
                : F("left"));
    #endif
      }

      if (alignConfirmFrameCount <
          ALIGN_CONFIRM_FRAMES) {

        return makeStopCommand(
            "confirming alignment reversal");
      }

      // 连续两帧方向一致，正式改变原地转向方向
      activeAlignDirection =
          requestedDirection;

      resetAlignConfirmationState();
    }
    else {
      /*
      * 当前帧仍在原来的转向方向，
      * 之前可能存在的换向确认作废。
      */
      resetAlignConfirmationState();
    }

    /*
    * 使用经过确认的方向计算转向量。
    * absoluteXError 仍然决定转向强度，
    * activeAlignDirection 决定转向方向。
    */
    const float confirmedXError =
        absoluteXError *
        activeAlignDirection;

    int turnSpeed =
        computeTurnSpeed(confirmedXError);

    /*
     * computeTurnSpeed 使用 0.10 的普通转弯死区。
     * 原地对准停止阈值是 0.08，因此在 0.08～0.10 内，
     * 如果仍处于对准状态，就按最小 PWM 继续转动。
     */
    if (turnSpeed == 0) {

        turnSpeed =
          (activeAlignDirection > 0)
              ? MIN_ALIGN_TURN_SPEED
              : -MIN_ALIGN_TURN_SPEED;

        turnSpeed *= TURN_SIGN;
    }

    if (turnSpeed > 0) {
      turnSpeed = constrain(
        turnSpeed,
        MIN_ALIGN_TURN_SPEED,
        MAX_ALIGN_TURN_SPEED);
    }
    else {
      turnSpeed = constrain(
        turnSpeed,
        -MAX_ALIGN_TURN_SPEED,
        -MIN_ALIGN_TURN_SPEED);
    }

    // 左侧有障碍，不允许向左原地转动
    if (activeAlignDirection < 0 &&
        isLeftBlocked(distance)) {
          return makeStopCommand(
              "left blocked while aligned");
        }

    // 右侧有障碍，不允许向右原地转动
    if (activeAlignDirection > 0 &&
        isRightBlocked(distance)) {
          return makeStopCommand(
              "right blocked while aligned");
        }    

    MotionCommand cmd;

    // 一侧正转，另一侧反转，实现原地转弯
    cmd.leftSpeed = turnSpeed;
    cmd.rightSpeed = -turnSpeed;
    cmd.reason = "aligned in place";

    return cmd;
  }

  resetAlignmentState();
  /*
   * 距离滞回：
   *
   * 尚未完成两帧确认时，只有目标达到重新启动阈值，
   * 才允许累计确认帧。停止阈值与重新启动阈值之间的
   * 1.12～1.16 区域保持停车，避免阈值附近反复启停。
   *
   * 一旦已经完成确认并开始运行，本判断不再阻止运动；
   * 运行中仍持续检查每条 TARGET，并在距离 <= 1.12 时停车。
   */
  const bool startAlreadyConfirmed =
      farTargetFrameCount >=
      FAR_TARGET_CONFIRM_FRAMES;

  if (!startAlreadyConfirmed &&
      target.relativeDistance <
          FOLLOW_RESTART_RELATIVE_DISTANCE) {

    resetAutoFollowConfirmation();

    return makeStopCommand(
        "waiting for restart distance");
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
