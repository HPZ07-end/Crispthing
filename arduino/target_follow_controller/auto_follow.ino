#include "config.h"

namespace {
float absoluteFloat(float value) {
  return value < 0.0f ? -value : value;
}

int computeTurnSpeed(float x) {
  x = constrain(x, -1.0f, 1.0f);
  return (int)(K_TURN * x * TURN_SIGN);
}

int computeForwardSpeedBySize(float size) {
  if (size <= SIZE_FAR) {
    return MAX_SPEED;
  }

  const float ratio = constrain(
      (SIZE_NEAR - size) / (SIZE_NEAR - SIZE_FAR), 0.0f, 1.0f);
  const int speed = MIN_SPEED + (int)(ratio * (MAX_SPEED - MIN_SPEED));
  return constrain(speed, MIN_SPEED, MAX_SPEED);
}

int computeForwardSpeed(const TargetData& target,
                        const DistanceData& distance) {
  const bool targetNearCenter =
      absoluteFloat(target.x) < TARGET_CENTER_X_THRESHOLD;

  if (targetNearCenter && distance.tofFrontCm > 0.0f) {
    if (distance.tofFrontCm < TOF_TOO_CLOSE_CM) {
      return 0;
    }
    if (distance.tofFrontCm > TOF_FAR_CM) {
      return MAX_SPEED;
    }

    const float ratio = constrain(
        (distance.tofFrontCm - TOF_TOO_CLOSE_CM) /
            (TOF_FAR_CM - TOF_TOO_CLOSE_CM),
        0.0f, 1.0f);
    const int speed = MIN_SPEED + (int)(ratio * (MAX_SPEED - MIN_SPEED));
    return constrain(speed, MIN_SPEED, MAX_SPEED);
  }

  return computeForwardSpeedBySize(target.size);
}
}  // namespace

void setupAutoFollow() {
  // 当前自主跟随模块无需额外硬件初始化。
}

bool parseTargetMessage(char* line, TargetData& target) {
  char* token = strtok(line, ",");
  if (token == NULL || strcmp(token, "TARGET") != 0) return false;

  token = strtok(NULL, ",");
  if (token == NULL) return false;
  target.visible = atoi(token) == 1;

  token = strtok(NULL, ",");
  if (token == NULL) return false;
  target.x = atof(token);

  token = strtok(NULL, ",");
  if (token == NULL) return false;
  target.size = atof(token);

  token = strtok(NULL, ",");
  if (token == NULL) return false;
  target.quality = atof(token);

  // 不允许多余字段。
  if (strtok(NULL, ",") != NULL) return false;

  target.receivedAt = millis();

#if DEBUG_PRINT
  Serial.print("TARGET: visible=");
  Serial.print(target.visible);
  Serial.print(", x=");
  Serial.print(target.x);
  Serial.print(", size=");
  Serial.print(target.size);
  Serial.print(", quality=");
  Serial.println(target.quality);
#endif

  return true;
}

MotionCommand computeAutoFollowCommand(const TargetData& target,
                                       const DistanceData& distance) {
  if (!target.visible) {
    return makeStopCommand("target not visible");
  }
  if (target.quality < QUALITY_MIN) {
    return makeStopCommand("low target quality");
  }
  if (target.size >= SIZE_NEAR) {
    return makeStopCommand("target too close by image size");
  }

  const int forwardSpeed = computeForwardSpeed(target, distance);
  int turnSpeed = computeTurnSpeed(target.x);

  // 对应方向有近障碍时，禁止继续向该侧修正。
  if (target.x < 0.0f && isLeftBlocked(distance)) {
    turnSpeed = 0;
  }
  if (target.x > 0.0f && isRightBlocked(distance)) {
    turnSpeed = 0;
  }

  MotionCommand cmd;
  cmd.leftSpeed = constrain(forwardSpeed + turnSpeed, -MAX_SPEED, MAX_SPEED);
  cmd.rightSpeed = constrain(forwardSpeed - turnSpeed, -MAX_SPEED, MAX_SPEED);
  cmd.reason = "auto follow";
  return cmd;
}
