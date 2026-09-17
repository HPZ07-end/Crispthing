#include "config.h"

namespace {

float cachedFrontDistanceCm = -1.0f;
unsigned long lastMeasurementTime = 0;
bool hasMeasuredFrontDistance = false;
bool frontBlocked = false;

float readFrontDistanceCm() {
#if SENSOR_ENABLED
  digitalWrite(ULTRA_FRONT_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_FRONT_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_FRONT_TRIG, LOW);

  const unsigned long duration =
      pulseIn(
          ULTRA_FRONT_ECHO,
          HIGH,
          ULTRA_ECHO_TIMEOUT_US);

  if (duration == 0) {
    return -1.0f;
  }

  return duration * 0.0343f / 2.0f;
#else
  return -1.0f;
#endif
}

void printFrontDistance(unsigned long now) {
#if DEBUG_PRINT
  static unsigned long lastPrintTime = 0;

  if (now - lastPrintTime < 250) {
    return;
  }
  lastPrintTime = now;

  Serial.print(F("Front distance: "));

  if (cachedFrontDistanceCm > 0.0f) {
    Serial.print(cachedFrontDistanceCm, 1);
    Serial.println(F(" cm"));
  } else {
    Serial.println(F("no valid echo"));
  }
#else
  (void)now;
#endif
}

}  // namespace

void setupObstacleSensors() {
#if SENSOR_ENABLED
  // 当前由 config.h 配置为 TRIG=A5、ECHO=A4。
  pinMode(ULTRA_FRONT_TRIG, OUTPUT);
  pinMode(ULTRA_FRONT_ECHO, INPUT);

  // 保证上电后 TRIG 默认保持低电平，不产生误触发脉冲。
  digitalWrite(ULTRA_FRONT_TRIG, LOW);
#endif
}

DistanceData makeInvalidDistanceData() {
  DistanceData d;
  d.ultraFrontCm = -1.0f;
  return d;
}

DistanceData readDistanceSensors() {
  DistanceData d = makeInvalidDistanceData();

#if SENSOR_ENABLED
  const unsigned long now = millis();

  if (!hasMeasuredFrontDistance ||
      now - lastMeasurementTime >= ULTRA_MEASURE_INTERVAL_MS) {
    cachedFrontDistanceCm = readFrontDistanceCm();
    lastMeasurementTime = now;
    hasMeasuredFrontDistance = true;

    printFrontDistance(now);
  }

  d.ultraFrontCm = cachedFrontDistanceCm;
#endif

  return d;
}

bool isFrontBlocked(const DistanceData& d) {
  // 无有效回波时保持原状态：
  // 尚未停车则不因远距离无回波误停；已经停车则不会因一次丢帧误恢复。
  if (d.ultraFrontCm <= 0.0f) {
    return frontBlocked;
  }

  if (!frontBlocked &&
      d.ultraFrontCm <= ULTRA_FRONT_STOP_CM) {
    frontBlocked = true;
  }
  else if (frontBlocked &&
           d.ultraFrontCm >= ULTRA_FRONT_RELEASE_CM) {
    frontBlocked = false;
  }

  return frontBlocked;
}

bool isLeftBlocked(const DistanceData& d) {
  (void)d;
  return false;
}

bool isRightBlocked(const DistanceData& d) {
  (void)d;
  return false;
}

bool isRearBlocked(const DistanceData& d) {
  (void)d;
  return false;
}

bool obstacleOverrideRequired(const DistanceData& d) {
  // V1 先把“正前方危险”作为强制避障条件。
  return isFrontBlocked(d);
}

MotionCommand computeObstacleCommand(const DistanceData& d) {
  (void)d;

  // 基础策略：检测到正前方障碍后立即停车，不执行绕行。
  return makeStopCommand("front obstacle stop");
}
