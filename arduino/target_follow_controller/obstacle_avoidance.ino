#include "config.h"

namespace {
enum UltrasonicStatus {
  ULTRA_VALID,
  ULTRA_TIMEOUT,
  ULTRA_BELOW_MIN,
  ULTRA_ABOVE_MAX
};

struct UltrasonicReading {
  unsigned long echoUs;
  float distanceCm;
  UltrasonicStatus status;
};

bool frontBlockedLatched = false;
uint8_t frontNearCount = 0;
uint8_t frontClearCount = 0;
unsigned long lastFrontStateSampleAt = 0;

UltrasonicReading readJSNSR04(int trigPin, int echoPin) {
  UltrasonicReading reading;
  reading.echoUs = 0;
  reading.distanceCm = -1.0f;
  reading.status = ULTRA_TIMEOUT;

#if SENSOR_ENABLED
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  reading.echoUs =
      pulseIn(echoPin, HIGH, ULTRA_ECHO_TIMEOUT_US);

  if (reading.echoUs == 0) {
    return reading;
  }

  reading.distanceCm =
      reading.echoUs * 0.0343f / 2.0f;

  if (reading.distanceCm < ULTRA_MIN_RELIABLE_CM) {
    reading.status = ULTRA_BELOW_MIN;
  } else if (reading.distanceCm > ULTRA_MAX_RELIABLE_CM) {
    reading.status = ULTRA_ABOVE_MAX;
    reading.distanceCm = -1.0f;
  } else {
    reading.status = ULTRA_VALID;
  }
#else
  (void)trigPin;
  (void)echoPin;
#endif

  return reading;
}

void updateFrontBlockState(const DistanceData& d) {
  // 同一采样值可能被安全逻辑读取多次，但只允许累计一次。
  if (d.sampledAt == 0 ||
      d.sampledAt == lastFrontStateSampleAt) {
    return;
  }

  lastFrontStateSampleAt = d.sampledAt;

  // 超时/超量程不用于解除已有停车状态。
  if (d.ultraFrontCm <= 0.0f) {
    frontNearCount = 0;
    frontClearCount = 0;
    return;
  }

  if (!frontBlockedLatched) {
    frontClearCount = 0;

    if (d.ultraFrontCm <= ULTRA_FRONT_SAFE_CM) {
      if (frontNearCount < ULTRA_BLOCK_CONFIRM_SAMPLES) {
        frontNearCount++;
      }

      if (frontNearCount >= ULTRA_BLOCK_CONFIRM_SAMPLES) {
        frontBlockedLatched = true;
        frontNearCount = 0;
      }
    } else {
      frontNearCount = 0;
    }
  } else {
    frontNearCount = 0;

    if (d.ultraFrontCm >= ULTRA_FRONT_RELEASE_CM) {
      if (frontClearCount < ULTRA_RELEASE_CONFIRM_SAMPLES) {
        frontClearCount++;
      }

      if (frontClearCount >= ULTRA_RELEASE_CONFIRM_SAMPLES) {
        frontBlockedLatched = false;
        frontClearCount = 0;
      }
    } else {
      frontClearCount = 0;
    }
  }
}

const __FlashStringHelper* ultrasonicStatusText(
    UltrasonicStatus status) {
  if (status == ULTRA_VALID) return F("VALID");
  if (status == ULTRA_BELOW_MIN) return F("BELOW_MIN");
  if (status == ULTRA_ABOVE_MAX) return F("ABOVE_MAX");
  return F("TIMEOUT");
}

void printFrontReading(
    const UltrasonicReading& reading,
    unsigned long sampledAt) {
#if DEBUG_PRINT && ULTRASONIC_TEST_PRINT_ENABLED
  Serial.print(F("ULTRA_FRONT,t_ms="));
  Serial.print(sampledAt);
  Serial.print(F(",echo_us="));
  Serial.print(reading.echoUs);
  Serial.print(F(",distance_cm="));

  if (reading.distanceCm > 0.0f) {
    Serial.print(reading.distanceCm, 1);
  } else {
    Serial.print(F("NA"));
  }

  Serial.print(F(",status="));
  Serial.print(ultrasonicStatusText(reading.status));
  Serial.print(F(",blocked="));
  Serial.println(frontBlockedLatched ? 1 : 0);
#else
  (void)reading;
  (void)sampledAt;
#endif
}

float readToFFrontCm() {
#if SENSOR_ENABLED && TOF_ENABLED
  // TODO：根据实际 ToF 型号加入 VL53L0X/VL53L1X 等库代码。
  return -1.0f;
#else
  return -1.0f;
#endif
}
}  // namespace

void setupObstacleSensors() {
#if SENSOR_ENABLED
#if ULTRA_LEFT_ENABLED
  pinMode(ULTRA_LEFT_TRIG, OUTPUT);
  pinMode(ULTRA_LEFT_ECHO, INPUT);
#endif
#if ULTRA_FRONT_ENABLED
  digitalWrite(ULTRA_FRONT_TRIG, LOW);
  pinMode(ULTRA_FRONT_TRIG, OUTPUT);
  pinMode(ULTRA_FRONT_ECHO, INPUT);
#endif
#if ULTRA_RIGHT_ENABLED
  pinMode(ULTRA_RIGHT_TRIG, OUTPUT);
  pinMode(ULTRA_RIGHT_ECHO, INPUT);
#endif
#if ULTRA_REAR_ENABLED
  pinMode(ULTRA_REAR_TRIG, OUTPUT);
  pinMode(ULTRA_REAR_ECHO, INPUT);
#endif
#endif
}

DistanceData makeInvalidDistanceData() {
  DistanceData d;
  d.ultraLeftCm = -1.0f;
  d.ultraFrontCm = -1.0f;
  d.ultraRightCm = -1.0f;
  d.ultraRearCm = -1.0f;
  d.tofFrontCm = -1.0f;
  d.sampledAt = 0;
  return d;
}

DistanceData readDistanceSensors() {
  static DistanceData cached = makeInvalidDistanceData();
  static unsigned long lastSampleAt = 0;
  static bool hasSample = false;

#if SENSOR_ENABLED
  const unsigned long now = millis();

  if (!hasSample ||
      now - lastSampleAt >= ULTRA_SAMPLE_INTERVAL_MS) {
    hasSample = true;
    lastSampleAt = now;
    cached.sampledAt = now;

#if ULTRA_LEFT_ENABLED
    cached.ultraLeftCm =
        readJSNSR04(ULTRA_LEFT_TRIG, ULTRA_LEFT_ECHO).distanceCm;
#endif

#if ULTRA_FRONT_ENABLED
    const UltrasonicReading front =
        readJSNSR04(ULTRA_FRONT_TRIG, ULTRA_FRONT_ECHO);

    cached.ultraFrontCm = front.distanceCm;
    updateFrontBlockState(cached);
    printFrontReading(front, now);
#endif

#if ULTRA_RIGHT_ENABLED
    cached.ultraRightCm =
        readJSNSR04(ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO).distanceCm;
#endif

#if ULTRA_REAR_ENABLED
    cached.ultraRearCm =
        readJSNSR04(ULTRA_REAR_TRIG, ULTRA_REAR_ECHO).distanceCm;
#endif

    cached.tofFrontCm = readToFFrontCm();
  }
#endif

  return cached;
}

bool isFrontBlocked(const DistanceData& d) {
  updateFrontBlockState(d);

#if SENSOR_ENABLED && ULTRA_FRONT_ENABLED
  const bool ultrasonicBlocked = frontBlockedLatched;
#else
  const bool ultrasonicBlocked = false;
#endif
  const bool tofBlocked =
      d.tofFrontCm > 0 && d.tofFrontCm < TOF_TOO_CLOSE_CM;
  return ultrasonicBlocked || tofBlocked;
}

bool isLeftBlocked(const DistanceData& d) {
#if SENSOR_ENABLED && ULTRA_LEFT_ENABLED
  return d.ultraLeftCm > 0 && d.ultraLeftCm < ULTRA_SIDE_SAFE_CM;
#else
  (void)d;
  return false;
#endif
}

bool isRightBlocked(const DistanceData& d) {
#if SENSOR_ENABLED && ULTRA_RIGHT_ENABLED
  return d.ultraRightCm > 0 && d.ultraRightCm < ULTRA_SIDE_SAFE_CM;
#else
  (void)d;
  return false;
#endif
}

bool isRearBlocked(const DistanceData& d) {
#if SENSOR_ENABLED && ULTRA_REAR_ENABLED
  return d.ultraRearCm > 0 && d.ultraRearCm < ULTRA_REAR_SAFE_CM;
#else
  (void)d;
  return false;
#endif
}

bool obstacleOverrideRequired(const DistanceData& d) {
  // 车头单传感器阶段只做最高优先级停车，不尝试盲目选择绕行方向。
  return isFrontBlocked(d);
}

MotionCommand computeObstacleCommand(const DistanceData& d) {
  (void)d;

  /*
   * 障碍解除后不能沿用障碍出现前的“已确认可跟随”状态。
   * 清除确认计数后，自动模式必须重新收到两条序号不同、
   * 且距离满足启动条件的 TARGET，才允许再次前进。
   */
  resetAutoFollowConfirmation();

  // 只有车头距离时无法判断左右哪边可通行，因此当前必须停车。
  return makeStopCommand("front obstacle stop");
}
