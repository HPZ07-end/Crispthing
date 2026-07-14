// 测试 4 个 JSN-SR04 + 1 个 ToF 距离传感器
/*
  SENSOR_TEST_ENABLED = 0:
    placeholder mode, all distance values are -1.

  SENSOR_TEST_ENABLED = 1:
    read four JSN-SR04 ultrasonic sensors.

  TOF_TEST_ENABLED = 0:
    ToF placeholder, tofFrontCm = -1.

  TOF_TEST_ENABLED = 1:
    add ToF library code in readToFFrontCm().
*/

#define SENSOR_TEST_ENABLED 0
#define TOF_TEST_ENABLED 0

// 四组超声波引脚
const int ULTRA_LEFT_TRIG  = 2;
const int ULTRA_LEFT_ECHO  = 3;

const int ULTRA_FRONT_TRIG = 4;
const int ULTRA_FRONT_ECHO = 11;

const int ULTRA_RIGHT_TRIG = 12;
const int ULTRA_RIGHT_ECHO = A0;

const int ULTRA_REAR_TRIG  = A1;
const int ULTRA_REAR_ECHO  = A2;

const unsigned long PRINT_INTERVAL_MS = 500;
unsigned long lastPrintTime = 0;

struct DistanceData {
  float ultraLeftCm;
  float ultraFrontCm;
  float ultraRightCm;
  float ultraRearCm;
  float tofFrontCm;
};

void setup() {
  Serial.begin(9600);

#if SENSOR_TEST_ENABLED
  pinMode(ULTRA_LEFT_TRIG, OUTPUT);
  pinMode(ULTRA_LEFT_ECHO, INPUT);

  pinMode(ULTRA_FRONT_TRIG, OUTPUT);
  pinMode(ULTRA_FRONT_ECHO, INPUT);

  pinMode(ULTRA_RIGHT_TRIG, OUTPUT);
  pinMode(ULTRA_RIGHT_ECHO, INPUT);

  pinMode(ULTRA_REAR_TRIG, OUTPUT);
  pinMode(ULTRA_REAR_ECHO, INPUT);
#endif

  Serial.println("Distance sensor test ready.");
  Serial.println("JSN-SR04 channels: left, front, right, rear.");
  Serial.println("ToF front channel is placeholder by default.");
}

void loop() {
  unsigned long now = millis();

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;

    DistanceData d = readDistanceSensors();
    printDistanceData(d);
  }
}

DistanceData readDistanceSensors() {
  DistanceData d;

#if SENSOR_TEST_ENABLED // 多个超声波不能同时触发
  d.ultraLeftCm = readJSNSR04Cm(ULTRA_LEFT_TRIG, ULTRA_LEFT_ECHO);
  delay(30);

  d.ultraFrontCm = readJSNSR04Cm(ULTRA_FRONT_TRIG, ULTRA_FRONT_ECHO);
  delay(30);

  d.ultraRightCm = readJSNSR04Cm(ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO);
  delay(30);

  d.ultraRearCm = readJSNSR04Cm(ULTRA_REAR_TRIG, ULTRA_REAR_ECHO);

  d.tofFrontCm = readToFFrontCm();
#else
  d.ultraLeftCm = -1;
  d.ultraFrontCm = -1;
  d.ultraRightCm = -1;
  d.ultraRearCm = -1;
  d.tofFrontCm = -1;
#endif

  return d;
}

float readJSNSR04Cm(int trigPin, int echoPin) {
#if SENSOR_TEST_ENABLED
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // 给Trig一个10微秒触发脉冲
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) { // 没测到有效回波
    return -1;
  }

  float distanceCm = duration * 0.0343 / 2.0; // 声速/2
  return distanceCm;
#else
  return -1;
#endif
}

float readToFFrontCm() {
#if SENSOR_TEST_ENABLED && TOF_TEST_ENABLED
  // TODO:
  // Add ToF library code here, for example VL53L0X / VL53L1X.
  // Return distance in cm.
  return -1;
#else
  return -1;
#endif
}

void printDistanceData(DistanceData d) {
  Serial.print("ultraLeftCm = ");
  Serial.print(d.ultraLeftCm);

  Serial.print(", ultraFrontCm = ");
  Serial.print(d.ultraFrontCm);

  Serial.print(", ultraRightCm = ");
  Serial.print(d.ultraRightCm);

  Serial.print(", ultraRearCm = ");
  Serial.print(d.ultraRearCm);

  Serial.print(", tofFrontCm = ");
  Serial.println(d.tofFrontCm);
}
