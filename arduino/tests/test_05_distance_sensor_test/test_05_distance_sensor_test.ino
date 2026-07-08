// 测试前方距离传感器
/*
  test_05_distance_sensor_test.ino

  Purpose:
    Test front distance sensor.

  Current support:
    SENSOR_TYPE = 0: no sensor, placeholder mode
    SENSOR_TYPE = 1: HC-SR04 ultrasonic sensor

  If using ToF sensor later, replace readFrontDistanceCm() with the corresponding library code.
*/

#define SENSOR_TYPE 0

// For HC-SR04 only.
const int TRIG_PIN = 11;
const int ECHO_PIN = 12;

const unsigned long PRINT_INTERVAL_MS = 500;
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(9600);

#if SENSOR_TYPE == 1
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
#endif

  Serial.println("Distance sensor test ready.");
}

void loop() {
  unsigned long now = millis();

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;

    float distance = readFrontDistanceCm();

    Serial.print("frontDistanceCm = ");
    Serial.println(distance);
  }
}

float readFrontDistanceCm() {
#if SENSOR_TYPE == 1
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return -1;
  }

  float distanceCm = duration * 0.0343 / 2.0;
  return distanceCm;
#else
  return -1;
#endif
}