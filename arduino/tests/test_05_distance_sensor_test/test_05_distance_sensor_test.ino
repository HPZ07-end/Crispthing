const int TRIG_PIN = 4;
const int ECHO_PIN = 11;

float readDistanceCm() {
  // 保证 Trig 初始为低电平
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  // 发送 10 微秒触发脉冲
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // 等待 Echo 返回，最多等待 30 ms
  unsigned long duration =
      pulseIn(ECHO_PIN, HIGH, 30000UL);

  // 超时，说明没有收到有效回波
  if (duration == 0) {
    return -1.0;
  }

  // 声速约 0.0343 cm/μs，除以 2 是往返距离
  return duration * 0.0343 / 2.0;
}

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  Serial.println("Front ultrasonic test ready.");
}

void loop() {
  float distanceCm = readDistanceCm();

  if (distanceCm < 0) {
    Serial.println("Distance: invalid");
  } else {
    Serial.print("Distance: ");
    Serial.print(distanceCm, 1);
    Serial.println(" cm");
  }

  delay(200);
}