const int EMERGENCY_STOP_PIN = A3;

void setup() {
  Serial.begin(9600);
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);

  Serial.println("Emergency stop test ready.");
}

void loop() {
  // NC 常闭接法
  // LOW：按钮释放，线路正常
  // HIGH：按钮按下或线路断开
  if (digitalRead(EMERGENCY_STOP_PIN) == HIGH) {
    Serial.println("EMERGENCY STOP ACTIVE");
  } else {
    Serial.println("Emergency stop released");
  }

  delay(200);
}