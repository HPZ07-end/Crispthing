// 测试串口输出
/*
  test_01_serial_print.ino

  Purpose:
    Check whether Serial Monitor works.

  Serial Monitor setting:
    Baud rate: 9600

  Expected result:
    Serial Monitor prints "loop running..." every second.
*/

void setup() {
  Serial.begin(9600);
  Serial.println("Arduino UNO serial print test started.");
}

void loop() {
  Serial.println("loop running...");
  delay(1000);
}