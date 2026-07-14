// 测试串口输出
/*
  test_01_serial_print.ino

  Purpose:
    Check whether Serial Monitor works.

  Serial Monitor setting:
    Baud rate: 9600

  Expected result:
    Serial Monitor prints "loop running..." every second.

  Arduino 能不能通过 Serial 向电脑发送文字
  串口监视器能不能正常显示信息
  波特率 9600 是否正常
*/

void setup() {
  Serial.begin(9600); // 开启串口通信，波特率是9600
  Serial.println("Arduino UNO serial print test started.");
}

void loop() {
  Serial.println("loop running..."); // 每秒打印一次
  delay(1000);
}