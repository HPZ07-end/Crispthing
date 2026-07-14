//测试简单字符指令解析
/*
  test_02_serial_command.ino

  Purpose:
    Test simple serial command parsing.

  Serial Monitor setting:
    Baud rate: 9600
    Line ending: Newline or No line ending both work

  Commands:
    f -> Forward
    b -> Backward
    l -> Turn left
    r -> Turn right
    s -> Stop

  Note:
    This program only prints actions. It does not control motors.

  电脑能不能向 Arduino 发送字符
  Arduino 能不能读取字符
  Arduino 能不能根据字符做判断
*/

void setup() {
  Serial.begin(9600);
  Serial.println("Serial command test ready.");
  Serial.println("Input f / b / l / r / s:");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == '\n' || cmd == '\r') {
      return;
    }

    if (cmd == 'f') {
      Serial.println("Forward");
    } else if (cmd == 'b') {
      Serial.println("Backward");
    } else if (cmd == 'l') {
      Serial.println("Turn left");
    } else if (cmd == 'r') {
      Serial.println("Turn right");
    } else if (cmd == 's') {
      Serial.println("Stop");
    } else {
      Serial.print("Unknown command: ");
      Serial.println(cmd);
    }
  }
}