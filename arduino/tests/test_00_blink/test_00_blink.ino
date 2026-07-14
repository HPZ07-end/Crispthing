//测试Arduino IDE、数据线、驱动和上传流程
/*
  test_00_blink.ino

  Purpose:
    Check whether Arduino UNO, USB cable, driver and Arduino IDE upload process work.

  Expected result:
    The built-in LED on Arduino UNO blinks every 0.5 second.

  Arduino UNO 板子正常
  USB 数据线正常
  Arduino IDE 能上传程序
  板载 LED 能正常闪烁
*/

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // 把Arduino板载LED对应的引脚设为输出模式
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH); // LED亮0.5秒 
  delay(500);

  digitalWrite(LED_BUILTIN, LOW); // LED灭0.5秒
  delay(500);
}