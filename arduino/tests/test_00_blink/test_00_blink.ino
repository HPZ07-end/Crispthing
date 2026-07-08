//测试Arduino IDE、数据线、驱动和上传流程
/*
  test_00_blink.ino

  Purpose:
    Check whether Arduino UNO, USB cable, driver and Arduino IDE upload process work.

  Expected result:
    The built-in LED on Arduino UNO blinks every 0.5 second.
*/

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);

  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}