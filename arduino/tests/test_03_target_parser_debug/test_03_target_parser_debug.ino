// 测试OpenBot TARGET数据解析和左右轮速度计算
// =======================
// OpenBot TARGET 数据调试版
// 只解析数据 + 计算速度，不真正驱动电机
// =======================

float QUALITY_MIN = 0.50;

float SIZE_NEAR = 0.30;
float SIZE_FAR = 0.08;

int MAX_SPEED = 160;
int MIN_SPEED = 60;

float K_TURN = 90.0;

unsigned long lastTargetTime = 0;
unsigned long TARGET_TIMEOUT = 500;

void setup() {
  Serial.begin(9600);
  Serial.println("TARGET parser ready.");
  Serial.println("Send: TARGET,visible,x,size,quality");
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("TARGET")) {
      handleTargetMessage(line);
      lastTargetTime = millis();
    }
  }

  if (millis() - lastTargetTime > TARGET_TIMEOUT) {
    // 这里先只打印，不真正停车
    // 防止一直刷屏，加一个简单延时
    delay(100);
  }
}

void handleTargetMessage(String line) {
  int p1 = line.indexOf(',');
  int p2 = line.indexOf(',', p1 + 1);
  int p3 = line.indexOf(',', p2 + 1);
  int p4 = line.indexOf(',', p3 + 1);

  if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0) {
    Serial.println("Invalid TARGET message.");
    return;
  }

  int visible = line.substring(p1 + 1, p2).toInt();
  float x = line.substring(p2 + 1, p3).toFloat();
  float size = line.substring(p3 + 1, p4).toFloat();
  float quality = line.substring(p4 + 1).toFloat();

  Serial.println("----- TARGET received -----");
  Serial.print("visible = ");
  Serial.println(visible);

  Serial.print("x = ");
  Serial.println(x);

  Serial.print("size = ");
  Serial.println(size);

  Serial.print("quality = ");
  Serial.println(quality);

  computeControl(visible, x, size, quality);
}

void computeControl(int visible, float x, float size, float quality) {
  if (visible == 0) {
    Serial.println("Decision: stop, target not visible.");
    printMotorSpeed(0, 0);
    return;
  }

  if (quality < QUALITY_MIN) {
    Serial.println("Decision: stop, low quality.");
    printMotorSpeed(0, 0);
    return;
  }

  int forwardSpeed = computeForwardSpeed(size);
  int turnSpeed = computeTurnSpeed(x);

  int leftSpeed = forwardSpeed - turnSpeed;
  int rightSpeed = forwardSpeed + turnSpeed;

  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  Serial.print("forwardSpeed = ");
  Serial.println(forwardSpeed);

  Serial.print("turnSpeed = ");
  Serial.println(turnSpeed);

  printMotorSpeed(leftSpeed, rightSpeed);
}

int computeForwardSpeed(float size) {
  if (size > SIZE_NEAR) {
    return 0;
  }

  if (size < SIZE_FAR) {
    return MAX_SPEED;
  }

  return MIN_SPEED;
}

int computeTurnSpeed(float x) {
  x = constrain(x, -1.0, 1.0);
  int turnSpeed = int(K_TURN * x);
  return turnSpeed;
}

void printMotorSpeed(int leftSpeed, int rightSpeed) {
  Serial.print("Motor command: L = ");
  Serial.print(leftSpeed);
  Serial.print(", R = ");
  Serial.println(rightSpeed);
}