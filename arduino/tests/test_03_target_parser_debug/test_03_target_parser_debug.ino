// 测试 OpenBot TARGET 数据解析和左右履带速度计算
// 只解析数据 + 计算速度，不真正驱动电机
/*
解析 TARGET,visible,x,size,quality
计算 forwardSpeed
计算 turnSpeed
计算 leftSpeed 和 rightSpeed
但不真正控制电机
*/

const float QUALITY_MIN = 0.50; // 识别质量的最低阈值：小于0.50就停车

const float SIZE_NEAR = 0.30; // 目标看起来比较小，认为比较远
const float SIZE_FAR = 0.08; // 目标看起来很大，认为太近

const int MAX_SPEED = 130; // 最大跟随速度
const int MIN_SPEED = 70; // 最小跟随速度

const float K_TURN = 70.0; // 转向灵敏度
#define TURN_SIGN 1 // 转向方向修正

const unsigned long TARGET_TIMEOUT_MS = 500; // 超时检测指标

unsigned long lastTargetTime = 0;
bool hasReceivedTarget = false;
bool timeoutReported = false;

void setup() {
  Serial.begin(9600);
  Serial.println("TARGET parser debug ready.");
  Serial.println("Send: TARGET,visible,x,size,quality");
  Serial.println("Example: TARGET,1,-0.25,0.12,0.85");
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("TARGET")) {
      handleTargetMessage(line); // 解析并计算
      lastTargetTime = millis();
      hasReceivedTarget = true;
      timeoutReported = false;
    } else if (line.length() > 0) {
      Serial.print("Unknown message: ");
      Serial.println(line);
    }
  }

  if (hasReceivedTarget && millis() - lastTargetTime > TARGET_TIMEOUT_MS) {
    if (!timeoutReported) {
      Serial.println("Decision: stop, target timeout.");
      printMotorSpeed(0, 0);
      timeoutReported = true;
    }
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
  if (visible == 0) { // 目标不可见，停车
    Serial.println("Decision: stop, target not visible.");
    printMotorSpeed(0, 0);
    return;
  }

  if (quality < QUALITY_MIN) { // quality太低，停车
    Serial.println("Decision: stop, low quality.");
    printMotorSpeed(0, 0);
    return;
  }

  if (size > SIZE_NEAR) { // size太大，目标可能太近，停车
    Serial.println("Decision: stop, target too close by size.");
    printMotorSpeed(0, 0);
    return;
  }

  // 否则，计算前进速度和转向速度
  int forwardSpeed = computeForwardSpeedBySize(size);
  int turnSpeed = computeTurnSpeed(x);

  // 差速小车
  int leftSpeed = forwardSpeed + turnSpeed;
  int rightSpeed = forwardSpeed - turnSpeed;

  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  Serial.print("forwardSpeed = ");
  Serial.println(forwardSpeed);

  Serial.print("turnSpeed = ");
  Serial.println(turnSpeed);

  printMotorSpeed(leftSpeed, rightSpeed);
}

int computeForwardSpeedBySize(float size) {
  if (size <= SIZE_FAR) {
    return MAX_SPEED;
  }

  float ratio = (SIZE_NEAR - size) / (SIZE_NEAR - SIZE_FAR);
  ratio = constrain(ratio, 0.0, 1.0);

  int speed = MIN_SPEED + int(ratio * (MAX_SPEED - MIN_SPEED));
  return constrain(speed, MIN_SPEED, MAX_SPEED);
}

int computeTurnSpeed(float x) {
  x = constrain(x, -1.0, 1.0);
  int turnSpeed = int(K_TURN * x * TURN_SIGN);
  return turnSpeed;
}

void printMotorSpeed(int leftSpeed, int rightSpeed) {
  Serial.print("Track command: L = ");
  Serial.print(leftSpeed);
  Serial.print(", R = ");
  Serial.println(rightSpeed);
}
