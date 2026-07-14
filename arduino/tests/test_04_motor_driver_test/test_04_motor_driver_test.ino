// 测试电机驱动板和左右电机
/*
  Important:
    1. Put the robot on a stand.
    2. Keep wheels off the ground.
    3. Confirm motor driver wiring before enabling motor output.
    4. Do not power motors from Arduino 5V.

  Serial commands:
    lf -> left motor forward
    lb -> left motor backward
    rf -> right motor forward
    rb -> right motor backward
    ff -> both motors forward
    ss -> stop

  单独测试左电机正反转
  单独测试右电机正反转
  测试两个电机一起前进
  测试停止
  测试急停按钮

  // 测试电机驱动板和左右履带
  // 第一次测试时必须让履带离地，不能落地测试
*/

#define MOTOR_TEST_ENABLED 0
#define EMERGENCY_STOP_ENABLED 1

// Placeholder pins. Modify after receiving the robot.
const int LEFT_PWM  = 5;
const int LEFT_IN1  = 7;
const int LEFT_IN2  = 8;

const int RIGHT_PWM = 6;
const int RIGHT_IN1 = 9;
const int RIGHT_IN2 = 10;

#define LEFT_TRACK_DIR 1
#define RIGHT_TRACK_DIR 1

// INPUT_PULLUP:
// not pressed = HIGH
// pressed     = LOW
const int EMERGENCY_STOP_PIN = 13;

const int TEST_SPEED = 80;
const unsigned long TEST_DURATION_MS = 600;

char serialBuffer[20];
int serialIndex = 0;

void setup() {
  Serial.begin(9600);

#if MOTOR_TEST_ENABLED
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  stopAllMotors();
#endif

#if EMERGENCY_STOP_ENABLED
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
#endif

  Serial.println("Motor driver test ready.");
  Serial.println("Commands: lf, lb, rf, rb, ff, ss");
  Serial.println("Motor output is disabled by default.");
}

void loop() {
  if (isEmergencyStopActive()) {
    Serial.println("Emergency stop active. Motors stopped.");
    stopAllMotors();
    delay(200);
    return;
  }

  readSerialLines();
}

void readSerialLines() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        handleCommand(serialBuffer);
        serialIndex = 0;
      }
    } else {
      if (serialIndex < (int)sizeof(serialBuffer) - 1) {
        serialBuffer[serialIndex++] = c;
      } else {
        serialIndex = 0;
        Serial.println("Command too long. Dropped.");
      }
    }
  }
}

void handleCommand(const char* cmd) {
  if (isEmergencyStopActive()) {
    stopAllMotors();
    Serial.println("Emergency stop active. Command ignored.");
    return;
  }

  if (strcmp(cmd, "lf") == 0) {
    Serial.println("Left track forward.");
    setMotor(LEFT_PWM, LEFT_IN1, LEFT_IN2, TEST_SPEED * LEFT_TRACK_DIR);
    delayAndStop();
  } else if (strcmp(cmd, "lb") == 0) {
    Serial.println("Left trackmotor backward.");
    setMotor(LEFT_PWM, LEFT_IN1, LEFT_IN2, -TEST_SPEED * LEFT_TRACK_DIR);
    delayAndStop();
  } else if (strcmp(cmd, "rf") == 0) {
    Serial.println("Right track forward.");
    setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, TEST_SPEED * LEFT_TRACK_DIR);
    delayAndStop();
  } else if (strcmp(cmd, "rb") == 0) {
    Serial.println("Right track backward.");
    setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, -TEST_SPEED * LEFT_TRACK_DIR);
    delayAndStop();
  } else if (strcmp(cmd, "ff") == 0) {
    Serial.println("Both tracks forward.");
    setMotor(LEFT_PWM, LEFT_IN1, LEFT_IN2, TEST_SPEED * LEFT_TRACK_DIR);
    setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, TEST_SPEED * LEFT_TRACK_DIR);
    delayAndStop();
  } else if (strcmp(cmd, "ss") == 0) {
    Serial.println("Stop all motors.");
    stopAllMotors();
  } else {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
  }
}

void delayAndStop() {
  unsigned long startTime = millis();

  while (millis() - startTime < TEST_DURATION_MS) {
    if (isEmergencyStopActive()) {
      Serial.println("Emergency stop during motor test.");
      stopAllMotors();
      return;
    }
    delay(10);
  }

  stopAllMotors();
}

bool isEmergencyStopActive() {
#if EMERGENCY_STOP_ENABLED
  return digitalRead(EMERGENCY_STOP_PIN) == LOW;
#else
  return false;
#endif
}

void setMotor(int pwmPin, int in1Pin, int in2Pin, int speed) {
#if MOTOR_TEST_ENABLED
  speed = constrain(speed, -255, 255);

  if (speed > 0) { // 正转
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) { // 反转
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, -speed);
  } else { // 停止
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);
  }
#else
  Serial.print("[Motor disabled] pwmPin = ");
  Serial.print(pwmPin);
  Serial.print(", speed = ");
  Serial.println(speed);
#endif
}

void stopAllMotors() {
#if MOTOR_TEST_ENABLED
  setMotor(LEFT_PWM, LEFT_IN1, LEFT_IN2, 0);
  setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, 0);
#else
  Serial.println("[Motor disabled] stopAllMotors()");
#endif
}
