/*
  target_follow_controller.ino

  Arduino side controller for OpenBot target-following robot.

  OpenBot sends:
    TARGET,visible,x,size,quality

  Current mode:
    - Parse TARGET message
    - Read emergency stop button
    - Reserve four JSN-SR04 ultrasonic channels
    - Reserve one front ToF channel
    - Compute left/right track speed
    - Print debug result
    - Motor output disabled by default
*/

// =======================
// Build switches
// =======================

#define MOTOR_ENABLED 0 // 1：驱动电机；0：只打印速度
#define SENSOR_ENABLED 0 // 1：读取 JSN-SR04；0：距离值是-1
#define TOF_ENABLED 0 // 1：读取ToF；0：距离值是-1
#define EMERGENCY_STOP_ENABLED 1 // 1：启用急停按钮
#define DEBUG_PRINT 1 // 1：打印调试信息

#define TURN_SIGN 1


// =======================
// Motor pins
// These are placeholders.
// Modify them according to the seller's wiring diagram.
// =======================

const int LEFT_PWM  = 5;
const int LEFT_IN1  = 7;
const int LEFT_IN2  = 8;

const int RIGHT_PWM = 6;
const int RIGHT_IN1 = 9;
const int RIGHT_IN2 = 10;

// 左右履带方向修正参数
// 如果某一侧履带“正转”时实际向后，就把对应值改成 -1
#define LEFT_TRACK_DIR 1
#define RIGHT_TRACK_DIR 1

// =======================
// JSN-SR04 ultrasonic pins
// =======================

const int ULTRA_LEFT_TRIG  = 2;
const int ULTRA_LEFT_ECHO  = 3;

const int ULTRA_FRONT_TRIG = 4;
const int ULTRA_FRONT_ECHO = 11;

const int ULTRA_RIGHT_TRIG = 12;
const int ULTRA_RIGHT_ECHO = A0;

const int ULTRA_REAR_TRIG  = A1;
const int ULTRA_REAR_ECHO  = A2;


// =======================
// ToF pins
// For most I2C ToF modules on Arduino UNO:
// SDA = A4, SCL = A5
// =======================


// =======================
// Emergency stop pin
// Use INPUT_PULLUP:
// not pressed = HIGH
// pressed     = LOW
// =======================

const int EMERGENCY_STOP_PIN = 13;


// =======================
// Control parameters
// Need real testing later.
// =======================

const float QUALITY_MIN = 0.50;

const float SIZE_FAR  = 0.08;
const float SIZE_NEAR = 0.30;

const int MIN_SPEED = 70;
const int MAX_SPEED = 130;

const float K_TURN = 70.0;

// 传感器安全阈值
const float ULTRA_FRONT_SAFE_CM = 45.0; // 前方
const float ULTRA_SIDE_SAFE_CM  = 35.0; // 左右
const float ULTRA_REAR_SAFE_CM  = 35.0; // 后方

const float TOF_TOO_CLOSE_CM = 80.0;
const float TOF_FAR_CM       = 180.0;

const float TARGET_CENTER_X_THRESHOLD = 0.25;

const unsigned long TARGET_TIMEOUT_MS = 500;
const unsigned long CONTROL_INTERVAL_MS = 50;


// =======================
// Data structures
// =======================

struct TargetData {
  bool visible; // 是否看见目标
  float x; // 目标左右偏移
  float size; // 目标大小
  float quality; // 识别质量
  unsigned long receivedAt; // 收到这条数据的实践，用于判断超时
};

struct DistanceData {
  float ultraLeftCm; // 左侧 JSN-SR04 距离
  float ultraFrontCm;
  float ultraRightCm;
  float ultraRearCm;
  float tofFrontCm;
};

struct MotionCommand {
  int leftSpeed;
  int rightSpeed;
  const char* reason;
};

enum RobotState {
  STATE_IDLE,
  STATE_AUTO,
  STATE_BYPASS,
  STATE_BACKUP,
  STATE_REACQUIRE,
  STATE_MANUAL,
  STATE_EMERGENCY
};


// =======================
// Global variables
// =======================

TargetData latestTarget;
DistanceData latestDistance;

RobotState currentState = STATE_IDLE;

bool hasReceivedTarget = false;
bool timeoutStopReported = false;
bool waitingStopReported = false;

char serialBuffer[100];
int serialIndex = 0;

unsigned long lastControlTime = 0;


// =======================
// Setup
// =======================

void setup() {
  Serial.begin(9600);

  setupMotorPins();
  setupSensorPins();
  setupEmergencyStopPin();

  stopCar();

  latestDistance = makeInvalidDistanceData();

  Serial.println("Arduino target follow controller ready.");
  Serial.println("Protocol: TARGET,visible,x,size,quality");
  Serial.println("Example: TARGET,1,-0.25,0.12,0.85");
}


// =======================
// Main loop
// =======================

void loop() {
  readSerialLines();

  unsigned long now = millis();
  if (now - lastControlTime < CONTROL_INTERVAL_MS) {
    return;
  }
  lastControlTime = now;

  latestDistance = readDistanceSensors();

  if (isEmergencyStopActive()) {
    MotionCommand cmd = makeStopCommand("emergency stop");
    currentState = STATE_EMERGENCY;
    applyMotionCommand(cmd);
    return;
  }

  if (!hasReceivedTarget) {
    if (!waitingStopReported) {
      MotionCommand cmd = makeStopCommand("waiting for target");
      currentState = STATE_IDLE;
      applyMotionCommand(cmd);
      waitingStopReported = true;
    }
    return;
  }

  if (now - latestTarget.receivedAt > TARGET_TIMEOUT_MS) {
    if (!timeoutStopReported) {
      MotionCommand cmd = makeStopCommand("target timeout");
      currentState = STATE_IDLE;
      applyMotionCommand(cmd);
      timeoutStopReported = true;
    }
    return;
  }

  MotionCommand cmd = computeMotionCommand(latestTarget, latestDistance);
  applyMotionCommand(cmd);
}


// =======================
// Serial reading
// =======================

void readSerialLines() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        handleLine(serialBuffer);
        serialIndex = 0;
      }
    } else {
      if (serialIndex < (int)sizeof(serialBuffer) - 1) {
        serialBuffer[serialIndex++] = c;
      } else {
        serialIndex = 0;
        Serial.println("Serial buffer overflow. Message dropped.");
      }
    }
  }
}


void handleLine(char* line) {
  if (startsWith(line, "TARGET")) {
    TargetData target;

    if (parseTargetMessage(line, target)) {
      latestTarget = target;
      hasReceivedTarget = true;
      timeoutStopReported = false;
      waitingStopReported = false;
    } else {
      Serial.println("Invalid TARGET message.");
    }
  } else {
    Serial.print("Unknown message: ");
    Serial.println(line);
  }
}


bool startsWith(const char* text, const char* prefix) {
  while (*prefix) {
    if (*text++ != *prefix++) {
      return false;
    }
  }
  return true;
}


// =======================
// Protocol parser
// Parse:
// TARGET,visible,x,size,quality
// =======================

bool parseTargetMessage(char* line, TargetData& target) {
  char* token = strtok(line, ",");

  if (token == NULL) return false;
  if (strcmp(token, "TARGET") != 0) return false;

  token = strtok(NULL, ",");
  if (token == NULL) return false;
  target.visible = atoi(token) == 1;

  token = strtok(NULL, ",");
  if (token == NULL) return false;
  target.x = atof(token);

  token = strtok(NULL, ",");
  if (token == NULL) return false;
  target.size = atof(token);

  token = strtok(NULL, ",");
  if (token == NULL) return false;
  target.quality = atof(token);

  target.receivedAt = millis();

#if DEBUG_PRINT
  Serial.println("----- TARGET received -----");
  Serial.print("visible = ");
  Serial.println(target.visible);

  Serial.print("x = ");
  Serial.println(target.x);

  Serial.print("size = ");
  Serial.println(target.size);

  Serial.print("quality = ");
  Serial.println(target.quality);
#endif

  return true;
}


// =======================
// State selection
// =======================

RobotState selectRobotState(TargetData target, DistanceData distance) {
  if (isEmergencyStopActive()) {
    return STATE_EMERGENCY;
  }

  if (isManualControlActive()) {
    return STATE_MANUAL;
  }

  if (isFrontBlocked(distance)) {
    return STATE_IDLE;
  }

  if (!target.visible) {
    return STATE_REACQUIRE;
  }

  if (target.quality < QUALITY_MIN) {
    return STATE_IDLE;
  }

  return STATE_AUTO;
}


// =======================
// Control logic
// =======================

MotionCommand computeMotionCommand(TargetData target, DistanceData distance) {
  currentState = selectRobotState(target, distance);

  if (currentState == STATE_EMERGENCY) {
    return makeStopCommand("emergency stop");
  }

  if (currentState == STATE_MANUAL) {
    return computeManualCommand();
  }

  if (currentState == STATE_REACQUIRE) {
    return makeStopCommand("target reacquire");
  }

  if (currentState == STATE_BYPASS) {
    return computeBypassCommand(distance);
  }

  if (currentState == STATE_BACKUP) {
    return computeBackupCommand(distance);
  }

  if (currentState == STATE_IDLE) {
    return makeStopCommand("idle or safety stop");
  }

  if (currentState == STATE_AUTO) {
    return computeAutoFollowCommand(target, distance);
  }

  return makeStopCommand("unknown state");
}


MotionCommand computeAutoFollowCommand(TargetData target, DistanceData distance) {
  if (!target.visible) {
    return makeStopCommand("target not visible");
  }

  if (target.quality < QUALITY_MIN) {
    return makeStopCommand("low quality");
  }

  if (isFrontBlocked(distance)) {
    return makeStopCommand("front blocked");
  }

  if (target.size > SIZE_NEAR) {
    return makeStopCommand("target too close by size");
  }

  int forwardSpeed = computeForwardSpeedWithToF(target, distance);
  int turnSpeed = computeTurnSpeed(target.x);

  if (target.x < 0 && isLeftBlocked(distance)) {
    turnSpeed = 0;
  }

  if (target.x > 0 && isRightBlocked(distance)) {
    turnSpeed = 0;
  }

  int leftSpeed = forwardSpeed + turnSpeed;
  int rightSpeed = forwardSpeed - turnSpeed;

  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  MotionCommand cmd;
  cmd.leftSpeed = leftSpeed;
  cmd.rightSpeed = rightSpeed;
  cmd.reason = "auto follow";
  return cmd;
}


MotionCommand computeManualCommand() {
  // TODO: Add joystick or upper-computer manual control.
  return makeStopCommand("manual not implemented");
}


MotionCommand computeBypassCommand(DistanceData distance) {
  // TODO: Add local obstacle bypass logic.
  return makeStopCommand("bypass not implemented");
}


MotionCommand computeBackupCommand(DistanceData distance) {
  // TODO: Add low-speed backup escape logic.
  return makeStopCommand("backup not implemented");
}


MotionCommand makeStopCommand(const char* reason) {
  MotionCommand cmd;
  cmd.leftSpeed = 0;
  cmd.rightSpeed = 0;
  cmd.reason = reason;
  return cmd;
}


int computeForwardSpeedWithToF(TargetData target, DistanceData distance) {
  bool targetNearCenter = absoluteFloat(target.x) < TARGET_CENTER_X_THRESHOLD;

  if (targetNearCenter && distance.tofFrontCm > 0) {
    if (distance.tofFrontCm < TOF_TOO_CLOSE_CM) {
      return 0;
    }

    if (distance.tofFrontCm > TOF_FAR_CM) {
      return MAX_SPEED;
    }

    float ratio = (distance.tofFrontCm - TOF_TOO_CLOSE_CM) / (TOF_FAR_CM - TOF_TOO_CLOSE_CM);
    ratio = constrain(ratio, 0.0, 1.0);

    int speed = MIN_SPEED + int(ratio * (MAX_SPEED - MIN_SPEED));
    return constrain(speed, MIN_SPEED, MAX_SPEED);
  }

  return computeForwardSpeedBySize(target.size);
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


float absoluteFloat(float value) {
  if (value < 0) {
    return -value;
  }
  return value;
}


// =======================
// Safety checks
// =======================

bool isEmergencyStopActive() {
#if EMERGENCY_STOP_ENABLED
  return digitalRead(EMERGENCY_STOP_PIN) == LOW;
#else
  return false;
#endif
}


bool isManualControlActive() {
  // TODO: Add joystick/manual-mode input.
  return false;
}


bool isFrontBlocked(DistanceData d) {
  if (d.ultraFrontCm > 0 && d.ultraFrontCm < ULTRA_FRONT_SAFE_CM) {
    return true;
  }

  if (d.tofFrontCm > 0 && d.tofFrontCm < TOF_TOO_CLOSE_CM) {
    return true;
  }

  return false;
}


bool isLeftBlocked(DistanceData d) {
  return d.ultraLeftCm > 0 && d.ultraLeftCm < ULTRA_SIDE_SAFE_CM;
}


bool isRightBlocked(DistanceData d) {
  return d.ultraRightCm > 0 && d.ultraRightCm < ULTRA_SIDE_SAFE_CM;
}


bool isRearBlocked(DistanceData d) {
  return d.ultraRearCm > 0 && d.ultraRearCm < ULTRA_REAR_SAFE_CM;
}


// =======================
// Sensor layer
// =======================

void setupSensorPins() {
#if SENSOR_ENABLED
  pinMode(ULTRA_LEFT_TRIG, OUTPUT);
  pinMode(ULTRA_LEFT_ECHO, INPUT);

  pinMode(ULTRA_FRONT_TRIG, OUTPUT);
  pinMode(ULTRA_FRONT_ECHO, INPUT);

  pinMode(ULTRA_RIGHT_TRIG, OUTPUT);
  pinMode(ULTRA_RIGHT_ECHO, INPUT);

  pinMode(ULTRA_REAR_TRIG, OUTPUT);
  pinMode(ULTRA_REAR_ECHO, INPUT);
#endif
}


DistanceData makeInvalidDistanceData() {
  DistanceData d;
  d.ultraLeftCm = -1;
  d.ultraFrontCm = -1;
  d.ultraRightCm = -1;
  d.ultraRearCm = -1;
  d.tofFrontCm = -1;
  return d;
}


DistanceData readDistanceSensors() {
  DistanceData d = makeInvalidDistanceData();

#if SENSOR_ENABLED
  d.ultraLeftCm = readJSNSR04Cm(ULTRA_LEFT_TRIG, ULTRA_LEFT_ECHO);
  delay(30);

  d.ultraFrontCm = readJSNSR04Cm(ULTRA_FRONT_TRIG, ULTRA_FRONT_ECHO);
  delay(30);

  d.ultraRightCm = readJSNSR04Cm(ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO);
  delay(30);

  d.ultraRearCm = readJSNSR04Cm(ULTRA_REAR_TRIG, ULTRA_REAR_ECHO);

  d.tofFrontCm = readToFFrontCm();
#endif

  return d;
}


float readJSNSR04Cm(int trigPin, int echoPin) {
#if SENSOR_ENABLED
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    return -1;
  }

  float distanceCm = duration * 0.0343 / 2.0;
  return distanceCm;
#else
  return -1;
#endif
}


float readToFFrontCm() {
#if SENSOR_ENABLED && TOF_ENABLED
  // TODO:
  // Add ToF library code here, for example VL53L0X / VL53L1X.
  // Return distance in cm.
  return -1;
#else
  return -1;
#endif
}


// =======================
// Emergency stop layer
// =======================

void setupEmergencyStopPin() {
#if EMERGENCY_STOP_ENABLED
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
#endif
}


// =======================
// Motion output
// =======================

void applyMotionCommand(MotionCommand cmd) {
#if DEBUG_PRINT
  Serial.print("State: ");
  printRobotState(currentState);

  Serial.print("Decision: ");
  Serial.println(cmd.reason);

  Serial.print("Track command: L = ");
  Serial.print(cmd.leftSpeed);
  Serial.print(", R = ");
  Serial.println(cmd.rightSpeed);
#endif

#if MOTOR_ENABLED
  setMotorSpeed(cmd.leftSpeed, cmd.rightSpeed);
#endif
}


void printRobotState(RobotState state) {
#if DEBUG_PRINT
  if (state == STATE_IDLE) {
    Serial.println("IDLE");
  } else if (state == STATE_AUTO) {
    Serial.println("AUTO");
  } else if (state == STATE_BYPASS) {
    Serial.println("BYPASS");
  } else if (state == STATE_BACKUP) {
    Serial.println("BACKUP");
  } else if (state == STATE_REACQUIRE) {
    Serial.println("REACQUIRE");
  } else if (state == STATE_MANUAL) {
    Serial.println("MANUAL");
  } else if (state == STATE_EMERGENCY) {
    Serial.println("EMERGENCY");
  } else {
    Serial.println("UNKNOWN");
  }
#endif
}


// =======================
// Motor driver layer
// =======================

void setupMotorPins() {
#if MOTOR_ENABLED
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
#endif
}


void setMotorSpeed(int leftSpeed, int rightSpeed) {
#if MOTOR_ENABLED
  setMotor(LEFT_PWM, LEFT_IN1, LEFT_IN2, leftSpeed * LEFT_TRACK_DIR);
  setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, rightSpeed * RIGHT_TRACK_DIR);
#endif
}


void setMotor(int pwmPin, int in1Pin, int in2Pin, int speed) {
#if MOTOR_ENABLED
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);
  }
#endif
}


void stopCar() {
#if MOTOR_ENABLED
  setMotorSpeed(0, 0);
#endif
}
