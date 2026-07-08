/*
  target_follow_controller.ino

  Arduino side controller for OpenBot target-following robot.

  OpenBot sends:
    TARGET,visible,x,size,quality

  Example:
    TARGET,1,-0.25,0.12,0.85

  Current mode:
    - Parse TARGET message
    - Compute left/right motor speed
    - Print debug result
    - Motor output disabled by default

  After hardware arrives:
    1. Test Blink
    2. Test Serial
    3. Upload this file
    4. Test TARGET parsing
    5. Set MOTOR_ENABLED to 1
    6. Modify motor pins according to seller wiring diagram
*/


/*
loop()
  ↓
readSerialLines()
  ↓
收到一整行 TARGET 数据
  ↓
handleLine()
  ↓
parseTargetMessage()
  ↓
得到 visible / x / size / quality
  ↓
computeMotionCommand()
  ↓
判断：
  - 目标是否可见
  - quality 是否足够
  - 前方距离是否安全
  - size 是否太大
  ↓
计算：
  - forwardSpeed
  - turnSpeed
  - leftSpeed
  - rightSpeed
  ↓
applyMotionCommand()
  ↓
调试模式：打印左右轮速度
电机模式：真正控制电机
*/

// =======================
// Build switches
// =======================

#define MOTOR_ENABLED 0
#define SENSOR_ENABLED 0
#define DEBUG_PRINT 1
#define TURN_SIGN 1


// =======================
// Motor pins 电机pin针
// These are placeholders.
// Modify them according to the seller's wiring diagram.
// =======================

const int LEFT_PWM  = 5;
const int LEFT_IN1  = 7;
const int LEFT_IN2  = 8;

const int RIGHT_PWM = 6;
const int RIGHT_IN1 = 9;
const int RIGHT_IN2 = 10;


// =======================
// Control parameters 参数调试
// Need real testing later.
// =======================

const float QUALITY_MIN = 0.50;

const float SIZE_FAR  = 0.08;
const float SIZE_NEAR = 0.30;

const int MIN_SPEED = 60;
const int MAX_SPEED = 160;

const float K_TURN = 90.0;

const float SAFE_DISTANCE_CM = 50.0;

const unsigned long TARGET_TIMEOUT_MS = 500;


// =======================
// Data structures
// =======================

struct TargetData {
  bool visible;
  float x;
  float size;
  float quality;
  unsigned long receivedAt;
};

struct MotionCommand {
  int leftSpeed;
  int rightSpeed;
  const char* reason;
};


// =======================
// Global variables
// =======================

TargetData latestTarget;
bool hasReceivedTarget = false;
bool timeoutStopReported = false;

char serialBuffer[100];
int serialIndex = 0;


// =======================
// Setup
// =======================

void setup() {
  Serial.begin(9600);

  setupMotorPins();
  stopCar();

  Serial.println("Arduino target follow controller ready.");
  Serial.println("Protocol: TARGET,visible,x,size,quality");
  Serial.println("Example: TARGET,1,-0.25,0.12,0.85");
}


// =======================
// Main loop
// =======================

void loop() { //循环执行
  readSerialLines(); //检查OpenBot有无通过串口发来新的数据

  if (hasReceivedTarget) { //如果受到过目标数据，开始检查超时
    unsigned long now = millis(); //获取当前时间：从Arduino开机到现在

    if (now - latestTarget.receivedAt > TARGET_TIMEOUT_MS) { //超时，将触发超时停车
      if (!timeoutStopReported) { //只有没报告过“目标超时停车”才执行一次停车
        MotionCommand cmd = makeStopCommand("target timeout"); //创建停止命令
        applyMotionCommand(cmd);
        timeoutStopReported = true;
      }
    }
  }
}


// =======================
// Serial reading
// =======================

void readSerialLines() { 
  //串口接收函数
  while (Serial.available() > 0) { //只要串口有数据就继续读
    char c = Serial.read();

    if (c == '\n' || c == '\r') { //整一条消息结束
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0'; //加结束符
        handleLine(serialBuffer); //把一整行交给handelLine()处理
        serialIndex = 0; //清空索引，准备接收下一条消息
      }
    } else {
      if (serialIndex < sizeof(serialBuffer) - 1) { //防止数组越界
        serialBuffer[serialIndex++] = c;
      } else {
        serialIndex = 0;
        Serial.println("Serial buffer overflow. Message dropped."); //消息太长，丢弃
      }
    }
  }
}


void handleLine(char* line) { 
  //处理整行的串口消息
  if (startsWith(line, "TARGET")) { //判断是否是TARGET消息
    TargetData target;

    if (parseTargetMessage(line, target)) { //解析串口数据
      latestTarget = target; //保存目标数据
      hasReceivedTarget = true;
      timeoutStopReported = false; //允许以后再次出发超时判断

      MotionCommand cmd = computeMotionCommand(target); //计算运动指令
      applyMotionCommand(cmd); //执行运动指令
    } else {
      Serial.println("Invalid TARGET message.");
    }
  } else {
    Serial.print("Unknown message: ");
    Serial.println(line);
  }
}


bool startsWith(const char* text, const char* prefix) {
  //判断一个字符串是否以某个前缀开头
  while (*prefix) { //只要prefix还没走到末尾，就继续比较，只要有一个字符串不同，就返回false
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

bool parseTargetMessage(char* line, TargetData& target) { //协议解析函数
  char* token = strtok(line, ","); //用逗号作为分隔符

  if (token == NULL) return false;
  if (strcmp(token, "TARGET") != 0) return false;

  token = strtok(NULL, ",");
  if (token == NULL) return false;
  target.visible = atoi(token) == 1; //atoi：把字符串转换成整数

  token = strtok(NULL, ",");
  if (token == NULL) return false;
  target.x = atof(token); //atof：把字符串转换成浮点数

  token = strtok(NULL, ",");
  if (token == NULL) return false;
  target.size = atof(token);

  token = strtok(NULL, ",");
  if (token == NULL) return false;
  target.quality = atof(token);

  target.receivedAt = millis(); //记录接收时间，用于超时保护

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
// Control logic
// =======================

MotionCommand computeMotionCommand(TargetData target) {
  // 跟随决策核心
  // 输入：TargetData target：visible, x, size, qquality
  // 输出：MotionCommand: 左轮速度、右轮速度、决策原因

  if (!target.visible) { //目标不可见就停车；之后可能加上蜂鸣器报警
    return makeStopCommand("target not visible");
  }

  if (target.quality < QUALITY_MIN) { //识别质量太低就停车；之后需要汇报信息
    return makeStopCommand("low quality");
  }

  float frontDistance = readFrontDistanceCm(); //读取前方距离传感器

  if (frontDistance > 0 && frontDistance < SAFE_DISTANCE_CM) { //前方太近就停车
    return makeStopCommand("obstacle too close");
  }

  if (target.size > SIZE_NEAR) { //目标看起来太近就停车
    return makeStopCommand("target too close by size");
  }

  //计算前进速度和转向速度
  int forwardSpeed = computeForwardSpeed(target.size);
  int turnSpeed = computeTurnSpeed(target.x);

  int leftSpeed = forwardSpeed - turnSpeed;
  int rightSpeed = forwardSpeed + turnSpeed;

  //限制速度范围
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  MotionCommand cmd;
  cmd.leftSpeed = leftSpeed;
  cmd.rightSpeed = rightSpeed;
  cmd.reason = "follow target";

#if DEBUG_PRINT
  Serial.print("forwardSpeed = ");
  Serial.println(forwardSpeed);

  Serial.print("turnSpeed = ");
  Serial.println(turnSpeed);
#endif

  return cmd;
}


MotionCommand makeStopCommand(const char* reason) {
  //生成一个停车指令
  MotionCommand cmd;
  cmd.leftSpeed = 0;
  cmd.rightSpeed = 0;
  cmd.reason = reason;
  return cmd;
}


int computeForwardSpeed(float size) {
  // 根据目标大小size计算前进速度
  if (size <= SIZE_FAR) {
    return MAX_SPEED;
  }

  // 线性映射：目标越远，速度越大；目标越近，速度越小
  float ratio = (SIZE_NEAR - size) / (SIZE_NEAR - SIZE_FAR);
  ratio = constrain(ratio, 0.0, 1.0);

  int speed = MIN_SPEED + int(ratio * (MAX_SPEED - MIN_SPEED));
  return constrain(speed, MIN_SPEED, MAX_SPEED);
}


int computeTurnSpeed(float x) {
  // 根据目标左右偏移x计算转向速度
  x = constrain(x, -1.0, 1.0); //防止OpenBot发来的x超过预期范围
  int turnSpeed = int(K_TURN * x * TURN_SIGN); //TURN_SIGN为方向修正参数，因为实际测试时有可能发现方向与指令相反
  return turnSpeed;
}


// =======================
// Sensor layer
// =======================

float readFrontDistanceCm() { 
  //距离传感器读取接口
#if SENSOR_ENABLED
  // TODO:
  // Replace this with real ToF / ultrasonic sensor reading.
  // Example:
  // return tofSensor.readRangeCm();
  return -1;
#else
  return -1;
#endif
}


// =======================
// Motion output
// =======================

void applyMotionCommand(MotionCommand cmd) {
  //执行运动命令
#if DEBUG_PRINT //打印调试信息
  Serial.print("Decision: ");
  Serial.println(cmd.reason);

  Serial.print("Motor command: L = ");
  Serial.print(cmd.leftSpeed);
  Serial.print(", R = ");
  Serial.println(cmd.rightSpeed);
#endif

#if MOTOR_ENABLED
  setMotorSpeed(cmd.leftSpeed, cmd.rightSpeed);
#endif
}


// =======================
// Motor driver layer
// =======================

void setupMotorPins() {
  // 初始化电机控制引脚
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
  // 统一设置左右两个电机的速度
#if MOTOR_ENABLED
  setMotor(LEFT_PWM, LEFT_IN1, LEFT_IN2, leftSpeed);
  setMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, rightSpeed);
#endif
}


void setMotor(int pwmPin, int in1Pin, int in2Pin, int speed) {
  //单个电机的底层控制函数：正转、反转、停止、速度大小
#if MOTOR_ENABLED
  speed = constrain(speed, -255, 255); // 限制速度范围

  if (speed > 0) { // 正转
    digitalWrite(in1Pin, HIGH); // IN1 = HIGH
    digitalWrite(in2Pin, LOW); // IN2 = LOW
    analogWrite(pwmPin, speed); // PWM = speed
  } else if (speed < 0) { // 反转
    digitalWrite(in1Pin, LOW); 
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, -speed); // PWM仍是正数，但方向引脚已经反过来，电机反转
  } else { // speed = 0：停止
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);
  }
#endif
}


void stopCar() {
  // 停车
#if MOTOR_ENABLED
  setMotorSpeed(0, 0);
#endif
}