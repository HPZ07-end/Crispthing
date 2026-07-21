#include "config.h"
#include <Wire.h>

// ============================================================
// 遥控安全状态：急停释放/失联后必须先回到中位
// ============================================================
#if I2C_REMOTE_ENABLED
static bool remoteNeutralRearmRequired = true;
static bool remoteRearmMessagePrinted = false;

static RemoteAction lastContinuousAction = REMOTE_STOP;
static unsigned long continuousMotionStartedAt = 0;
static bool deadmanMessagePrinted = false;
static bool linkTimeoutMessagePrinted = false;
#endif

namespace {

// ============================================================
// 串口输入缓存
// ============================================================
char serialBuffer[80];
uint8_t serialIndex = 0;

#if I2C_REMOTE_ENABLED

// ============================================================
// I2C 遥控板状态
// ============================================================
unsigned long lastRemotePollTime = 0;
unsigned long lastI2cRetryTime = 0;
unsigned long lastI2cHealthCheckTime = 0;

// 最近一次成功解析出完整遥控帧的时间。
// 只在完整帧到达时更新，半帧不会触发断线。
unsigned long lastValidRemotePacketTime = 0;
bool hasValidRemotePacket = false;

bool remoteBoardFound = false;

// 遥控数据流重新组帧
uint8_t remoteFrameBuffer[REMOTE_PACKET_LENGTH] = {0};
uint8_t remoteFrameIndex = 0;

// 用于检测数据变化和按键边沿
uint8_t previousPacket[REMOTE_PACKET_LENGTH] = {0};
bool hasPreviousPacket = false;

uint8_t previousDpad = 0xFF;
uint8_t previousKeys = 0xFF;

#endif

// ============================================================
// 通用辅助函数
// ============================================================
bool startsWith(const char* text, const char* prefix) {
  while (*prefix) {
    if (*text++ != *prefix++) {
      return false;
    }
  }

  return true;
}

void setRemoteAction(RemoteAction action) {
  manualModeActive = true;
  remoteAction = action;
  lastRemoteCommandTime = millis();
}

// ============================================================
// 电脑串口遥控
// u：前进
// b：后退
// l：左转
// r：右转
// s：停止
// +：加速
// -：减速
// a：自动模式
// ============================================================
void handleRemoteCharacter(char command) {
  command = (char)tolower(command);

  if (command == 'a') {
    manualModeActive = false;
    remoteAction = REMOTE_STOP;

    Serial.println(F("Mode: AUTO"));
    return;
  }

  if (command == 'u') {
    setRemoteAction(REMOTE_FORWARD);

  } else if (command == 'b') {
    setRemoteAction(REMOTE_BACKWARD);

  } else if (command == 'l') {
    setRemoteAction(REMOTE_LEFT);

  } else if (command == 'r') {
    setRemoteAction(REMOTE_RIGHT);

  } else if (command == 's') {
    setRemoteAction(REMOTE_STOP);

  } else if (command == '+') {
    remoteSpeed =
        constrain(remoteSpeed + 10, MIN_SPEED, MAX_SPEED);

  } else if (command == '-') {
    remoteSpeed =
        constrain(remoteSpeed - 10, MIN_SPEED, MAX_SPEED);

  } else {
    Serial.println(F("Unknown remote command."));
    return;
  }

#if DEBUG_PRINT
  Serial.print(F("Serial remote: "));
  Serial.print(command);
  Serial.print(F(", speed="));
  Serial.println(remoteSpeed);
#endif
}

// ============================================================
// 解析手机控制指令
//
// CMD,序号,STOP
// CMD,序号,MANUAL
// CMD,序号,AUTO
// CMD,序号,ESTOP
// ============================================================
bool handleCommandMessage(char* line) {
  char* token = strtok(line, ",");

  // 第一个字段必须是 CMD
  if (token == NULL ||
      strcmp(token, "CMD") != 0) {
    return false;
  }

  // 读取序号
  token = strtok(NULL, ",");

  if (token == NULL) {
    return false;
  }

  char* endPointer = NULL;

  const unsigned long sequence =
      strtoul(token, &endPointer, 10);

  // 序号必须完全由数字组成
  if (token[0] == '\0' ||
      endPointer == NULL ||
      *endPointer != '\0') {
    return false;
  }

  // 读取动作
  token = strtok(NULL, ",");

  if (token == NULL) {
    return false;
  }

  char* action = token;

  // 不允许出现多余字段
  if (strtok(NULL, ",") != NULL) {
    return false;
  }

  // ----------------------------------------------------------
  // STOP：停车锁定，但不改变当前手动/自动模式
  // ----------------------------------------------------------
  if (strcmp(action, "STOP") == 0) {
    commandStopActive = true;
    remoteAction = REMOTE_STOP;
    stopCar();  // 立即把左右电机输出清零
    requireRemoteNeutralRearm();
  }

  // ----------------------------------------------------------
  // MANUAL：进入手动模式
  //
  // MANUAL 同时作为软件 ESTOP 的安全解除动作。
  // 解除后仍要求手柄方向键先回到中位。
  // ----------------------------------------------------------
  else if (strcmp(action, "MANUAL") == 0) {
    softwareEmergencyActive = false;
    commandStopActive = false;

    manualModeActive = true;
    remoteAction = REMOTE_STOP;

    requireRemoteNeutralRearm();

  }

  // ----------------------------------------------------------
  // AUTO：进入自动模式
  //
  // 软件 ESTOP 锁定期间不允许直接进入 AUTO；
  // 必须先发送 MANUAL 完成安全解除。
  // ----------------------------------------------------------
  else if (strcmp(action, "AUTO") == 0) {
    if (softwareEmergencyActive) {
      Serial.println(
          F("CMD AUTO rejected: ESTOP latched; send MANUAL first"));

      return true;
    }

    commandStopActive = false;

    manualModeActive = false;
    remoteAction = REMOTE_STOP;

    // 进入自动模式后必须等待一条新的 TARGET，
    // 防止直接使用切换模式前的旧目标数据。
    hasReceivedTarget = false;

    resetAutoFollowConfirmation();
    requireRemoteNeutralRearm();
  }

  // ----------------------------------------------------------
  // ESTOP：软件急停锁定
  // ----------------------------------------------------------
  else if (strcmp(action, "ESTOP") == 0) {
    softwareEmergencyActive = true;
    commandStopActive = true;

    remoteAction = REMOTE_STOP;

    stopCar();  // 立即停车

    requireRemoteNeutralRearm();
  }

  else {
    return false;
  }

#if DEBUG_PRINT
  Serial.print(F("CMD: seq="));
  Serial.print(sequence);

  Serial.print(F(", action="));
  Serial.println(action);
#endif

  return true;
}

// OpenBot原有程序可能自动发送这些后台消息。
// 当前自定义控制只使用TARGET和CMD，因此全部忽略。
bool isOpenBotBackgroundMessage(const char* line) {
  const size_t length = strlen(line);

  // f：OpenBot功能查询
  if (strcmp(line, "f") == 0) {
    return true;
  }

  if (length > 1) {
    const char type = line[0];

    // h750、c0,0、s100、v250、w500等
    if (type == 'h' ||
        type == 'c' ||
        type == 's' ||
        type == 'v' ||
        type == 'w') {
      return true;
    }
  }

  return false;
}

void handleSerialLine(char* line) {
  // 手机端模式与安全控制
  if (startsWith(line, "CMD,")) {
    if (!handleCommandMessage(line)) {
      Serial.println(F("Invalid CMD message."));
    }

    return;
  }

  // 手机端目标跟随数据
  if (startsWith(line, "TARGET,")) {
    TargetData target;

    if (parseTargetMessage(line, target)) {
      latestTarget = target;
      hasReceivedTarget = true;
    } else {
      Serial.println(F("Invalid TARGET message."));
    }

    return;
  }

  // 忽略OpenBot原有后台协议，禁止其直接控制电机
  if (isOpenBotBackgroundMessage(line)) {
    return;
  }

  // 电脑串口单字符调试
  if (strlen(line) == 1) {
    handleRemoteCharacter(line[0]);
    return;
  }

#if DEBUG_PRINT
  Serial.print(F("Unknown message: "));
  Serial.println(line);
#endif
}

void readSerialInput() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';

        handleSerialLine(serialBuffer);

        serialIndex = 0;
      }

    } else if (serialIndex < sizeof(serialBuffer) - 1) {
      serialBuffer[serialIndex++] = c;

    } else {
      serialIndex = 0;

      Serial.println(F("Serial buffer overflow."));
    }
  }
}

#if I2C_REMOTE_ENABLED

// ============================================================
// I2C 遥控板底层通信
// ============================================================
bool checkRemoteBoard() {
  Wire.beginTransmission(REMOTE_I2C_ADDRESS);

  const uint8_t error = Wire.endTransmission();

  return error == 0;
}

// 按键低电平有效：对应位为0表示按下
bool buttonPressed(uint8_t value, uint8_t mask) {
  return (value & mask) == 0;
}

// 检测“刚刚按下”
bool buttonPressedEdge(
    uint8_t value,
    uint8_t previous,
    uint8_t mask) {

  return buttonPressed(value, mask) &&
         !buttonPressed(previous, mask);
}

bool packetChanged(const uint8_t* packet) {
  if (!hasPreviousPacket) {
    return true;
  }

  for (uint8_t i = 0;
       i < REMOTE_PACKET_LENGTH;
       ++i) {

    if (packet[i] != previousPacket[i]) {
      return true;
    }
  }

  return false;
}

void rememberPacket(const uint8_t* packet) {
  memcpy(
      previousPacket,
      packet,
      REMOTE_PACKET_LENGTH);

  hasPreviousPacket = true;
}

void printPacket(const uint8_t* packet) {
#if DEBUG_PRINT
  Serial.print(F("REMOTE RAW:"));

  for (uint8_t i = 0;
       i < REMOTE_PACKET_LENGTH;
       ++i) {

    Serial.print(' ');

    if (packet[i] < 0x10) {
      Serial.print('0');
    }

    Serial.print(packet[i], HEX);
  }

  Serial.println();
#else
  (void)packet;
#endif
}

// ============================================================
// 数据流重新组帧
//
// 标准数据包：
// FF 41 5A DPAD KEYS LX LY RX RY
//
// 转接板每次读取的起点不一定在FF处，
// 因此不能简单地把每次9字节都当成完整数据包。
// ============================================================
bool feedRemoteByte(
    uint8_t value,
    uint8_t* completedPacket) {

  // 等待第一个帧头字节：FF
  if (remoteFrameIndex == 0) {
    if (value == 0xFF) {
      remoteFrameBuffer[0] = value;
      remoteFrameIndex = 1;
    }

    return false;
  }

  // 等待第二个帧头字节：41
  if (remoteFrameIndex == 1) {
    if (value == 0x41) {
      remoteFrameBuffer[1] = value;
      remoteFrameIndex = 2;

    } else if (value == 0xFF) {
      // 连续出现FF，重新作为帧头起点
      remoteFrameBuffer[0] = 0xFF;
      remoteFrameIndex = 1;

    } else {
      remoteFrameIndex = 0;
    }

    return false;
  }

  // 等待第三个帧头字节：5A
  if (remoteFrameIndex == 2) {
    if (value == 0x5A) {
      remoteFrameBuffer[2] = value;
      remoteFrameIndex = 3;

    } else if (value == 0xFF) {
      remoteFrameBuffer[0] = 0xFF;
      remoteFrameIndex = 1;

    } else {
      remoteFrameIndex = 0;
    }

    return false;
  }

  // 收集剩余6字节
  remoteFrameBuffer[remoteFrameIndex++] = value;

  if (remoteFrameIndex >= REMOTE_PACKET_LENGTH) {
    memcpy(
        completedPacket,
        remoteFrameBuffer,
        REMOTE_PACKET_LENGTH);

    remoteFrameIndex = 0;

    return true;
  }

  return false;
}

// 本次没有拼出完整帧时返回false，
// 但这不代表遥控板已经断开。
bool readRemotePacket(uint8_t* packet) {
  Wire.requestFrom(
      (uint8_t)REMOTE_I2C_ADDRESS,
      (uint8_t)REMOTE_PACKET_LENGTH);

  bool completePacketFound = false;

  while (Wire.available() > 0) {
    const uint8_t value =
        (uint8_t)Wire.read();

    if (feedRemoteByte(value, packet)) {
      completePacketFound = true;
    }
  }

  return completePacketFound;
}

// ============================================================
// 解析完整遥控数据包
// ============================================================
void applyRemotePacket(const uint8_t* packet) {
  const unsigned long now = millis();

  const uint8_t dpad = packet[3];
  const uint8_t keys = packet[4];

  const uint8_t directionCode = dpad | 0x0F;
  const uint8_t faceCode = keys | 0x0F;
  const uint8_t previousFaceCode = previousKeys | 0x0F;

  // 完整有效帧到达：刷新真正的遥控链路看门狗。
  lastValidRemotePacketTime = now;
  hasValidRemotePacket = true;
  lastRemoteCommandTime = now;
  linkTimeoutMessagePrinted = false;

  // SELECT：进入自主模式，同时清除手动动作。
  if (buttonPressedEdge(dpad, previousDpad, 0x01)) {
    manualModeActive = false;
    remoteAction = REMOTE_STOP;
    remoteNeutralRearmRequired = true;
    Serial.println(F("Mode: AUTO"));
  }

  // START：进入遥控模式，但必须先松开方向键再解锁。
  if (buttonPressedEdge(dpad, previousDpad, 0x08)) {
    manualModeActive = true;
    remoteAction = REMOTE_STOP;
    remoteNeutralRearmRequired = true;
    remoteRearmMessagePrinted = false;
    Serial.println(F("Mode: REMOTE; release D-pad to arm"));
  }

  // Y：加速。
  if (faceCode == 0xEF &&
      previousFaceCode != 0xEF) {
    remoteSpeed = constrain(
        remoteSpeed + 10,
        MIN_SPEED,
        MAX_SPEED);

    Serial.print(F("Remote speed="));
    Serial.println(remoteSpeed);
  }

  // X：减速。
  if (faceCode == 0xDF &&
      previousFaceCode != 0xDF) {
    remoteSpeed = constrain(
        remoteSpeed - 10,
        MIN_SPEED,
        MAX_SPEED);

    Serial.print(F("Remote speed="));
    Serial.println(remoteSpeed);
  }

  // A：立即停车，并要求方向键先回到中位。
  const bool aHeld = (faceCode == 0xBF);
  if (aHeld) {
    manualModeActive = true;
    remoteAction = REMOTE_STOP;
    remoteNeutralRearmRequired = true;
    remoteRearmMessagePrinted = false;
    lastContinuousAction = REMOTE_STOP;
    continuousMotionStartedAt = 0;

    previousDpad = dpad;
    previousKeys = keys;
    return;
  }

  /*
   * 急停释放、A 停车、失联或连续运动超时后：
   * 必须先收到方向键完全松开的 FF，才允许再次运动。
   */
  if (remoteNeutralRearmRequired) {
    remoteAction = REMOTE_STOP;

    if (directionCode == 0xFF) {
      remoteNeutralRearmRequired = false;
      remoteRearmMessagePrinted = false;
      lastContinuousAction = REMOTE_STOP;
      continuousMotionStartedAt = 0;
      deadmanMessagePrinted = false;
      Serial.println(F("REMOTE REARMED"));
    } else if (!remoteRearmMessagePrinted) {
      Serial.println(F("REMOTE LOCKED: release D-pad"));
      remoteRearmMessagePrinted = true;
    }

    previousDpad = dpad;
    previousKeys = keys;
    return;
  }

  if (!manualModeActive) {
    previousDpad = dpad;
    previousKeys = keys;
    return;
  }

  RemoteAction requestedAction = REMOTE_STOP;

  switch (directionCode) {
    case 0xEF:
      requestedAction = REMOTE_FORWARD;
      break;

    case 0xBF:
      requestedAction = REMOTE_BACKWARD;
      break;

    case 0xCF:
      requestedAction = REMOTE_LEFT;
      break;

    case 0xDF:
      requestedAction = REMOTE_RIGHT;
      break;

    default:
      requestedAction = REMOTE_STOP;
      break;
  }

  if (requestedAction == REMOTE_STOP) {
    remoteAction = REMOTE_STOP;
    lastContinuousAction = REMOTE_STOP;
    continuousMotionStartedAt = 0;
    deadmanMessagePrinted = false;
  } else {
    // 换方向或从停止开始时，重新计算连续运动时间。
    if (requestedAction != lastContinuousAction) {
      lastContinuousAction = requestedAction;
      continuousMotionStartedAt = now;
      deadmanMessagePrinted = false;
    }

    /*
     * 若转接板在手柄断开后仍重复最后一帧，
     * 单纯依靠“完整帧超时”无法识别。
     * 因此增加最大连续运动时间，超时后强制停车并要求回中。
     */
    if (now - continuousMotionStartedAt >
        REMOTE_MAX_CONTINUOUS_MS) {
      remoteAction = REMOTE_STOP;
      remoteNeutralRearmRequired = true;
      remoteRearmMessagePrinted = false;

      if (!deadmanMessagePrinted) {
        Serial.println(
            F("DEADMAN TIMEOUT -> STOP; release D-pad"));
        deadmanMessagePrinted = true;
      }
    } else {
      remoteAction = requestedAction;
    }
  }

  previousDpad = dpad;
  previousKeys = keys;
}

// ============================================================
// 定期读取I2C遥控数据
// ============================================================
void readI2cRemote() {
  const unsigned long now = millis();

  // 尚未找到遥控板：每隔 1 秒重新检测
  if (!remoteBoardFound) {
    if (now - lastI2cRetryTime < 1000) {
      return;
    }

    lastI2cRetryTime = now;
    remoteBoardFound = checkRemoteBoard();

    if (remoteBoardFound) {
      Serial.println(F("I2C remote board FOUND at 0x19"));

      remoteFrameIndex = 0;
      hasPreviousPacket = false;
      previousDpad = 0xFF;
      previousKeys = 0xFF;
      hasValidRemotePacket = false;
      lastValidRemotePacketTime = 0;
      remoteAction = REMOTE_STOP;
      remoteNeutralRearmRequired = true;
      remoteRearmMessagePrinted = false;
      lastContinuousAction = REMOTE_STOP;
      continuousMotionStartedAt = 0;
      lastRemoteCommandTime = now;
    } else {
      remoteAction = REMOTE_STOP;
      lastRemoteCommandTime = now;

      Serial.println(F("I2C remote board NOT FOUND at 0x19"));
    }

    return;
  }

  // 控制 I2C 读取周期
  if (now - lastRemotePollTime < REMOTE_POLL_INTERVAL_MS) {
    return;
  }

  lastRemotePollTime = now;

  uint8_t packet[REMOTE_PACKET_LENGTH];
  bool packetReady = false;

  /*
   * 该转接板的 9 字节读取起点并不固定。
   * 一次 requestFrom() 没拼出完整帧是正常现象，
   * 不能立刻判定断线，也不能清空 remoteFrameIndex。
   *
   * 每个控制周期最多连续读取 3 组数据，
   * 提高持续按键及组合按键的响应速度。
   */
  for (uint8_t attempt = 0;
       attempt < 3 && !packetReady;
       ++attempt) {

    packetReady = readRemotePacket(packet);
  }

  if (!packetReady) {
    // 只有 I2C 设备本身确实不响应时才判定断线
    if (now - lastI2cHealthCheckTime >= 1000) {
      lastI2cHealthCheckTime = now;

      if (!checkRemoteBoard()) {
        remoteBoardFound = false;
        remoteFrameIndex = 0;
        hasPreviousPacket = false;
  
        remoteAction = REMOTE_STOP;
        hasValidRemotePacket = false;
        lastValidRemotePacketTime = 0;
        remoteNeutralRearmRequired = true;
        remoteRearmMessagePrinted = false;
        lastContinuousAction = REMOTE_STOP;
        continuousMotionStartedAt = 0;
        lastRemoteCommandTime = now;

        Serial.println(F("I2C remote disconnected -> STOP"));
      }
    }

    return;
  }

  if (packetChanged(packet)) {
    printPacket(packet);
    rememberPacket(packet);
  }

  applyRemotePacket(packet);
}

#endif  // I2C_REMOTE_ENABLED

}  // namespace

// ============================================================
// 初始化遥控模块
// ============================================================
void setupRemoteControl() {
  Serial.println(F("REMOTE CODE VERSION V8 REARM+DEADMAN"));
#if I2C_REMOTE_ENABLED

  // Arduino UNO：
  // SDA = A4
  // SCL = A5
  Wire.begin();
  Wire.setClock(100000UL);

  #if defined(WIRE_HAS_TIMEOUT)
  // I2C总线被干扰拉死时自动超时复位
  Wire.setWireTimeout(25000, true);
  #endif

  delay(50);

  remoteBoardFound =
      checkRemoteBoard();

  if (remoteBoardFound) {
    Serial.println(
        F("I2C remote board FOUND at 0x19"));
  } else {
    Serial.println(
        F("I2C remote board NOT FOUND at 0x19"));
  }

#endif
}

// ============================================================
// 主程序每轮调用
// ============================================================
void readSerialCommunication() {
  readSerialInput();

#if I2C_REMOTE_ENABLED
  readI2cRemote();
#endif
}

// ============================================================
// 外部安全模块调用：急停、失联后要求方向键先回中
// ============================================================
void requireRemoteNeutralRearm() {
  remoteAction = REMOTE_STOP;
  lastRemoteCommandTime = millis();

#if I2C_REMOTE_ENABLED
  remoteNeutralRearmRequired = true;
  remoteRearmMessagePrinted = false;
  lastContinuousAction = REMOTE_STOP;
  continuousMotionStartedAt = 0;
  deadmanMessagePrinted = false;
#endif
}

// ============================================================
// 根据遥控动作生成左右履带速度
// ============================================================
MotionCommand computeRemoteCommand() {
  MotionCommand command;

  const int speed =
      constrain(remoteSpeed, 0, MAX_SPEED);

  command.reason = "remote control";

  if (remoteAction == REMOTE_FORWARD) {
    command.leftSpeed = speed;
    command.rightSpeed = speed;

  } else if (remoteAction == REMOTE_BACKWARD) {
    command.leftSpeed = -speed;
    command.rightSpeed = -speed;

  } else if (remoteAction == REMOTE_LEFT) {
    command.leftSpeed = -speed;
    command.rightSpeed = speed;

  } else if (remoteAction == REMOTE_RIGHT) {
    command.leftSpeed = speed;
    command.rightSpeed = -speed;

  } else {
    command.leftSpeed = 0;
    command.rightSpeed = 0;
    command.reason = "remote stop";
  }

  return command;
}

// ============================================================
// 遥控命令超时判断
// ============================================================
bool isRemoteCommandTimedOut(unsigned long now) {
  if (!manualModeActive ||
      remoteAction == REMOTE_STOP) {
    return false;
  }

#if I2C_REMOTE_ENABLED
  if (!hasValidRemotePacket ||
      now - lastValidRemotePacketTime >
          REMOTE_FAILSAFE_MS) {

    remoteAction = REMOTE_STOP;
    remoteNeutralRearmRequired = true;
    remoteRearmMessagePrinted = false;
    lastContinuousAction = REMOTE_STOP;
    continuousMotionStartedAt = 0;

    if (!linkTimeoutMessagePrinted) {
      Serial.println(
          F("REMOTE LINK TIMEOUT -> STOP; release D-pad"));
      linkTimeoutMessagePrinted = true;
    }

    return true;
  }
#endif

  return now - lastRemoteCommandTime >
         REMOTE_TIMEOUT_MS;
}