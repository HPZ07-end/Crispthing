#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

// 串口参数
const unsigned long SERIAL_BAUD_RATE = 9600;

// 接收缓冲区
const size_t SERIAL_BUFFER_SIZE = 96;
char serialBuffer[SERIAL_BUFFER_SIZE];
size_t serialBufferIndex = 0;

// 新 TARGET 协议对应的数据结构
struct TargetData {
  unsigned long sequence;
  bool valid;
  float xError;
  float distanceCm;
  float similarity;
};

// 解析：
// TARGET,序号,目标有效,x偏差,距离,相似度
bool parseTargetMessage(char* line, TargetData& target) {
  char* token = strtok(line, ",");

  // 消息类型
  if (token == NULL || strcmp(token, "TARGET") != 0) {
    return false;
  }

  // 序号
  token = strtok(NULL, ",");
  if (token == NULL) {
    return false;
  }
  target.sequence = strtoul(token, NULL, 10);

  // 目标是否有效
  token = strtok(NULL, ",");
  if (token == NULL) {
    return false;
  }

  const int validValue = atoi(token);

  if (validValue != 0 && validValue != 1) {
    return false;
  }

  target.valid = (validValue == 1);

  // x 偏差
  token = strtok(NULL, ",");
  if (token == NULL) {
    return false;
  }
  target.xError = atof(token);

  // 距离，单位 cm
  token = strtok(NULL, ",");
  if (token == NULL) {
    return false;
  }
  target.distanceCm = atof(token);

  // 相似度
  token = strtok(NULL, ",");
  if (token == NULL) {
    return false;
  }
  target.similarity = atof(token);

  // 不允许额外字段
  if (strtok(NULL, ",") != NULL) {
    return false;
  }

  // 范围检查
  if (target.xError < -1.0f || target.xError > 1.0f) {
    return false;
  }

  if (target.similarity < 0.0f ||
      target.similarity > 1.0f) {
    return false;
  }

  // 距离允许 -1 表示无效
  if (target.distanceCm < 0.0f &&
      target.distanceCm != -1.0f) {
    return false;
  }

  return true;
}

void printTargetData(const TargetData& target) {
  Serial.println("TARGET parsed successfully");

  Serial.print("sequence: ");
  Serial.println(target.sequence);

  Serial.print("valid: ");
  Serial.println(target.valid ? 1 : 0);

  Serial.print("xError: ");
  Serial.println(target.xError, 3);

  Serial.print("distanceCm: ");
  Serial.println(target.distanceCm, 2);

  Serial.print("similarity: ");
  Serial.println(target.similarity, 3);

  Serial.println("--------------------");
}

void handleSerialLine(char* line) {
  TargetData target;

  if (parseTargetMessage(line, target)) {
    printTargetData(target);
  } else {
    Serial.print("Invalid TARGET message: ");
    Serial.println(line);
  }
}

void readSerialLines() {
  while (Serial.available() > 0) {
    const char incomingChar = Serial.read();

    // 忽略 Windows 风格换行中的 \r
    if (incomingChar == '\r') {
      continue;
    }

    if (incomingChar == '\n') {
      if (serialBufferIndex > 0) {
        serialBuffer[serialBufferIndex] = '\0';
        handleSerialLine(serialBuffer);
        serialBufferIndex = 0;
      }
      continue;
    }

    // 防止缓冲区溢出
    if (serialBufferIndex < SERIAL_BUFFER_SIZE - 1) {
      serialBuffer[serialBufferIndex++] = incomingChar;
    } else {
      serialBufferIndex = 0;
      Serial.println("Serial buffer overflow");
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  Serial.println("TARGET parser test ready");
  Serial.println(
      "Format: TARGET,seq,valid,xError,distanceCm,similarity");
}

void loop() {
  readSerialLines();
}