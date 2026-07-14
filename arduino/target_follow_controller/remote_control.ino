#include "config.h"

namespace {
char serialBuffer[100];
int serialIndex = 0;

bool startsWith(const char* text, const char* prefix) {
  while (*prefix) {
    if (*text++ != *prefix++) {
      return false;
    }
  }
  return true;
}

void handleRemoteCharacter(char command) {
  command = (char)tolower(command);

  if (command == 'a') {
    manualModeActive = false;
    remoteAction = REMOTE_STOP;
    Serial.println("Mode: AUTO");
    return;
  }

  manualModeActive = true;
  lastRemoteCommandTime = millis();

  if (command == 'f') {
    remoteAction = REMOTE_FORWARD;
  } else if (command == 'b') {
    remoteAction = REMOTE_BACKWARD;
  } else if (command == 'l') {
    remoteAction = REMOTE_LEFT;
  } else if (command == 'r') {
    remoteAction = REMOTE_RIGHT;
  } else if (command == 's') {
    remoteAction = REMOTE_STOP;
  } else {
    Serial.println("Unknown remote command.");
    return;
  }

#if DEBUG_PRINT
  Serial.print("Remote command: ");
  Serial.println(command);
#endif
}

void handleSerialLine(char* line) {
  if (startsWith(line, "TARGET,")) {
    TargetData target;
    if (parseTargetMessage(line, target)) {
      latestTarget = target;
      hasReceivedTarget = true;
    } else {
      Serial.println("Invalid TARGET message.");
    }
    return;
  }

  // V1 遥控：串口发送单字符并换行
  // f=前进，b=后退，l=左转，r=右转，s=停止，a=返回自动模式
  if (strlen(line) == 1) {
    handleRemoteCharacter(line[0]);
    return;
  }

  Serial.print("Unknown message: ");
  Serial.println(line);
}
}  // namespace

void setupRemoteControl() {
  // 当前使用 Arduino 主串口，无额外引脚初始化。
}

void readSerialCommunication() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        handleSerialLine(serialBuffer);
        serialIndex = 0;
      }
    } else if (serialIndex < (int)sizeof(serialBuffer) - 1) {
      serialBuffer[serialIndex++] = c;
    } else {
      serialIndex = 0;
      Serial.println("Serial buffer overflow. Message dropped.");
    }
  }
}

MotionCommand computeRemoteCommand() {
  MotionCommand cmd;
  cmd.reason = "remote control";

  const int speed = constrain(remoteSpeed, 0, MAX_SPEED);

  if (remoteAction == REMOTE_FORWARD) {
    cmd.leftSpeed = speed;
    cmd.rightSpeed = speed;
  } else if (remoteAction == REMOTE_BACKWARD) {
    cmd.leftSpeed = -speed;
    cmd.rightSpeed = -speed;
  } else if (remoteAction == REMOTE_LEFT) {
    cmd.leftSpeed = -speed;
    cmd.rightSpeed = speed;
  } else if (remoteAction == REMOTE_RIGHT) {
    cmd.leftSpeed = speed;
    cmd.rightSpeed = -speed;
  } else {
    cmd.leftSpeed = 0;
    cmd.rightSpeed = 0;
    cmd.reason = "remote stop";
  }

  return cmd;
}

bool isRemoteCommandTimedOut(unsigned long now) {
  if (!manualModeActive || remoteAction == REMOTE_STOP) {
    return false;
  }
  return now - lastRemoteCommandTime > REMOTE_TIMEOUT_MS;
}
