#include "config.h"

// 供其他控制文件读取的电池状态
float latestBatteryVoltage = -1.0f;
bool batteryVoltageValid = false;

namespace {

const uint8_t BATTERY_AVERAGE_SAMPLES = 8;
const float ADC_REFERENCE_VOLTAGE = 5.0f;

unsigned long lastBatterySampleTime = 0;
unsigned long lastBatteryPrintTime = 0;

float readBatteryVoltageOnce() {
  // 丢弃第一次读取，减少模拟通道切换造成的误差
  analogRead(BATTERY_VOLTAGE_PIN);

  unsigned long adcSum = 0;

  for (uint8_t i = 0;
       i < BATTERY_AVERAGE_SAMPLES;
       ++i) {

    adcSum += analogRead(BATTERY_VOLTAGE_PIN);
    delayMicroseconds(200);
  }

  const float averageAdc =
      (float)adcSum / BATTERY_AVERAGE_SAMPLES;

  // UNO ADC：0～1023 对应约 0～5V
  const float moduleOutputVoltage =
      averageAdc *
      ADC_REFERENCE_VOLTAGE /
      1023.0f;

  // 恢复为分压前的电池电压
  return moduleOutputVoltage *
         BATTERY_DIVIDER_RATIO *
         BATTERY_CALIBRATION;
}

}  // namespace


void setupBatteryMonitor() {
#if BATTERY_MONITOR_ENABLED
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);

  latestBatteryVoltage = -1.0f;
  batteryVoltageValid = false;

#if DEBUG_PRINT
  Serial.println(F("Battery monitor enabled."));
#endif
#endif
}


void updateBatteryMonitor(unsigned long now) {
#if BATTERY_MONITOR_ENABLED
  if (now - lastBatterySampleTime <
      BATTERY_SAMPLE_INTERVAL_MS) {
    return;
  }

  lastBatterySampleTime = now;

  const float newVoltage =
      readBatteryVoltageOnce();

  if (!batteryVoltageValid) {
    latestBatteryVoltage = newVoltage;
    batteryVoltageValid = true;
  } else {
    /*
     * 指数滤波：
     * 75% 使用上一结果，25% 使用本次结果，
     * 减少电机噪声导致的数值跳动。
     */
    latestBatteryVoltage =
        latestBatteryVoltage * 0.75f +
        newVoltage * 0.25f;
  }

#if DEBUG_PRINT
  // 每秒打印一次，避免占用过多串口带宽
  if (now - lastBatteryPrintTime >= 1000) {
    lastBatteryPrintTime = now;

    Serial.print(F("battery_v="));
    Serial.println(latestBatteryVoltage, 2);
  }
#endif

#else
  (void)now;
#endif
}


bool isBatteryVoltageValid() {
  return batteryVoltageValid;
}


float getBatteryVoltage() {
  return latestBatteryVoltage;
}