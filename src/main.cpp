// ========== PID & Filter 測試模式 ==========
// 用 Serial Plotter 觀察波形（115200 baud）
// 上傳後打開 Arduino IDE → Tools → Serial Plotter

#include <Arduino.h>
#include "Math_Layer.h"
#include "config.h"

// --- 選擇測試模式（改這裡切換）---
// 0 = Filter 階躍響應
// 1 = Filter 濾雜訊
// 2 = PID 階躍追蹤（模擬一階系統）
// 3 = PID 方波追蹤
#define TEST_MODE 2

Filter filter(FilterConfig::ALPHA_FIXED);
PID_Controller pid(PidConfig::PID_KP, PidConfig::PID_KI, PidConfig::PID_KD);

uint32_t lastTime = 0;
const uint16_t INTERVAL_MS = 20; // 20ms = 50Hz，與 PID timeStep 一致
uint32_t step = 0;

// 模擬的「馬達」位置（一階系統）
float simPosition = 0.0f;

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(A0));
}

void loop() {
  uint32_t now = millis();
  if (now - lastTime < INTERVAL_MS) return;
  lastTime = now;
  step++;

#if TEST_MODE == 0
  // ===== Filter 階躍響應 =====
  // 在 step=50 時從 0 跳到 500
  int16_t raw = (step > 50) ? 500 : 0;
  int16_t filtered = filter.update(raw);

  Serial.print("Raw:");
  Serial.print(raw);
  Serial.print(",Filtered:");
  Serial.println(filtered);

  // 300 步後重來
  if (step > 300) { step = 0; filter.reset(); }

#elif TEST_MODE == 1
  // ===== Filter 濾雜訊 =====
  // 真實值 200，加上 ±50 的隨機雜訊
  int16_t trueVal = 200;
  int16_t noise = random(-50, 51);
  int16_t raw = trueVal + noise;
  int16_t filtered = filter.update(raw);

  Serial.print("True:");
  Serial.print(trueVal);
  Serial.print(",Raw:");
  Serial.print(raw);
  Serial.print(",Filtered:");
  Serial.println(filtered);

#elif TEST_MODE == 2
  // ===== PID 階躍追蹤（模擬一階系統）=====
  // setpoint 在 step=50 跳到 200
  int16_t setpoint = (step > 50) ? 200 : 0;

  int16_t feedback = (int16_t)simPosition;
  int16_t output = pid.compute(setpoint, feedback);

  // 模擬一階慣性系統：position += gain * output * dt
  // gain=0.05 讓系統不會瞬間到位，能看到 PID 的動態行為
  simPosition += 0.05f * output;
  simPosition = constrain(simPosition, -500.0f, 500.0f);

  Serial.print("Setpoint:");
  Serial.print(setpoint);
  Serial.print(",Position:");
  Serial.print(feedback);
  Serial.print(",Output:");
  Serial.println(output);

  // 500 步後重來
  if (step > 500) { step = 0; pid.reset(); simPosition = 0; }

#elif TEST_MODE == 3
  // ===== PID 方波追蹤 =====
  // 每 200 步切換 setpoint：0 ↔ 150
  int16_t setpoint = ((step / 200) % 2 == 0) ? 150 : 0;

  int16_t feedback = (int16_t)simPosition;
  int16_t output = pid.compute(setpoint, feedback);

  simPosition += 0.05f * output;
  simPosition = constrain(simPosition, -500.0f, 500.0f);

  Serial.print("Setpoint:");
  Serial.print(setpoint);
  Serial.print(",Position:");
  Serial.print(feedback);
  Serial.print(",Output:");
  Serial.println(output);

#endif
}

