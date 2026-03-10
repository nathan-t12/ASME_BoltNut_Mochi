#include <Arduino.h>
#include "Math_Layer.h"
#include "Hardware_Layer.h"
#include "Comms_Layer.h"
#include "config.h"


//Motor(Kp,Ki,Kd,pwmPin,dirPin1,dirPin2,encoderPin1,encoderPin2)

Motor motor1(5.0,0,0,9,8,7,2,3);

void motor1EncoderISR() {
    motor1.encoderISR();
}
void setup() {
  Serial.begin(115200);
  // randomSeed(analogRead(A0));
  motor1.begin();
  initTimer1_20ms();
  attachInterrupt(digitalPinToInterrupt(2), motor1EncoderISR, RISING);
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH); // STBY
}

int16_t totalCount = 0;
uint8_t sampleCount = 0;

void loop() {
  if (timerFlag) {
    timerFlag = false;
    motor1.setSpeed(33);

    noInterrupts();
    int16_t count = motor1.encoderCount;
    motor1.encoderCount = 0;
    interrupts();

    totalCount += count;
    sampleCount++;

    // ✅ 每 10 個週期 (200ms) 取平均
    if (sampleCount >= 10) {
        Serial.print("Avg Speed: ");
        Serial.println(totalCount / sampleCount);
        totalCount = 0;
        sampleCount = 0;
    }
  }
}

