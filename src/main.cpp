#include <Arduino.h>
#include "Math_Layer.h"
#include "Hardware_Layer.h"
#include "Comms_Layer.h"
#include "config.h"

#include <IBusBM.h>
#include <Servo.h>
//Motor(Kp,Ki,Kd,pwmPin,dirPin1,dirPin2,encoderPin1,encoderPin2)
IBusBM ibus;
uint32_t lastStatusMs = 0;

void setup() {
  Serial.begin(115200);
  ibus.begin(Serial2);
  delay(100);
  Serial.println("BOOT: serial monitor ready");
}


void loop() {
  uint16_t ch1 = ibus.readChannel(1);
  if (ch1 > 0) {
    int16_t targetCount = mapC1ToTargetCountLUT(ch1);

    Serial.print("CH0:");
    Serial.print(ibus.readChannel(0));
    Serial.print(",CH1:");
    Serial.print(ch1);
    Serial.print(",CH2:");
    Serial.print(ibus.readChannel(2));
    Serial.print(",CH3:");
    Serial.print(ibus.readChannel(3));
    Serial.print(",CH4:");
    Serial.print(ibus.readChannel(4));
    Serial.print(",CH5:");
    Serial.print(ibus.readChannel(5));
    Serial.print(",TargetCount:");
    Serial.println(targetCount);
  } else if (millis() - lastStatusMs >= 500) {
    lastStatusMs = millis();
    Serial.println("Waiting for iBus signal...");
  }
  delay(20);
}

