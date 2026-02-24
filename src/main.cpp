// 套件依賴
#include <Arduino.h>
#include <Servo.h>

// 自訂的函式庫
#include "Hardware_Layer.h"
#include "Math_Layer.h" 
#include "Comms_Layer.h"


void setup() {

  Serial.begin(115200);
  initTimer1_20ms();

}

void loop() {
  if (timerFlag){
    timerFlag = false;
    Serial.println(millis());

  }
}

