// 套件依賴
#include <Arduino.h>
#include <Servo.h>


// 自訂的函式庫
#include "Hardware_Layer.h"
#include "Math_Layer.h" 
#include "Comms_Layer.h"
#include "config.h"



//Motor(Kp, Ki, Kd, pwmPin, dirPin1, dirPin2, encoderPin)

void setup() {

  Serial.begin(115200);

  initTimer1_20ms();
  PID_Controller pid(PidConfig::PID_KP, PidConfig::PID_KI, PidConfig::PID_KD);
}

void loop() {


  if (timerFlag){
    timerFlag = false;
    //update encoder
  }
}

