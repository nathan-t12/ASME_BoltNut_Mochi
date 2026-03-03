// 套件依賴
#include <Arduino.h>
#include <Servo.h>


// 自訂的函式庫
#include "Hardware_Layer.h"
#include "Math_Layer.h" 
#include "Comms_Layer.h"
#include "config.h"


const int motorPWM = 9;
const int motorDirection =8;
const int motorDirection2 =7;
const int STBY =6;

long count =0;
void motorCNT() {
  count++;
}

void setup() {

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(2), motorCNT, RISING); 
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDirection, OUTPUT);
  pinMode(motorDirection2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(2, INPUT_PULLUP);

  initTimer1_20ms();
  PID_Controller pid(PidConfig::PID_KP, PidConfig::PID_KI, PidConfig::PID_KD);
}

void loop() {

  digitalWrite(STBY,HIGH);
  digitalWrite(motorDirection,HIGH);
  digitalWrite(motorDirection2,LOW);
  delay(2000);
  analogWrite(motorPWM, 100);
  if (timerFlag){
    timerFlag = false;
    Serial.println(count);
  }
}

