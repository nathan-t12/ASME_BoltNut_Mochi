#ifndef HARDWARE_LAYER_H
#define HARDWARE_LAYER_H
#include "Math_Layer.h"
#include <Arduino.h>
#pragma once

extern volatile bool timerFlag; 

void initTimer1_20ms(); // 初始化函式的宣告

class Motor{
    public:
        Motor(float Kp, float Ki, float Kd, uint8_t pwmPin, uint8_t dirPin1, uint8_t dirPin2, uint8_t encoderPin1, uint8_t encoderPin2);
        void setSpeed(int16_t speed);
        void updateEncoder();
        void encoderISR();
    private:
        uint8_t pwmPin;
        uint8_t dirPin1;
        uint8_t dirPin2;
        uint8_t encoderPin1;
        uint8_t encoderPin2;
        PID_Controller incrementPID;
        volatile uint16_t encoderCount = 0;
        int16_t currentSpeed = 0;
};

#endif