#ifndef HARDWARE_LAYER_H
#define HARDWARE_LAYER_H
#pragma once
#include <Arduino.h>
#include "Math_Layer.h"

extern volatile bool timerFlag; 

void initControlTimer20ms(); // 初始化函式的宣告

class Motor{
    public:
        Motor(float Kp, float Ki, float Kd, uint8_t pwmPin, uint8_t dirPin1, uint8_t dirPin2, uint8_t encoderPin1, uint8_t encoderPin2);
        void setSpeed(int16_t speed);
        void updateEncoder(int16_t target);
        void updateFeedbackOnly();
        void stopAndResetControl();
        void begin();
        void encoderISR();
        int16_t getCurrentSpeed() const { return currentSpeed; }
        int16_t getLastPWM()      const { return lastPWM; } 
        volatile int16_t encoderCount = 0;
    private:
        uint8_t pwmPin;
        uint8_t dirPin1;
        uint8_t dirPin2;
        uint8_t encoderPin1;
        uint8_t encoderPin2;
        PID_Controller incrementPID;
        int16_t currentSpeed = 0;
        int16_t lastPWM = 0;
        Filter encoderFilter;
};

#endif