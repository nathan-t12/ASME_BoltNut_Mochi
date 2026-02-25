#ifndef MATH_LAYER_H
#define MATH_LAYER_H
#pragma once
#include <Arduino.h>

class PID_Controller {
public:
    PID_Controller(float p, float i, float d);
    int16_t compute(int16_t setpoint, int16_t input);
    void reset();

private:
    // 縮放後的整數增益
    int16_t Kp;
    int16_t Ki;
    int16_t Kd;

    int32_t integral;       // 積分項需要較大範圍
    int16_t previousError;
    unsigned long lastTime;
};

#endif