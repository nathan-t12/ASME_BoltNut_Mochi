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
    int32_t Ki;
    int32_t Kd;

    // 積分項需要較大範圍      
    int16_t previousError1;
    int16_t previousError2;
    int16_t previousOutput;
};

#endif