#include "Math_Layer.h"
#include "config.h"

PID_Controller::PID_Controller(float p, float i, float d) 
    : Kp(static_cast<int16_t>(p * PidConfig::SCALE_FACTOR))
    , Ki(static_cast<int16_t>(i * PidConfig::SCALE_FACTOR))
    , Kd(static_cast<int16_t>(d * PidConfig::SCALE_FACTOR))
    , integral(0)
    , previousError(0)
    , lastTime(0)
{
}

int16_t PID_Controller::compute(int16_t setpoint, int16_t input) {
    int16_t error = setpoint - input;

    // 比例項：位元右移代替除法
    int32_t pTerm = (int32_t)Kp * error >> PidConfig::SCALE_SHIFT;

    // 積分項
    integral += (int32_t)Ki * error;
    // 積分限制
    if (integral >  PidConfig::INTEGRAL_LIMIT) integral =  PidConfig::INTEGRAL_LIMIT;
    if (integral < -PidConfig::INTEGRAL_LIMIT) integral = -PidConfig::INTEGRAL_LIMIT;
    int32_t iTerm = integral >> PidConfig::SCALE_SHIFT;

    // 微分項
    int16_t dTerm = (int32_t)Kd * (error - previousError) >> PidConfig::SCALE_SHIFT;

    previousError = error;

    // 加總並限制輸出
    int32_t output = pTerm + iTerm + dTerm;
    if (output >  PidConfig::OUTPUT_MAX) output =  PidConfig::OUTPUT_MAX;
    if (output < -PidConfig::OUTPUT_MIN) output =  PidConfig::OUTPUT_MIN;

    return static_cast<int16_t>(output);
}

void PID_Controller::reset() {
    integral = 0;
    previousError = 0;
    lastTime = 0;
}