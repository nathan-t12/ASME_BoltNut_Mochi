#include "Math_Layer.h"
#include "config.h"


Filter::Filter(uint8_t alphaFixed) 
    : alphaFixed(alphaFixed)
    , previousValue(0) 
{
}

int16_t Filter::update(int16_t newValue) {
    int32_t result = ((int32_t)alphaFixed * newValue 
                   + (int32_t)(FilterConfig::SCALE_FACTOR - alphaFixed) * previousValue) 
                   >> FilterConfig::SCALE_SHIFT;

    previousValue = static_cast<int16_t>(result);
    return previousValue;
}


void Filter::reset() {
    previousValue = 0;
}

PID_Controller::PID_Controller(float p, float i, float d) 
    : Kp(static_cast<int16_t>(p * PidConfig::SCALE_FACTOR))
    , Ki(static_cast<int32_t>(i * PidConfig::SCALE_FACTOR * PidConfig::timeStep))
    , Kd(static_cast<int32_t>(d * PidConfig::SCALE_FACTOR / PidConfig::timeStep))
    , previousError1(0)
    , previousError2(0)
    , previousOutput(0)
{
}

int16_t PID_Controller::compute(int16_t setpoint, int16_t input) {
    int16_t error = setpoint - input;

    int64_t delta = (int32_t)Kp *(error - previousError1)
     + (int32_t)Ki * error 
     + (int32_t)Kd * (error - 2 * previousError1 + previousError2);

    int16_t output = previousOutput + static_cast<int16_t>(delta >> PidConfig::SCALE_SHIFT);
    
    output = constrain(output, PidConfig::OUTPUT_MIN, PidConfig::OUTPUT_MAX);

    previousError2 = previousError1;
    previousError1 = error;
    previousOutput = output;

    return static_cast<int16_t>(output);
}

void PID_Controller::reset() {
    previousError1 = 0;
    previousError2 = 0;
    previousOutput = 0;
}