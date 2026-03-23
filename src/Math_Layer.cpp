#include "Math_Layer.h"
#include "config.h"


Filter::Filter(uint16_t alphaFixed) 
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

ServoInputFilter::ServoInputFilter(uint16_t initialValue)
    : filteredValue(initialValue)
{
}

uint16_t ServoInputFilter::updateWithEndpointSnap(uint16_t inputValue,
                                                  uint16_t minValue,
                                                  uint16_t maxValue,
                                                  uint8_t filterShift,
                                                  uint8_t endpointBand) {
    if (inputValue <= static_cast<uint16_t>(minValue + endpointBand)) {
        filteredValue = minValue;
        return filteredValue;
    }

    if (inputValue >= static_cast<uint16_t>(maxValue - endpointBand)) {
        filteredValue = maxValue;
        return filteredValue;
    }

    int16_t delta = static_cast<int16_t>(inputValue) - static_cast<int16_t>(filteredValue);
    filteredValue += static_cast<int16_t>(delta >> filterShift);
    return filteredValue;
}

void ServoInputFilter::reset(uint16_t value) {
    filteredValue = value;
}

uint8_t mapServo2Lookup(uint16_t chValue) {
    // Lookup table with focus on endpoint certainty.
    static constexpr uint16_t kInputTable[] = {1000, 1200, 1400, 1600, 1800, 2000};
    static constexpr uint8_t kAngleTable[] = {180, 170, 145, 105, 55, ServoMotor::SERVO2_MIN_ANGLE};

    if (chValue <= kInputTable[0]) {
        return kAngleTable[0];
    }
    if (chValue >= kInputTable[5]) {
        return kAngleTable[5];
    }

    for (uint8_t i = 0; i < 5; ++i) {
        uint16_t x0 = kInputTable[i];
        uint16_t x1 = kInputTable[i + 1];
        if (chValue <= x1) {
            uint8_t y0 = kAngleTable[i];
            uint8_t y1 = kAngleTable[i + 1];
            uint16_t dx = x1 - x0;
            uint16_t ox = chValue - x0;
            uint8_t angle = static_cast<uint8_t>(
                y0 + (static_cast<int16_t>(y1) - static_cast<int16_t>(y0)) * static_cast<int16_t>(ox) / static_cast<int16_t>(dx));
            return angle;
        }
    }

    return kAngleTable[5];
}

uint8_t mapServo3CenterPeak(uint16_t chValue) {
    // Piecewise mapping: INPUT_MIN -> LEFT_ANGLE, CENTER_INPUT -> PEAK_ANGLE, INPUT_MAX -> RIGHT_ANGLE
    uint16_t clamped = constrain(chValue, ServoMotor::INPUT_MIN, ServoMotor::INPUT_MAX);
    if (clamped <= ServoMotor::SERVO3_CENTER_INPUT) {
        return static_cast<uint8_t>(
            ServoMotor::SERVO3_LEFT_ANGLE +
            (static_cast<uint32_t>(clamped - ServoMotor::INPUT_MIN) *
             (ServoMotor::SERVO3_PEAK_ANGLE - ServoMotor::SERVO3_LEFT_ANGLE)) /
            (ServoMotor::SERVO3_CENTER_INPUT - ServoMotor::INPUT_MIN));
    }
    return static_cast<uint8_t>(
        ServoMotor::SERVO3_PEAK_ANGLE -
        (static_cast<uint32_t>(clamped - ServoMotor::SERVO3_CENTER_INPUT) *
         (ServoMotor::SERVO3_PEAK_ANGLE - ServoMotor::SERVO3_RIGHT_ANGLE)) /
        (ServoMotor::INPUT_MAX - ServoMotor::SERVO3_CENTER_INPUT));
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