#include <Arduino.h>
#include <Servo.h>
#include "Math_Layer.h"
#include "Hardware_Layer.h"
#include "Comms_Layer.h"
#include "config.h"
#include <IBusBM.h>
//Motor(Kp,Ki,Kd,pwmPin,dirPin1,dirPin2,encoderPin1,encoderPin2)
IBusBM ibus;

// 0: 閉迴路 (C1 -> target count -> PID)
// 1: 開迴路 (C1 -> 直接 PWM)
#define MOTOR_TEST_MODE 1

Motor motor1(PidConfig::PID_KP, PidConfig::PID_KI, PidConfig::PID_KD,
             MotorPins::M1_PWM_PIN, MotorPins::M1_DIR_PIN1, MotorPins::M1_DIR_PIN2, MotorPins::M1_ENCODER_PIN1, MotorPins::M1_ENCODER_PIN2);

Motor motor2(PidConfig::PID_KP, PidConfig::PID_KI, PidConfig::PID_KD,
             MotorPins::M2_PWM_PIN, MotorPins::M2_DIR_PIN1, MotorPins::M2_DIR_PIN2, MotorPins::M2_ENCODER_PIN1, MotorPins::M2_ENCODER_PIN2);

Motor motor3(PidConfig::PID_KP, PidConfig::PID_KI, PidConfig::PID_KD,
             MotorPins::M3_PWM_PIN, MotorPins::M3_DIR_PIN1, MotorPins::M3_DIR_PIN2, MotorPins::M3_ENCODER_PIN1, MotorPins::M3_ENCODER_PIN2);

Motor motor4(PidConfig::PID_KP, PidConfig::PID_KI, PidConfig::PID_KD,
             MotorPins::M4_PWM_PIN, MotorPins::M4_DIR_PIN1, MotorPins::M4_DIR_PIN2, MotorPins::M4_ENCODER_PIN1, MotorPins::M4_ENCODER_PIN2);

Servo servo1;
Servo servo2;
Servo servo3;

void motor1EncoderISR() {
    motor1.encoderISR();
}

void motor2EncoderISR() {
    motor2.encoderISR();
}

void motor3EncoderISR() {
    motor3.encoderISR();
}

void motor4EncoderISR() {
    motor4.encoderISR();
}

uint32_t lastPrintMs = 0;
uint16_t lastCh1 = CommsMapConfig::C1_CENTER;
int16_t lastTargetCount = 0;
int16_t lastPwmCmd = 0;
int16_t lastSpeedOL1 = 0;
int16_t lastSpeedOL2 = 0;
int16_t lastSpeedOL3 = 0;
int16_t lastSpeedOL4 = 0;
uint8_t lastServo1Angle = ServoMotor::SERVO1_MIN_ANGLE;
uint8_t lastServo2Angle = ServoMotor::SERVO2_MIN_ANGLE;
uint8_t lastServo3Angle = ServoMotor::SERVO3_MIN_ANGLE;

constexpr uint16_t SERVO_INPUT_CENTER = (ServoMotor::INPUT_MIN + ServoMotor::INPUT_MAX) / 2;
constexpr uint8_t SERVO_DISCRETE_HYST_BAND = 35;
constexpr uint8_t SERVO_SWITCH_CONFIRM_COUNT = 2;
constexpr uint8_t SERVO_FILTER_SHIFT_S1_S3 = ServoMotor::FILTER_SHIFT;
constexpr uint8_t SERVO_FILTER_SHIFT_S2 = 1;
constexpr uint8_t SERVO_WRITE_DEADBAND_S1_S3 = ServoMotor::WRITE_DEADBAND;
constexpr uint8_t SERVO_WRITE_DEADBAND_S2 = 1;
ServoInputFilter servo1Filter(SERVO_INPUT_CENTER);
ServoInputFilter servo2Filter(SERVO_INPUT_CENTER);
ServoInputFilter servo3Filter(SERVO_INPUT_CENTER);

uint8_t servo1PendingAngle = ServoMotor::SERVO1_MIN_ANGLE;
uint8_t servo3PendingAngle = ServoMotor::SERVO3_MIN_ANGLE;
uint8_t servo1SwitchCount = 0;
uint8_t servo3SwitchCount = 0;

static uint8_t mapServo1DiscreteWithHysteresis(uint16_t chValue, uint8_t lastAngle) {
    // 3-level mapping with hysteresis around 1400 and 1800.
    if (lastAngle <= 0) {
        if (chValue >= static_cast<uint16_t>(1400 + SERVO_DISCRETE_HYST_BAND)) {
            return 45;
        }
        return 0;
    }

    if (lastAngle >= 90) {
        if (chValue <= static_cast<uint16_t>(1800 - SERVO_DISCRETE_HYST_BAND)) {
            return 45;
        }
        return 90;
    }

    if (chValue <= static_cast<uint16_t>(1400 - SERVO_DISCRETE_HYST_BAND)) {
        return 0;
    }
    if (chValue >= static_cast<uint16_t>(1800 + SERVO_DISCRETE_HYST_BAND)) {
        return 90;
    }
    return 45;
}

static uint8_t mapServo2Lookup(uint16_t chValue) {
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

static uint8_t mapServo3DiscreteWithHysteresis(uint16_t chValue, uint8_t lastAngle) {
    // 5-level mapping with hysteresis around 1200/1400/1600/1800.
    if (lastAngle <= 40) {
        if (chValue >= static_cast<uint16_t>(1200 + SERVO_DISCRETE_HYST_BAND)) {
            return 60;
        }
        return 40;
    }

    if (lastAngle <= 60) {
        if (chValue <= static_cast<uint16_t>(1200 - SERVO_DISCRETE_HYST_BAND)) {
            return 40;
        }
        if (chValue >= static_cast<uint16_t>(1400 + SERVO_DISCRETE_HYST_BAND)) {
            return 80;
        }
        return 60;
    }

    if (lastAngle <= 80) {
        if (chValue <= static_cast<uint16_t>(1400 - SERVO_DISCRETE_HYST_BAND)) {
            return 60;
        }
        if (chValue >= static_cast<uint16_t>(1600 + SERVO_DISCRETE_HYST_BAND)) {
            return 100;
        }
        return 80;
    }

    if (lastAngle <= 100) {
        if (chValue <= static_cast<uint16_t>(1600 - SERVO_DISCRETE_HYST_BAND)) {
            return 80;
        }
        if (chValue >= static_cast<uint16_t>(1800 + SERVO_DISCRETE_HYST_BAND)) {
            return 120;
        }
        return 100;
    }

    if (chValue <= static_cast<uint16_t>(1800 - SERVO_DISCRETE_HYST_BAND)) {
        return 100;
    }
    return 120;
}

void setup() {
    Serial.begin(115200);
    ibus.begin(Serial2);

    motor1.begin();
    motor2.begin();
    motor3.begin();
    motor4.begin();
    
    servo1.attach(ServoMotor::Servo1_PWM_PIN);
    servo2.attach(ServoMotor::Servo2_PWM_PIN);
    servo3.attach(ServoMotor::Servo3_PWM_PIN);
    servo1.write(ServoMotor::SERVO1_MAX_ANGLE);
    servo2.write(ServoMotor::SERVO2_MIN_ANGLE);
    servo3.write(ServoMotor::SERVO3_MIN_ANGLE);
    
    initControlTimer20ms();

    pinMode(MotorPins::STBY1, OUTPUT);
    pinMode(MotorPins::STBY2, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(MotorPins::M1_ENCODER_PIN1), motor1EncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MotorPins::M2_ENCODER_PIN1), motor2EncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MotorPins::M3_ENCODER_PIN1), motor3EncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MotorPins::M4_ENCODER_PIN1), motor4EncoderISR, RISING);
    digitalWrite(MotorPins::STBY1, HIGH);
    digitalWrite(MotorPins::STBY2, HIGH);

    delay(100);
    Serial.println("Motor1~Motor4 + Servo1/Servo2/Servo3 Test Started");
}

void loop() {
    // 20ms 週期執行閉迴路更新（與 PID timeStep 一致）
    if (timerFlag) {
        timerFlag = false;

        uint16_t ch1 = ibus.readChannel(1);

        // 接收器失效保護：讀不到資料就停車
        if (ch1 >= CommsMapConfig::C1_MIN && ch1 <= CommsMapConfig::C1_MAX) {
            lastCh1 = ch1;
            lastTargetCount = mapC1ToTargetCountLUT(ch1);
        } else {
            lastCh1 = CommsMapConfig::C1_CENTER;
            lastTargetCount = 0;
        }

#if MOTOR_TEST_MODE == 0
        if (lastTargetCount == 0) {
            motor1.stopAndResetControl();
            motor2.stopAndResetControl();
            motor3.stopAndResetControl();
            motor4.stopAndResetControl();
            lastPwmCmd = 0;
        } else {
            motor1.updateEncoder(lastTargetCount);
            motor2.updateEncoder(lastTargetCount);
            motor3.updateEncoder(lastTargetCount);
            motor4.updateEncoder(lastTargetCount);
            lastPwmCmd = (motor1.getLastPWM() + motor2.getLastPWM() + motor3.getLastPWM() + motor4.getLastPWM()) / 4;
        }
#else
        // 開迴路：用 CH1 直接控制 PWM，先確認馬達與驅動硬體路徑正常
        int16_t delta = static_cast<int16_t>(lastCh1) - static_cast<int16_t>(CommsMapConfig::C1_CENTER);
        if (abs(delta) <= static_cast<int16_t>(CommsMapConfig::C1_DEADBAND)) {
            lastPwmCmd = 0;
        } else {
            lastPwmCmd = map(delta,
                             -500,
                             500,
                             PidConfig::OUTPUT_MIN,
                             PidConfig::OUTPUT_MAX);
        }
        motor1.setSpeed(lastPwmCmd);
        motor2.setSpeed(lastPwmCmd);
        motor3.setSpeed(lastPwmCmd);
        motor4.setSpeed(lastPwmCmd);

        // 開迴路下仍更新回授，便於觀察速度是否有變化
        motor1.updateFeedbackOnly();
        motor2.updateFeedbackOnly();
        motor3.updateFeedbackOnly();
        motor4.updateFeedbackOnly();

        lastSpeedOL1 = motor1.getCurrentSpeed();
        lastSpeedOL2 = motor2.getCurrentSpeed();
        lastSpeedOL3 = motor3.getCurrentSpeed();
        lastSpeedOL4 = motor4.getCurrentSpeed();
#endif
        
        // Servo control (independent of motor test mode)
        uint16_t ch5 = ibus.readChannel(5);
        uint16_t ch2 = ibus.readChannel(2);
        uint16_t ch4 = ibus.readChannel(4);
        
        if (ch5 >= ServoMotor::INPUT_MIN && ch5 <= ServoMotor::INPUT_MAX) {
            uint8_t cmdAngle1 = lastServo1Angle;
            uint16_t filteredCh1 = servo1Filter.updateWithEndpointSnap(ch5,
                                                                        ServoMotor::INPUT_MIN,
                                                                        ServoMotor::INPUT_MAX,
                                                                        SERVO_FILTER_SHIFT_S1_S3,
                                                                        ServoMotor::ENDPOINT_BAND);
            cmdAngle1 = mapServo1DiscreteWithHysteresis(filteredCh1, lastServo1Angle);
            if (cmdAngle1 != lastServo1Angle) {
                if (cmdAngle1 == servo1PendingAngle) {
                    if (servo1SwitchCount < SERVO_SWITCH_CONFIRM_COUNT) {
                        servo1SwitchCount++;
                    }
                } else {
                    servo1PendingAngle = cmdAngle1;
                    servo1SwitchCount = 1;
                }

                if (servo1SwitchCount >= SERVO_SWITCH_CONFIRM_COUNT &&
                    abs(static_cast<int16_t>(cmdAngle1) - static_cast<int16_t>(lastServo1Angle)) >= SERVO_WRITE_DEADBAND_S1_S3) {
                    lastServo1Angle = cmdAngle1;
                    servo1.write(lastServo1Angle);
                    servo1SwitchCount = 0;
                }
            } else {
                servo1SwitchCount = 0;
            }
        }
        
        if (ch2 >= ServoMotor::INPUT_MIN && ch2 <= ServoMotor::INPUT_MAX) {
            uint8_t cmdAngle2 = lastServo2Angle;
            uint16_t filteredCh2 = servo2Filter.updateWithEndpointSnap(ch2,
                                                                        ServoMotor::INPUT_MIN,
                                                                        ServoMotor::INPUT_MAX,
                                                                        SERVO_FILTER_SHIFT_S2,
                                                                        ServoMotor::ENDPOINT_BAND);
            cmdAngle2 = mapServo2Lookup(filteredCh2);
            if (abs(static_cast<int16_t>(cmdAngle2) - static_cast<int16_t>(lastServo2Angle)) >= SERVO_WRITE_DEADBAND_S2) {
                lastServo2Angle = cmdAngle2;
                servo2.write(lastServo2Angle);
            }
        }

        if (ch4 >= ServoMotor::INPUT_MIN && ch4 <= ServoMotor::INPUT_MAX) {
            uint8_t cmdAngle3 = lastServo3Angle;
            uint16_t filteredCh3 = servo3Filter.updateWithEndpointSnap(ch4,
                                                                        ServoMotor::INPUT_MIN,
                                                                        ServoMotor::INPUT_MAX,
                                                                        SERVO_FILTER_SHIFT_S1_S3,
                                                                        ServoMotor::ENDPOINT_BAND);
            cmdAngle3 = mapServo3DiscreteWithHysteresis(filteredCh3, lastServo3Angle);
            if (cmdAngle3 != lastServo3Angle) {
                if (cmdAngle3 == servo3PendingAngle) {
                    if (servo3SwitchCount < SERVO_SWITCH_CONFIRM_COUNT) {
                        servo3SwitchCount++;
                    }
                } else {
                    servo3PendingAngle = cmdAngle3;
                    servo3SwitchCount = 1;
                }

                if (servo3SwitchCount >= SERVO_SWITCH_CONFIRM_COUNT &&
                    abs(static_cast<int16_t>(cmdAngle3) - static_cast<int16_t>(lastServo3Angle)) >= SERVO_WRITE_DEADBAND_S1_S3) {
                    lastServo3Angle = cmdAngle3;
                    servo3.write(lastServo3Angle);
                    servo3SwitchCount = 0;
                }
            } else {
                servo3SwitchCount = 0;
            }
        }
    }

    // 10Hz 輸出監看資料給 Serial Monitor / Plotter
    if (millis() - lastPrintMs >= 100) {
        lastPrintMs = millis();

    #if MOTOR_TEST_MODE == 1
        Serial.print(",Speed1(OL):");
        Serial.print(lastSpeedOL1);
        Serial.print(",Speed2(OL):");
        Serial.print(lastSpeedOL2);
        Serial.print(",Speed3(OL):");
        Serial.print(lastSpeedOL3);
        Serial.print(",Speed4(OL):");
        Serial.print(lastSpeedOL4);
    #endif

        Serial.print(",Servo1Angle:");
        Serial.print(lastServo1Angle);
        Serial.print(",CH2Raw:");

        Serial.print(",CH2F:");

        Serial.print(",Servo2Angle:");
        Serial.print(lastServo2Angle);
        Serial.print(",Servo3Angle:");
        Serial.println(lastServo3Angle);
    }
}

