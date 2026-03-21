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
uint8_t lastServo1Angle = 0;
uint8_t lastServo2Angle = 0;

void setup() {
    Serial.begin(115200);
    ibus.begin(Serial2);

    motor1.begin();
    motor2.begin();
    motor3.begin();
    motor4.begin();
    
    servo1.attach(ServoMotor::Servo1_PWM_PIN);
    servo2.attach(ServoMotor::Servo2_PWM_PIN);
    servo1.write(0);
    servo2.write(0);
    
    initTimer1_20ms();

    pinMode(MotorPins::STBY1, OUTPUT);
    pinMode(MotorPins::STBY2, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(MotorPins::M1_ENCODER_PIN1), motor1EncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MotorPins::M2_ENCODER_PIN1), motor2EncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MotorPins::M3_ENCODER_PIN1), motor3EncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MotorPins::M4_ENCODER_PIN1), motor4EncoderISR, RISING);
    digitalWrite(MotorPins::STBY1, HIGH);
    digitalWrite(MotorPins::STBY2, HIGH);

    delay(100);
    Serial.println("Motor1~Motor4 + Servo1/Servo2 Test Started");
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
        
        if (ch5 >= CommsMapConfig::SERVO_MIN && ch5 <= CommsMapConfig::SERVO_MAX) {
            lastServo1Angle = mapChannelToServoAngle(ch5);
            servo1.write(lastServo1Angle);
        }
        
        if (ch2 >= CommsMapConfig::SERVO_MIN && ch2 <= CommsMapConfig::SERVO_MAX) {
            lastServo2Angle = mapChannelToServoAngle(ch2);
            servo2.write(lastServo2Angle);
        }
    }

    // 10Hz 輸出監看資料給 Serial Monitor / Plotter
    if (millis() - lastPrintMs >= 100) {
        lastPrintMs = millis();
        Serial.print("CH1:");
        Serial.print(lastCh1);
        Serial.print(",Target:");
        Serial.print(lastTargetCount);
        Serial.print(",CmdPWM:");
        Serial.print(lastPwmCmd);
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
        Serial.print(",Speed1:");
        Serial.print(motor1.getCurrentSpeed());
        Serial.print(",PWM1:");
        Serial.print(motor1.getLastPWM());
        Serial.print(",Speed2:");
        Serial.print(motor2.getCurrentSpeed());
        Serial.print(",PWM2:");
        Serial.print(motor2.getLastPWM());
        Serial.print(",Speed3:");
        Serial.print(motor3.getCurrentSpeed());
        Serial.print(",PWM3:");
        Serial.print(motor3.getLastPWM());
        Serial.print(",Speed4:");
        Serial.print(motor4.getCurrentSpeed());
        Serial.print(",PWM4:");
        Serial.print(motor4.getLastPWM());
        Serial.print(",Servo1Angle:");
        Serial.print(lastServo1Angle);
        Serial.print(",Servo2Angle:");
        Serial.println(lastServo2Angle);
    }
}

