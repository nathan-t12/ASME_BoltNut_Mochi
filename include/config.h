#ifndef CONFIG_H
#define CONFIG_H
#pragma once
#include <Arduino.h>
// ========== 腳位定義 ==========

// maxcount 38
// mincount 8

// DC Motor
namespace MotorPins {
    // 1A
    constexpr uint8_t M1_PWM_PIN = 6;
    constexpr uint8_t M1_DIR_PIN1 = 32;
    constexpr uint8_t M1_DIR_PIN2 = 33;
    constexpr uint8_t M1_ENCODER_PIN1 = 18; // Interrupt Pin
    constexpr uint8_t M1_ENCODER_PIN2 = 19; // Interrupt Pin

    //1B
    constexpr uint8_t M2_PWM_PIN = 7;
    constexpr uint8_t M2_DIR_PIN1 = 35;
    constexpr uint8_t M2_DIR_PIN2 = 34;
    constexpr uint8_t M2_ENCODER_PIN1 = 20; // Interrupt Pin
    constexpr uint8_t M2_ENCODER_PIN2 = 21; // Interrupt Pin

    //2A
    constexpr uint8_t M3_PWM_PIN = 4;
    constexpr uint8_t M3_DIR_PIN1 = 28;
    constexpr uint8_t M3_DIR_PIN2 = 29;
    constexpr uint8_t M3_ENCODER_PIN1 = 2; // Interrupt Pin
    constexpr uint8_t M3_ENCODER_PIN2 = 8; // Interrupt Pin

    //2B
    constexpr uint8_t M4_PWM_PIN = 5;
    constexpr uint8_t M4_DIR_PIN1 = 31;
    constexpr uint8_t M4_DIR_PIN2 = 30;
    constexpr uint8_t M4_ENCODER_PIN1 = 3; // Interrupt Pin
    constexpr uint8_t M4_ENCODER_PIN2 = 9; // Interrupt Pin

    constexpr uint8_t STBY1 = 37;
    constexpr uint8_t STBY2 = 36;
};

// Servo Motor
namespace ServoMotor
{
    constexpr uint8_t Servo1_PWM_PIN = 10;
    constexpr uint8_t Servo2_PWM_PIN = 38;
    constexpr uint8_t Servo3_PWM_PIN = 39;

    constexpr uint16_t INPUT_MIN = 1000;
    constexpr uint16_t INPUT_MAX = 2000;
    constexpr uint8_t FILTER_SHIFT = 3;
    constexpr uint8_t FILTER_SHIFT_S1 = FILTER_SHIFT + 1;
    constexpr uint8_t FILTER_SHIFT_S2 = 0;
    constexpr uint8_t FILTER_SHIFT_S3 = FILTER_SHIFT;
    constexpr uint8_t WRITE_DEADBAND = 2;
    constexpr uint8_t ENDPOINT_BAND = 20;

    constexpr uint8_t SERVO1_MIN_ANGLE = 3;
    constexpr uint8_t SERVO1_MAX_ANGLE = 90;

    constexpr uint8_t SERVO2_MIN_ANGLE = 20;
    constexpr uint8_t SERVO2_MAX_ANGLE = 170;

    constexpr uint8_t SERVO3_MIN_ANGLE = 53;
    constexpr uint8_t SERVO3_MAX_ANGLE = 110;

    constexpr uint16_t SERVO3_CENTER_INPUT = 1500;
    constexpr uint8_t SERVO3_LEFT_ANGLE = 55;    // at INPUT_MIN
    constexpr uint8_t SERVO3_PEAK_ANGLE = 115;   // at CENTER_INPUT
    constexpr uint8_t SERVO3_RIGHT_ANGLE = 63;   // at INPUT_MAX
    
}


// FlySky

// ========== 各參數調配 ==========

// PID參數

namespace PidConfig {
    // 縮放因子
    constexpr uint8_t SCALE_SHIFT = 10;           // 2^10 = 1024
    constexpr int16_t SCALE_FACTOR = (1 << SCALE_SHIFT); // 1024
    
    constexpr float timeStep = 0.02f; //s
    constexpr float PID_KP  = 2.0f;
    constexpr float PID_KI  = 0.3f;
    constexpr float PID_KD  = 0.1f;

    // 輸出限制
    constexpr int16_t OUTPUT_MAX      =  255;
    constexpr int16_t OUTPUT_MIN      = -255;
    constexpr int32_t INTEGRAL_LIMIT  =  128L * SCALE_FACTOR;
}

// 底盤輸出調校（開迴路）
namespace DriveConfig {
    // 開迴路總輸出上限（0~255）
    constexpr int16_t OPEN_LOOP_MAX_PWM = 230;

    // 單顆馬達補償倍率（百分比），100=不補償
    constexpr uint16_t MOTOR1_GAIN_PERCENT = 100;
    constexpr uint16_t MOTOR2_GAIN_PERCENT = 100;
    constexpr uint16_t MOTOR3_GAIN_PERCENT = 105;
    constexpr uint16_t MOTOR4_GAIN_PERCENT = 105;

    // 低速轉向補償（克服靜摩擦）
    // RIGHT_TURN_SIGN: +1 代表 turnCmd > 0 為右轉；-1 代表 turnCmd < 0 為右轉
    constexpr int8_t RIGHT_TURN_SIGN = 1;
    constexpr int16_t LOW_SPEED_BASE_PWM_THRESHOLD = 90;
    constexpr int16_t M1_LEFT_TURN_BOOST_PWM = 18;   // motor1/3 左轉時補強
    constexpr int16_t M4_RIGHT_TURN_BOOST_PWM = 18;  // motor4/2 右轉時補強
}

// 濾波參數
namespace FilterConfig {
    constexpr uint8_t  SCALE_SHIFT  = 5;    
    constexpr uint16_t SCALE_FACTOR = (1 << SCALE_SHIFT);        
    constexpr uint16_t  ALPHA_FIXED  = 0.3f * SCALE_FACTOR;  // 0.1~1.0
}

#endif