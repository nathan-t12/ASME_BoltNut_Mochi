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
    constexpr uint8_t M2_DIR_PIN1 = 34;
    constexpr uint8_t M2_DIR_PIN2 = 35;
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
    constexpr uint8_t M4_DIR_PIN1 = 30;
    constexpr uint8_t M4_DIR_PIN2 = 31;
    constexpr uint8_t M4_ENCODER_PIN1 = 3; // Interrupt Pin
    constexpr uint8_t M4_ENCODER_PIN2 = 9; // Interrupt Pin

    constexpr uint8_t STBY1 = 37;
    constexpr uint8_t STBY2 = 36;
};

// Servo Motor
namespace ServoMotor
{
    constexpr uint8_t Servo1_PWM_PIN = 10;
    constexpr uint8_t Servo2_PWM_PIN = 40;
    constexpr uint8_t Servo3_PWM_PIN = 41;

    constexpr uint16_t INPUT_MIN = 1000;
    constexpr uint16_t INPUT_MAX = 2000;
    constexpr uint8_t FILTER_SHIFT = 3;
    constexpr uint8_t WRITE_DEADBAND = 2;
    constexpr uint8_t ENDPOINT_BAND = 20;

    constexpr uint8_t SERVO1_MIN_ANGLE = 0;
    constexpr uint8_t SERVO1_MAX_ANGLE = 90;

    constexpr uint8_t SERVO2_MIN_ANGLE = 25;
    constexpr uint8_t SERVO2_MAX_ANGLE = 170;

    constexpr uint8_t SERVO3_MIN_ANGLE = 45;
    constexpr uint8_t SERVO3_MAX_ANGLE = 115;
    
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
    constexpr float PID_KI  = 0.5f;
    constexpr float PID_KD  = 0.1f;

    // 輸出限制
    constexpr int16_t OUTPUT_MAX      =  255;
    constexpr int16_t OUTPUT_MIN      = -255;
    constexpr int32_t INTEGRAL_LIMIT  =  128L * SCALE_FACTOR;
}

// 濾波參數
namespace FilterConfig {
    constexpr uint8_t  SCALE_SHIFT  = 5;    
    constexpr uint16_t SCALE_FACTOR = (1 << SCALE_SHIFT);        
    constexpr uint16_t  ALPHA_FIXED  = 0.3f * SCALE_FACTOR;  // 0.1~1.0
}

#endif