#ifndef CONFIG_H
#define CONFIG_H
#pragma once
#include <Arduino.h>
// ========== 腳位定義 ==========

// Encoder

// DC Motor

// Servo Motor

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
    constexpr int32_t INTEGRAL_LIMIT  =  128 * SCALE_FACTOR;
}

// 濾波參數
namespace FilterConfig {
    constexpr uint8_t  SCALE_SHIFT  = 8;              
    constexpr uint8_t  ALPHA_FIXED  = 0.3f * (1 << SCALE_SHIFT);  // 0.1~1.0
}

#endif