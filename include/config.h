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
    constexpr uint8_t SCALE_SHIFT = 8;           // 2^8 = 256
    constexpr int16_t SCALE_FACTOR = (1 << SCALE_SHIFT); // 256

    // 浮點參數 → 轉換時使用
    constexpr float LEFT_KP  = 2.0f;
    constexpr float LEFT_KI  = 0.5f;
    constexpr float LEFT_KD  = 0.1f;

    // 輸出限制
    constexpr int16_t OUTPUT_MAX      =  255;
    constexpr int16_t OUTPUT_MIN      = -255;
    constexpr int16_t INTEGRAL_LIMIT  =  100 * SCALE_FACTOR;
}

// 濾波參數

#endif