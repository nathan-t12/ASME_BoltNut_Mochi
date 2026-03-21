#ifndef COMMS_LAYER_H
#define COMMS_LAYER_H
#pragma once
#include <Arduino.h>


//左搖桿

// C3 1000~2000 1500常態 左右
// c1 1000~2000 1500常態 前後

//右搖桿
// c0 1000~2000 1500常態 左右
// c2 1000~2000 1500常態 前後

// 上面兩顆
// c4 1000~2000
// c5 1000~2000 

// 腳位設計
// c1 前進後退
// c3 轉彎
// c2 夾轉仰俯
// c4 夾爪開合
// c5 載物台角度

namespace CommsMapConfig {
	constexpr uint16_t C1_MIN = 1000;
	constexpr uint16_t C1_MAX = 2000;
	constexpr uint16_t C1_CENTER = 1500;
	constexpr uint16_t C1_DEADBAND = 20;
	constexpr int16_t TARGET_MIN_COUNT = 5;
	constexpr int16_t TARGET_MAX_COUNT = 35;

	// Servo mapping: 1000 -> 0°, 2000 -> 90°
	constexpr uint16_t SERVO_MIN = 1000;
	constexpr uint16_t SERVO_MAX = 2000;
	constexpr uint8_t SERVO_MIN_ANGLE = 0;
	constexpr uint8_t SERVO_MAX_ANGLE = 90;
}

int16_t mapC1ToTargetCountLUT(uint16_t c1);
uint8_t mapChannelToServoAngle(uint16_t chValue);

#endif