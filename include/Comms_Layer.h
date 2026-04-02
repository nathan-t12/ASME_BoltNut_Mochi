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

	constexpr uint16_t C3_MIN = 1000;
	constexpr uint16_t C3_MAX = 2000;
	constexpr uint16_t C3_CENTER = 1500;
	constexpr uint16_t C3_DEADBAND = 20;
	constexpr int16_t TURN_PWM_MAX = 255;
	constexpr uint16_t TURN_GAIN_PERCENT = 130;
}

int16_t mapC1ToTargetCountLUT(uint16_t c1);
int16_t mapC1ToOpenLoopPwmLUT(uint16_t c1);
int16_t mapC3ToTurnPwmLUT(uint16_t c3);

#endif