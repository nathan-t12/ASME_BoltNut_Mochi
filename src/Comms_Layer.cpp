#include "Comms_Layer.h"

int16_t mapC1ToTargetCountLUT(uint16_t c1) {
    static constexpr uint8_t kCubicCountTable[] = {
        8, 8, 8, 9, 9, 10, 11, 12,
        14, 16, 18, 21, 24, 28, 33, 38
    };

    constexpr uint16_t kSpan      = CommsMapConfig::C1_MAX - CommsMapConfig::C1_CENTER;
    constexpr uint16_t kActiveSpan = kSpan - CommsMapConfig::C1_DEADBAND;
    constexpr uint8_t  kTableMaxIndex = static_cast<uint8_t>(
        sizeof(kCubicCountTable) / sizeof(kCubicCountTable[0]) - 1);

    uint16_t clamped  = constrain(c1, CommsMapConfig::C1_MIN, CommsMapConfig::C1_MAX);
    int16_t  delta    = static_cast<int16_t>(clamped) - static_cast<int16_t>(CommsMapConfig::C1_CENTER);
    uint16_t absDelta = static_cast<uint16_t>(abs(delta));

    //  死區回傳 0，交給上層邏輯判斷是否停止
    if (absDelta <= CommsMapConfig::C1_DEADBAND) {
        return 0;
    }

    uint16_t active = absDelta - CommsMapConfig::C1_DEADBAND;
    uint8_t  index  = static_cast<uint8_t>(
        (static_cast<uint32_t>(active) * kTableMaxIndex) / kActiveSpan);

    // 防止 index 越界
    index = constrain(index, 0, kTableMaxIndex);

    int16_t count = static_cast<int16_t>(kCubicCountTable[index]);
    return (delta >= 0) ? count : -count;
}

int16_t mapC1ToOpenLoopPwmLUT(uint16_t c1) {
    static constexpr uint8_t kPwmTable[] = {
        18, 20, 23, 27, 32, 39, 48, 60,
        74, 92, 114, 138, 164, 188, 210, 255
    };

    constexpr uint16_t kSpan = CommsMapConfig::C1_MAX - CommsMapConfig::C1_CENTER;
    constexpr uint16_t kActiveSpan = kSpan - CommsMapConfig::C1_DEADBAND;
    constexpr uint8_t kTableMaxIndex = static_cast<uint8_t>(sizeof(kPwmTable) / sizeof(kPwmTable[0]) - 1);

    uint16_t clamped = constrain(c1, CommsMapConfig::C1_MIN, CommsMapConfig::C1_MAX);
    int16_t delta = static_cast<int16_t>(clamped) - static_cast<int16_t>(CommsMapConfig::C1_CENTER);
    uint16_t absDelta = static_cast<uint16_t>(abs(delta));

    if (absDelta <= CommsMapConfig::C1_DEADBAND) {
        return 0;
    }

    uint16_t active = absDelta - CommsMapConfig::C1_DEADBAND;
    uint8_t index = static_cast<uint8_t>((static_cast<uint32_t>(active) * kTableMaxIndex) / kActiveSpan);
    index = constrain(index, 0, kTableMaxIndex);

    int16_t pwm = static_cast<int16_t>(kPwmTable[index]);
    return (delta >= 0) ? pwm : -pwm;
}

int16_t mapC3ToTurnPwmLUT(uint16_t c3) {
    static constexpr uint8_t kTurnTable[] = {
        10, 12, 14, 17, 21, 26, 33, 42,
        52, 64, 78, 95, 114, 136, 158, 180
    };

    constexpr uint16_t kSpan = CommsMapConfig::C3_MAX - CommsMapConfig::C3_CENTER;
    constexpr uint16_t kActiveSpan = kSpan - CommsMapConfig::C3_DEADBAND;
    constexpr uint8_t kTableMaxIndex = static_cast<uint8_t>(sizeof(kTurnTable) / sizeof(kTurnTable[0]) - 1);

    uint16_t clamped = constrain(c3, CommsMapConfig::C3_MIN, CommsMapConfig::C3_MAX);
    int16_t delta = static_cast<int16_t>(clamped) - static_cast<int16_t>(CommsMapConfig::C3_CENTER);
    uint16_t absDelta = static_cast<uint16_t>(abs(delta));

    if (absDelta <= CommsMapConfig::C3_DEADBAND) {
        return 0;
    }

    uint16_t active = absDelta - CommsMapConfig::C3_DEADBAND;
    uint8_t index = static_cast<uint8_t>((static_cast<uint32_t>(active) * kTableMaxIndex) / kActiveSpan);
    index = constrain(index, 0, kTableMaxIndex);

    int16_t turn = static_cast<int16_t>(kTurnTable[index]);
    turn = static_cast<int16_t>((static_cast<int32_t>(turn) * CommsMapConfig::TURN_GAIN_PERCENT) / 100);
    turn = constrain(turn, 0, CommsMapConfig::TURN_PWM_MAX);
    return (delta >= 0) ? turn : -turn;
}