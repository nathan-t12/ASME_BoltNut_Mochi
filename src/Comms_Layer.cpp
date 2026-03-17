#include "Comms_Layer.h"

int16_t mapC1ToTargetCountLUT(uint16_t c1) {
	static constexpr uint8_t kCubicCountTable[] = {
		8, 8, 8, 9, 9, 10, 11, 12,
		14, 16, 18, 21, 24, 28, 33, 38
	};

	constexpr uint16_t kSpan = CommsMapConfig::C1_MAX - CommsMapConfig::C1_CENTER; // 500
	constexpr uint16_t kActiveSpan = kSpan - CommsMapConfig::C1_DEADBAND; // 470
	constexpr uint8_t kTableMaxIndex = static_cast<uint8_t>(sizeof(kCubicCountTable) / sizeof(kCubicCountTable[0]) - 1);

	uint16_t clamped = constrain(c1, CommsMapConfig::C1_MIN, CommsMapConfig::C1_MAX);
	int16_t delta = static_cast<int16_t>(clamped) - static_cast<int16_t>(CommsMapConfig::C1_CENTER);
	uint16_t absDelta = static_cast<uint16_t>(abs(delta));

	if (absDelta <= CommsMapConfig::C1_DEADBAND) {
		return 0;
	}

	uint16_t active = absDelta - CommsMapConfig::C1_DEADBAND;
	uint8_t index = static_cast<uint8_t>((static_cast<uint32_t>(active) * kTableMaxIndex) / kActiveSpan);
	int16_t count = static_cast<int16_t>(kCubicCountTable[index]);

	return (delta >= 0) ? count : -count;
}