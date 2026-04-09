# BoltNut_Mochie

An Arduino-based multi-motor and multi-servo control system for autonomous robotic platforms.  
**BoltNut_Mochie** integrates four DC motors with encoders, three servo motors, and iBus RC receiver input to deliver robust open-loop and closed-loop control modes with lookup-table (LUT) optimization.

## Overview

**Platform:** Arduino Mega 2560 (ATmega2560)  
**Input:** iBus protocol from RC receiver  
**Output:** 4× DC motor PWM + direction, 3× servo PWM

### Key Features

- **Dual-mode motor control**
  - Closed-loop: encoder-based PID regulation
  - Open-loop: LUT-driven differential steering with dynamic turn compensation
  
- **Multi-servo support**
  - Per-servo input filtering and custom mapping functions
  - Centre-peak mapping for asymmetric response (e.g., servo3)
  - Lookup-table interpolation for smooth actuation

- **Low-speed turn assist**
  - Automatic PWM boost for right-side wheels during slow turns
  - Overcomes static friction for smooth response at minimal speeds

- **Efficient control loop**
  - Timer2-based 20ms periodic control tick (avoids Servo library ISR conflicts)
  - Scaled fixed-point arithmetic for real-time computation

## Hardware Setup

### Motor Connections (Arduino Mega 2560)
```
Motor 1 (M1): PWM=6,    DIR=32/33, ENC_INT=18/19
Motor 2 (M2): PWM=7,    DIR=35/34, ENC_INT=20/21
Motor 3 (M3): PWM=4,    DIR=28/29, ENC_INT=2/8
Motor 4 (M4): PWM=5,    DIR=31/30, ENC_INT=3/9
Standby:      STBY1=37, STBY2=36
```

### Servo Connections
```
Servo 1 (S1): PWM=10
Servo 2 (S2): PWM=38
Servo 3 (S3): PWM=39
```

### RC Receiver (iBus)
```
Serial2 RX connected to iBus data line
```

## RC Channel Mapping

| Channel | Function | Range | Mode |
|---------|----------|-------|------|
| C1 | Forward/Reverse | 1000–2000 | Both |
| C3 | Turn | 1000–2000 | Open-loop |
| C5 | Servo 1 (discrete) | 1000–2000 | Both |
| C2 | Servo 2 (continuous) | 1000–2000 | Both |
| C0 | Servo 3 (centre-peak) | 1000–2000 | Both |

## Control Modes

### Closed-Loop (MOTOR_TEST_MODE = 0)
- **C1 → Target Count LUT → PID per motor**
- Encoder feedback provides speed regulation
- Ideal for precise speed synchronization and load handling

### Open-Loop (MOTOR_TEST_MODE = 1 – Default)
- **C1 → Base PWM LUT + C3 → Turn PWM LUT**
- Differential mixing: Left = Base + Turn, Right = Base – Turn
- Per-motor gain compensation (100–110%)
- Low-speed right-turn boost (18 PWM units below 90 base PWM)
- Full response, suitable for dynamic manoeuvring

## Software Architecture

```
BoltNut_Mochie/
├── include/
│   ├── config.h              # Pin definitions, tuning parameters (PID, filter, servo ranges)
│   ├── Comms_Layer.h         # RC channel mapping & LUT declarations
│   ├── Hardware_Layer.h      # Motor class, Timer2 setup
│   └── Math_Layer.h          # Filter, PID, servo mapping functions
├── src/
│   ├── main.cpp              # Main loop: control integration, mode dispatch
│   ├── Comms_Layer.cpp       # LUT implementations for C1/C3
│   ├── Hardware_Layer.cpp    # Motor PWM/direction, encoder ISR, Timer2 ISR
│   └── Math_Layer.cpp        # Filter, PID, servo lookup/centre-peak mappers
├── platformio.ini            # Build configuration
└── README.md
```

### Layer Responsibilities

1. **Comms Layer** – Channel-to-command mapping via table lookup
2. **Math Layer** – Exponential filtering, PID loop, servo angle interpolation
3. **Hardware Layer** – PWM/direction I/O, encoder counting, timing
4. **Main** – Orchestrates mode selection, applies mixers, commands actuators

## Tuning & Configuration

All key parameters centralized in `include/config.h`:

### Motor Tuning (DriveConfig)
```cpp
constexpr uint16_t MOTOR1_GAIN_PERCENT = 100;     // Gain: 100 = no boost
constexpr uint16_t MOTOR3_GAIN_PERCENT = 105;     // +5% for imbalance
constexpr int16_t  LOW_SPEED_BASE_PWM_THRESHOLD = 90;  // Below 90 PWM → boost
constexpr int16_t  M4_RIGHT_TURN_BOOST_PWM = 18;  // +18 PWM for inner wheel
```

### PID Tuning (PidConfig)
```cpp
constexpr float PID_KP  = 2.0f;   // Proportional gain
constexpr float PID_KI  = 0.3f;   // Integral gain
constexpr float PID_KD  = 0.1f;   // Derivative gain
```

### Servo Parameters (ServoMotor)
```cpp
constexpr uint8_t FILTER_SHIFT_S1 = 4;       // Servo1: stronger filtering
constexpr uint8_t FILTER_SHIFT_S2 = 0;       // Servo2: no filter (raw)
constexpr uint8_t FILTER_SHIFT_S3 = 3;       // Servo3: standard filter
```

### Turn Compensation (CommsMapConfig)
```cpp
constexpr uint16_t TURN_GAIN_PERCENT = 130;  // Turn command ×1.3
constexpr int16_t  TURN_PWM_MAX = 255;       // Saturation cap
```

## Compilation & Deployment

### Prerequisites
- **PlatformIO** with Arduino AVR support
- **Libraries:**
  - `bmellink/IBusBM @ ^1.1.4`
  - `arduino-libraries/Servo @ ^1.2.2`

### Build
```bash
pio run -e megaatmega2560
```

### Upload (Windows)
```bash
pio run -e megaatmega2560 --target upload --upload-port COM10
```

### Monitor Console
```bash
pio device monitor -b 115200
```

## Performance Notes

- **Control frequency:** ~50 Hz (20 ms tick via Timer2)
- **Servo update:** Per-loop for smooth, jitter-free response
- **Static friction compensation:** Active only when base PWM ≤ 90 and turning
- **Memory usage:** ~10.9% RAM, ~3.9% Flash (plenty of headroom for extensions)

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| One wheel slower | Mechanical friction / motor imbalance | Increase `MOTOR*_GAIN_PERCENT` |
| Poor low-speed turns | Static friction on inner wheels | Increase `M4_RIGHT_TURN_BOOST_PWM` |
| Servo jitter | Input noise / excessive filter | Reduce `FILTER_SHIFT_S*` or increase `WRITE_DEADBAND` |
| Encoder noise spikes | Electrical interference | Add ferrite on signal lines |

## License

Open source. Use, modify, and redistribute freely with attribution.


---

**For questions or customization requests**, refer to the inline code comments in `include/config.h` and `src/Math_Layer.cpp`.


