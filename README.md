# ASME_BoltNut_Mochie

ASME_BoltNut_Mochie 是一個基於 Arduino Mega 2560 的多馬達/多伺服控制專案，
使用 IBus 遙控通道作為輸入，整合以下能力：

1. 4 顆 DC 馬達（含編碼器）
2. 3 顆伺服馬達
3. 開迴路與閉迴路雙模式
4. LUT 映射優化，降低即時計算負擔

## 功能總覽

1. 馬達控制
- 閉迴路模式：C1 經 LUT 映射為目標計數，進入 PID 控制。
- 開迴路模式：C1 與 C3 都經 LUT，形成前進/轉向混控。
- 4 輪混控：左側（M1/M3）、右側（M2/M4）差速控制。

2. 伺服控制
- Servo1：離散三段角。
- Servo2：查表角度映射，並保留診斷輸出。
- Servo3：中心峰值分段映射。

3. 時脈與中斷
- 控制迴圈使用 Timer2 產生約 20ms Flag，避免與 Servo 函式庫衝突。
- 伺服由 Arduino Servo 函式庫控制。
- 4 路 encoder 中斷輸入用於速度回授。

## 軟體架構

1. 通訊與映射層
- 檔案：include/Comms_Layer.h, src/Comms_Layer.cpp
- 責任：通道數值轉換、LUT 映射（C1/C3 等）

2. 數學層
- 檔案：include/Math_Layer.h, src/Math_Layer.cpp
- 責任：濾波器、PID、Servo 查表映射函式

3. 硬體層
- 檔案：include/Hardware_Layer.h, src/Hardware_Layer.cpp
- 責任：PWM 輸出、方向控制、encoder 計數、控制時序旗標

4. 主控流程
- 檔案：src/main.cpp
- 責任：整合遙控輸入、模式切換、馬達與伺服命令下發、序列監看

## 通道與控制對應

1. C1：前進/後退
- 閉迴路：C1 -> 目標計數 -> PID
- 開迴路：C1 -> 基礎 PWM

2. C3：轉向
- 開迴路：C3 -> 轉向 PWM（LUT）

3. C5：Servo1
- 離散三段（0 / 45 / 90）+ 遲滯

4. C2：Servo2
- 查表映射（極值優先）

5. C0：Servo3
- 中心峰值映射：
  - INPUT_MIN（1000）-> SERVO3_LEFT_ANGLE
  - SERVO3_CENTER_INPUT（1500）-> SERVO3_PEAK_ANGLE
  - INPUT_MAX（2000）-> SERVO3_RIGHT_ANGLE

## 馬達實作細節

1. 閉迴路模式
- 每次控制週期讀取 encoder 增量。
- 濾波器平滑速度估計。
- PID 計算輸出並限制於 PWM 範圍。

2. 開迴路模式
- C1 LUT 得到 BasePWM。
- C3 LUT 得到 TurnPWM。
- 左右輪命令：
  - Left = constrain(BasePWM + TurnPWM)
  - Right = constrain(BasePWM - TurnPWM)

3. 安全性
- 遙控讀值超界時回中心或安全值。
- encoder 異常尖峰過濾。
- 目標為零時提供停止/重置控制狀態。

## 伺服實作細節

1. Servo1
- 透過輸入濾波與遲滯減少門檻附近跳動。

2. Servo2
- 使用 mapServo2Lookup，查表加分段內插。
- 監看輸出含 CH2Raw、CH2F、Servo2Angle，便於診斷。

3. Servo3
- 使用 mapServo3CenterPeak。
- 主要參數集中於 config.h 的 ServoMotor namespace：
  - SERVO3_CENTER_INPUT
  - SERVO3_LEFT_ANGLE
  - SERVO3_PEAK_ANGLE
  - SERVO3_RIGHT_ANGLE

## 主要設定參數

1. 腳位與伺服參數
- 檔案：include/config.h
- 內容：馬達腳位、伺服腳位、伺服映射參數

2. 通道映射常數
- 檔案：include/Comms_Layer.h
- 內容：C1/C3 deadband、範圍與 LUT 對應常數

3. 控制與濾波
- 檔案：include/config.h
- 內容：PidConfig、FilterConfig

## 序列監看輸出

目前主程式會週期輸出以下資訊（用於調機與除錯）：

1. CH1, CH3
2. BasePWM, TurnPWM
3. Lcmd, Rcmd
4. Speed1~Speed4（開迴路回授）
5. Servo1Angle, Servo2Angle, Servo3Angle
6. CH2Raw, CH2F（Servo2 診斷）

## 開發與執行

1. 環境
- VS Code + PlatformIO
- Board: megaatmega2560

2. 編譯
```bash
pio run -e megaatmega2560
```

3. 上傳
```bash
pio run -e megaatmega2560 --target upload --upload-port COM10
```

4. 監看序列埠
```bash
pio device monitor -b 115200
```

## 專案目錄

```text
ASME_1/
├── include/
│   ├── config.h
│   ├── Comms_Layer.h
│   ├── Hardware_Layer.h
│   └── Math_Layer.h
├── src/
│   ├── main.cpp
│   ├── Comms_Layer.cpp
│   ├── Hardware_Layer.cpp
│   └── Math_Layer.cpp
├── platformio.ini
└── README.md
```

<!-- ## 後續建議

1. 將 Servo1 離散門檻與遲滯常數外移到 config.h，便於現場快速調機。
2. 增加轉向增益隨速度衰減 LUT，提升高速穩定性。
3. 補一份實機校正紀錄（各通道端點、伺服實際角、輪組方向）方便交接。 -->


