# ASME_1 專案規範與開發指南

這是一個基於 PlatformIO 開發的 Arduino Mega 2560 專案。為了確保團隊開發順利，請所有開發成員務必遵守以下規範。

---

## 🛠 開發環境設定

- **IDE**: VS Code
- **Plugin**: PlatformIO IDE
- **Board**: Arduino Mega 2560
- **Baud Rate**: 115200 (請確保程式碼與設定檔一致)

---

## 🌿 Git 分支管理

為了保護主分支（`main`）的穩定性，請遵循以下流程：

- ⛔ **禁止直接推送到 `main` 分支**

### 開發流程

1. 從 `main` 建立自己的開發分支：
   ```bash
   git checkout -b feat/your-feature-name
   ```

2. 完成功能後，再發起 **Pull Request (PR)** 進行合併。

### ⚠️ 注意

- 在每次 `push` 之前，請務必先執行 `git pull` 以確保本地端程式碼與遠端同步。
- 🚫 **絕對禁止** 使用 `git push --force`。

---

## 📝 Commit 提交規範

提交訊息請統一使用 **`動詞: 描述內容`** 的格式。

### 常用動詞

- `init`: 專案初始化
- `add`: 新增功能、檔案或函式庫
- `fix`: 修復錯誤（Bug）
- `docs`: 修改文件（如 README）
- `refactor`: 重構程式碼（不影響功能的結構調整）
- `hotfix`: 緊急修復
- `feat`: 完成功能

### 範例

```
add: 新增伺服馬達夾取邏輯
fix: 修正感測器讀取錯誤
docs: 更新 README 開發規範
```

---

## 💻 程式碼風格 (Coding Style)

為了保持程式碼的可讀性，請遵守以下命名規則：

| 類型 | 規則 | 範例 |
|------|------|------|
| 變數 / 函式 | lowerCamelCase (小駝峰) | `motorSpeed`, `getSensorData()` |
| 類別 (Class) | PascalCase (大駝峰) | `ArmController` |
| 常量 / 宏 | SCREAMING_SNAKE (全大寫) | `MAX_PWM`, `SERVO_PIN` |
| 檔案名稱 | snake_case (蛇形命名) | `motor_control.cpp` |

---

## ⚙️ 硬體與函式庫管理

### 硬體定義

- 🚫 **禁止**在 `src` 的邏輯程式中直接使用「魔術數字」（例如：`digitalWrite(13, HIGH)`）。
- ✅ 請將所有腳位皆有定義，請到config.h查看或更改

### 函式庫管理

- 🚫 **禁止**手動下載 `.zip` 檔放置於專案內。
- ✅ 所有需要的函式庫必須寫在 `platformio.ini` 的 `lib_deps` 中。
- 如果不知道怎麼寫請看教學有教

---

## 📂 專案結構簡述

```
ASME_1/
├── .pio/            # 編譯產物 (已自動忽略，勿上傳)
├── include/         # 放置標頭檔 (.h)，如 config.h
├── lib/             # 自訂函式庫目錄
├── src/             # 放置主要的原始碼 (.cpp, .ino)
│   └── main.cpp     # 主程式入口
├── test/            # 測試程式目錄
├── platformio.ini   # 專案環境設定檔
└── README.md        # 專案說明文件
```

---

## 抽象層

第一層：IBUS通訊&非線性映射
第二層：主要邏輯與 FSM
第三層：數學演算
第四層：硬體控制

---

## 📦 已安裝的函式庫

- `Servo` (v1.3.0) - 伺服馬達控制

---

## 🚀 快速開始

1. 安裝 PlatformIO IDE 插件
2. 開啟專案資料夾
3. 編譯並上傳：
   ```bash
   pio run --target upload
   ```
4. 開啟序列埠監視器：
   ```bash
   pio device monitor
   ```

---


