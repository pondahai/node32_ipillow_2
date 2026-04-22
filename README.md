# iPillow System (node32_ipillow_2)

這是一個基於 ESP32 的智慧枕頭控制系統，旨在通過氣壓傳感器、氣泵和電磁閥自動調整枕頭的高度，以提供最佳的睡眠舒適度。

## 1. 程式原理 (Program Principles)

系統的核心是一個狀態機 (State Machine)，負責管理從初始化、排氣、充氣到待機和自動高度調整的整個生命週期。

### 核心運作邏輯：
*   **物理數據計算**：系統根據使用者的性別、年齡、身高和體重，利用內建的數據表 (`data_tables.h`) 計算出建議的頭部和頸部支撐高度。
*   **氣壓監測**：使用三個 HX710B 氣壓傳感器即時監測監控室 (Monitor)、頸部氣囊 (Neck) 和頭部氣囊 (Head) 的壓力。
*   **氣動控制**：透過 74HC595 移位暫存器控制 5 個電磁閥 (V1-V5) 和 1 個氣泵 (Motor) 的啟閉，實現對氣囊的精確充氣或排氣。
*   **中位數濾波**：為了減少感測器讀數中的突波雜訊，實作了中位數濾波器 (Median Filter) 結合移動平均 (Moving Average) 來處理氣壓數據。
*   **藍牙互動**：支援透過藍牙 (Bluetooth Serial) 接收手機端的指令，進行模式切換、參數設定、手動控制及數據回傳。

## 2. 變數說明 (Variable Descriptions)

### 系統狀態 (SystemState)
*   `INIT`: 系統初始化。
*   `DRAIN_ALL`: 排除所有氣囊中的氣體。
*   `FILL_MONITOR`: 填充監控室氣囊。
*   `FILL_NECK`: 填充頸部氣囊。
*   `FILL_HEAD`: 填充頭部氣囊。
*   `STANDBY`: 待機模式，監測是否需要調整高度。
*   `ADJUSTING_HEIGHT`: 正在動態調整高度。
*   `MANUAL_CONTROL`: 手動控制模式。

### 關鍵硬體接腳與常數
*   `I2C_SDA/SCL`, `TFT_*`: LCD 顯示屏與 I2C 通訊接腳。
*   `S1/S2/S3_SCK_PIN/SDI_PIN`: 三組 HX710B 氣壓傳感器的時鐘與數據線。
*   `LCLK`, `DCLK`, `D595`: 74HC595 移位暫存器的控制接腳。
*   `PWM_PIN`: 氣泵馬達的 PWM 速度控制。

### 全域控制變數
*   `headNumber / neckNumber`: 目標高度 (cm)。
*   `currentHeadNumber / currentNeckNumber`: 當前估計的高度。
*   `pressureMonitor / pressureNeck / pressureHead`: 處理後的即時氣壓值。
*   `s1 ~ s6`: 內部控制標記，對應馬達與閥門的開關狀態。

## 3. 副程式說明 (Subroutine Descriptions)

### 核心生命週期
*   `setup()`: 初始化串列埠、595 暫存器、LCD、藍牙、壓力感測器及 PWM 輸出。
*   `loop()`: 主迴路，負責讀取壓力、處理藍牙指令及執行狀態機。

### 壓力處理
*   `readPressureSensors()`: 每 500ms 讀取一次所有感測器，並應用濾波。
*   `get_median_filter(newVal, buf)`: 實作 3 點中位數濾波，去除單次讀值突波。
*   `read_air_pressure(i, pressure, base)`: 封裝 HX710B 的底層讀取邏輯。

### 硬體控制
*   `init595()` / `set595(port, high_low)`: 初始化與設定 74HC595 的輸出位元。
*   `s_to_io()`: 將全域標記 `s1-s6` 的狀態同步到硬體（閥門與馬達），包含馬達的緩啟動邏輯。

### 邏輯計算與通訊
*   `calculation(gender, age, height, weight)`: 根據輸入參數在 `data_tables.h` 中檢索百分位數，並計算出推薦的頭部/頸部寬度與建議壓力。
*   `lookup_table(...)`: 在 PROGMEM 存儲的數據表中執行線性搜索與值檢索。
*   `parseBluetoothCommand(command)`: 解析接收到的字串指令（如 `SET`, `GET`, `MODE`, `MANUAL` 等）。
*   `displayImage(index)`: 在 LCD 上繪製存儲在 `g933.h` 中的圖形數據。

### 氣動執行動作
*   `drainAllChambers()` / `drainMonitorChamber()`: 執行排氣程序。
*   `fillMonitorChamber()` / `fillNeckChamber()` / `fillHeadChamber()`: 執行初始充氣程序。
*   `adjustHeight()`: 根據目標高度與當前高度的差異，計算所需的充/排氣時間並執行調整。
