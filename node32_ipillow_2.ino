// board: ESP32 Dev Module
// partition schema: Huge APP
// 添加必要的頭文件
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include <BluetoothSerial.h>
#include <HX710B.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include <movingAvg.h>
#include "data_tables.h"
#include "g933.h" 
#include <sys/time.h> 

// LCD and Touch configuration
#define I2C_SDA 21
#define I2C_SCL 22
#define TFT_CS 15
#define TFT_DC 0
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_RST -1
Adafruit_ST7789 *tft = NULL;

// Pressure sensors
#define S1_SCK_PIN 18 // detect(monitor)
#define S1_SDI_PIN 5
#define S2_SCK_PIN 23 // neck
#define S2_SDI_PIN 19
#define S3_SCK_PIN 32 // head
#define S3_SDI_PIN 33
HX710B air_press1(S1_SCK_PIN, S1_SDI_PIN); // detect
HX710B air_press2(S2_SCK_PIN, S2_SDI_PIN); // neck
HX710B air_press3(S3_SCK_PIN, S3_SDI_PIN); // head

// 595 Shift Register
#define LCLK 25
#define DCLK 27
#define D595 17
#define MOTOR 0
#define V1 5
#define V2 4
#define V3 3
#define V4 2
#define V5 1

#define PWM_PIN 12       // P12 對應 GPIO12
// #define PWM_CHANNEL 0    // 可用 0~15
#define PWM_FREQ 100    // 5KHz 頻率
#define PWM_RESOLUTION 8 // 8-bit (0~255)
uint8_t pwm_duty = 255; // 190=8V 210=9V 230=10V

// Timing constants
// --- MODIFIED --- Renamed variables for clarity
uint16_t MonitorInitialFillTime = 15000;
uint16_t NeckInitialFillTime = 7500;
uint16_t HeadInitialFillTime = 15000;
uint16_t NeckPumpingTimeIntervalPerCM = 3583;
uint16_t HeadPumpingTimeIntervalPerCM = 5835;

// State machine states
// --- REVERTED --- Removed partial reset states. Back to original.
enum SystemState {
  INIT,
  DRAIN_ALL,
  FILL_MONITOR,
  FILL_NECK,
  FILL_HEAD,
  STANDBY,
  ADJUSTING_HEIGHT,
  RESET_MONITOR,
  REFILL_MONITOR,
  MANUAL_CONTROL, // 
};

// Global variables
BluetoothSerial SerialBT;
unsigned char reg595;
SystemState currentState = INIT;
bool experiment_status = false; 

// Pressure variables
float pressureMonitor = 0, pressureMonitorBase = 0;
float pressureNeck = 0, pressureNeckBase = 0;
float pressureHead = 0, pressureHeadBase = 0;

// Height settings
int headNumber = 7;    // Default head height (cm)
int neckNumber = 10;   // Default neck height (cm)
int currentHeadNumber = 0;
int currentNeckNumber = 0;

int HSF = headNumber; //  7~14cm
int N1SF = neckNumber; //  10~16c
int N2SP;
int HLF = 13;
int N1LF = 15;
int N2LP;

// Control flags
bool s1 = false, s2 = false, s3 = false, s4 = false, s5 = false, s6 = false;

// Timing variables
unsigned long pumpStartTime = 0;
unsigned long requiredPumpTime = 0;

// Moving average filters
movingAvg pressureMonitorAvg(11);
movingAvg pressureNeckAvg(11);
movingAvg pressureHeadAvg(11);

// calculations
int pub_gender;
unsigned int pub_age;
unsigned int pub_height;
unsigned int pub_weight;
unsigned int pub_head_width;
unsigned int pub_neck_width;
unsigned int pub_shoulder_width;

// 圖像數據陣列
const unsigned char* images[] = {g933_header_data};
unsigned int image_widths[] = {g933_width};
unsigned int image_heights[] = {g933_height};
const unsigned char (*cmaps[])[256][3] = {&g933_header_data_cmap};
int currentImage = 0;

void logLocalTime() {
  struct tm timeinfo;
  char timeString[20];
  
  // 獲取當前時間
  time_t now;
  time(&now);

  // Unix時間戳從1970年開始，如果時間小於一個合理的近期值，表示尚未同步
  if (now < 86400) { 
    // Serial.println("Time not yet synchronized");
    return;
  }

  // 將時間轉換為本地時間結構
  localtime_r(&now, &timeinfo);
  
  // 格式化時間字串 YYYY-MM-DD HH:MM:SS
  strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
  
  // 印出時間
  Serial.print("Current Local Time: ");
  Serial.println(timeString);
}

void displayImage(int index) {
  tft->fillScreen(ST77XX_BLACK);
  const unsigned char* data = images[index];
  int width = image_widths[index];
  int height = image_heights[index];
  const unsigned char (*cmap)[256][3] = cmaps[index];
  tft->setAddrWindow(0, 0, width - 1, height - 1);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      uint8_t pixel[3];
      pixel[0] = (*cmap)[(unsigned char)data[0]][0];
      pixel[1] = (*cmap)[(unsigned char)data[0]][1];
      pixel[2] = (*cmap)[(unsigned char)data[0]][2];
      data++;
      // if(pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0){
      // }else{
        uint16_t color = tft->color565(pixel[0], pixel[1], pixel[2]);
        tft->pushColor(color);
      // }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("iPillow System Initializing...");


  // Initialize 595 shift register
  init595();
  s_to_io();
  Serial.println("Shift register initialized");
  // delay(1000);

  // Initialize LCD
  tft = new Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
  tft->init(240, 280);
  tft->setRotation(1);
  tft->fillScreen(ST77XX_BLACK);

  Serial.println("Drawing splash screen...");
  displayImage(currentImage); // 顯示初始圖像

  // Initialize Bluetooth
  String BTADDR = printDeviceAddressOnLCD();
  String BTNAME = "iPillow" + BTADDR;
  SerialBT.begin(BTNAME);
  Serial.println(BTNAME + ": " + "Bluetooth initialized");


  // Initialize pressure sensors
  air_press1.init();
  air_press2.init();
  air_press3.init();
  Serial.println("Pressure sensors initialized");

  // Initialize moving averages
  pressureMonitorAvg.begin();
  pressureNeckAvg.begin();
  pressureHeadAvg.begin();
  for(int i=0;i<10;i++){
    pressureMonitorAvg.reading(10000);
    pressureNeckAvg.reading(10000);
    pressureHeadAvg.reading(10000);
  }
  Serial.println("Moving averages initialized");

  // Start with draining all chambers
  currentState = DRAIN_ALL;
  Serial.println("Starting state: DRAIN_ALL");

  // PWM init.
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_PIN, 0);
}

int old_state = -1;

void loop() {
  // Read all pressure sensors
  readPressureSensors();

  // Handle Bluetooth communication
  handleBluetooth();

  // State machine
  // --- REVERTED --- Removed cases for partial reset states
  switch(currentState) {
    case INIT:
      if(old_state != currentState){
        old_state = currentState;
        Serial.println("State: INIT (should not be here)");
      }
      currentState = DRAIN_ALL;
      break;

    case DRAIN_ALL:
      if(old_state != currentState){
        old_state = currentState;
        Serial.println("State: DRAIN_ALL");
        headNumber = 7;
        neckNumber = 10;
      }
      drainAllChambers();
      break;

    case FILL_MONITOR:
      if(old_state != currentState){
        old_state = currentState;
        Serial.println("State: FILL_MONITOR");
      }
      fillMonitorChamber();
      break;

    case RESET_MONITOR:
      if(old_state != currentState){
        old_state = currentState;
        Serial.println("State: RESET_MONITOR");
      }
      drainMonitorChamber();
      break;
    case REFILL_MONITOR:
      if(old_state != currentState){
        old_state = currentState;
        Serial.println("State: FILL_MONITOR_ONLY");
      }
      refillMonitorChamber();
      break;

    case FILL_NECK:
      if(old_state != currentState){
        old_state = currentState;
        Serial.println("State: FILL_NECK");
      }
      fillNeckChamber();
      break;

    case FILL_HEAD:
      if(old_state != currentState){
        old_state = currentState;
        Serial.println("State: FILL_HEAD");
      }
      fillHeadChamber();
      break;

    case STANDBY:
      if(old_state != currentState){
        old_state = currentState;
        Serial.println("State: STANDBY");
      }
      standbyOperations();
      break;

    case ADJUSTING_HEIGHT:
      if(old_state != currentState){
        old_state = currentState;
        Serial.println("State: ADJUSTING_HEIGHT");
      }
      adjustHeight();
      break;

    case MANUAL_CONTROL:
      if(old_state != currentState){
        old_state = currentState;
        Serial.println("State: MANUAL_CONTROL");
        Serial.println("System is now in manual control mode.");
        Serial.println("Use 'S,<1-6>,<0-1>' to control outputs.");
        Serial.println("Use 'RESET' to exit manual mode.");
      }
      // 在手動模式下，我們不做任何事，只等待藍牙指令。
      // 藍牙指令的處理在 handleBluetooth() 中完成。
      break;

  }

  static unsigned long lastTimeLog = 0;
  if (millis() - lastTimeLog >= 5000) {
    lastTimeLog = millis();
    logLocalTime();
  }


  // Send pressure data via Bluetooth every second
  static unsigned long lastBTUpdate = 0;
  if (millis() - lastBTUpdate >= 1000) {
    lastBTUpdate = millis();
  }

  delay(10); // Small delay to prevent watchdog triggers
}

// Initialize 595 shift register
void init595() {
  pinMode(DCLK, OUTPUT);
  pinMode(LCLK, OUTPUT);
  pinMode(D595, OUTPUT);
  reg595 = 0x0;
}

// Set 595 shift register outputs
void set595(int port, int high_low) {
  if (high_low == HIGH) {
    reg595 |= (0x80 >> port);
  } else {
    reg595 &= ~(0x80 >> port);
  }

  digitalWrite(LCLK, LOW);
  shiftOut(D595, DCLK, LSBFIRST, reg595);
  digitalWrite(LCLK, HIGH);
}

// Control valves and motor
void s_to_io() {
  set595(V1, s2 ? HIGH : LOW);
  delay(50);
  set595(V2, s3 ? HIGH : LOW);
  delay(50);
  set595(V3, s4 ? HIGH : LOW);
  delay(50);
  set595(V4, s5 ? HIGH : LOW);
  delay(50);
  set595(V5, s6 ? HIGH : LOW);
  delay(50);
  if(s1){
    // motor on
    for(int i=100 ;i>0; i-=10){
      delay(100);
      set595(MOTOR, HIGH);
      ledcWrite(PWM_PIN, pwm_duty-i);
    }
  }else{
    // motor off
    set595(MOTOR, LOW);
    ledcWrite(PWM_PIN, 0);
  }
  delay(50);
}

// --- 新增：中位數濾波專用的歷史緩衝區 (檢疫站) ---
// 大小設為 3，足以過濾單次突波，且反應最快
uint32_t bufMonitor[3] = {0,0,0}; 
uint32_t bufNeck[3] = {0,0,0};
uint32_t bufHead[3] = {0,0,0};

// --- 新增：中位數計算函數 ---
// 這就是「過濾器」，負責存入新值並吐出中位數
uint32_t get_median_filter(uint32_t newVal, uint32_t *buf) {
  // 1. 推入新值 (由後往前覆蓋，丟棄最舊的)
  buf[0] = buf[1];
  buf[1] = buf[2];
  buf[2] = newVal;

  // 2. 複製出來排序 (以免打亂原本的時間順序)
  uint32_t temp[3] = {buf[0], buf[1], buf[2]};

  // 3. 簡單排序 (Bubble Sort for 3 elements)
  // 只需要找出中間值，不用排得很完美，這三行夠了
  if (temp[0] > temp[1]) { uint32_t t = temp[0]; temp[0] = temp[1]; temp[1] = t; }
  if (temp[1] > temp[2]) { uint32_t t = temp[1]; temp[1] = temp[2]; temp[2] = t; }
  if (temp[0] > temp[1]) { uint32_t t = temp[0]; temp[0] = temp[1]; temp[1] = t; }

  // 4. 回傳中間那個 (中位數)
  return temp[1];
}

// void readPressureSensors() {
//   static unsigned long lastReadTime = 0;
//   if (millis() - lastReadTime >= 100) { // Read every 100ms
//     float pressure;
//     uint32_t raw_data = 0;

//     // --- 1. Monitor Sensor ---
//     // 先讀取原始值 (注意：這裡要確認 read 函數能取得 raw data)
//     // 假設您的 read_air_pressure 有辦法改寫成回傳 raw，或者我們直接操作 sensor
//     // 為了不改動您太多底層，我們假設您能取得 raw data
//     // 這裡示範邏輯：
    
//     // 讀取 Monitor
//     if (air_press1.read(&raw_data, 1000UL) == HX710B_OK) { // 使用較短 timeout
//         // [關鍵步驟]：過濾突波
//         uint32_t cleanData = get_median_filter(raw_data, bufMonitor);
        
//         // 將乾淨的數據餵給原本的移動平均
//         pressureMonitor = pressureMonitorAvg.reading(raw_data);
        
//         // 處理 Base (這裡也要用乾淨數據)
//         if (pressureMonitorBase == 0 && cleanData != 0) {
//              pressureMonitorBase = cleanData;
//         }
//     }

//     // --- 2. Neck Sensor ---
//     if (air_press2.read(&raw_data, 1000UL) == HX710B_OK) {
//         uint32_t cleanData = get_median_filter(raw_data, bufNeck);
//         pressureNeck = pressureNeckAvg.reading(raw_data);
        
//         if (pressureNeckBase == 0 && cleanData != 0) {
//              pressureNeckBase = cleanData;
//         }
//     }

//     // --- 3. Head Sensor ---
//     if (air_press3.read(&raw_data, 1000UL) == HX710B_OK) {
//         uint32_t cleanData = get_median_filter(raw_data, bufHead);
//         pressureHead = pressureHeadAvg.reading(raw_data);
        
//         if (pressureHeadBase == 0 && cleanData != 0) {
//              pressureHeadBase = cleanData;
//         }
//     }

//     lastReadTime = millis();
//   }
// }


// Read all pressure sensors
void readPressureSensors() {
  static unsigned long lastReadTime = 0;
  if (millis() - lastReadTime > 500) { // Read every 500ms
    float pressure = 0;
    uint32_t cleanData;
    Serial.print("sensor raw data: ");
    // Read monitor pressure
    pressure = 0;
    read_air_pressure(1, &pressure, &pressureMonitorBase);
    
    Serial.print(pressure);
    cleanData = get_median_filter(pressure, bufMonitor);
    pressureMonitor = pressureMonitorAvg.reading((int)pressure);
    // 實驗: 在移動平均佇列中塞0
    pressureMonitorAvg.reading((int)0);
    pressureMonitorAvg.reading((int)0);
    // pressureMonitorAvg.reading((int)0);
 
    Serial.print(" ");
    // Read neck pressure
    pressure = 0;
    read_air_pressure(2, &pressure, &pressureNeckBase);
    
    Serial.print(pressure);
    cleanData = get_median_filter(pressure, bufNeck);
    pressureNeck = pressureNeckAvg.reading((int)pressure);
    pressureNeckAvg.reading((int)0);
    pressureNeckAvg.reading((int)0);
    // pressureNeckAvg.reading((int)0);
   
    Serial.print(" ");
    // Read head pressure
    pressure = 0;
    read_air_pressure(3, &pressure, &pressureHeadBase);
   
    Serial.print(pressure);
    cleanData = get_median_filter(pressure, bufHead);
    pressureHead = pressureHeadAvg.reading((int)pressure);
    pressureHeadAvg.reading((int)0);
    pressureHeadAvg.reading((int)0);
    // pressureHeadAvg.reading((int)0);
  
    Serial.println();
    lastReadTime = millis();
  }
}

// Read individual pressure sensor
void read_air_pressure(int i, float *pressure, float *pressure_base) {
  uint32_t data_raw = 0;
  HX710B* sensor = nullptr;

  switch(i) {
    case 1: sensor = &air_press1; break;
    case 2: sensor = &air_press2; break;
    case 3: sensor = &air_press3; break;
  }

    if (sensor && sensor->read(&data_raw, 1000UL) == HX710B_OK) {
      *pressure = (uint32_t)data_raw;
      
      if (*pressure_base == 0 && *pressure != 0) {
        *pressure_base = *pressure;
      }
    }
  
}

// Drain all chambers
void drainAllChambers() {
  static bool draining = false;
  static unsigned long drainStartTime = 0;

  if (!draining) {
    Serial.println("Starting to drain all chambers");

    // Start draining all chambers
    s1 = true;  // Motor on
    s2 = true;  // Detect valve open
    s3 = true;  // Neck valve open
    s4 = true;  // Head valve open
    s5 = false; // Pump in off
    s6 = false; // Pump in off
    s_to_io();

    draining = true;
    drainStartTime = millis();
    return;
  }

  if ((pressureMonitor < 1200000 && pressureNeck < 1000 && pressureHead < 1000) ||
      (millis() - drainStartTime > 180000)) {
    Serial.println("Draining complete or timeout reached");

    // Stop draining
    s1 = false; s2 = false; s3 = false; s4 = false; s5 = false; s6 = false;
    s_to_io();

    draining = false;
    currentState = FILL_MONITOR;
  }
}

// Fill monitor chamber
void fillMonitorChamber() {
  static bool filling = false;

  if (!filling) {
    Serial.println("Starting to fill monitor chamber");

    s1 = true; s2 = true; s3 = false; s4 = false; s5 = true; s6 = true;
    s_to_io();

    filling = true;
    pumpStartTime = millis();
    // --- MODIFIED --- Use new variable name
    requiredPumpTime = MonitorInitialFillTime;
    return;
  }

  if (millis() - pumpStartTime >= requiredPumpTime) {
    Serial.println("Monitor chamber filling complete");

    s1 = false; s2 = false; s3 = false; s4 = false; s5 = false; s6 = false;
    s_to_io();

    filling = false;
    currentState = FILL_NECK;
  }
}

// drain monitor chamber
void drainMonitorChamber() {
  static bool draining = false;
  static unsigned long drainStartTime = 0;

  if (!draining) {
    Serial.println("Starting to drain monitor chamber");

    s1 = true; s2 = true; s3 = false; s4 = false; s5 = false; s6 = false;
    s_to_io();

    draining = true;
    drainStartTime = millis();
    return;
  }

  if ((pressureMonitor < 1000 ) || (millis() - drainStartTime > 180000)) {
    Serial.println("Draining complete or timeout reached");

    s1 = false; s2 = false; s3 = false; s4 = false; s5 = false; s6 = false;
    s_to_io();

    draining = false;
    currentState = REFILL_MONITOR;
  }
}
// Refill monitor chamber
void refillMonitorChamber() {
  static bool filling = false;

  if (!filling) {
    Serial.println("Starting to fill monitor chamber");

    s1 = true; s2 = true; s3 = false; s4 = false; s5 = true; s6 = true;
    s_to_io();

    filling = true;
    pumpStartTime = millis();
    // --- MODIFIED --- Use new variable name
    requiredPumpTime = MonitorInitialFillTime;
    return;
  }

  if (millis() - pumpStartTime >= requiredPumpTime) {
    Serial.println("Monitor chamber filling complete");

    s1 = false; s2 = false; s3 = false; s4 = false; s5 = false; s6 = false;
    s_to_io();

    filling = false;
    currentState = STANDBY;
  }
}

// Fill neck chamber
void fillNeckChamber() {
  static bool filling = false;

  if (!filling) {
    s1 = true; s2 = false; s3 = true; s4 = false; s5 = true; s6 = true;
    s_to_io();

    filling = true;
    pumpStartTime = millis();
    // --- MODIFIED --- Use new variable name
    requiredPumpTime = NeckInitialFillTime;
    return;
  }

  if (millis() - pumpStartTime >= requiredPumpTime) {
    s1 = false; s2 = false; s3 = false; s4 = false; s5 = false; s6 = false;
    s_to_io();

    filling = false;
    currentNeckNumber = neckNumber;
    currentState = FILL_HEAD;
  }
}

// Fill head chamber
void fillHeadChamber() {
  static bool filling = false;

  if (!filling) {
    s1 = true; s2 = false; s3 = false; s4 = true; s5 = true; s6 = true;
    s_to_io();

    filling = true;
    pumpStartTime = millis();
    // --- MODIFIED --- Use new variable name
    requiredPumpTime = HeadInitialFillTime;
    return;
  }

  if (millis() - pumpStartTime >= requiredPumpTime) {
    s1 = false; s2 = false; s3 = false; s4 = false; s5 = false; s6 = false;
    s_to_io();

    filling = false;
    currentHeadNumber = headNumber;
    currentState = STANDBY;
  }
}

// Standby operations
void standbyOperations() {
  if (headNumber != currentHeadNumber || neckNumber != currentNeckNumber) {
    currentState = ADJUSTING_HEIGHT;
    return;
  }
}

// Adjust pillow height
void adjustHeight() {
  static bool adjusting = false;
  static bool adjustingHead = false;
  static bool adjustingNeck = false;
  static unsigned long adjustTimeNeeded = 0;

  if (!adjusting) {
    adjustingHead = (headNumber != currentHeadNumber);
    adjustingNeck = (neckNumber != currentNeckNumber);

    if (adjustingHead) {
      int cmDiff = headNumber - currentHeadNumber;
      adjustTimeNeeded = abs(cmDiff) * HeadPumpingTimeIntervalPerCM;

      Serial.print("Adjusting head height from "); Serial.print(currentHeadNumber);
      Serial.print("cm to "); Serial.print(headNumber);
      Serial.print("cm, time needed: "); Serial.print(adjustTimeNeeded); Serial.println("ms");

      s1 = true; s4 = true;
      if (cmDiff > 0) { s5 = true; s6 = true; } else { s5 = false; s6 = false; }
    } else if (adjustingNeck) {
      int cmDiff = neckNumber - currentNeckNumber;
      adjustTimeNeeded = abs(cmDiff) * NeckPumpingTimeIntervalPerCM;

      Serial.print("Adjusting neck height from "); Serial.print(currentNeckNumber);
      Serial.print("cm to "); Serial.print(neckNumber);
      Serial.print("cm, time needed: "); Serial.print(adjustTimeNeeded); Serial.println("ms");

      s1 = true; s3 = true;
      if (cmDiff > 0) { s5 = true; s6 = true; } else { s5 = false; s6 = false; }
    }

    s_to_io();
    adjusting = true;
    pumpStartTime = millis();
    return;
  }

  if (millis() - pumpStartTime >= adjustTimeNeeded) {
    s1 = false; s2 = false; s3 = false; s4 = false; s5 = false; s6 = false;
    s_to_io();

    if (adjustingHead) { currentHeadNumber = headNumber; }
    else if (adjustingNeck) { currentNeckNumber = neckNumber; }

    adjusting = false;
    currentState = STANDBY;
  }
}

// Handle Bluetooth communication
void handleBluetooth() {
  if (SerialBT.available()) {
    String received = SerialBT.readStringUntil('\n');
    received.trim();
    parseBluetoothCommand(received);
  }
}

// Parse Bluetooth commands
void parseBluetoothCommand(String command) {
  Serial.print("Received BT command: ");
  Serial.println(command);
  String originalCommand = command; 
  command.toUpperCase();

  int commaIndex;
  String parts[10];
  int partIndex = 0;

  while ((commaIndex = command.indexOf(',')) != -1 && partIndex < 10) {
    parts[partIndex++] = command.substring(0, commaIndex);
    command = command.substring(commaIndex + 1);
  }
  if (command.length() > 0 && partIndex < 10) {
    parts[partIndex++] = command;
  }

  if (parts[0].equalsIgnoreCase("EXPERIMENT")) {
    if (partIndex >= 2 && parts[1].equalsIgnoreCase("OK")) {
        experiment_status = true;
        Serial.println("Experiment status has been ENABLED.");
        SerialBT.println("OK"); // 回應確認指令已收到
        return; // 處理完畢，直接返回
    }
  }


  if(parts[0].equalsIgnoreCase("SYNCTIME")) {
    if (partIndex >= 2) {
      // 從原始指令中提取時間戳字串，以避免大小寫轉換問題
      String timestampStr = originalCommand.substring(originalCommand.indexOf(',') + 1);
      
      // 將時間戳字串轉換為 time_t (長整型)
      time_t received_time = atol(timestampStr.c_str());

      // 建立 timeval 結構來設定時間
      struct timeval tv;
      tv.tv_sec = received_time; // 設定秒
      tv.tv_usec = 0;             // 設定微秒

      // 設定系統時間
      settimeofday(&tv, NULL);

      Serial.print("System time synchronized to: ");
      Serial.println(timestampStr);

      // 將原始指令原封不動傳回
      SerialBT.println(originalCommand);
      
      // 顯示一次同步後的時間
      logLocalTime();
    }
    return; // 處理完畢，直接返回
  }


  if(parts[0].equalsIgnoreCase("MANUAL")) {
    if (currentState != MANUAL_CONTROL) {
      Serial.println("Entering MANUAL_CONTROL mode.");
      // 進入手動模式前，關閉所有輸出，確保初始狀態乾淨
      s1 = s2 = s3 = s4 = s5 = s6 = false;
      s_to_io();
      currentState = MANUAL_CONTROL;
    }
    return; // 處理完畢，直接返回
  }

  if (currentState == MANUAL_CONTROL) {
    if (parts[0].equalsIgnoreCase("S")) {
      if (partIndex >= 3) { // 確保指令有三個部分: 'S', index, state
        int index = parts[1].toInt();
        int state = parts[2].toInt();

        Serial.print("Manual control: s");
        Serial.print(index);
        Serial.print(" -> ");
        Serial.println(state == 1 ? "ON" : "OFF");

        switch(index) {
          case 1: s1 = (state == 1); break;
          case 2: s2 = (state == 1); break;
          case 3: s3 = (state == 1); break;
          case 4: s4 = (state == 1); break;
          case 5: s5 = (state == 1); break;
          case 6: s6 = (state == 1); break;
          default:
            Serial.println("Invalid index. Use 1-6.");
            return;
        }
        s_to_io(); // 將更新後的 s 狀態寫入硬體
      } else {
        Serial.println("Invalid 'S' command format. Use: S,<index>,<state>");
      }
    }
    // 在手動模式下，我們可能不希望處理其他指令，所以可以在這裡加上 return
    // return;  // 如果你希望手動模式下屏蔽所有其他指令，可以取消這行的註解
  }

  if(parts[0] == "USER") {
    calculation(parts[1].toInt(), parts[2].toInt(), parts[3].toInt(), parts[4].toInt());
    HLF = (pub_shoulder_width - pub_head_width) / 2 / 10;
    N1LF = (pub_shoulder_width - pub_neck_width) / 2 / 10;
    Serial.print("pub_shoulder_width: "); Serial.println(pub_shoulder_width);
    Serial.print("pub_head_width: "); Serial.println(pub_head_width);
    Serial.print("pub_neck_width: "); Serial.println(pub_neck_width);
  }

  if(parts[0] == "GET") {
    if (parts[1] == "INFT" ) {
      if (parts[2] == "ALL" ) {
        // --- MODIFIED --- Use new variable names
        String data = "Monitor: " + String(MonitorInitialFillTime) + "\n" + \
                      "Neck: " + String(NeckInitialFillTime) + "\n" + \
                      "Head: " + String(HeadInitialFillTime) + "\n";
        SerialBT.println(data);
      }
    }
  }

  if(parts[0] == "SET") {
    // --- MODIFIED --- All SET commands now only update parameters without triggering a state change.
    if (parts[1] == "MONITOR" ) {
      MonitorInitialFillTime = parts[2].toInt();
      Serial.print("Monitor Initial Fill Time set to: ");
      Serial.println(MonitorInitialFillTime);
    }
    else if (parts[1] == "NECK") {
      NeckInitialFillTime = parts[2].toInt();
      Serial.print("Neck Initial Fill Time set to: ");
      Serial.println(NeckInitialFillTime);
    }
    else if (parts[1] == "HEAD") {
      HeadInitialFillTime = parts[2].toInt();
      Serial.print("Head Initial Fill Time set to: ");
      Serial.println(HeadInitialFillTime);
    }
    else if (parts[1] == "NORM" ) {
      if (parts[4] == "HEAD" ) {
        headNumber = parts[5].toInt();
        headNumber = constrain(headNumber, 5, 20);
      } else if (parts[4] == "NECK" ) {
        neckNumber = parts[5].toInt();
        neckNumber = constrain(neckNumber, 5, 20);
      }
    }
    else if (parts[1] == "BEAU" ) {
      if (parts[2] == "HEAD" ) {
        Serial.println("BEAU HEAD " + String(parts[3]));
      } else if (parts[2] == "NECK" ) {
        Serial.println("BEAU NECK " + String(parts[3]));
      }
    }
    SerialBT.println("MCU,OK"); 
  }

  if(parts[0] == "RESET") {
    currentState = DRAIN_ALL;
  }

  if (parts[0] == "MODE") {
    // parts[1] (NORM|BEAU)
    SerialBT.println("MODE,OK");
  }

  if (parts[0] == "INIT") {
    if (parts[1] == "NORM" ) {
      if (parts[2] == "L" ) {
        String data = "gender=" + String(pub_gender) + "\n" + \
        "age=" + String(pub_age) + "\n" + \
        "height=" + String(pub_height) + "\n" + \
        "weight=" + String(pub_weight) + "\n" + \
        "head_width=" + String(pub_head_width) + "\n" + \
        "neck_width=" + String(pub_neck_width) + "\n" + \
        "shoulder_width=" + String(pub_shoulder_width) + "\n" + \
        "headNumber=" + String(headNumber) + "\n" + \
        "neckNumber=" + String(neckNumber) + "\n" + \
        "HSF=" + String(HSF) + "\n" + \
        "N1SF=" + String(N1SF) + "\n" + \
        "HLF=" + String(HLF) + "\n" + \
        "N1LF=" + String(N1LF) + "\n" + \
        "N2LP=" + String(pressureMonitor) + "\n";
        if (experiment_status) {
          SerialBT.println(data);
        }
        Serial.print("HLF: "); Serial.println(HLF); 
        Serial.print("N1LF: "); Serial.println(N1LF);
        headNumber = HLF;
        neckNumber = N1LF;
        SerialBT.println("INIT,OK,"+String(headNumber)+","+String(neckNumber));        
      }
      if (parts[2] == "S" ) {
        SerialBT.println("INIT,OK");
      }
    }
    if (parts[1] == "BEAU" ) {
        Serial.print("HLF: "); Serial.println(HLF); 
        Serial.print("N1LF: "); Serial.println(N1LF);
        headNumber = HLF;
        neckNumber = N1LF;
        SerialBT.println("INIT,OK,"+String(headNumber)+","+String(neckNumber));        
    }
  }

  if (parts[0] == "I"){
    sendSevenData();
  }
  if (parts[0] == "P"){
    sendPressureData();
  }
  if (parts[0] == "MOTORPWM"){
    pwm_duty = parts[1].toInt();
    s_to_io();
  }
  if (parts[0] == "DEBUG") {
    String data = "gender=" + String(pub_gender) + "\n" + \
      "age=" + String(pub_age) + "\n" + \
      "height=" + String(pub_height) + "\n" + \
      "weight=" + String(pub_weight) + "\n" + \
      "head_width=" + String(pub_head_width) + "\n" + \
      "neck_width=" + String(pub_neck_width) + "\n" + \
      "shoulder_width=" + String(pub_shoulder_width) + "\n" + \
      "headNumber=" + String(headNumber) + "\n" + \
      "neckNumber=" + String(neckNumber) + "\n" + \
      "currentHeadNumber=" + String(currentHeadNumber) + "\n" + \
      "currentNeckNumber=" + String(currentNeckNumber) + "\n" + \
      "HSF=" + String(HSF) + "\n" + \
      "N1SF=" + String(N1SF) + "\n" + \
      "HLF=" + String(HLF) + "\n" + \
      "N1LF=" + String(N1LF) + "\n" + \
      "N2LP=" + String(pressureMonitor) + "\n";
    SerialBT.println(data);
  }
}
// diff, state, onoff, last5, prev5, predict_pose, pose
//   
void sendSevenData() {
  String data = "0\n " + String(currentState) + "\n "+ String((s6<<5 | s5<<4 | s4<<3 | s3<<2 | s2<<1 | s1<<0)) +" 0\n 0\n 0\n 0\n";

  SerialBT.println(data);

}

// Send pressure data via Bluetooth
void sendPressureData() {
  String data = String(pressureMonitor) + " " + String(pressureNeck) + " " + String(pressureHead);
  SerialBT.println(data);
}



// 修改printDeviceAddressOnLCD函數
String printDeviceAddressOnLCD() {
  if(!btStarted()) {
    btStart();
  }
  esp_bluedroid_init();
  esp_bluedroid_enable();

  const uint8_t* point = esp_bt_dev_get_address();
  if(point == NULL) {
    tft->setCursor(10, 10);
    tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft->setTextSize(2);
    tft->print("iPillow BT ERR");
    return String("");
  }

  char addr[13];
  sprintf(addr, "%02X%02X", point[4], point[5]);
  Serial.print("bt addr: ");
  Serial.println(addr);

  tft->setCursor(10, 10);
  tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft->setTextSize(2);
  tft->print("  iPillow ");
  tft->println(addr);
  return String(addr);
}

//
// calculation
//


void lookup_table(const unsigned int table[][2] PROGMEM, unsigned int age_n, unsigned int *p5, unsigned int *p95){


  if(age_n < 7)             *p5=pgm_read_word(&table[0][0]), *p95=pgm_read_word(&table[0][1]);
  if(age_n >= 7 && age_n < 8) *p5=pgm_read_word(&table[1][0]), *p95=pgm_read_word(&table[1][1]);
  if(age_n >= 8 && age_n < 9) *p5=pgm_read_word(&table[2][0]), *p95=pgm_read_word(&table[2][1]);
  if(age_n >= 9 && age_n < 10) *p5=pgm_read_word(&table[3][0]), *p95=pgm_read_word(&table[3][1]);
  if(age_n >= 10 && age_n < 11) *p5=pgm_read_word(&table[4][0]), *p95=pgm_read_word(&table[4][1]);
  if(age_n >= 11 && age_n < 12) *p5=pgm_read_word(&table[5][0]), *p95=pgm_read_word(&table[5][1]);
  if(age_n >= 12 && age_n < 13) *p5=pgm_read_word(&table[6][0]), *p95=pgm_read_word(&table[6][1]);
  if(age_n >= 13 && age_n < 14) *p5=pgm_read_word(&table[7][0]), *p95=pgm_read_word(&table[7][1]);
  if(age_n >= 14 && age_n < 15) *p5=pgm_read_word(&table[8][0]), *p95=pgm_read_word(&table[8][1]);
  if(age_n >= 15 && age_n < 16) *p5=pgm_read_word(&table[9][0]), *p95=pgm_read_word(&table[9][1]);
  if(age_n >= 16 && age_n < 17) *p5=pgm_read_word(&table[10][0]), *p95=pgm_read_word(&table[10][1]);
  if(age_n >= 17 && age_n < 18) *p5=pgm_read_word(&table[11][0]), *p95=pgm_read_word(&table[11][1]);
  if(age_n >= 18 && age_n < 25) *p5=pgm_read_word(&table[12][0]), *p95=pgm_read_word(&table[12][1]);
  if(age_n >= 25 && age_n < 35) *p5=pgm_read_word(&table[13][0]), *p95=pgm_read_word(&table[13][1]);
  if(age_n >= 35 && age_n < 45) *p5=pgm_read_word(&table[14][0]), *p95=pgm_read_word(&table[14][1]);
  if(age_n >= 45 )           *p5=pgm_read_word(&table[15][0]), *p95=pgm_read_word(&table[15][1]);
}


void calculation(int gender, unsigned int input_age, unsigned int input_height, unsigned int input_weight){
  unsigned int age_n=input_age;
  unsigned int input;
  unsigned int delta95;
  unsigned int delta5;
  unsigned int p95,p5;
  unsigned int percent_height;
  unsigned int percent_weight;
  unsigned int percent_average;
  unsigned int head_width;
  unsigned int neck_width;
  unsigned int shoulder_width;
  unsigned int head_pressure;
  unsigned int neck_pressure;
  float y;

    if(gender == 0) {
      lookup_table(female_heights,age_n,&p5,&p95);
    }else{
      lookup_table(male_heights,age_n,&p5,&p95);
    }
    input= input_height * 10;
    delta95 = p95-input;
    delta5 = input-p5;
    percent_height=(95*delta5+05*delta95)/(delta95+delta5);
    if(percent_height>100)percent_height=100;

    if(gender == 0) {
      lookup_table(female_weights,age_n,&p5,&p95);
    }else{
      lookup_table(male_weights,age_n,&p5,&p95);
    }
    input= input_weight;
    delta95 = p95-input;
    delta5 = input-p5;
    percent_weight=(95*delta5+05*delta95)/(delta95+delta5);
    if(percent_weight>100)percent_weight=100;

    percent_average=(percent_height+percent_weight)/2;
    if(gender == 0) {
      lookup_table(female_head_width,age_n,&p5,&p95);
    }else{
      lookup_table(male_head_width,age_n,&p5,&p95);
    }
    head_width=((percent_average-5)*p95+(95-percent_average)*p5)/90;

    if(gender == 0) {
      lookup_table(female_neck_width,age_n,&p5,&p95);
    }else{
      lookup_table(male_neck_width,age_n,&p5,&p95);
    }
    neck_width=((percent_average-5)*p95+(95-percent_average)*p5)/90;

    if(gender == 0) {
      lookup_table(female_shoulder_width,age_n,&p5,&p95);
    }else{
      lookup_table(male_shoulder_width,age_n,&p5,&p95);
    }
    shoulder_width=((percent_average-5)*p95+(95-percent_average)*p5)/90;

    y=(0.048*((float)head_width/10.0) - 0.278) *1000.0;
    head_pressure = (y>0)?y:100;
    y=(0.1289*((float)neck_width/10.0) -1.5896) *1000.0;
    neck_pressure = (y>0)?y:100;

  pub_gender = gender;
  pub_age = input_age;
  pub_height = input_height;
  pub_weight = input_weight;
  pub_head_width = head_width;
  pub_neck_width = neck_width;
  pub_shoulder_width = shoulder_width;
}