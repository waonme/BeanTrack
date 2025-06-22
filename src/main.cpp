#include <M5Unified.h>
#include <M5UnitKmeterISO.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define KM_SDA   21
#define KM_SCL   22
#define I2C_FREQ 100000L
#define KM_ADDR  KMETER_DEFAULT_ADDR

constexpr uint16_t PERIOD_MS   = 1000;       // 1 秒周期
constexpr uint16_t BUF_SIZE    = 900;        // 15 分記録
constexpr float    TEMP_MIN    = 20.0f;      // グラフ下限
constexpr float    TEMP_MAX    = 270.0f;     // グラフ上限（緊急停止域表示用）
constexpr float    TEMP_DANGER = 245.0f;     // 危険温度
constexpr float    TEMP_CRITICAL = 260.0f;   // 緊急停止温度
// UI最適化：レイアウト定数
constexpr int16_t  HEADER_HEIGHT = 50;      // ヘッダー部分の高さ
constexpr int16_t  FOOTER_HEIGHT = 20;      // フッター部分の高さ
constexpr int16_t  GRAPH_X0    = 10;
constexpr int16_t  GRAPH_Y0    = HEADER_HEIGHT;
constexpr int16_t  GRAPH_W     = 300;
constexpr int16_t  GRAPH_H     = 240 - HEADER_HEIGHT - FOOTER_HEIGHT;  // 170px

constexpr uint8_t  LCD_BRIGHTNESS = 1;

enum DisplayMode {
  MODE_GRAPH = 0,
  MODE_STATS = 1,
  MODE_ROR = 2,
  MODE_GUIDE = 3,
  MODE_COUNT = 4
};

enum SystemState {
  STATE_STANDBY = 0,
  STATE_RUNNING = 1
};

enum RoastLevel {
  ROAST_LIGHT = 0,        // 浅煎り
  ROAST_MEDIUM_LIGHT = 1, // 中浅煎り
  ROAST_MEDIUM = 2,       // 中煎り
  ROAST_MEDIUM_DARK = 3,  // 中深煎り
  ROAST_DARK = 4,         // 深煎り
  ROAST_FRENCH = 5,       // フレンチロースト
  ROAST_COUNT = 6
};

enum RoastStage {
  STAGE_PREHEAT = 0,      // 予熱
  STAGE_CHARGE = 1,       // 投入
  STAGE_DRYING = 2,       // 蒸らし・乾燥
  STAGE_MAILLARD = 3,     // メイラード反応
  STAGE_FIRST_CRACK = 4,  // 1ハゼ
  STAGE_DEVELOPMENT = 5,  // 発達段階
  STAGE_SECOND_CRACK = 6, // 2ハゼ
  STAGE_FINISH = 7        // 排出
};

enum FirePower {
  FIRE_OFF = 0,      // 火力OFF
  FIRE_VERY_LOW = 1, // 極弱火
  FIRE_LOW = 2,      // 弱火
  FIRE_MEDIUM = 3,   // 中火
  FIRE_HIGH = 4,     // 強火
  FIRE_MAX = 5       // 最大火力
};

struct RoastTarget {
  float temp_min;
  float temp_max;
  float ror_min;
  float ror_max;
  uint16_t time_min;  // 秒
  uint16_t time_max;  // 秒
  const char* advice;
  FirePower recommended_fire; // 推奨火力
};

float    buf[BUF_SIZE];
uint16_t head = 0;
uint16_t count = 0;

// BLE UUIDs for Nordic UART Service
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

M5UnitKmeterISO kmeter;
BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t next_tick = 0;
uint8_t  km_err    = 0;
float    current_temp = 0;
uint32_t last_ble_send = 0;
constexpr uint32_t BLE_SEND_INTERVAL = 1000;  // Send data every 1 second

DisplayMode display_mode = MODE_GRAPH;
SystemState system_state = STATE_STANDBY;
bool need_full_redraw = true;
float last_temp = NAN;
int16_t last_graph_x = -1;
int16_t last_graph_y = -1;

float temp_min_recorded = INFINITY;
float temp_max_recorded = -INFINITY;
float temp_sum = 0.0f;

// RoR (Rate of Rise) calculation
float current_ror = 0.0f;
float current_ror_15s = 0.0f;  // 15秒RoR for quick response
float ror_buf[BUF_SIZE];
uint16_t ror_count = 0;
constexpr uint16_t ROR_INTERVAL = 60;  // 60 seconds for RoR calculation
constexpr uint16_t ROR_INTERVAL_15S = 15;  // 15 seconds for quick RoR

// Stall detection
bool stall_warning_active = false;
uint32_t last_stall_check = 0;

// Safety features
bool first_crack_confirmation_needed = false;
bool first_crack_confirmed = false;

// Non-blocking emergency alert system
bool emergency_active = false;
uint32_t emergency_beep_start = 0;
int emergency_beep_count = 0;
constexpr int MAX_EMERGENCY_BEEPS = 10;
constexpr uint32_t EMERGENCY_BEEP_INTERVAL = 300; // 300ms intervals

// Non-blocking stage change beeps
bool stage_beep_active = false;
uint32_t stage_beep_start = 0;
int stage_beep_count = 0;
constexpr int MAX_STAGE_BEEPS = 3;
constexpr uint32_t STAGE_BEEP_INTERVAL = 300; // 300ms between beeps
int stage_beep_frequencies[3] = {1000, 1200, 1500};
int stage_beep_durations[3] = {200, 200, 300};

// Non-blocking critical warning beeps
bool critical_beep_active = false;
uint32_t critical_beep_start = 0;
int critical_beep_count = 0;
constexpr int MAX_CRITICAL_BEEPS = 3;
constexpr uint32_t CRITICAL_BEEP_INTERVAL = 250; // 250ms between beeps

// Hysteresis values as constexpr
constexpr float FIRE_HYSTERESIS = 0.5f;  // °C hysteresis for fire power changes

// Melody system for elegant notifications
struct Melody {
  int notes[8];      // 8音のメロディ
  int duration_ms;   // 各音の長さ
};

// Theodore提言による各イベント用メロディ
constexpr Melody MELODY_STAGE_CHANGE = {{262, 294, 330, 349, 392, 440, 494, 523}, 150};  // C-D-E-F-G-A-B-C上昇
constexpr Melody MELODY_FIRST_CRACK = {{523, 494, 440, 392, 349, 330, 294, 262}, 120};    // C-B-A-G-F-E-D-C下降
constexpr Melody MELODY_EMERGENCY = {{880, 831, 784, 740, 698, 659, 622, 587}, 100};      // 高音から滑落
constexpr Melody MELODY_COMPLETION = {{392, 523, 392, 523, 392, 523, 659, 523}, 200};    // G-C完成旋律

// Temperature prediction for gas burner
class TemperaturePredictor {
private:
  float temp_history[10];
  int history_index = 0;
  bool history_full = false;
  
public:
  void addTemperature(float temp) {
    temp_history[history_index] = temp;
    history_index = (history_index + 1) % 10;
    if (history_index == 0) history_full = true;
  }
  
  float predictTemperatureIn30s() {
    if (!history_full && history_index < 3) return current_temp;
    
    float recent_ror = calculateRecentRoR();
    return current_temp + (recent_ror * 0.5f);  // 30秒後の予測
  }
  
  float calculateRecentRoR() {
    int samples = history_full ? 10 : history_index;
    if (samples < 2) return 0.0f;
    
    // 正しい最古インデックスの計算
    int oldestIdx = history_full ? history_index : 0;
    float oldest = temp_history[oldestIdx];
    float newest = temp_history[(history_index - 1 + 10) % 10];
    return (newest - oldest) / samples * 60.0f;  // °C/分
  }
};

TemperaturePredictor predictor;

// ローストレベル依存の安全温度計算
float getDangerTemp(RoastLevel level) {
  // French/Dark では深煎り用の高い閾値を使用
  if (level == ROAST_FRENCH || level == ROAST_DARK) {
    return 255.0f;  // French用：255°Cまで安全
  } else {
    return 245.0f;  // 通常焙煎：245°C
  }
}

float getCriticalTemp(RoastLevel level) {
  // French/Dark では緊急停止温度も高く設定
  if (level == ROAST_FRENCH || level == ROAST_DARK) {
    return 270.0f;  // French用：270°Cで緊急停止
  } else {
    return 260.0f;  // 通常焙煎：260°C
  }
}

// Roast guide variables
RoastLevel selected_roast_level = ROAST_MEDIUM;
RoastStage current_stage = STAGE_PREHEAT;
uint32_t roast_start_time = 0;
bool roast_guide_active = false;
float stage_start_temp = 0.0f;
uint32_t stage_start_time = 0;
bool first_crack_detected = false;
bool second_crack_detected = false;

// Audio notification variables
uint32_t last_beep_time = 0;
bool stage_change_beep_played = false;
bool critical_temp_warning_active = false;
uint32_t last_critical_warning = 0;

// Fire power recommendation
FirePower current_recommended_fire = FIRE_MEDIUM;
FirePower last_recommended_fire = FIRE_MEDIUM;

// Button long press detection
uint32_t btnC_press_start = 0;
bool btnC_long_press_handled = false;
uint32_t btnB_press_start = 0;
bool btnB_long_press_handled = false;
constexpr uint32_t LONG_PRESS_DURATION = 2000;  // 2 seconds

void drawGraph();
void drawCurrentValue();
void drawStats();
void drawRoR();
void drawGuide();
void addNewGraphPoint();
void handleButtons();
void drawStandbyScreen();
float getAverageTemp();
float calculateRoR();
float calculateRoR15s();
void updateRoRBuffer();
void checkStallCondition();
void updateRoastStage();
void drawRoastLevelSelection();
RoastTarget getRoastTarget(RoastStage stage, RoastLevel level);
const char* getRoastLevelName(RoastLevel level);
const char* getRoastStageName(RoastStage stage);
uint32_t getRoastElapsedTime();
float getStageElapsedTime();
void sendBLEData();
const char* getFirePowerName(FirePower fire);
FirePower calculateRecommendedFire();
void playBeep(int duration_ms, int frequency = 1000);
void playMelody(const Melody& melody);
void playStageChangeBeep();
void playCriticalWarningBeep();
void updateFirePowerRecommendation();
void forceNextStage();
void checkEmergencyConditions();
void handleNonBlockingBeeps();
float getNextStageKeyTemp(RoastStage stage, RoastLevel level);
const char* getGasAdjustmentAdvice(FirePower current_fire, FirePower target_fire);
void drawFooter(const char* instructions);
float getDangerTemp(RoastLevel level);
float getCriticalTemp(RoastLevel level);

// BLE Server callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      M5_LOGI("BLE Client connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      M5_LOGI("BLE Client disconnected");
    }
};

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  // I2C明示的初期化（M5Unifiedの実装変更に対応）
  Wire.begin(KM_SDA, KM_SCL, I2C_FREQ);
  
  while (!kmeter.begin(&Wire, KM_ADDR, KM_SDA, KM_SCL, I2C_FREQ)) {
    M5_LOGE("KMeterISO not found…再試行中");
    delay(500);
  }

  // Initialize BLE
  BLEDevice::init("M5Stack-Thermometer");
  
  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristic for TX (server to client)
  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  
  M5_LOGI("BLE started - Device name: M5Stack-Thermometer");

  M5.Lcd.setRotation(1);
  M5.Lcd.setBrightness(LCD_BRIGHTNESS);  // バッテリー寿命延長のため明度を調整
  M5.Lcd.fillScreen(TFT_BLACK);
  
  // Initialize statistics
  temp_min_recorded = INFINITY;
  temp_max_recorded = -INFINITY;
  temp_sum = 0.0f;
  current_ror = 0.0f;
  ror_count = 0;
  
  // Initialize roast guide
  selected_roast_level = ROAST_MEDIUM;
  current_stage = STAGE_PREHEAT;
  roast_guide_active = false;
  first_crack_detected = false;
  second_crack_detected = false;
  first_crack_confirmation_needed = false;
  first_crack_confirmed = false;
  
  // Draw initial standby screen
  drawStandbyScreen();
  
  next_tick = millis();
}


/**
 * ヘッダー領域の最適化表示（温度＋追加情報）
 */
void drawCurrentValue() {
  // ヘッダー領域全体をクリア
  M5.Lcd.fillRect(0, 0, 320, HEADER_HEIGHT, TFT_BLACK);
  
  // メイン温度表示
  M5.Lcd.setCursor(0, 5);
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
  M5.Lcd.printf("TEMP: %6.2f C", current_temp);
  
  // 火力推奨インジケーター表示（焙煎ガイド有効時のみ）
  if (roast_guide_active && system_state == STATE_RUNNING) {
    M5.Lcd.setCursor(220, 5);
    M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
    
    // 火力に応じて色を変える
    uint16_t fire_color = TFT_WHITE;
    switch(current_recommended_fire) {
      case FIRE_OFF: fire_color = TFT_DARKGREY; break;
      case FIRE_VERY_LOW: fire_color = TFT_BLUE; break;
      case FIRE_LOW: fire_color = TFT_CYAN; break;
      case FIRE_MEDIUM: fire_color = TFT_YELLOW; break;
      case FIRE_HIGH: fire_color = TFT_ORANGE; break;
      case FIRE_MAX: fire_color = TFT_RED; break;
    }
    
    M5.Lcd.setTextColor(fire_color);
    M5.Lcd.printf("[%s]", getFirePowerName(current_recommended_fire));
    M5.Lcd.setTextColor(TFT_WHITE);
  }
  
  // 第2行：RoR＋ステージ情報（空いていた余白を活用）
  M5.Lcd.setCursor(0, 25);
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_12);
  if (count >= ROR_INTERVAL) {
    M5.Lcd.printf("RoR: %.1f C/min", current_ror);
  } else {
    M5.Lcd.printf("RoR: Wait %ds", ROR_INTERVAL - count);
  }
  
  // 焙煎ガイド情報
  if (roast_guide_active) {
    M5.Lcd.setCursor(150, 25);
    M5.Lcd.printf("Stage: %s", getRoastStageName(current_stage));
    
    M5.Lcd.setCursor(0, 35);
    M5.Lcd.printf("Time: %02d:%02d", getRoastElapsedTime() / 60, getRoastElapsedTime() % 60);
  }
}

/**
 * 現在の buf 内容でグラフを再描画
 */
void drawGraph() {
  if (count == 0) return;

  M5.Lcd.startWrite();
  M5.Lcd.fillRect(GRAPH_X0-1, GRAPH_Y0-1, GRAPH_W+2, GRAPH_H+2, TFT_BLACK);
  M5.Lcd.drawRect(GRAPH_X0-1, GRAPH_Y0-1, GRAPH_W+2, GRAPH_H+2, TFT_WHITE);
  
  // ローストレベル依存の危険域表示
  float current_danger_temp = getDangerTemp(selected_roast_level);
  float current_critical_temp = getCriticalTemp(selected_roast_level);
  
  // 危険域（Danger〜Critical）を赤褐色帯で表示
  float danger_y_start = GRAPH_Y0 + GRAPH_H - (current_danger_temp - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * GRAPH_H;
  float danger_y_end = GRAPH_Y0 + GRAPH_H - (current_critical_temp - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * GRAPH_H;
  M5.Lcd.fillRect(GRAPH_X0, (int)danger_y_end, GRAPH_W, (int)(danger_y_start - danger_y_end), TFT_MAROON);
  
  // 緊急停止域（Critical〜TEMP_MAX）をより濃い赤で表示
  float critical_y_top = GRAPH_Y0;  // グラフ上端（TEMP_MAX=270°C位置）
  M5.Lcd.fillRect(GRAPH_X0, (int)critical_y_top, GRAPH_W, (int)(danger_y_end - critical_y_top), TFT_RED);

  // 温度範囲（固定）
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
  M5.Lcd.setCursor(GRAPH_X0 + GRAPH_W + 4, GRAPH_Y0 - 8);
  M5.Lcd.printf("%.0f", TEMP_MAX);
  M5.Lcd.setCursor(GRAPH_X0 + GRAPH_W + 4, GRAPH_Y0 + GRAPH_H - 8);
  M5.Lcd.printf("%.0f", TEMP_MIN);

  // 時間目盛
  M5.Lcd.setCursor(GRAPH_X0, GRAPH_Y0 + GRAPH_H + 6);
  M5.Lcd.printf("[15min]");

  // 折れ線
  uint16_t start = (count < BUF_SIZE) ? 0 : head;
  float prevX = -1, prevY = -1;
  for (uint16_t i = 0; i < count; ++i) {
    uint16_t idx = (start + i) % BUF_SIZE;
    float v = buf[idx];

    // 範囲外は無視（NaNでも可）
    if (v < TEMP_MIN || v > TEMP_MAX) continue;

    float x = GRAPH_X0 + (float)i / (BUF_SIZE - 1) * GRAPH_W;
    float y = GRAPH_Y0 + GRAPH_H - (v - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * GRAPH_H;

    if (prevX >= 0) {
      M5.Lcd.drawLine((int)prevX, (int)prevY, (int)x, (int)y, TFT_CYAN);
    }
    prevX = x;
    prevY = y;
  }

  M5.Lcd.endWrite();
}

void handleButtons() {
  // Handle Button A and B only when running
  if (system_state == STATE_RUNNING) {
    if (M5.BtnA.wasPressed()) {
      display_mode = (DisplayMode)((display_mode + 1) % MODE_COUNT);
      need_full_redraw = true;
      M5.Lcd.fillRect(0, GRAPH_Y0, 320, 240 - GRAPH_Y0, TFT_BLACK);
    }
    
  // Handle Button B press and long press
  if (M5.BtnB.isPressed()) {
    if (btnB_press_start == 0) {
      btnB_press_start = millis();
      btnB_long_press_handled = false;
    }
    
    // Check for long press (2 seconds) - Manual stage advance
    if (!btnB_long_press_handled && (millis() - btnB_press_start) >= LONG_PRESS_DURATION) {
      if (roast_guide_active && current_stage < STAGE_FINISH) {
        forceNextStage();
        btnB_long_press_handled = true;
        
        // Visual feedback
        M5.Lcd.fillRect(60, 100, 200, 40, TFT_BLACK);
        M5.Lcd.drawRect(60, 100, 200, 40, TFT_YELLOW);
        M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
        M5.Lcd.setTextColor(TFT_YELLOW);
        M5.Lcd.setCursor(70, 115);
        M5.Lcd.printf("MANUAL STAGE ADVANCE");
        M5.Lcd.setTextColor(TFT_WHITE);
        delay(300);
        need_full_redraw = true;
      }
    }
  } else if (btnB_press_start > 0) {
    // Button released - short press
    if (!btnB_long_press_handled) {
      // 1ハゼ確認処理
      if (first_crack_confirmation_needed && roast_guide_active) {
        first_crack_confirmed = true;
        first_crack_confirmation_needed = false;
        
        // 視覚的フィードバック
        M5.Lcd.fillRect(60, 100, 200, 40, TFT_BLACK);
        M5.Lcd.drawRect(60, 100, 200, 40, TFT_GREEN);
        M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
        M5.Lcd.setTextColor(TFT_GREEN);
        M5.Lcd.setCursor(70, 115);
        M5.Lcd.printf("1ST CRACK CONFIRMED");
        M5.Lcd.setTextColor(TFT_WHITE);
        playBeep(200, 1200);
        delay(300);
        need_full_redraw = true;
      } else if (display_mode == MODE_GUIDE && !roast_guide_active) {
        // 焙煎レベル変更
        selected_roast_level = (RoastLevel)((selected_roast_level + 1) % ROAST_COUNT);
      } else {
        // Reset statistics
        temp_min_recorded = INFINITY;
        temp_max_recorded = -INFINITY;
        temp_sum = 0.0f;
        for (int i = 0; i < count; i++) {
          float temp = buf[(head - count + i + BUF_SIZE) % BUF_SIZE];
          if (temp < temp_min_recorded) temp_min_recorded = temp;
          if (temp > temp_max_recorded) temp_max_recorded = temp;
          temp_sum += temp;
        }
        need_full_redraw = true;
      }
    }
    btnB_press_start = 0;
  }
  }
  
  // Handle Button C for start/stop and long press clear
  if (M5.BtnC.isPressed()) {
    if (btnC_press_start == 0) {
      btnC_press_start = millis();
      btnC_long_press_handled = false;
    }
    
    // Check for long press (2 seconds)
    if (!btnC_long_press_handled && (millis() - btnC_press_start) >= LONG_PRESS_DURATION) {
      // Long press: Clear all data and reset emergency state
      count = 0;
      head = 0;
      temp_min_recorded = INFINITY;
      temp_max_recorded = -INFINITY;
      temp_sum = 0.0f;
      current_ror = 0.0f;
      ror_count = 0;
      current_stage = STAGE_PREHEAT;
      roast_guide_active = false;
      first_crack_detected = false;
      second_crack_detected = false;
      emergency_active = false;  // Theodore提言：緊急停止状態もリセット
      emergency_beep_count = 0;
      need_full_redraw = true;
      
      // Visual feedback for clear
      M5.Lcd.fillScreen(TFT_BLACK);
      M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_24);
      M5.Lcd.setCursor(80, 120);
      M5.Lcd.println("*** DATA CLEARED ***");
      delay(500);
      
      if (system_state == STATE_RUNNING) {
        M5.Lcd.fillScreen(TFT_BLACK);
        M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("Real-Time Temperature");
        need_full_redraw = true;
      } else {
        drawStandbyScreen();
      }
      
      btnC_long_press_handled = true;
    }
  } else if (btnC_press_start > 0) {
    // Button released
    if (!btnC_long_press_handled) {
      // Short press: Start/Stop toggle
      if (system_state == STATE_STANDBY) {
        // Start monitoring
        system_state = STATE_RUNNING;
        M5.Lcd.fillScreen(TFT_BLACK);
        M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("Real-Time Temperature");
        need_full_redraw = true;
        next_tick = millis();
      } else {
        // Stop monitoring or start roast guide
        if (display_mode == MODE_GUIDE && !roast_guide_active) {
          // 焙煎ガイド開始
          roast_guide_active = true;
          current_stage = STAGE_PREHEAT;
          roast_start_time = millis();
          stage_start_time = millis();
          stage_start_temp = current_temp;
          first_crack_detected = false;
          second_crack_detected = false;
        } else {
          // Stop monitoring
          system_state = STATE_STANDBY;
          roast_guide_active = false;
          drawStandbyScreen();
        }
      }
    }
    btnC_press_start = 0;
  }
}

void addNewGraphPoint() {
  if (count < 2) return;
  
  // Get current and previous temperature values
  uint16_t current_idx = (head - 1 + BUF_SIZE) % BUF_SIZE;
  uint16_t prev_idx = (head - 2 + BUF_SIZE) % BUF_SIZE;
  
  float curr_temp = buf[current_idx];
  float prev_temp = buf[prev_idx];
  
  // Skip if out of range
  if (curr_temp < TEMP_MIN || curr_temp > TEMP_MAX || 
      prev_temp < TEMP_MIN || prev_temp > TEMP_MAX) return;
  
  // Calculate positions
  float x1, y1, x2, y2;
  
  if (count < BUF_SIZE) {
    // Not full buffer yet
    x1 = GRAPH_X0 + (float)(count - 2) / (BUF_SIZE - 1) * GRAPH_W;
    x2 = GRAPH_X0 + (float)(count - 1) / (BUF_SIZE - 1) * GRAPH_W;
  } else {
    // Full buffer, shift everything
    need_full_redraw = true;
    drawGraph();
    return;
  }
  
  y1 = GRAPH_Y0 + GRAPH_H - (prev_temp - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * GRAPH_H;
  y2 = GRAPH_Y0 + GRAPH_H - (curr_temp - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * GRAPH_H;
  
  // Draw new line segment
  M5.Lcd.drawLine((int)x1, (int)y1, (int)x2, (int)y2, TFT_CYAN);
}

void drawStats() {
  if (count == 0) {
    M5.Lcd.fillRect(0, GRAPH_Y0, 320, 240 - GRAPH_Y0, TFT_BLACK);
    M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
    M5.Lcd.setCursor(20, GRAPH_Y0 + 20);
    M5.Lcd.println("No data available");
    return;
  }
  
  M5.Lcd.fillRect(0, GRAPH_Y0, 320, 240 - GRAPH_Y0, TFT_BLACK);
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
  
  int y_pos = GRAPH_Y0 + 20;
  M5.Lcd.setCursor(20, y_pos);
  M5.Lcd.printf(">> Temperature Stats <<");
  
  y_pos += 30;
  M5.Lcd.setCursor(20, y_pos);
  M5.Lcd.printf("* Current: %.2f C", current_temp);
  
  y_pos += 25;
  M5.Lcd.setCursor(20, y_pos);
  M5.Lcd.printf("^ Maximum: %.2f C", temp_max_recorded);
  
  y_pos += 25;
  M5.Lcd.setCursor(20, y_pos);
  M5.Lcd.printf("v Minimum: %.2f C", temp_min_recorded);
  
  y_pos += 25;
  M5.Lcd.setCursor(20, y_pos);
  M5.Lcd.printf("~ Average: %.2f C", getAverageTemp());
  
  y_pos += 25;
  M5.Lcd.setCursor(20, y_pos);
  M5.Lcd.printf("# Data Points: %d", count);
  
  // Button instructions（統一フッターに移動）
  drawFooter("[A]Mode [B]Reset [C]Stop");
}

float getAverageTemp() {
  if (count == 0) return 0.0f;
  return temp_sum / count;
}

void drawStandbyScreen() {
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_24);
  M5.Lcd.setCursor(50, 80);
  M5.Lcd.println("*** Coffee Roast Monitor ***");
  
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
  M5.Lcd.setCursor(80, 120);
  M5.Lcd.println("--- STANDBY MODE ---");
  
  M5.Lcd.setCursor(40, 160);
  M5.Lcd.println("> Press Button C to START");
  
  M5.Lcd.setCursor(20, 190);
  M5.Lcd.println("> Hold Button C (2sec) to CLEAR");
  
  // Show data count if available
  if (count > 0) {
    M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_12);
    M5.Lcd.setCursor(10, 220);
    M5.Lcd.printf("Stored: %d points", count);
  }
}

float calculateRoR() {
  if (count < ROR_INTERVAL) {
    return 0.0f;  // Not enough data for RoR calculation
  }
  
  // Get temperature from 60 seconds ago
  uint16_t old_idx = (head - ROR_INTERVAL + BUF_SIZE) % BUF_SIZE;
  uint16_t current_idx = (head - 1 + BUF_SIZE) % BUF_SIZE;
  
  float old_temp = buf[old_idx];
  float current_temp_val = buf[current_idx];
  
  // 実測時間補正: サンプリング遅延を考慮（Theodore提言によるオーバーフロー防止）
  float actual_time_interval = (float)ROR_INTERVAL; // 基本は60秒
  if (millis() - next_tick > (uint32_t)PERIOD_MS * 2) {
    // サンプリング遅延が発生している場合の補正
    float delay_factor = (float)(millis() - next_tick) / ((float)PERIOD_MS * (float)ROR_INTERVAL);
    actual_time_interval = (float)ROR_INTERVAL * (1.0f + delay_factor);
  }
  
  // Calculate RoR: (ΔT) / (実測秒 / 60.0f) - 安全な除算
  if (actual_time_interval > 0.0f) {
    return (current_temp_val - old_temp) / (actual_time_interval / 60.0f);
  } else {
    return 0.0f;  // 異常値回避
  }
}

float calculateRoR15s() {
  if (count < ROR_INTERVAL_15S) {
    return 0.0f;  // Not enough data for 15s RoR calculation
  }
  
  // Get temperature from 15 seconds ago
  uint16_t old_idx = (head - ROR_INTERVAL_15S + BUF_SIZE) % BUF_SIZE;
  uint16_t current_idx = (head - 1 + BUF_SIZE) % BUF_SIZE;
  
  float old_temp = buf[old_idx];
  float current_temp_val = buf[current_idx];
  
  // Calculate 15s RoR and scale to per-minute
  return (current_temp_val - old_temp) * 4.0f;  // 15s * 4 = 60s
}

void checkStallCondition() {
  if (!roast_guide_active || millis() - last_stall_check < 5000) {
    return; // Check every 5 seconds
  }
  
  last_stall_check = millis();
  
  // Check for stall: RoR < 1°C/min for more than 60s after minimum stage time
  if (current_ror_15s < 1.0f && getStageElapsedTime() > 60) {
    if (!stall_warning_active) {
      stall_warning_active = true;
      playBeep(300, 1500); // Warning beep
    }
  } else {
    stall_warning_active = false;
  }
}

void updateRoRBuffer() {
  if (count >= ROR_INTERVAL) {
    // 最新値は head-1 の位置に格納
    uint16_t last_idx = (head + BUF_SIZE - 1) % BUF_SIZE;
    ror_buf[last_idx] = current_ror;
    if (ror_count < BUF_SIZE) ror_count++;
  }
}

void drawRoR() {
  M5.Lcd.fillRect(0, GRAPH_Y0, 320, 240 - GRAPH_Y0, TFT_BLACK);
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
  
  int y_pos = GRAPH_Y0 + 20;
  M5.Lcd.setCursor(20, y_pos);
  M5.Lcd.printf(">> Rate of Rise (RoR) <<");
  
  y_pos += 30;
  M5.Lcd.setCursor(20, y_pos);
  M5.Lcd.printf("Current: %.1f C/min", current_ror);
  
  y_pos += 25;
  M5.Lcd.setCursor(20, y_pos);
  if (count < ROR_INTERVAL) {
    M5.Lcd.printf("Wait %d more seconds", ROR_INTERVAL - count);
  } else {
    M5.Lcd.printf("10min projection: +%.0f C", current_ror * 10);
  }
  
  // RoR interpretation based on Scott Rao guidelines and roast stage
  y_pos += 30;
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
  M5.Lcd.setCursor(20, y_pos);
  
  // Stage-aware RoR evaluation
  if (roast_guide_active) {
    RoastTarget target = getRoastTarget(current_stage, selected_roast_level);
    if (current_ror > target.ror_max + 3) {
      M5.Lcd.setTextColor(TFT_RED);
      M5.Lcd.printf("[!] RoR: Too High for %s", getRoastStageName(current_stage));
    } else if (current_ror > target.ror_max) {
      M5.Lcd.setTextColor(TFT_ORANGE);
      M5.Lcd.printf("[^] RoR: High for %s", getRoastStageName(current_stage));
    } else if (current_ror >= target.ror_min) {
      M5.Lcd.setTextColor(TFT_GREEN);
      M5.Lcd.printf("[OK] RoR: Good for %s", getRoastStageName(current_stage));
    } else if (current_ror >= target.ror_min - 2) {
      M5.Lcd.setTextColor(TFT_CYAN);
      M5.Lcd.printf("[v] RoR: Low for %s", getRoastStageName(current_stage));
    } else {
      M5.Lcd.setTextColor(TFT_BLUE);
      M5.Lcd.printf("[!] RoR: Too Low - Risk of stall");
    }
  } else {
    // General RoR evaluation when not using roast guide
    if (current_ror > 20) {
      M5.Lcd.setTextColor(TFT_RED);
      M5.Lcd.printf("[!] RoR: Excessive (>20 C/min) - Risk flick");
    } else if (current_ror > 15) {
      M5.Lcd.setTextColor(TFT_ORANGE);
      M5.Lcd.printf("[^] RoR: Very High (15-20 C/min)");
    } else if (current_ror > 8) {
      M5.Lcd.setTextColor(TFT_YELLOW);
      M5.Lcd.printf("[^] RoR: High (8-15 C/min)");
    } else if (current_ror > 3) {
      M5.Lcd.setTextColor(TFT_GREEN);
      M5.Lcd.printf("[OK] RoR: Moderate (3-8 C/min)");
    } else if (current_ror > 0) {
      M5.Lcd.setTextColor(TFT_CYAN);
      M5.Lcd.printf("[v] RoR: Low (0-3 C/min)");
    } else {
      M5.Lcd.setTextColor(TFT_BLUE);
      M5.Lcd.printf("[-] RoR: Cooling (%.1f C/min)", current_ror);
    }
  }
  M5.Lcd.setTextColor(TFT_WHITE);
  
  // Simple RoR trend graph
  if (ror_count > 1) {
    y_pos += 40;
    M5.Lcd.setCursor(20, y_pos);
    M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_12);
    M5.Lcd.printf("RoR Trend (last 5min):");
    
    // Draw mini RoR graph
    int graph_y = y_pos + 20;
    int graph_x = 20;
    int graph_w = 280;
    int graph_h = 30;
    
    M5.Lcd.drawRect(graph_x, graph_y, graph_w, graph_h, TFT_WHITE);
    
    // Draw RoR trend line
    uint16_t points_to_show = (ror_count < 300) ? ror_count : 300;  // Last 5 minutes
    if (points_to_show > 1) {
      float prevX = -1, prevY = -1;
      for (uint16_t i = 0; i < points_to_show; ++i) {
        uint16_t idx = (head - points_to_show + i + BUF_SIZE) % BUF_SIZE;
        float ror_val = ror_buf[idx];
        
        float x = graph_x + (float)i / (points_to_show - 1) * graph_w;
        float y = graph_y + graph_h/2 - (ror_val / 20.0f) * (graph_h/2);  // Scale: ±20°C/min
        y = constrain(y, graph_y, graph_y + graph_h);
        
        if (prevX >= 0) {
          M5.Lcd.drawLine((int)prevX, (int)prevY, (int)x, (int)y, TFT_ORANGE);
        }
        prevX = x;
        prevY = y;
      }
    }
  }
  
  // Button instructions（統一フッターに移動）
  drawFooter("[A]Mode [B]Reset [C]Stop");
}

RoastTarget getRoastTarget(RoastStage stage, RoastLevel level) {
  // Scott Rao氏とSCAガイドラインに基づく現実的な焙煎プロファイル
  static const RoastTarget profiles[8][6] = {
    // STAGE_PREHEAT - 投入前予熱（180-200°C空焼き）
    {{180, 200, 0, 0, 0, 0, "Preheat roaster to 200C for light roast", FIRE_HIGH},
     {180, 200, 0, 0, 0, 0, "Preheat roaster to 190-200C", FIRE_HIGH},
     {180, 200, 0, 0, 0, 0, "Preheat roaster to 190C", FIRE_HIGH},
     {180, 200, 0, 0, 0, 0, "Preheat roaster to 180-190C", FIRE_HIGH},
     {180, 200, 0, 0, 0, 0, "Preheat roaster to 180C", FIRE_HIGH},
     {180, 200, 0, 0, 0, 0, "Preheat roaster to 180C", FIRE_HIGH}},
    
    // STAGE_CHARGE - 投入～転換点（1.5-2分、RoRピーク18-20→下降開始）
    {{80, 120, 15, 20, 90, 120, "Charge at 200C! RoR peak 18-20 C/min", FIRE_MEDIUM},
     {80, 120, 15, 18, 90, 120, "Charge at 190-200C! RoR peak ~18 C/min", FIRE_MEDIUM},
     {80, 120, 12, 15, 105, 135, "Charge at 190C! RoR peak ~15 C/min", FIRE_MEDIUM},
     {80, 120, 12, 15, 105, 150, "Charge at 180-190C! RoR peak 12-15 C/min", FIRE_MEDIUM},
     {80, 120, 10, 12, 120, 150, "Charge at 180C! RoR peak ~12 C/min", FIRE_MEDIUM},
     {80, 120, 10, 12, 120, 180, "Charge at 180C! RoR controlled start", FIRE_MEDIUM}},
    
    // STAGE_DRYING - 乾燥フェーズ（～150°C、240-360秒、RoR 10-15→下降）
    {{120, 150, 10, 15, 240, 300, "Drying phase - high RoR 10-15 C/min", FIRE_MEDIUM},
     {120, 150, 10, 12, 240, 330, "Drying phase - RoR 10-12 C/min", FIRE_MEDIUM},
     {120, 150, 8, 10, 270, 360, "Drying phase - RoR ~10 C/min", FIRE_MEDIUM},
     {120, 150, 8, 10, 270, 390, "Drying phase - RoR 8-10 C/min", FIRE_MEDIUM},
     {120, 150, 6, 8, 300, 420, "Drying phase - RoR ~8 C/min", FIRE_LOW},
     {120, 150, 6, 8, 300, 450, "Drying phase - RoR <8 C/min stable", FIRE_LOW}},
    
    // STAGE_MAILLARD - メイラード反応（150-195°C、180-300秒、RoR 5-8→下降）
    {{150, 195, 5, 8, 180, 240, "Maillard reaction - RoR 5-8 C/min", FIRE_LOW},
     {150, 195, 5, 8, 180, 270, "Maillard reaction - RoR 5-8 C/min", FIRE_LOW},
     {150, 200, 4, 5, 210, 300, "Maillard reaction - RoR ~5 C/min", FIRE_LOW},
     {150, 200, 3, 5, 240, 330, "Maillard reaction - RoR 3-5 C/min", FIRE_LOW},
     {150, 200, 2, 3, 270, 360, "Maillard reaction - RoR ~3 C/min", FIRE_LOW},
     {150, 200, 1, 3, 300, 420, "Maillard reaction - RoR <3 C/min", FIRE_VERY_LOW}},
    
    // STAGE_FIRST_CRACK - 1ハゼ（195-200°C、60-120秒、RoR急降下→谷）
    {{190, 200, 2, 5, 60, 90, "1st crack! RoR crash then stabilize", FIRE_VERY_LOW},
     {195, 200, 2, 5, 60, 90, "1st crack! Manage RoR crash", FIRE_VERY_LOW},
     {195, 205, 2, 4, 60, 90, "1st crack! RoR valley control", FIRE_VERY_LOW},
     {195, 205, 1, 4, 60, 120, "1st crack! RoR descent control", FIRE_VERY_LOW},
     {195, 205, 1, 3, 60, 120, "1st crack! Low RoR maintenance", FIRE_VERY_LOW},
     {195, 205, 1, 3, 60, 120, "1st crack! Gentle RoR control", FIRE_VERY_LOW}},
    
    // STAGE_DEVELOPMENT - 発達段階（DTR 15-25%、RoR 0-5→0へ）
    {{195, 205, 2, 5, 60, 120, "Light dev: 1-2min, RoR 5→0 C/min", FIRE_VERY_LOW},
     {200, 210, 2, 5, 90, 150, "Med-light dev: 1.5-2min, RoR <5 C/min", FIRE_VERY_LOW},
     {205, 218, 1, 3, 120, 180, "Medium dev: 2-3min, RoR 3-5→0 C/min", FIRE_VERY_LOW},
     {210, 225, 1, 3, 180, 210, "Med-dark dev: ~3min, RoR ≤3 C/min", FIRE_VERY_LOW},
     {220, 235, 1, 2, 180, 240, "Dark dev: 3-4min, RoR 2-3→0 C/min", FIRE_OFF},
     {230, 245, 0, 1, 240, 300, "French dev: 4min+, RoR ≈0 C/min", FIRE_OFF}},
    
    // STAGE_SECOND_CRACK - 2ハゼ（深煎りのみ、RoR≈0維持）
    {{225, 235, 0, 1, 30, 90, "Light 2nd crack - finish soon!", FIRE_OFF},
     {225, 240, 0, 1, 45, 105, "Med-light 2nd crack", FIRE_OFF},
     {230, 245, 0, 1, 60, 120, "Medium 2nd crack", FIRE_OFF},
     {230, 250, 0, 1, 90, 150, "Med-dark rolling 2nd crack", FIRE_OFF},
     {235, 255, 0, 1, 120, 180, "Dark rolling 2nd crack - watch oil", FIRE_OFF},
     {240, 260, 0, 1, 90, 150, "French intense 2nd crack - risk!", FIRE_OFF}},
    
    // STAGE_FINISH - 排出（各レベルの最終温度）
    {{200, 205, 0, 0, 0, 0, "Light roast complete at 205C", FIRE_OFF},
     {205, 210, 0, 0, 0, 0, "Med-light complete at 210C", FIRE_OFF},
     {210, 218, 0, 0, 0, 0, "Medium complete at 218C", FIRE_OFF},
     {220, 225, 0, 0, 0, 0, "Med-dark complete at 225C", FIRE_OFF},
     {230, 235, 0, 0, 0, 0, "Dark complete at 235C", FIRE_OFF},
     {240, 250, 0, 0, 0, 0, "French complete at 245C", FIRE_OFF}}
  };
  
  return profiles[stage][level];
}

const char* getRoastLevelName(RoastLevel level) {
  switch(level) {
    case ROAST_LIGHT: return "Light Roast";
    case ROAST_MEDIUM_LIGHT: return "Medium-Light";
    case ROAST_MEDIUM: return "Medium Roast";
    case ROAST_MEDIUM_DARK: return "Medium-Dark";
    case ROAST_DARK: return "Dark Roast";
    case ROAST_FRENCH: return "French Roast";
    default: return "Unknown";
  }
}

const char* getRoastStageName(RoastStage stage) {
  switch(stage) {
    case STAGE_PREHEAT: return "Preheat";
    case STAGE_CHARGE: return "Charge";
    case STAGE_DRYING: return "Drying";
    case STAGE_MAILLARD: return "Maillard";
    case STAGE_FIRST_CRACK: return "1st Crack";
    case STAGE_DEVELOPMENT: return "Development";
    case STAGE_SECOND_CRACK: return "2nd Crack";
    case STAGE_FINISH: return "Finish";
    default: return "Unknown";
  }
}

uint32_t getRoastElapsedTime() {
  if (roast_start_time == 0) return 0;
  return (millis() - roast_start_time) / 1000;
}

float getStageElapsedTime() {
  if (stage_start_time == 0) return 0;
  return (millis() - stage_start_time) / 1000.0f;
}

void updateRoastStage() {
  if (!roast_guide_active) return;
  
  uint32_t total_elapsed = getRoastElapsedTime();
  float stage_elapsed = getStageElapsedTime();
  RoastStage prev_stage = current_stage;
  RoastTarget current_target = getRoastTarget(current_stage, selected_roast_level);
  
  // Scott Rao氏のガイドラインに基づく段階判定ロジック
  switch(current_stage) {
    case STAGE_PREHEAT:
      // 予熱完了：温度条件またはユーザー操作で投入段階へ
      if ((current_temp >= 180 && current_temp <= 200) || stage_elapsed > 300) {
        current_stage = STAGE_CHARGE;
        stage_start_time = millis();
        stage_start_temp = current_temp;
      }
      break;
      
    case STAGE_CHARGE:
      {
        // 転換点通過の判定：1.5-2分の範囲で転換点検出
        bool turning_point_passed = (stage_elapsed > 90 && stage_elapsed < 180);
        bool temp_rising = (current_ror > 5);  // 上昇に転じている
        
        if (turning_point_passed && temp_rising) {
          current_stage = STAGE_DRYING;
          stage_start_time = millis();
          stage_start_temp = current_temp;
        }
        // 安全措置：3分経過で強制移行
        else if (stage_elapsed > 180) {
          current_stage = STAGE_DRYING;
          stage_start_time = millis();
          stage_start_temp = current_temp;
        }
      }
      break;
      
    case STAGE_DRYING:
      {
        // 乾燥フェーズ：温度150°C到達かつ適正時間経過
        bool temp_reached = (current_temp >= 150);
        bool min_time_met = (stage_elapsed >= current_target.time_min);
        bool max_time_exceeded = (stage_elapsed > current_target.time_max);
        
        // 全体時間の30-40%経過も考慮（SCAガイドライン）
        bool time_ratio_ok = (total_elapsed > 240);  // 最低4分は乾燥
        
        if ((temp_reached && min_time_met && time_ratio_ok) || max_time_exceeded) {
          current_stage = STAGE_MAILLARD;
          stage_start_time = millis();
          stage_start_temp = current_temp;
        }
      }
      break;
      
    case STAGE_MAILLARD:
      {
        // メイラード反応：190-200°C近辺でRoRが5°C/min以下
        bool temp_threshold = (current_temp >= 190);
        bool ror_controlled = (current_ror <= 8);  // まだ下降中
        bool min_time_met = (stage_elapsed >= current_target.time_min);
        
        // 1ハゼ検出のための条件
        if (temp_threshold && ror_controlled && min_time_met) {
          if (!first_crack_confirmed && current_temp >= 195) {
            first_crack_confirmation_needed = true;
          }
          
          // 自動移行または手動確認
          if (first_crack_confirmed || (current_temp >= 200 && current_ror <= 5)) {
            current_stage = STAGE_FIRST_CRACK;
            stage_start_time = millis();
            stage_start_temp = current_temp;
            first_crack_detected = true;
            first_crack_confirmation_needed = false;
            first_crack_confirmed = false;
          }
        }
        
        // 強制移行（温度が高すぎる場合）
        if (current_temp >= 205) {
          current_stage = STAGE_FIRST_CRACK;
          stage_start_time = millis();
          stage_start_temp = current_temp;
          first_crack_detected = true;
        }
      }
      break;
      
    case STAGE_FIRST_CRACK:
      {
        // 1ハゼ：60-120秒で終了、RoRクラッシュ～回復を監視
        bool min_time_met = (stage_elapsed >= 60);
        bool max_time_reached = (stage_elapsed >= 120);
        
        if (min_time_met || max_time_reached) {
          current_stage = STAGE_DEVELOPMENT;
          stage_start_time = millis();
          stage_start_temp = current_temp;
        }
      }
      break;
      
    case STAGE_DEVELOPMENT:
      {
        RoastTarget dev_target = getRoastTarget(STAGE_DEVELOPMENT, selected_roast_level);
        
        // DTR計算（Development Time Ratio）
        float total_time_so_far = total_elapsed;
        float dev_time = stage_elapsed;
        float current_dtr = (dev_time / total_time_so_far) * 100.0f;
        
        // 条件判定
        bool min_time_met = (dev_time >= dev_target.time_min);
        bool temp_target_reached = (current_temp >= dev_target.temp_max);
        bool dtr_sufficient = (current_dtr >= 15.0f);  // 最低15%のDTR
        bool max_dtr_exceeded = (current_dtr >= 25.0f); // 最大25%のDTR
        
        // 深煎り判定：2ハゼ域に進むか終了か
        if ((min_time_met && temp_target_reached && dtr_sufficient) || max_dtr_exceeded) {
          if (selected_roast_level >= ROAST_MEDIUM_DARK && current_temp >= 220) {
            current_stage = STAGE_SECOND_CRACK;
            stage_start_time = millis();
            stage_start_temp = current_temp;
            second_crack_detected = true;
          } else {
            current_stage = STAGE_FINISH;
            stage_start_time = millis();
          }
        }
        
        // 緊急停止条件
        if (current_temp >= getCriticalTemp(selected_roast_level)) {
          current_stage = STAGE_FINISH;
          stage_start_time = millis();
        }
      }
      break;
      
    case STAGE_SECOND_CRACK:
      {
        RoastTarget sc_target = getRoastTarget(STAGE_SECOND_CRACK, selected_roast_level);
        
        // 2ハゼ：時間と温度の両方で判定
        bool min_time_met = (stage_elapsed >= sc_target.time_min);
        bool temp_target_reached = (current_temp >= sc_target.temp_max);
        bool critical_temp_reached = (current_temp >= getCriticalTemp(selected_roast_level));
        
        if ((min_time_met && temp_target_reached) || critical_temp_reached) {
          current_stage = STAGE_FINISH;
          stage_start_time = millis();
        }
      }
      break;
      
    case STAGE_FINISH:
      // 排出段階、手動でリセット
      break;
  }
  
  // 段階が変わった場合に音声通知
  if (prev_stage != current_stage && !stage_change_beep_played) {
    playStageChangeBeep();
    stage_change_beep_played = true;
  } else if (prev_stage == current_stage) {
    stage_change_beep_played = false;
  }
}

void drawRoastLevelSelection() {
  M5.Lcd.fillRect(0, GRAPH_Y0, 320, 240 - GRAPH_Y0, TFT_BLACK);
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
  
  int y_pos = GRAPH_Y0 + 10;
  M5.Lcd.setCursor(60, y_pos);
  M5.Lcd.printf(">> Select Roast Level <<");
  
  y_pos += 30;
  
  // 2列レイアウトで表示
  for (int i = 0; i < ROAST_COUNT; i++) {
    int col = i % 2;  // 0 or 1
    int row = i / 2;  // 0, 1, 2
    
    int x_pos = col == 0 ? 20 : 170;  // 左列と右列
    int current_y = y_pos + row * 20;
    
    M5.Lcd.setCursor(x_pos, current_y);
    if (i == selected_roast_level) {
      M5.Lcd.setTextColor(TFT_YELLOW);
      M5.Lcd.printf("> %s", getRoastLevelName((RoastLevel)i));
      M5.Lcd.setTextColor(TFT_WHITE);
    } else {
      M5.Lcd.printf("  %s", getRoastLevelName((RoastLevel)i));
    }
  }
  
  // ボタン指示（統一フッターに移動）
  drawFooter("[B]Change [C]Start");
}

void drawGuide() {
  // ガイド表示領域：ヘッダーとフッターを除いた範囲
  int content_height = 240 - HEADER_HEIGHT - FOOTER_HEIGHT;
  M5.Lcd.fillRect(0, GRAPH_Y0, 320, content_height, TFT_BLACK);
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
  
  int y_pos = GRAPH_Y0 + 10;
  
  // 焙煎レベルと段階表示
  M5.Lcd.setCursor(10, y_pos);
  M5.Lcd.printf("%s - %s", getRoastLevelName(selected_roast_level), getRoastStageName(current_stage));
  
  // 段階プログレスバー
  y_pos += 15;
  int progress_width = 280;
  int progress_x = 20;
  int stage_progress = (current_stage * progress_width) / 7;  // 8段階
  M5.Lcd.drawRect(progress_x, y_pos, progress_width, 8, TFT_WHITE);
  M5.Lcd.fillRect(progress_x + 1, y_pos + 1, stage_progress, 6, TFT_GREEN);
  
  // 現在ステージ内の詳細進行率
  y_pos += 10;
  RoastTarget stage_target = getRoastTarget(current_stage, selected_roast_level);
  float stage_elapsed = getStageElapsedTime();
  float stage_total_time = stage_target.time_max > 0 ? stage_target.time_max : stage_target.time_min + 60;
  float stage_progress_pct = stage_elapsed / stage_total_time;
  if (stage_progress_pct > 1.0f) stage_progress_pct = 1.0f;
  
  int detail_progress = (int)(stage_progress_pct * progress_width);
  M5.Lcd.drawRect(progress_x, y_pos, progress_width, 6, TFT_DARKGREY);
  uint16_t detail_color = stage_progress_pct > 0.8f ? TFT_YELLOW : TFT_CYAN;
  M5.Lcd.fillRect(progress_x + 1, y_pos + 1, detail_progress, 4, detail_color);
  
  // 段階インジケーター
  for (int i = 0; i < 8; i++) {
    int dot_x = progress_x + (i * progress_width / 7);
    uint16_t color = (i <= current_stage) ? TFT_YELLOW : TFT_DARKGREY;
    M5.Lcd.fillCircle(dot_x, y_pos + 4, 3, color);
  }
  
  y_pos += 25;  // プログレスバー拡張分のスペースを追加
  M5.Lcd.setCursor(10, y_pos);
  M5.Lcd.printf("Time: %02d:%02d", getRoastElapsedTime() / 60, getRoastElapsedTime() % 60);
  
  // 現在の目標値
  RoastTarget target = getRoastTarget(current_stage, selected_roast_level);
  
  y_pos += 20;
  M5.Lcd.setCursor(10, y_pos);
  // 三層ガイド表示（初心者向け明確化）
  M5.Lcd.setTextColor(TFT_CYAN);
  M5.Lcd.printf("Maintain: %.0f-%.0f C", target.temp_min, target.temp_max);
  M5.Lcd.setTextColor(TFT_WHITE);
  
  y_pos += 15;
  M5.Lcd.setCursor(10, y_pos);
  // 次段階の鍵温度表示
  float next_key_temp = getNextStageKeyTemp(current_stage, selected_roast_level);
  if (next_key_temp > 0) {
    bool key_temp_reached = current_temp >= next_key_temp;
    M5.Lcd.setTextColor(key_temp_reached ? TFT_GREEN : TFT_RED);
    M5.Lcd.printf("Next Step: %s%.0f C", key_temp_reached ? "OK" : ">=", next_key_temp);
    M5.Lcd.setTextColor(TFT_WHITE);
  } else {
    M5.Lcd.setTextColor(TFT_YELLOW);
    M5.Lcd.printf("Next Step: Time Based");
    M5.Lcd.setTextColor(TFT_WHITE);
  }
  
  y_pos += 15;
  M5.Lcd.setCursor(10, y_pos);
  // 最小時間表示
  float elapsed = getStageElapsedTime();
  bool min_time_met = elapsed >= target.time_min;
  M5.Lcd.setTextColor(min_time_met ? TFT_GREEN : TFT_YELLOW);
  M5.Lcd.printf("Min Time: %s%ds (%.0fs)", min_time_met ? "OK" : "", target.time_min, elapsed);
  M5.Lcd.setTextColor(TFT_WHITE);
  
  y_pos += 16;
  M5.Lcd.setCursor(10, y_pos);
  M5.Lcd.printf("RoR: %.1f-%.1f C/min", target.ror_min, target.ror_max);
  
  // 火力推奨表示
  y_pos += 16;
  M5.Lcd.setCursor(10, y_pos);
  M5.Lcd.setTextColor(TFT_ORANGE);
  M5.Lcd.printf("Fire: %s", getFirePowerName(current_recommended_fire));
  M5.Lcd.setTextColor(TFT_WHITE);
  
  // 現在の状態評価
  y_pos += 18;
  M5.Lcd.setCursor(10, y_pos);
  if (current_temp < target.temp_min) {
    M5.Lcd.setTextColor(TFT_BLUE);
    M5.Lcd.printf("[v] Temp: LOW (%.1f C)", current_temp);
  } else if (current_temp > target.temp_max) {
    M5.Lcd.setTextColor(TFT_RED);
    M5.Lcd.printf("[!] Temp: HIGH (%.1f C)", current_temp);
  } else {
    M5.Lcd.setTextColor(TFT_GREEN);
    M5.Lcd.printf("[OK] Temp: OK (%.1f C)", current_temp);
  }
  M5.Lcd.setTextColor(TFT_WHITE);
  
  y_pos += 20;
  M5.Lcd.setCursor(10, y_pos);
  if (current_ror < target.ror_min) {
    M5.Lcd.setTextColor(TFT_BLUE);
    M5.Lcd.printf("[v] RoR: LOW (%.1f)", current_ror);
  } else if (current_ror > target.ror_max) {
    M5.Lcd.setTextColor(TFT_RED);
    M5.Lcd.printf("[!] RoR: HIGH (%.1f)", current_ror);
  } else {
    M5.Lcd.setTextColor(TFT_GREEN);
    M5.Lcd.printf("[OK] RoR: OK (%.1f)", current_ror);
  }
  M5.Lcd.setTextColor(TFT_WHITE);
  
  // 表示可能領域の制限チェック
  int max_y = 240 - FOOTER_HEIGHT - 15;  // フッター上部マージン
  
  // 1ハゼ確認表示（優先度高）
  if (first_crack_confirmation_needed && y_pos < max_y - 25) {
    y_pos += 15;
    M5.Lcd.setTextColor(TFT_YELLOW);
    M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_12);
    M5.Lcd.setCursor(10, y_pos);
    M5.Lcd.printf(">>> 1st Crack? Press B <<<");
    M5.Lcd.setTextColor(TFT_WHITE);
  }
  
  // 重要警告（優先度最高）
  if (current_stage == STAGE_FINISH && y_pos < max_y - 20) {
    y_pos += 15;
    M5.Lcd.setTextColor(TFT_RED);
    M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_12);
    M5.Lcd.setCursor(30, y_pos);
    M5.Lcd.printf("*** DROP BEANS NOW! ***");
    M5.Lcd.setTextColor(TFT_WHITE);
  }
  
  // 下部情報（スペースが許す場合のみ）
  if (y_pos < max_y - 35) {
    y_pos += 12;
    M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_12);
    M5.Lcd.setCursor(10, y_pos);
    M5.Lcd.printf("Pred: %.0f°C | %s", predictor.predictTemperatureIn30s(), 
                  getGasAdjustmentAdvice(last_recommended_fire, current_recommended_fire));
  }
  
  // ボタン指示（統一フッターに移動）
  if (first_crack_confirmation_needed) {
    drawFooter("[A]Mode [B]Confirm 1st Crack [C]Stop");
  } else if (roast_guide_active && current_stage < STAGE_FINISH) {
    drawFooter("[A]Mode [B-Hold]Next Stage [C]Stop");
  } else {
    drawFooter("[A]Mode [C]Stop");
  }
}

const char* getFirePowerName(FirePower fire) {
  switch(fire) {
    case FIRE_OFF: return "OFF";
    case FIRE_VERY_LOW: return "極弱火";
    case FIRE_LOW: return "弱火";
    case FIRE_MEDIUM: return "中火";
    case FIRE_HIGH: return "強火";
    case FIRE_MAX: return "最大火力";
    default: return "不明";
  }
}

FirePower calculateRecommendedFire() {
  if (!roast_guide_active) return FIRE_MEDIUM;
  
  RoastTarget target = getRoastTarget(current_stage, selected_roast_level);
  FirePower base_fire = target.recommended_fire;
  
  // Scott Rao原則：「常に下降するRoR」を維持する火力制御
  float ror_trend = 0.0f;
  if (count >= 3) {
    // 直近3点のRoR傾向を計算
    uint16_t idx1 = (head - 3 + BUF_SIZE) % BUF_SIZE;
    uint16_t idx2 = (head - 1 + BUF_SIZE) % BUF_SIZE;
    ror_trend = buf[idx2] - buf[idx1];  // 簡易的な傾向
  }
  
  // 段階別の高度な火力制御
  switch(current_stage) {
    case STAGE_CHARGE:
      // 投入段階：RoRピーク後の下降開始をサポート
      if (current_ror > 18 && ror_trend > 0) {
        base_fire = FIRE_LOW;  // 早めに火力を落とす
      } else if (current_ror < 10) {
        base_fire = FIRE_MEDIUM;  // 十分な熱を与える
      }
      break;
      
    case STAGE_DRYING:
      // 乾燥段階：10-15°C/minを維持しつつ下降
      if (current_ror > target.ror_max + 2) {
        base_fire = (base_fire > FIRE_OFF) ? (FirePower)(base_fire - 1) : FIRE_OFF;
      } else if (current_ror < target.ror_min - 2) {
        base_fire = (base_fire < FIRE_MAX) ? (FirePower)(base_fire + 1) : FIRE_MAX;
      }
      break;
      
    case STAGE_MAILLARD:
      // メイラード段階：滑らかな下降を最優先
      if (ror_trend > 0.5f) {  // RoRが上昇傾向
        base_fire = (base_fire > FIRE_OFF) ? (FirePower)(base_fire - 1) : FIRE_OFF;
      } else if (current_ror > target.ror_max) {
        base_fire = FIRE_VERY_LOW;  // 1ハゼ前に火力を十分下げる
      } else if (current_ror < target.ror_min && current_temp < 180) {
        base_fire = (base_fire < FIRE_LOW) ? (FirePower)(base_fire + 1) : FIRE_LOW;
      }
      break;
      
    case STAGE_FIRST_CRACK:
      // 1ハゼ：クラッシュ防止とフィック回避が最優先
      base_fire = FIRE_VERY_LOW;  // 基本は極低火
      if (current_ror < -1.0f) {  // 急激なクラッシュ
        base_fire = FIRE_LOW;  // 少し火力を戻す
      } else if (ror_trend > 0.2f) {  // フィックの兆候
        base_fire = FIRE_OFF;  // 即座に火を切る
      }
      break;
      
    case STAGE_DEVELOPMENT:
      // 発達段階：RoR 0に向けて緩やかに制御
      if (current_ror > 3) {
        base_fire = FIRE_OFF;  // 火力を大幅削減
      } else if (current_ror > 1) {
        base_fire = FIRE_VERY_LOW;
      } else if (current_ror < 0.5f && getStageElapsedTime() < target.time_min) {
        base_fire = FIRE_VERY_LOW;  // 最低限の熱は維持
      } else {
        base_fire = FIRE_OFF;  // 基本はオフ
      }
      break;
      
    case STAGE_SECOND_CRACK:
    case STAGE_FINISH:
      // 2ハゼ以降：火力オフが基本
      base_fire = FIRE_OFF;
      break;
      
    default:
      // 予熱段階
      base_fire = FIRE_HIGH;
      break;
  }
  
  // 温度安全措置（最優先）
  float current_danger_temp = getDangerTemp(selected_roast_level);
  if (current_temp > current_danger_temp - 5) {
    base_fire = FIRE_OFF;  // 危険温度接近時は火力カット
  } else if (current_temp > target.temp_max + 3) {
    base_fire = (base_fire > FIRE_OFF) ? (FirePower)(base_fire - 1) : FIRE_OFF;
  }
  
  // ヒステリシス適用（振動防止）
  if (abs(base_fire - current_recommended_fire) <= 1) {
    // 微小な変更は無視
    if (current_ror >= target.ror_min - 1 && 
        current_ror <= target.ror_max + 1 &&
        abs(ror_trend) < 0.3f) {
      return current_recommended_fire;
    }
  }
  
  return base_fire;
}

void playBeep(int duration_ms, int frequency) {
  // M5Stackのスピーカーでビープ音を鳴らす
  M5.Speaker.tone(frequency, duration_ms);
}

void playStageChangeBeep() {
  // 段階変更時の3音ビープ（非ブロッキング開始）
  stage_beep_active = true;
  stage_beep_start = millis();
  stage_beep_count = 0;
}

void playCriticalWarningBeep() {
  // 緊急警告時の断続ビープ（非ブロッキング開始）
  critical_beep_active = true;
  critical_beep_start = millis();
  critical_beep_count = 0;
}

void forceNextStage() {
  if (!roast_guide_active || current_stage >= STAGE_FINISH) return;
  
  RoastStage prev_stage = current_stage;
  
  // 次の段階に強制移行
  switch(current_stage) {
    case STAGE_PREHEAT:
      current_stage = STAGE_CHARGE;
      break;
    case STAGE_CHARGE:
      current_stage = STAGE_DRYING;
      break;
    case STAGE_DRYING:
      current_stage = STAGE_MAILLARD;
      break;
    case STAGE_MAILLARD:
      current_stage = STAGE_FIRST_CRACK;
      first_crack_detected = true;
      break;
    case STAGE_FIRST_CRACK:
      current_stage = STAGE_DEVELOPMENT;
      break;
    case STAGE_DEVELOPMENT:
      if (selected_roast_level >= ROAST_DARK) {
        current_stage = STAGE_SECOND_CRACK;
        second_crack_detected = true;
      } else {
        current_stage = STAGE_FINISH;
      }
      break;
    case STAGE_SECOND_CRACK:
      current_stage = STAGE_FINISH;
      break;
    default:
      break;
  }
  
  // 段階変更時の処理
  if (prev_stage != current_stage) {
    stage_start_time = millis();
    stage_start_temp = current_temp;
    playStageChangeBeep();
  }
}

void checkEmergencyConditions() {
  // ローストレベル依存の危険温度チェック
  float current_danger_temp = getDangerTemp(selected_roast_level);
  float current_critical_temp = getCriticalTemp(selected_roast_level);
  
  if (current_temp >= current_danger_temp) {
    // 警告表示（ローストレベル情報付き）
    M5.Lcd.fillRect(0, 100, 320, 40, TFT_RED);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);
    M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_24);
    M5.Lcd.setCursor(20, 110);
    M5.Lcd.printf("!!! DANGER TEMP: %.1f°C !!!", current_temp);
    M5.Lcd.setCursor(20, 125);
    M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_12);
    M5.Lcd.printf("(%s Limit: %.0f°C)", getRoastLevelName(selected_roast_level), current_danger_temp);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    
    // 連続警告音
    if (millis() - last_critical_warning > 1000) {
      playCriticalWarningBeep();
      last_critical_warning = millis();
    }
  }
  
  // 緊急停止温度（非ブロッキング）
  if (current_temp >= current_critical_temp && !emergency_active) {
    current_stage = STAGE_FINISH;
    roast_guide_active = false;
    emergency_active = true;
    emergency_beep_start = millis();
    emergency_beep_count = 0;
    
    M5.Lcd.fillScreen(TFT_RED);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);
    M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_36);
    M5.Lcd.setCursor(50, 100);
    M5.Lcd.printf("EMERGENCY STOP!");
  }
  
  // 非ブロッキング緊急警告音処理
  if (emergency_active && emergency_beep_count < MAX_EMERGENCY_BEEPS) {
    uint32_t elapsed = millis() - emergency_beep_start;
    uint32_t beep_cycle = emergency_beep_count * EMERGENCY_BEEP_INTERVAL * 2; // ON+OFF時間
    
    if (elapsed >= beep_cycle && elapsed < beep_cycle + EMERGENCY_BEEP_INTERVAL) {
      // ビープ開始
      if (elapsed == beep_cycle || (elapsed - beep_cycle) < 50) { // 開始タイミングのみ
        M5.Speaker.tone(3000, EMERGENCY_BEEP_INTERVAL);
      }
    } else if (elapsed >= beep_cycle + EMERGENCY_BEEP_INTERVAL * 2) {
      // 次のビープへ
      emergency_beep_count++;
    }
  }
  
  // 緊急警告の終息処理
  if (emergency_active && emergency_beep_count >= MAX_EMERGENCY_BEEPS) {
    emergency_active = false;
  }
  
  // 温度が安全域に戻った場合の復旧処理（Theodore提言による改善）
  // 緊急停止からの復旧は手動操作（ボタンC長押し）でのみ可能
  if (emergency_active && current_temp < current_danger_temp - 10.0f) {
    // 温度が十分下がったことを表示（復旧は手動のみ）
    M5.Lcd.fillRect(0, 150, 320, 30, TFT_GREEN);
    M5.Lcd.setTextColor(TFT_BLACK, TFT_GREEN);
    M5.Lcd.setCursor(60, 160);
    M5.Lcd.printf("TEMP SAFE - Hold C to Reset");
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  }
  
  // RoR異常チェック（ガスコンロ特有）
  if (current_ror > 20) {  // 20°C/分以上は異常
    M5.Lcd.setTextColor(TFT_ORANGE);
    M5.Lcd.setCursor(10, 180);
    M5.Lcd.printf("!! RoR TOO HIGH - REDUCE HEAT !!");
    M5.Lcd.setTextColor(TFT_WHITE);
  }
}

const char* getGasAdjustmentAdvice(FirePower current_fire, FirePower target_fire) {
  if (current_fire == target_fire) return "火力維持";
  
  int diff = target_fire - current_fire;
  switch(diff) {
    case -2: case -3: return "すぐに弱火に！";
    case -1: return "少し火を弱める";
    case 1: return "少し火を強める";
    case 2: case 3: return "すぐに火を強く！";
    default: return "火力調整";
  }
}

void updateFirePowerRecommendation() {
  if (!roast_guide_active) return;
  
  FirePower new_fire = calculateRecommendedFire();
  
  // 火力推奨が変わった場合の通知
  if (new_fire != last_recommended_fire) {
    current_recommended_fire = new_fire;
    last_recommended_fire = new_fire;
    
    // 火力変更の音声通知
    if (millis() - last_beep_time > 3000) {  // 3秒間隔制限
      playBeep(500, 800);  // 低音で火力変更を通知
      last_beep_time = millis();
    }
  }
  
  // 基本的な警告チェック
  RoastTarget target = getRoastTarget(current_stage, selected_roast_level);
  bool critical_temp = (current_temp > target.temp_max + 10) || 
                      (current_stage == STAGE_FINISH && current_temp > target.temp_max);
  
  if (critical_temp && (millis() - last_critical_warning > 5000)) {
    playCriticalWarningBeep();
    last_critical_warning = millis();
    critical_temp_warning_active = true;
  } else if (!critical_temp) {
    critical_temp_warning_active = false;
  }
}

void sendBLEData() {
  if (millis() - last_ble_send < BLE_SEND_INTERVAL) {
    return;  // Don't send too frequently
  }
  
  if (!deviceConnected) {
    return;  // No client connected
  }
  
  last_ble_send = millis();
  
  // 静的バッファでヒープ断片化を防止（Theodoreの提言により容量拡張）
  static char json_buffer[768];
  int pos = 0;
  
  pos += snprintf(json_buffer + pos, sizeof(json_buffer) - pos,
    "{\"timestamp\":%lu,\"temperature\":%.2f,\"ror\":%.1f,\"state\":\"%s\",\"mode\":%d,\"count\":%d",
    millis(), current_temp, current_ror,
    (system_state == STATE_RUNNING) ? "running" : "standby",
    display_mode, count);
  
  if (roast_guide_active && pos < sizeof(json_buffer) - 150) {
    int written = snprintf(json_buffer + pos, sizeof(json_buffer) - pos,
      ",\"roast\":{\"level\":\"%s\",\"stage\":\"%s\",\"elapsed\":%lu,\"fire\":\"%s\"}",
      getRoastLevelName(selected_roast_level),
      getRoastStageName(current_stage),
      getRoastElapsedTime(),
      getFirePowerName(current_recommended_fire));
    if (written > 0 && written < sizeof(json_buffer) - pos) pos += written;
  }
  
  if (count > 0 && pos < sizeof(json_buffer) - 80) {
    int written = snprintf(json_buffer + pos, sizeof(json_buffer) - pos,
      ",\"stats\":{\"min\":%.2f,\"max\":%.2f,\"avg\":%.2f}",
      temp_min_recorded, temp_max_recorded, getAverageTemp());
    if (written > 0 && written < sizeof(json_buffer) - pos) pos += written;
  }
  
  if (pos < sizeof(json_buffer) - 2) {
    strcat(json_buffer, "}\n");
  }
  
  pTxCharacteristic->setValue(json_buffer);
  pTxCharacteristic->notify();
}

void loop() {
  M5.update();
  handleButtons();
  handleNonBlockingBeeps();

  // Handle BLE disconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(300); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    M5_LOGI("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // Handle BLE connection
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  if (system_state == STATE_STANDBY) {
    // In standby mode, just handle buttons
    return;
  }

  if (millis() >= next_tick) {
    next_tick += PERIOD_MS;

    km_err = kmeter.getReadyStatus();
    if (km_err == 0) {
      current_temp = kmeter.getCelsiusTempValue() / 100.0f;

      // Update statistics
      if (current_temp < temp_min_recorded) temp_min_recorded = current_temp;
      if (current_temp > temp_max_recorded) temp_max_recorded = current_temp;
      temp_sum += current_temp;

      buf[head] = current_temp;
      head = (head + 1) % BUF_SIZE;
      if (count < BUF_SIZE) ++count;

      // Calculate and update RoR (both 15s and 60s)
      current_ror = calculateRoR();
      current_ror_15s = calculateRoR15s();
      updateRoRBuffer();
      
      // Check for stall condition
      checkStallCondition();
      
      // Add temperature to predictor
      predictor.addTemperature(current_temp);
      
      // Check emergency conditions
      checkEmergencyConditions();

      // Update fire power recommendations and audio notifications
      updateFirePowerRecommendation();

      // Send BLE data
      sendBLEData();

      drawCurrentValue();
      
      if (display_mode == MODE_GRAPH) {
        if (need_full_redraw) {
          drawGraph();
          need_full_redraw = false;
        } else {
          addNewGraphPoint();
        }
      } else if (display_mode == MODE_STATS) {
        drawStats();
      } else if (display_mode == MODE_ROR) {
        drawRoR();
      } else if (display_mode == MODE_GUIDE) {
        if (roast_guide_active) {
          updateRoastStage();
          drawGuide();
        } else {
          drawRoastLevelSelection();
        }
      }
    } else {
      M5.Lcd.fillRect(0, 30, 320, 30, TFT_BLACK);
      M5.Lcd.setCursor(0, 30);
      M5.Lcd.printf("KMeter Err: %d", km_err);
    }
  }
}
void handleNonBlockingBeeps() {
  // Handle stage change beeps
  if (stage_beep_active) {
    uint32_t elapsed = millis() - stage_beep_start;
    uint32_t beep_time = stage_beep_count * STAGE_BEEP_INTERVAL;
    
    if (elapsed >= beep_time && stage_beep_count < MAX_STAGE_BEEPS) {
      playBeep(stage_beep_durations[stage_beep_count], stage_beep_frequencies[stage_beep_count]);
      stage_beep_count++;
    }
    
    if (stage_beep_count >= MAX_STAGE_BEEPS) {
      stage_beep_active = false;
    }
  }
  
  // Handle critical warning beeps
  if (critical_beep_active) {
    uint32_t elapsed = millis() - critical_beep_start;
    uint32_t beep_time = critical_beep_count * CRITICAL_BEEP_INTERVAL;
    
    if (elapsed >= beep_time && critical_beep_count < MAX_CRITICAL_BEEPS) {
      playBeep(150, 2000);
      critical_beep_count++;
    }
    
    if (critical_beep_count >= MAX_CRITICAL_BEEPS) {
      critical_beep_active = false;
    }
  }
}// 次段階進行に必要な鍵温度を返す（初心者向け明確化）
float getNextStageKeyTemp(RoastStage stage, RoastLevel level) {
  switch(stage) {
    case STAGE_PREHEAT:
      return 150.0f;  // Charge段階へ
    case STAGE_CHARGE:
      return 0.0f;    // 時間ベース
    case STAGE_DRYING:
      return 150.0f;  // Maillard段階へ
    case STAGE_MAILLARD:
      return 180.0f;  // 1st Crack段階への鍵温度
    case STAGE_FIRST_CRACK:
      return 0.0f;    // 時間ベース
    case STAGE_DEVELOPMENT:
      // 焙煎レベルに応じた仕上げ温度
      switch(level) {
        case ROAST_LIGHT: return 200.0f;
        case ROAST_MEDIUM_LIGHT: return 205.0f;
        case ROAST_MEDIUM: return 210.0f;
        case ROAST_MEDIUM_DARK: return 220.0f;
        case ROAST_DARK: return 225.0f;  // 2nd Crack域へ
        case ROAST_FRENCH: return 225.0f;
        default: return 210.0f;
      }
    case STAGE_SECOND_CRACK:
      // 各レベルの最終温度
      switch(level) {
        case ROAST_DARK: return 250.0f;
        case ROAST_FRENCH: return 260.0f;
        default: return 235.0f;
      }
    case STAGE_FINISH:
    default:
      return 0.0f;  // 完了
  }
}

/**
 * Theodore提言のエレガントなメロディ再生
 */
void playMelody(const Melody& melody) {
  for (int i = 0; i < 8; i++) {
    if (melody.notes[i] > 0) {
      M5.Speaker.tone(melody.notes[i], melody.duration_ms);
      delay(melody.duration_ms + 50);  // 小さな間隔を追加
    }
  }
}

/**
 * フッター領域の統一描画（画面下部のボタン指示）
 */
void drawFooter(const char* instructions) {
  int footer_y = 240 - FOOTER_HEIGHT;
  M5.Lcd.fillRect(0, footer_y, 320, FOOTER_HEIGHT, TFT_BLACK);
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_12);
  M5.Lcd.setTextColor(TFT_WHITE);
  M5.Lcd.setCursor(5, footer_y + 4);
  M5.Lcd.printf("%s", instructions);
}