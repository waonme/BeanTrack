#include <M5Unified.h>
#include <M5UnitKmeterISO.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <stdarg.h>

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

// セオドア提言：Sprite最適化用
static LGFX_Sprite graph_sprite(&M5.Lcd);
static bool sprite_initialized = false;

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

// セオドア提言：メモリ効率化のため int16_t に変更（0.1°C刻み）
int16_t  buf[BUF_SIZE];  // 0.1°C単位で格納（例：25.3°C → 253）
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

// セオドア提言：差分描画用キャッシュ変数
float last_displayed_temp = NAN;
float last_displayed_ror = NAN;
FirePower last_displayed_fire = FIRE_OFF;
RoastStage last_displayed_stage = STAGE_PREHEAT;
uint32_t last_displayed_time = 0;
bool last_roast_guide_state = false;
int last_ror_wait_seconds = -1;

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

// セオドア提言：自動復旧システム
bool auto_recovery_available = false;
uint32_t recovery_dialog_start = 0;
bool recovery_dialog_active = false;

// Non-blocking stage change beeps
bool stage_beep_active = false;
uint32_t stage_beep_start = 0;
int stage_beep_count = 0;
constexpr int MAX_STAGE_BEEPS = 3;
constexpr uint32_t STAGE_BEEP_INTERVAL = 300; // 300ms between beeps
int stage_beep_frequencies[3] = {1000, 1200, 1500};
int stage_beep_durations[3] = {200, 200, 300};

// セオドア提言：非ブロッキングメロディシステム
bool melody_active = false;
uint32_t melody_start = 0;
int melody_note_index = 0;
int melody_notes[8];
int melody_duration = 0;
uint32_t melody_note_start = 0;

// 非ブロッキング初期化待機
static uint32_t init_retry_timer = 0;
static bool init_waiting = false;

// 非ブロッキング復旧成功表示
static uint32_t recovery_display_start = 0;
static bool recovery_display_active = false;

// 非ブロッキングBLE再接続待機
static uint32_t ble_restart_timer = 0;
static bool ble_restart_pending = false;

// セオドア提言：BLE差分パケット送信システム
static uint32_t last_full_data_send = 0;
constexpr uint32_t FULL_DATA_INTERVAL = 15000; // 15秒間隔で完全データ送信

// Non-blocking critical warning beeps
bool critical_beep_active = false;
uint32_t critical_beep_start = 0;
int critical_beep_count = 0;
constexpr int MAX_CRITICAL_BEEPS = 3;
constexpr uint32_t CRITICAL_BEEP_INTERVAL = 250; // 250ms between beeps

// Hysteresis values as constexpr
constexpr float FIRE_HYSTERESIS = 0.5f;  // °C hysteresis for fire power changes

// ティッカーフッター機能
struct TickerMessage {
  char text[128];
  uint32_t added_time;
};

constexpr int TICKER_MAX_MESSAGES = 10;
constexpr uint32_t TICKER_MESSAGE_DURATION = 5000; // 各メッセージ5秒表示
constexpr uint32_t TICKER_SCROLL_SPEED = 50; // スクロール速度(ms)
constexpr int TICKER_Y_POSITION = 220; // フッター位置

TickerMessage ticker_messages[TICKER_MAX_MESSAGES];
int ticker_message_count = 0;
int ticker_current_index = 0;
int ticker_scroll_offset = 0;
uint32_t ticker_last_scroll = 0;
uint32_t ticker_message_start = 0;
bool ticker_enabled = false;

// Melody system for elegant notifications
struct Melody {
  int notes[8];      // 8音のメロディ
  int duration_ms;   // 各音の長さ
};

// セオドア提言：メロディもPROGMEMで最適化
const Melody PROGMEM MELODY_STAGE_CHANGE = {{262, 294, 330, 349, 392, 440, 494, 523}, 150};  // C-D-E-F-G-A-B-C上昇
const Melody PROGMEM MELODY_FIRST_CRACK = {{523, 494, 440, 392, 349, 330, 294, 262}, 120};    // C-B-A-G-F-E-D-C下降
const Melody PROGMEM MELODY_EMERGENCY = {{880, 831, 784, 740, 698, 659, 622, 587}, 100};      // 高音から滑落
const Melody PROGMEM MELODY_COMPLETION = {{392, 523, 392, 523, 392, 523, 659, 523}, 200};    // G-C完成旋律

// セオドア提言：関数宣言（非ブロッキング関数群）
void updateMelody();
void playMelody(const Melody& melody_pgm);

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

// 関数宣言
void addTickerMessage(const char* format, ...);
void updateTickerSystemInfo();
void updateTickerFooter();

// セオドア提言：温度バッファの型変換ヘルパー関数
inline void setTempToBuffer(uint16_t index, float temp) {
  buf[index] = (int16_t)(temp * 10.0f);  // 0.1°C刻みで格納
}

inline float getTempFromBuffer(uint16_t index) {
  return buf[index] * 0.1f;  // float に戻す
}

// セオドア提言：転換点検出用の移動平均RoR計算
float calculateMovingAverageRoR(int window_size = 5) {
  if (count < window_size + 1) return 0.0f;
  
  float sum_ror = 0.0f;
  int valid_samples = 0;
  
  for (int i = 1; i <= window_size; i++) {
    uint16_t current_idx = (head - i + BUF_SIZE) % BUF_SIZE;
    uint16_t prev_idx = (head - i - 1 + BUF_SIZE) % BUF_SIZE;
    
    float current_temp = getTempFromBuffer(current_idx);
    float prev_temp = getTempFromBuffer(prev_idx);
    
    // 1秒間隔でのRoR計算
    float point_ror = (current_temp - prev_temp) * 60.0f;  // °C/min
    sum_ror += point_ror;
    valid_samples++;
  }
  
  return valid_samples > 0 ? sum_ror / valid_samples : 0.0f;
}

// セオドア提言：転換点検出（RoRの符号反転を検出）
bool detectTurningPoint() {
  if (count < 10) return false;  // 最低10サンプル必要
  
  float current_ma_ror = calculateMovingAverageRoR(3);  // 短期移動平均
  float prev_ma_ror = 0.0f;
  
  // 3秒前の移動平均RoRを計算
  if (count >= 13) {
    // 一時的にheadを3つ戻して計算
    uint16_t temp_head = (head - 3 + BUF_SIZE) % BUF_SIZE;
    uint16_t temp_count = count - 3;
    
    float sum_ror = 0.0f;
    for (int i = 1; i <= 3; i++) {
      uint16_t current_idx = (temp_head - i + BUF_SIZE) % BUF_SIZE;
      uint16_t prev_idx = (temp_head - i - 1 + BUF_SIZE) % BUF_SIZE;
      
      float current_temp = getTempFromBuffer(current_idx);
      float prev_temp = getTempFromBuffer(prev_idx);
      sum_ror += (current_temp - prev_temp) * 60.0f;
    }
    prev_ma_ror = sum_ror / 3.0f;
  }
  
  // 転換点条件：負から正への転換（±1°C/minの閾値）
  return (prev_ma_ror < -1.0f && current_ma_ror > 1.0f);
}

// セオドア提言：定数統一でコード長20%削減
constexpr float DANGER_TEMPS[ROAST_COUNT] = {
  245.0f,  // ROAST_LIGHT
  245.0f,  // ROAST_MEDIUM_LIGHT  
  245.0f,  // ROAST_MEDIUM
  245.0f,  // ROAST_MEDIUM_DARK
  255.0f,  // ROAST_DARK（深煎り用）
  255.0f   // ROAST_FRENCH（深煎り用）
};

constexpr float CRITICAL_TEMPS[ROAST_COUNT] = {
  260.0f,  // ROAST_LIGHT
  260.0f,  // ROAST_MEDIUM_LIGHT
  260.0f,  // ROAST_MEDIUM
  260.0f,  // ROAST_MEDIUM_DARK
  270.0f,  // ROAST_DARK（深煎り用）
  270.0f   // ROAST_FRENCH（深煎り用）
};

// 統一された温度取得関数
inline float getDangerTemp(RoastLevel level) {
  return DANGER_TEMPS[level];
}

inline float getCriticalTemp(RoastLevel level) {
  return CRITICAL_TEMPS[level];
}

// セオドア提言：LGFX環境向けclamp関数
template <typename T>
constexpr T clamp(T value, T min_val, T max_val) {
  return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
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
  
  // セオドア提言：Sprite初期化（真のスクロール実装）
  graph_sprite.createSprite(GRAPH_W, GRAPH_H);
  sprite_initialized = true;

  // I2C明示的初期化（M5Unifiedの実装変更に対応）
  Wire.begin(KM_SDA, KM_SCL, I2C_FREQ);
  
  // 非ブロッキング初期化（セオドア提言：スタートアップも非ブロッキング化）
  if (!kmeter.begin(&Wire, KM_ADDR, KM_SDA, KM_SCL, I2C_FREQ)) {
    M5_LOGE("KMeterISO not found…再試行中");
    init_waiting = true;
    init_retry_timer = millis();
    // 初期化失敗時は一旦setup()を抜けてloop()で再試行
    return;
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
 * セオドア提言：差分描画によるフラッシュ防止ヘッダー描画
 */
void drawCurrentValue() {
  bool needs_full_clear = need_full_redraw;
  
  // 全体クリアが必要な場合のみ実行
  if (needs_full_clear) {
    M5.Lcd.fillRect(0, 0, 320, HEADER_HEIGHT, TFT_BLACK);
    // キャッシュをリセットして全体再描画を促す
    last_displayed_temp = NAN;
    last_displayed_ror = NAN;
    last_displayed_fire = (FirePower)-1;
    last_displayed_stage = (RoastStage)-1;
    last_displayed_time = UINT32_MAX;
    last_roast_guide_state = !roast_guide_active;
    last_ror_wait_seconds = -1;
  }
  
  // 1. メイン温度表示（変化時のみ更新）
  if (needs_full_clear || abs(current_temp - last_displayed_temp) > 0.05f) {
    // テキスト領域のみクリア
    M5.Lcd.fillRect(0, 5, 200, 16, TFT_BLACK);
    M5.Lcd.setCursor(0, 5);
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.printf("TEMP: %6.2f C", current_temp);
    last_displayed_temp = current_temp;
  }
  
  // 2. 火力推奨インジケーター表示（変化時のみ）
  bool fire_display_needed = (roast_guide_active && system_state == STATE_RUNNING);
  if (needs_full_clear || 
      fire_display_needed != last_roast_guide_state ||
      (fire_display_needed && current_recommended_fire != last_displayed_fire)) {
    
    // 火力表示領域をクリア
    M5.Lcd.fillRect(220, 5, 100, 16, TFT_BLACK);
    
    if (fire_display_needed) {
      M5.Lcd.setCursor(220, 5);
      M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
      
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
    last_displayed_fire = current_recommended_fire;
    last_roast_guide_state = fire_display_needed;
  }
  
  // 3. RoR表示（変化時のみ）
  bool ror_changed = false;
  if (count >= ROR_INTERVAL) {
    ror_changed = (abs(current_ror - last_displayed_ror) > 0.05f);
  } else {
    int wait_seconds = ROR_INTERVAL - count;
    ror_changed = (wait_seconds != last_ror_wait_seconds);
    last_ror_wait_seconds = wait_seconds;
  }
  
  if (needs_full_clear || ror_changed) {
    M5.Lcd.fillRect(0, 25, 140, 12, TFT_BLACK);
    M5.Lcd.setCursor(0, 25);
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
    M5.Lcd.setTextColor(TFT_WHITE);
    if (count >= ROR_INTERVAL) {
      M5.Lcd.printf("RoR: %.1f C/min", current_ror);
      last_displayed_ror = current_ror;
    } else {
      M5.Lcd.printf("RoR: Wait %ds", ROR_INTERVAL - count);
    }
  }
  
  // 4. 焙焙ステージ情報（変化時のみ）
  if (roast_guide_active) {
    if (needs_full_clear || current_stage != last_displayed_stage) {
      M5.Lcd.fillRect(150, 25, 170, 12, TFT_BLACK);
      M5.Lcd.setCursor(150, 25);
      M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
      M5.Lcd.setTextColor(TFT_WHITE);
      M5.Lcd.printf("Stage: %s", getRoastStageName(current_stage));
      last_displayed_stage = current_stage;
    }
    
    uint32_t current_elapsed = getRoastElapsedTime();
    if (needs_full_clear || current_elapsed != last_displayed_time) {
      M5.Lcd.fillRect(0, 35, 120, 12, TFT_BLACK);
      M5.Lcd.setCursor(0, 35);
      M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
      M5.Lcd.setTextColor(TFT_WHITE);
      M5.Lcd.printf("Time: %02d:%02d", current_elapsed / 60, current_elapsed % 60);
      last_displayed_time = current_elapsed;
    }
  } else if (last_roast_guide_state != roast_guide_active) {
    // 焙焙ガイドが停止した場合、関連領域をクリア
    M5.Lcd.fillRect(150, 25, 170, 12, TFT_BLACK);
    M5.Lcd.fillRect(0, 35, 120, 12, TFT_BLACK);
    last_roast_guide_state = roast_guide_active;
  }
}

/**
 * 現在の buf 内容でグラフを再描画
 */
void drawGraph() {
  if (count == 0 || !sprite_initialized) return;

  // 画面上の枠線を描画
  M5.Lcd.fillRect(GRAPH_X0-1, GRAPH_Y0-1, GRAPH_W+2, GRAPH_H+2, TFT_BLACK);
  M5.Lcd.drawRect(GRAPH_X0-1, GRAPH_Y0-1, GRAPH_W+2, GRAPH_H+2, TFT_WHITE);
  
  // Sprite内容をクリアして背景を再構築
  graph_sprite.fillSprite(TFT_BLACK);
  
  // ローストレベル依存の危険域表示（Sprite内）
  float current_danger_temp = getDangerTemp(selected_roast_level);
  float current_critical_temp = getCriticalTemp(selected_roast_level);
  
  // セオドア提言：色覚配慮の危険域表示（Sprite座標系で）
  float danger_y_start = GRAPH_H - (current_danger_temp - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * GRAPH_H;
  float danger_y_end = GRAPH_H - (current_critical_temp - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * GRAPH_H;
  
  // 注意域（Danger〜Critical）
  if (danger_y_start > danger_y_end) {
    graph_sprite.fillRect(0, (int)danger_y_end, GRAPH_W, (int)(danger_y_start - danger_y_end), TFT_OLIVE);
  }
  
  // 緊急停止域（Critical〜TEMP_MAX）縞模様
  int critical_height = (int)danger_y_end;
  for (int y = 0; y < critical_height; y += 4) {
    uint16_t color = (y % 8 < 4) ? TFT_RED : TFT_DARKGREY;
    graph_sprite.drawFastHLine(0, y, GRAPH_W, color);
  }

  // 折れ線をSprite内に描画
  uint16_t start = (count < BUF_SIZE) ? 0 : head;
  float prevX = -1, prevY = -1;
  for (uint16_t i = 0; i < count; ++i) {
    uint16_t idx = (start + i) % BUF_SIZE;
    float v = getTempFromBuffer(idx);

    // 範囲外は無視
    if (v < TEMP_MIN || v > TEMP_MAX) continue;

    float x = (float)i / (BUF_SIZE - 1) * GRAPH_W;
    float y = GRAPH_H - (v - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * GRAPH_H;

    if (prevX >= 0) {
      graph_sprite.drawLine((int)prevX, (int)prevY, (int)x, (int)y, TFT_CYAN);
    }
    prevX = x;
    prevY = y;
  }

  // Spriteを画面に転送
  graph_sprite.pushSprite(GRAPH_X0, GRAPH_Y0);
  
  // 画面上に軸ラベルを描画（Sprite外）
  M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
  M5.Lcd.setCursor(GRAPH_X0 + GRAPH_W + 4, GRAPH_Y0 - 8);
  M5.Lcd.printf("%.0f", TEMP_MAX);
  M5.Lcd.setCursor(GRAPH_X0 + GRAPH_W + 4, GRAPH_Y0 + GRAPH_H - 8);
  M5.Lcd.printf("%.0f", TEMP_MIN);

  // 時間目盛
  M5.Lcd.setCursor(GRAPH_X0, GRAPH_Y0 + GRAPH_H + 6);
  M5.Lcd.printf("[15min]");
}

void handleButtons() {
  // Handle Button A and B only when running
  if (system_state == STATE_RUNNING) {
    // Check for A+B combination for ticker toggle
    static bool combo_handled = false;
    if (M5.BtnA.isPressed() && M5.BtnB.isPressed()) {
      if (!combo_handled) {
        ticker_enabled = !ticker_enabled;
        combo_handled = true;
        
        // Visual feedback
        M5.Lcd.fillRect(60, 100, 200, 40, TFT_BLACK);
        M5.Lcd.drawRect(60, 100, 200, 40, ticker_enabled ? TFT_GREEN : TFT_RED);
        M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
        M5.Lcd.setTextColor(ticker_enabled ? TFT_GREEN : TFT_RED);
        M5.Lcd.setCursor(70, 115);
        M5.Lcd.printf("TICKER %s", ticker_enabled ? "ENABLED" : "DISABLED");
        M5.Lcd.setTextColor(TFT_WHITE);
        playBeep(100, ticker_enabled ? 1000 : 800);
        
        // Clear ticker area if disabling
        if (!ticker_enabled) {
          M5.Lcd.fillRect(0, TICKER_Y_POSITION, 320, 20, TFT_BLACK);
        } else {
          // Add initial message when enabling
          ticker_message_count = 0;
          addTickerMessage("ティッカーフッター有効化");
        }
        
        need_full_redraw = true;
        
        // 非ブロッキング表示（1秒後に消去）
        recovery_display_start = millis();
        recovery_display_active = true;
      }
    } else if (!M5.BtnA.isPressed() && !M5.BtnB.isPressed()) {
      combo_handled = false;  // Reset when both buttons are released
    }
    
    if (M5.BtnA.wasPressed() && !M5.BtnB.isPressed()) {
      // セオドア提言：緊急時の自動復旧機能
      if (emergency_active && auto_recovery_available) {
        // 自動復旧実行
        emergency_active = false;
        emergency_beep_count = 0;
        auto_recovery_available = false;
        recovery_dialog_active = false;
        need_full_redraw = true;
        
        // 復旧成功メッセージ（非ブロッキング表示）
        M5.Lcd.fillScreen(TFT_BLACK);
        M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
        M5.Lcd.setTextColor(TFT_GREEN);
        M5.Lcd.setCursor(80, 120);
        M5.Lcd.printf("AUTO RECOVERY SUCCESS");
        M5.Lcd.setTextColor(TFT_WHITE);
        playBeep(300, 1000);
        recovery_display_start = millis();
        recovery_display_active = true;
      } else {
        // 通常のモード切り替え
        display_mode = (DisplayMode)((display_mode + 1) % MODE_COUNT);
        need_full_redraw = true;
        M5.Lcd.fillRect(0, GRAPH_Y0, 320, 240 - GRAPH_Y0, TFT_BLACK);
      }
    }
    
  // Handle Button B press and long press
  if (M5.BtnB.isPressed()) {
    if (btnB_press_start == 0) {
      btnB_press_start = millis();
      btnB_long_press_handled = false;
    }
    
    // Check for long press (2 seconds) - Manual stage advance
    uint32_t now = millis();
    if (!btnB_long_press_handled && (now - btnB_press_start) >= LONG_PRESS_DURATION) {
      if (roast_guide_active && current_stage < STAGE_FINISH) {
        forceNextStage();
        btnB_long_press_handled = true;
        
        // Visual feedback（非ブロッキング化）
        static uint32_t feedback_start = 0;
        static bool feedback_active = false;
        
        if (!feedback_active) {
          M5.Lcd.fillRect(60, 100, 200, 40, TFT_BLACK);
          M5.Lcd.drawRect(60, 100, 200, 40, TFT_YELLOW);
          M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
          M5.Lcd.setTextColor(TFT_YELLOW);
          M5.Lcd.setCursor(70, 115);
          M5.Lcd.printf("MANUAL STAGE ADVANCE");
          M5.Lcd.setTextColor(TFT_WHITE);
          feedback_start = millis();
          feedback_active = true;
        } else if (millis() - feedback_start > 300) {
          need_full_redraw = true;
          feedback_active = false;
        }
      }
    }
  } else if (btnB_press_start > 0) {
    // Button released - short press
    if (!btnB_long_press_handled) {
      // 1ハゼ確認処理
      if (first_crack_confirmation_needed && roast_guide_active) {
        first_crack_confirmed = true;
        first_crack_confirmation_needed = false;
        
        // 視覚的フィードバック（非ブロッキング化）
        static uint32_t crack_feedback_start = 0;
        static bool crack_feedback_active = false;
        
        if (!crack_feedback_active) {
          M5.Lcd.fillRect(60, 100, 200, 40, TFT_BLACK);
          M5.Lcd.drawRect(60, 100, 200, 40, TFT_GREEN);
          M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
          M5.Lcd.setTextColor(TFT_GREEN);
          M5.Lcd.setCursor(70, 115);
          M5.Lcd.printf("1ST CRACK CONFIRMED");
          M5.Lcd.setTextColor(TFT_WHITE);
          playBeep(200, 1200);
          crack_feedback_start = millis();
          crack_feedback_active = true;
        } else if (millis() - crack_feedback_start > 300) {
          need_full_redraw = true;
          crack_feedback_active = false;
        }
      } else if (display_mode == MODE_GUIDE && !roast_guide_active) {
        // 焙煎レベル変更
        selected_roast_level = (RoastLevel)((selected_roast_level + 1) % ROAST_COUNT);
      } else {
        // Reset statistics
        temp_min_recorded = INFINITY;
        temp_max_recorded = -INFINITY;
        temp_sum = 0.0f;
        for (int i = 0; i < count; i++) {
          float temp = getTempFromBuffer((head - count + i + BUF_SIZE) % BUF_SIZE);
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
    uint32_t now = millis();
    if (!btnC_long_press_handled && (now - btnC_press_start) >= LONG_PRESS_DURATION) {
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
      
      // Visual feedback for clear（非ブロッキング化）
      static uint32_t clear_feedback_start = 0;
      static bool clear_feedback_active = false;
      
      if (!clear_feedback_active) {
        M5.Lcd.fillScreen(TFT_BLACK);
        M5.Lcd.setFont(&fonts::lgfxJapanGothic_24);
        M5.Lcd.setCursor(80, 120);
        M5.Lcd.println("*** DATA CLEARED ***");
        clear_feedback_start = millis();
        clear_feedback_active = true;
      } else if (millis() - clear_feedback_start > 500) {
        clear_feedback_active = false;
        
        if (system_state == STATE_RUNNING) {
          M5.Lcd.fillScreen(TFT_BLACK);
          M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
          M5.Lcd.setCursor(0, 0);
          M5.Lcd.println("Real-Time Temperature");
          need_full_redraw = true;
        } else {
          drawStandbyScreen();
        }
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
        M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
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

// セオドア提言：Sprite使用による真のスクロールグラフ実装
void addNewGraphPoint() {
  if (count < 2 || !sprite_initialized) return;
  
  uint16_t current_idx = (head - 1 + BUF_SIZE) % BUF_SIZE;
  uint16_t prev_idx = (head - 2 + BUF_SIZE) % BUF_SIZE;
  
  float curr_temp = getTempFromBuffer(current_idx);
  float prev_temp = getTempFromBuffer(prev_idx);
  
  // Skip if out of range
  if (curr_temp < TEMP_MIN || curr_temp > TEMP_MAX || 
      prev_temp < TEMP_MIN || prev_temp > TEMP_MAX) return;
  
  if (count < BUF_SIZE) {
    // バッファがまだ満杯でない場合：従来の方式でSprite内に描画
    float x1 = (float)(count - 2) / (BUF_SIZE - 1) * GRAPH_W;
    float x2 = (float)(count - 1) / (BUF_SIZE - 1) * GRAPH_W;
    float y1 = GRAPH_H - (prev_temp - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * GRAPH_H;
    float y2 = GRAPH_H - (curr_temp - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * GRAPH_H;
    
    graph_sprite.drawLine((int)x1, (int)y1, (int)x2, (int)y2, TFT_CYAN);
    
    // Spriteを画面に転送
    graph_sprite.pushSprite(GRAPH_X0, GRAPH_Y0);
  } else {
    // バッファ満杯：真のスクロール描画（Sprite効率活用）
    
    // 1. Sprite内容を1ピクセル左にスクロール
    graph_sprite.scroll(-1, 0);
    
    // 2. 右端の列をクリア（危険域背景も含む）
    float current_danger_temp = getDangerTemp(selected_roast_level);
    float current_critical_temp = getCriticalTemp(selected_roast_level);
    
    // 背景を再描画（右端1列のみ）
    int x_col = GRAPH_W - 1;
    for (int y = 0; y < GRAPH_H; y++) {
      uint16_t bg_color = TFT_BLACK;
      
      // Y座標から実際の温度を計算
      float temp_at_y = TEMP_MIN + (1.0f - (float)y / GRAPH_H) * (TEMP_MAX - TEMP_MIN);
      
      if (temp_at_y >= current_critical_temp) {
        // 緊急停止域（縞模様）
        bg_color = (y % 8 < 4) ? TFT_RED : TFT_DARKGREY;
      } else if (temp_at_y >= current_danger_temp) {
        // 注意域（暗い色）
        bg_color = TFT_OLIVE;
      }
      
      graph_sprite.drawPixel(x_col, y, bg_color);
    }
    
    // 3. 最新の線分を右端に描画
    float y1 = GRAPH_H - (prev_temp - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * GRAPH_H;
    float y2 = GRAPH_H - (curr_temp - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * GRAPH_H;
    
    graph_sprite.drawLine(GRAPH_W - 2, (int)y1, GRAPH_W - 1, (int)y2, TFT_CYAN);
    
    // 4. Spriteを画面に転送
    graph_sprite.pushSprite(GRAPH_X0, GRAPH_Y0);
  }
}

void drawStats() {
  if (count == 0) {
    M5.Lcd.fillRect(0, GRAPH_Y0, 320, 240 - GRAPH_Y0, TFT_BLACK);
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
    M5.Lcd.setCursor(20, GRAPH_Y0 + 20);
    M5.Lcd.println("No data available");
    return;
  }
  
  M5.Lcd.fillRect(0, GRAPH_Y0, 320, 240 - GRAPH_Y0, TFT_BLACK);
  M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
  
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
  M5.Lcd.setFont(&fonts::lgfxJapanGothic_24);
  M5.Lcd.setCursor(50, 80);
  M5.Lcd.println("*** Coffee Roast Monitor ***");
  
  M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
  M5.Lcd.setCursor(80, 120);
  M5.Lcd.println("--- STANDBY MODE ---");
  
  M5.Lcd.setCursor(40, 160);
  M5.Lcd.println("> Press Button C to START");
  
  M5.Lcd.setCursor(20, 190);
  M5.Lcd.println("> Hold Button C (2sec) to CLEAR");
  
  // Show data count if available
  if (count > 0) {
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
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
  
  float old_temp = getTempFromBuffer(old_idx);
  float current_temp_val = getTempFromBuffer(current_idx);
  
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
  
  float old_temp = getTempFromBuffer(old_idx);
  float current_temp_val = getTempFromBuffer(current_idx);
  
  // Calculate 15s RoR and scale to per-minute
  return (current_temp_val - old_temp) * 4.0f;  // 15s * 4 = 60s
}

void checkStallCondition() {
  uint32_t now = millis();
  if (!roast_guide_active || (now - last_stall_check) < 5000) {
    return; // Check every 5 seconds
  }
  
  last_stall_check = now;
  
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
  M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
  
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
  M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
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
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
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
  // セオドア提言：PROGMEM最適化で12KB RAM削減
  static const RoastTarget PROGMEM profiles[8][6] = {
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
  
  // PROGMEMから読み取り
  RoastTarget target;
  memcpy_P(&target, &profiles[stage][level], sizeof(RoastTarget));
  return target;
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

// セオドア提言：millis()オーバーフロー対策（減算形式）
uint32_t getRoastElapsedTime() {
  if (roast_start_time == 0) return 0;
  uint32_t now = millis();
  return (now - roast_start_time) / 1000;  // unsigned減算でオーバーフロー安全
}

float getStageElapsedTime() {
  if (stage_start_time == 0) return 0;
  uint32_t now = millis();
  return (now - stage_start_time) / 1000.0f;  // unsigned減算でオーバーフロー安全
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
        // セオドア提言：改善された転換点検出
        bool min_time_met = (stage_elapsed > 90);  // 最低1.5分は投入段階維持
        bool turning_point = detectTurningPoint();  // 移動平均RoRによる転換点検出
        bool backup_condition = (stage_elapsed > 120 && current_ror > 8);  // 予備条件
        
        if (min_time_met && (turning_point || backup_condition)) {
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
  M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
  
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
  M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
  
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
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
    M5.Lcd.setCursor(10, y_pos);
    M5.Lcd.printf(">>> 1st Crack? Press B <<<");
    M5.Lcd.setTextColor(TFT_WHITE);
  }
  
  // 重要警告（優先度最高）
  if (current_stage == STAGE_FINISH && y_pos < max_y - 20) {
    y_pos += 15;
    M5.Lcd.setTextColor(TFT_RED);
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
    M5.Lcd.setCursor(30, y_pos);
    M5.Lcd.printf("*** DROP BEANS NOW! ***");
    M5.Lcd.setTextColor(TFT_WHITE);
  }
  
  // 下部情報（スペースが許す場合のみ）
  if (y_pos < max_y - 35) {
    y_pos += 12;
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
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
    ror_trend = getTempFromBuffer(idx2) - getTempFromBuffer(idx1);  // 簡易的な傾向
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

// セオドア提言：非ブロッキング三段階音響警告システム
void playTemperatureWarning(float temp, float danger_temp, float critical_temp) {
  uint32_t now = millis();
  static uint32_t last_warning_beep = 0;
  static uint32_t warning_beep_start = 0;
  static int warning_beep_step = 0;
  static bool warning_active = false;
  
  if ((now - last_warning_beep) < 2000 && !warning_active) return;  // 2秒間隔制限
  
  if (!warning_active) {
    // 新しい警告シーケンス開始
    warning_beep_start = now;
    warning_beep_step = 0;
    warning_active = true;
    last_warning_beep = now;
    
    if (temp >= critical_temp) {
      // 緊急段階：最初のビープ開始
      M5.Speaker.tone(2000, 100);
    } else if (temp >= danger_temp) {
      // 警告段階：最初のビープ開始
      M5.Speaker.tone(1500, 200);
    } else if (temp >= danger_temp - 5) {
      // 注意段階：単発ビープ
      M5.Speaker.tone(1000, 300);
      warning_active = false;  // 単発なので即終了
    }
  } else {
    // 進行中の警告シーケンス処理
    uint32_t elapsed = now - warning_beep_start;
    
    if (temp >= critical_temp) {
      // 緊急段階：3回連続ビープ
      if (elapsed >= 150 && warning_beep_step == 0) {
        M5.Speaker.tone(2000, 100);
        warning_beep_step = 1;
      } else if (elapsed >= 300 && warning_beep_step == 1) {
        M5.Speaker.tone(2000, 100);
        warning_beep_step = 2;
      } else if (elapsed >= 450) {
        warning_active = false;
      }
    } else if (temp >= danger_temp) {
      // 警告段階：2回連続ビープ
      if (elapsed >= 300 && warning_beep_step == 0) {
        M5.Speaker.tone(1500, 200);
        warning_beep_step = 1;
      } else if (elapsed >= 600) {
        warning_active = false;
      }
    }
  }
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
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_24);
    M5.Lcd.setCursor(20, 110);
    M5.Lcd.printf("!!! DANGER TEMP: %.1f°C !!!", current_temp);
    M5.Lcd.setCursor(20, 125);
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
    M5.Lcd.printf("(%s Limit: %.0f°C)", getRoastLevelName(selected_roast_level), current_danger_temp);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    
    // セオドア提言：三段階音響警告システム使用
    playTemperatureWarning(current_temp, current_danger_temp, current_critical_temp);
  }
  
  // 緊急停止温度（非ブロッキング）
  if (current_temp >= current_critical_temp && !emergency_active) {
    current_stage = STAGE_FINISH;
    roast_guide_active = false;
    emergency_active = true;
    uint32_t now = millis();
    emergency_beep_start = now;
    emergency_beep_count = 0;
    
    M5.Lcd.fillScreen(TFT_RED);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_36);
    M5.Lcd.setCursor(50, 100);
    M5.Lcd.printf("EMERGENCY STOP!");
  }
  
  // 非ブロッキング緊急警告音処理
  if (emergency_active && emergency_beep_count < MAX_EMERGENCY_BEEPS) {
    uint32_t now = millis();
    uint32_t elapsed = now - emergency_beep_start;
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
  
  // セオドア提言：改善された自動復旧システム（追加安全条件）
  if (emergency_active && current_temp < current_danger_temp - 10.0f) {
    bool ror_safe = (current_ror < 0);  // RoRがマイナス（冷却中）
    bool temp_stable = (abs(current_ror) < 5.0f);  // RoRが±5°C/min以内で安定
    bool sufficient_cooldown = (current_temp < current_danger_temp - 15.0f);  // 15°C以上冷却
    
    // より厳密な安全条件チェック
    bool all_conditions_safe = ror_safe && (temp_stable || sufficient_cooldown);
    
    if (all_conditions_safe && !auto_recovery_available) {
      auto_recovery_available = true;
      recovery_dialog_start = millis();
      recovery_dialog_active = true;
    }
    
    if (auto_recovery_available && recovery_dialog_active) {
      // 自動復旧提案ダイアログ（セオドア提言：改良版 - 詳細条件表示・拡張安全チェック）
      M5.Lcd.fillRect(25, 105, 270, 125, TFT_DARKGREEN);
      M5.Lcd.drawRect(25, 105, 270, 125, TFT_GREEN);
      M5.Lcd.setTextColor(TFT_WHITE, TFT_DARKGREEN);
      M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
      M5.Lcd.setCursor(35, 115);
      M5.Lcd.printf("INTELLIGENT RECOVERY READY:");
      M5.Lcd.setCursor(35, 130);
      M5.Lcd.printf("Temp: %.1f\u00b0C (Safe: <%.0f\u00b0C)", current_temp, current_danger_temp - 10);
      M5.Lcd.setCursor(35, 145);
      M5.Lcd.printf("RoR: %.1f°C/min (Cooling)", current_ror);
      M5.Lcd.setCursor(35, 160);
      M5.Lcd.printf("%s", (abs(current_ror) < 5.0f) ? "Temperature Stable" : "Cool Down Active");
      M5.Lcd.setCursor(35, 180);
      M5.Lcd.printf("System Ready for Safe Recovery");
      M5.Lcd.setCursor(35, 205);
      M5.Lcd.printf("[A] Auto Reset [C] Manual Control");
      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
      
      // 5秒経過で自動的にダイアログを非表示
      if (millis() - recovery_dialog_start > 5000) {
        recovery_dialog_active = false;
      }
      
      // セオドア提言：30秒経過で安全のため自動復旧オプションを無効化
      if (millis() - recovery_dialog_start > 30000) {
        auto_recovery_available = false;
        recovery_dialog_active = false;
      }
    } else if (auto_recovery_available && !recovery_dialog_active) {
      // 簡易表示
      M5.Lcd.fillRect(0, 150, 320, 30, TFT_GREEN);
      M5.Lcd.setTextColor(TFT_BLACK, TFT_GREEN);
      M5.Lcd.setCursor(40, 160);
      M5.Lcd.printf("SAFE - [A]Auto Reset [C]Manual Reset");
      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    }
  } else {
    auto_recovery_available = false;
    recovery_dialog_active = false;
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
    uint32_t now = millis();
    if ((now - last_beep_time) > 3000) {  // 3秒間隔制限
      playBeep(500, 800);  // 低音で火力変更を通知
      last_beep_time = now;
    }
  }
  
  // 基本的な警告チェック
  RoastTarget target = getRoastTarget(current_stage, selected_roast_level);
  bool critical_temp = (current_temp > target.temp_max + 10) || 
                      (current_stage == STAGE_FINISH && current_temp > target.temp_max);
  
  uint32_t now = millis();
  if (critical_temp && ((now - last_critical_warning) > 5000)) {
    playCriticalWarningBeep();
    last_critical_warning = now;
    critical_temp_warning_active = true;
  } else if (!critical_temp) {
    critical_temp_warning_active = false;
  }
}

/**
 * セオドア提言：BLE差分パケット送信最適化
 * 温度/RoRデータは1秒間隔、統計データは15秒間隔で送信し帯域節約
 */
void sendBLEData() {
  uint32_t now = millis();
  if ((now - last_ble_send) < BLE_SEND_INTERVAL) {
    return;  // Don't send too frequently
  }
  
  if (!deviceConnected) {
    return;  // No client connected
  }
  
  last_ble_send = now;
  bool send_full_data = (now - last_full_data_send) >= FULL_DATA_INTERVAL;
  
  // セオドア提言：パケットサイズ最適化のため軽量JSONまたはフルJSON
  StaticJsonDocument<512> doc;
  
  // 必須データ（毎秒送信）
  doc["timestamp"] = now;
  doc["temperature"] = serialized(String(current_temp, 2));
  doc["ror"] = serialized(String(current_ror, 1));
  doc["state"] = (system_state == STATE_RUNNING) ? "running" : "standby";
  doc["type"] = send_full_data ? "full" : "lite";  // パケットタイプ識別
  
  if (send_full_data) {
    // 15秒間隔でのみ送信する重いデータ
    doc["mode"] = display_mode;
    doc["count"] = count;
    
    // 焙煎ガイドデータ
    if (roast_guide_active) {
      JsonObject roast = doc.createNestedObject("roast");
      roast["level"] = getRoastLevelName(selected_roast_level);
      roast["stage"] = getRoastStageName(current_stage);
      roast["elapsed"] = getRoastElapsedTime();
      roast["fire"] = getFirePowerName(current_recommended_fire);
    }
    
    // 統計データ
    if (count > 0) {
      JsonObject stats = doc.createNestedObject("stats");
      stats["min"] = serialized(String(temp_min_recorded, 2));
      stats["max"] = serialized(String(temp_max_recorded, 2));
      stats["avg"] = serialized(String(getAverageTemp(), 2));
    }
    
    last_full_data_send = now;
  }
  
  // JSON文字列にシリアライズ
  static String json_string;
  json_string = "";
  serializeJson(doc, json_string);
  json_string += "\n";
  
  pTxCharacteristic->setValue(json_string.c_str());
  pTxCharacteristic->notify();
}

void loop() {
  M5.update();
  handleButtons();
  handleNonBlockingBeeps();
  
  // セオドア提言：完全非ブロッキング初期化リトライ
  if (init_waiting && millis() - init_retry_timer >= 500) {
    if (kmeter.begin(&Wire, KM_ADDR, KM_SDA, KM_SCL, I2C_FREQ)) {
      M5_LOGI("KMeterISO initialization successful!");
      init_waiting = false;
    } else {
      M5_LOGE("KMeterISO still not found…再試行");
      init_retry_timer = millis(); // Reset timer for next retry
    }
  }
  
  // 非ブロッキング復旧成功表示処理
  if (recovery_display_active && millis() - recovery_display_start >= 1000) {
    M5.Lcd.fillScreen(TFT_BLACK);
    recovery_display_active = false;
    need_full_redraw = true;
  }

  // Handle BLE disconnection（非ブロッキング再接続）
  if (!deviceConnected && oldDeviceConnected && !ble_restart_pending) {
    ble_restart_timer = millis();
    ble_restart_pending = true;
    oldDeviceConnected = deviceConnected;
  }
  
  // 非ブロッキングBLE再接続処理
  if (ble_restart_pending && millis() - ble_restart_timer >= 300) {
    pServer->startAdvertising(); // restart advertising
    M5_LOGI("Start advertising");
    ble_restart_pending = false;
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

      setTempToBuffer(head, current_temp);
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
      
      // Update ticker system information periodically
      updateTickerSystemInfo();

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
  
  // Update ticker footer every loop iteration for smooth scrolling
  updateTickerFooter();
}
void handleNonBlockingBeeps() {
  // Handle stage change beeps
  if (stage_beep_active) {
    uint32_t now = millis();
    uint32_t elapsed = now - stage_beep_start;
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
    uint32_t now = millis();
    uint32_t elapsed = now - critical_beep_start;
    uint32_t beep_time = critical_beep_count * CRITICAL_BEEP_INTERVAL;
    
    if (elapsed >= beep_time && critical_beep_count < MAX_CRITICAL_BEEPS) {
      playBeep(150, 2000);
      critical_beep_count++;
    }
    
    if (critical_beep_count >= MAX_CRITICAL_BEEPS) {
      critical_beep_active = false;
    }
  }
  
  // セオドア提言：メロディの非ブロッキング更新
  updateMelody();
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
 * セオドア提言：完全非ブロッキングメロディ再生
 */
void playMelody(const Melody& melody_pgm) {
  if (melody_active) return;  // 既に再生中の場合は無視
  
  // PROGMEMから読み取り
  Melody melody;
  memcpy_P(&melody, &melody_pgm, sizeof(Melody));
  
  // メロディデータをコピー
  for (int i = 0; i < 8; i++) {
    melody_notes[i] = melody.notes[i];
  }
  melody_duration = melody.duration_ms;
  
  // 非ブロッキング再生開始
  melody_active = true;
  melody_start = millis();
  melody_note_index = 0;
  melody_note_start = millis();
  
  // 最初の音を再生
  if (melody_notes[0] > 0) {
    M5.Speaker.tone(melody_notes[0], melody_duration);
  }
}

/**
 * メロディの非ブロッキング更新処理
 */
void updateMelody() {
  if (!melody_active) return;
  
  uint32_t now = millis();
  uint32_t note_elapsed = now - melody_note_start;
  
  // 現在の音符の再生時間終了チェック
  if (note_elapsed >= melody_duration + 50) {  // 50msの間隔を設ける
    melody_note_index++;
    
    if (melody_note_index >= 8 || melody_notes[melody_note_index] <= 0) {
      // メロディ終了
      melody_active = false;
      M5.Speaker.stop();
      return;
    }
    
    // 次の音符を再生
    M5.Speaker.tone(melody_notes[melody_note_index], melody_duration);
    melody_note_start = now;
  }
}

/**
 * フッター領域の統一描画（画面下部のボタン指示）
 */
void drawFooter(const char* instructions) {
  int footer_y = 240 - FOOTER_HEIGHT;
  M5.Lcd.fillRect(0, footer_y, 320, FOOTER_HEIGHT, TFT_BLACK);
  M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
  M5.Lcd.setTextColor(TFT_WHITE);
  M5.Lcd.setCursor(5, footer_y + 4);
  M5.Lcd.printf("%s", instructions);
}

/**
 * ティッカーフッター：メッセージ追加
 */
void addTickerMessage(const char* format, ...) {
  if (!ticker_enabled) return;
  
  char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  // 既存メッセージと重複チェック
  for (int i = 0; i < ticker_message_count; i++) {
    if (strcmp(ticker_messages[i].text, buffer) == 0) {
      return; // 重複メッセージは追加しない
    }
  }
  
  // メッセージ追加（リングバッファ形式）
  int index = ticker_message_count % TICKER_MAX_MESSAGES;
  strncpy(ticker_messages[index].text, buffer, sizeof(ticker_messages[index].text) - 1);
  ticker_messages[index].added_time = millis();
  
  if (ticker_message_count < TICKER_MAX_MESSAGES) {
    ticker_message_count++;
  }
}

/**
 * ティッカーフッター：システム情報自動収集
 */
void updateTickerSystemInfo() {
  if (!ticker_enabled || system_state != STATE_RUNNING) return;
  
  static uint32_t last_update = 0;
  if (millis() - last_update < 10000) return; // 10秒間隔で更新
  last_update = millis();
  
  // 温度情報
  if (current_temp > 50.0f) {
    float ror = calculateRoR();
    addTickerMessage("温度: %.1f°C | RoR: %.1f°C/min", current_temp, ror);
  }
  
  // BLE接続状態
  if (deviceConnected) {
    addTickerMessage("BLE接続中");
  }
  
  // 統計情報
  if (count > 60) {
    float mean_temp = (count > 0) ? temp_sum / count : 0.0f;
    addTickerMessage("平均温度: %.1f°C | 最高: %.1f°C", mean_temp, temp_max_recorded);
  }
  
  // 焙煎ステージ情報
  if (roast_guide_active && current_stage > STAGE_PREHEAT) {
    const char* stage_names[] = {"予熱", "乾燥", "第一クラック", "展開", "第二クラック", "仕上げ"};
    addTickerMessage("焙煎ステージ: %s", stage_names[current_stage]);
  }
}

/**
 * ティッカーフッター：描画更新
 */
void updateTickerFooter() {
  if (!ticker_enabled || ticker_message_count == 0) return;
  
  uint32_t now = millis();
  
  // メッセージ切り替えタイミング
  if (now - ticker_message_start > TICKER_MESSAGE_DURATION) {
    ticker_current_index = (ticker_current_index + 1) % ticker_message_count;
    ticker_scroll_offset = 320; // 右端から開始
    ticker_message_start = now;
  }
  
  // スクロール更新
  if (now - ticker_last_scroll > TICKER_SCROLL_SPEED) {
    ticker_scroll_offset -= 2; // 2ピクセルずつスクロール
    ticker_last_scroll = now;
    
    // メッセージが左端を超えたら右端に戻す
    const char* current_msg = ticker_messages[ticker_current_index].text;
    int msg_width = M5.Lcd.textWidth(current_msg);
    if (ticker_scroll_offset < -msg_width) {
      ticker_scroll_offset = 320;
    }
  }
  
  // 描画
  M5.Lcd.fillRect(0, TICKER_Y_POSITION, 320, 20, TFT_BLACK);
  M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
  M5.Lcd.setTextColor(TFT_CYAN);
  M5.Lcd.setCursor(ticker_scroll_offset, TICKER_Y_POSITION + 4);
  M5.Lcd.printf("%s", ticker_messages[ticker_current_index].text);
}