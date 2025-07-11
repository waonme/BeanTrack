#include <M5Unified.h>
#include <M5UnitKmeterISO.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <stdarg.h>

#include "Audio/MelodyPlayer.h"
#include "Display/TickerFooter.h"
#include "Statistics/TemperatureStatistics.h"
#include "Safety/SafetySystem.h"
#include "BLE/BLEManager.h"
#include "RoastGuide/RoastGuide.h"

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

// 理想プロファイル定義
struct ProfilePoint {
  uint16_t sec;  // 経過秒
  float    temp; // 目標温度 (°C)
};

// 各焙煎レベルの理想プロファイル（時刻と温度の折れ線）
constexpr ProfilePoint PROFILE_LIGHT[] = {
  {   0,  25 },   // 投入直後 BT
  { 240, 150 },   // 乾燥終点
  { 420, 190 },   // メイラード終点（1ハゼ直前）
  { 450, 195 },   // 1ハゼ開始
  { 510, 200 },   // 1ハゼ終点
  { 540, 205 }    // 排出
};

constexpr ProfilePoint PROFILE_MEDIUM[] = {
  {   0,  25 },
  { 300, 150 },
  { 480, 200 },
  { 510, 202 },
  { 600, 210 },
  { 660, 218 }
};

constexpr ProfilePoint PROFILE_MEDIUM_DARK[] = {
  {   0,  25 },
  { 330, 150 },
  { 540, 200 },
  { 570, 203 },
  { 720, 220 },
  { 780, 225 }
};

constexpr ProfilePoint PROFILE_DARK[] = {
  {   0,  25 },
  { 360, 150 },
  { 600, 200 },
  { 630, 205 },
  { 840, 225 },
  { 900, 230 }
};

constexpr ProfilePoint PROFILE_FRENCH[] = {
  {   0,  25 },
  { 360, 150 },
  { 630, 200 },
  { 660, 205 },
  { 900, 230 },
  { 960, 238 }
};

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

// RoastLevel, RoastStage, FirePower, and RoastTarget are now defined in RoastGuide module

// セオドア提言：メモリ効率化のため int16_t に変更（0.1°C刻み）
int16_t  buf[BUF_SIZE];  // 0.1°C単位で格納（例：25.3°C → 253）
uint16_t head = 0;
uint16_t count = 0;


M5UnitKmeterISO kmeter;
uint32_t next_tick = 0;
uint8_t  km_err    = 0;
float    current_temp = 0;

DisplayMode display_mode = MODE_GRAPH;
SystemState system_state = STATE_STANDBY;
bool need_full_redraw = true;
float last_temp = NAN;
int16_t last_graph_x = -1;
int16_t last_graph_y = -1;

// セオドア提言：差分描画用キャッシュ変数
float last_displayed_temp = NAN;
float last_displayed_ror = NAN;
RoastGuide::FirePower last_displayed_fire = RoastGuide::FIRE_OFF;
RoastGuide::RoastStage last_displayed_stage = RoastGuide::STAGE_PREHEAT;
uint32_t last_displayed_time = 0;
bool last_roast_guide_state = false;
int last_ror_wait_seconds = -1;


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


// Non-blocking stage change beeps
bool stage_beep_active = false;
uint32_t stage_beep_start = 0;
int stage_beep_count = 0;
constexpr int MAX_STAGE_BEEPS = 3;
constexpr uint32_t STAGE_BEEP_INTERVAL = 300; // 300ms between beeps
int stage_beep_frequencies[3] = {1000, 1200, 1500};
int stage_beep_durations[3] = {200, 200, 300};

// セオドア提言：非ブロッキングメロディシステム

// 非ブロッキング初期化待機
static uint32_t init_retry_timer = 0;
static bool init_waiting = false;

// 非ブロッキング復旧成功表示
static uint32_t recovery_display_start = 0;
static bool recovery_display_active = false;

// 非ブロッキングBLE再接続待機
static uint32_t ble_restart_timer = 0;
static bool ble_restart_pending = false;

// 火力推奨
static RoastGuide::FirePower last_recommended_fire = RoastGuide::FIRE_MEDIUM;

// セオドア提言：BLE差分パケット送信システム
static uint32_t last_full_data_send = 0;
constexpr uint32_t FULL_DATA_INTERVAL = 15000; // 15秒間隔で完全データ送信


// Hysteresis values as constexpr
constexpr float FIRE_HYSTERESIS = 0.5f;  // °C hysteresis for fire power changes

// ティッカーフッター機能

// Melody system for elegant notifications

// セオドア提言：メロディもPROGMEMで最適化
const MelodyPlayer::Melody PROGMEM MELODY_STAGE_CHANGE = {{262, 294, 330, 349, 392, 440, 494, 523}, 150};  // C-D-E-F-G-A-B-C上昇
const MelodyPlayer::Melody PROGMEM MELODY_FIRST_CRACK = {{523, 494, 440, 392, 349, 330, 294, 262}, 120};    // C-B-A-G-F-E-D-C下降
const MelodyPlayer::Melody PROGMEM MELODY_EMERGENCY = {{880, 831, 784, 740, 698, 659, 622, 587}, 100};      // 高音から滑落
const MelodyPlayer::Melody PROGMEM MELODY_COMPLETION = {{392, 523, 392, 523, 392, 523, 659, 523}, 200};    // G-C完成旋律

// セオドア提言：関数宣言（非ブロッキング関数群）

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

// モジュラー化対応ラッパー関数
inline void playMelodyWrapper(const MelodyPlayer::Melody& melody_pgm) {
    MELODY_PLAYER->playMelody(melody_pgm);
}

inline void updateMelodyWrapper() {
    MELODY_PLAYER->update();
}

// ティッカーフッターラッパー関数
inline void addTickerMessageWrapper(const char* format, ...) {
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    TICKER->addMessage("%s", buffer);
}

inline void updateTickerFooterWrapper() {
    TICKER->update();
}

inline void setTickerEnabledWrapper(bool enable) {
    TICKER->setEnabled(enable);
}

inline bool isTickerEnabledWrapper() {
    return TICKER->isEnabled();
}



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

// Temperature thresholds now managed by RoastGuide module
// Wrapper functions to maintain compatibility
inline float getDangerTemp(RoastGuide::RoastLevel level) {
  return ROAST_GUIDE->getDangerTemp(level);
}

inline float getCriticalTemp(RoastGuide::RoastLevel level) {
  return ROAST_GUIDE->getCriticalTemp(level);
}

// セオドア提言：LGFX環境向けclamp関数
template <typename T>
constexpr T clamp(T value, T min_val, T max_val) {
  return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

// Roast guide variables removed - now managed by RoastGuide module
// Use ROAST_GUIDE->getSelectedLevel(), ROAST_GUIDE->getCurrentStage(), etc.
float stage_start_temp = 0.0f;  // Still used locally

// Audio notification variables
uint32_t last_beep_time = 0;
bool stage_change_beep_played = false;
bool critical_temp_warning_active = false;
uint32_t last_critical_warning = 0;

// Fire power recommendation - removed, now use ROAST_GUIDE for fire recommendations

// Button long press detection
uint32_t btnC_press_start = 0;
bool btnC_long_press_handled = false;
uint32_t btnB_press_start = 0;
bool btnB_long_press_handled = false;
constexpr uint32_t LONG_PRESS_DURATION = 2000;  // 2 seconds

void drawGraph();
void drawIdealCurve(const ProfilePoint* profile, size_t len);
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
void drawRoastLevelSelection();
// RoastTarget getRoastTarget now delegated to RoastGuide module
// const char* getRoastLevelName now delegated to RoastGuide module
// const char* getRoastStageName removed - use ROAST_GUIDE methods
uint32_t getRoastElapsedTime();
float getStageElapsedTime();
void sendBLEData();
const char* getFirePowerName(RoastGuide::FirePower fire);
RoastGuide::FirePower calculateRecommendedFire();
void playBeep(int duration_ms, int frequency = 1000);
// playMelody is now handled by MelodyPlayer class
void playStageChangeBeep();
void playCriticalWarningBeep();
void updateFirePowerRecommendation();
void forceNextStage();
void checkEmergencyConditions();
void handleNonBlockingBeeps();
float getNextStageKeyTemp(RoastGuide::RoastStage stage, RoastGuide::RoastLevel level);
const char* getGasAdjustmentAdvice(RoastGuide::FirePower current_fire, RoastGuide::FirePower target_fire);
void drawFooter(const char* instructions);
float getDangerTemp(RoastGuide::RoastLevel level);
float getCriticalTemp(RoastGuide::RoastLevel level);

// 統計ラッパー関数（モジュラー化対応）
inline float getMinTemp() {
  return TEMP_STATS->getMin();
}

inline float getMaxTemp() {
  return TEMP_STATS->getMax();
}

inline float getAverageTemp() {
  return TEMP_STATS->getAverage();
}

inline void updateStats(float temp) {
  TEMP_STATS->addTemperature(temp);
}

inline void resetStats() {
  TEMP_STATS->reset();
}

inline void recalculateStatsFromBuffer() {
  // Create a temporary float buffer for the statistics module
  float* temp_buf = new float[count];
  for (uint16_t i = 0; i < count; i++) {
    temp_buf[i] = getTempFromBuffer((head - count + i + BUF_SIZE) % BUF_SIZE);
  }
  TEMP_STATS->recalculateFromBuffer(temp_buf, count, count);
  delete[] temp_buf;
}

// BLE接続状態ラッパー関数
inline bool isBLEConnected() {
  return BLE_MGR->isConnected();
}

// RoastGuide helper functions
inline const char* getStageName(RoastGuide::RoastStage stage) {
  static const char* stage_names[] = {
    "Preheat", "Charge", "Drying", "Maillard", 
    "First Crack", "Development", "Second Crack", "Finish"
  };
  return stage_names[(int)stage];
}

inline RoastGuide::FirePower getRecommendedFire() {
  if (!ROAST_GUIDE->isActive()) return RoastGuide::FIRE_MEDIUM;
  auto target = ROAST_GUIDE->getRoastTarget(ROAST_GUIDE->getCurrentStage(), ROAST_GUIDE->getSelectedLevel());
  return target.fire;
}

// Helper variables for timing (managed locally)
static uint32_t roast_start_time = 0;
static uint32_t stage_start_time = 0;

// Ticker wrapper function (needs to be after statistics wrappers)
inline void updateTickerSystemInfoWrapper() {
    // モジュラー版では、TickerFooterが自動的にシステム情報を収集する
    // 必要に応じて情報を追加
    if (TICKER->isEnabled() && system_state == STATE_RUNNING) {
        static uint32_t last_update = 0;
        if (millis() - last_update >= 10000) { // 10秒間隔
            last_update = millis();
            
            // 温度情報
            if (current_temp > 50.0f) {
                TICKER->addMessage("温度: %.1f°C", current_temp);
            }
            
            // BLE接続状態
            if (isBLEConnected()) {
                TICKER->addMessage("BLE接続中");
            }
            
            // 統計情報
            if (count > 60) {
                TICKER->addMessage("平均温度: %.1f°C | 最高: %.1f°C", getAverageTemp(), getMaxTemp());
            }
        }
    }
}

// 安全システムラッパー関数（モジュラー化対応）
inline bool isEmergencyActive() {
  return SAFETY->getState().emergency_active;
}

inline bool isAutoRecoveryAvailable() {
  return SAFETY->getState().auto_recovery_available;
}

inline bool isRecoveryDialogActive() {
  return SAFETY->getState().recovery_dialog_active;
}

inline void setEmergencyActive(bool active) {
  if (!active) {
    SAFETY->resetEmergency();
  }
  // Note: Setting to true is handled by checkEmergencyConditions
}

inline void executeAutoRecovery() {
  SAFETY->executeAutoRecovery();
}

inline void showRecoveryDialog(bool show) {
  SAFETY->showRecoveryDialog(show);
}

inline void setAutoRecoveryAvailable(bool available) {
  // Handled internally by SafetySystem
}


void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  
  // セオドア提言：Sprite初期化（真のスクロール実装）
  graph_sprite.createSprite(GRAPH_W, GRAPH_H);
  sprite_initialized = true;

  // MelodyPlayer初期化
  MELODY_PLAYER->begin();

  // TickerFooter初期化
  TICKER->begin();

  // TemperatureStatistics初期化
  TEMP_STATS->begin();

  // SafetySystem初期化
  SAFETY->begin();

  // RoastGuide初期化
  ROAST_GUIDE->begin();

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

  // BLEManager初期化
  BLE_MGR->begin("M5Stack-Thermometer");
  
  // データ要求コールバック設定
  BLE_MGR->setDataRequestCallback([](JsonDocument& doc, bool fullData) {
    // この関数はsendBLEDataの内容を移植
    if (fullData) {
      doc["type"] = "full";
    } else {
      doc["type"] = "lite";
    }
    
    doc["timestamp"] = millis();
    doc["temp"] = serialized(String(current_temp, 2));
    doc["ror"] = serialized(String(current_ror, 2));
    doc["state"] = system_state;
    
    if (fullData) {
      doc["mode"] = display_mode;
      doc["count"] = count;
      
      if (ROAST_GUIDE->isActive()) {
        JsonObject roast = doc["roast"].to<JsonObject>();
        roast["active"] = true;
        roast["level"] = ROAST_GUIDE->getRoastLevelName(ROAST_GUIDE->getSelectedLevel());
        roast["stage"] = getStageName(ROAST_GUIDE->getCurrentStage());  // Helper function needed
        roast["elapsed"] = getRoastElapsedTime();
        roast["fire"] = getFirePowerName(getRecommendedFire());  // Helper function needed
      }
      
      if (count > 0) {
        JsonObject stats = doc["stats"].to<JsonObject>();
        stats["min"] = serialized(String(getMinTemp(), 2));
        stats["max"] = serialized(String(getMaxTemp(), 2));
        stats["avg"] = serialized(String(getAverageTemp(), 2));
      }
    }
  });

  M5.Lcd.setRotation(1);
  M5.Lcd.setBrightness(LCD_BRIGHTNESS);  // バッテリー寿命延長のため明度を調整
  M5.Lcd.fillScreen(TFT_BLACK);
  
  // Initialize statistics
  resetStats();
  current_ror = 0.0f;
  ror_count = 0;
  
  // Initialize roast guide through RoastGuide module
  ROAST_GUIDE->stop();  // Ensure it's stopped
  // Other roast guide state now managed by RoastGuide module
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
    last_displayed_fire = (RoastGuide::FirePower)-1;
    last_displayed_stage = (RoastGuide::RoastStage)-1;
    last_displayed_time = UINT32_MAX;
    last_roast_guide_state = !ROAST_GUIDE->isActive();
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
  bool fire_display_needed = (ROAST_GUIDE->isActive() && system_state == STATE_RUNNING);
  RoastGuide::FirePower current_recommended_fire = getRecommendedFire();
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
        case RoastGuide::FIRE_OFF: fire_color = TFT_DARKGREY; break;
        case RoastGuide::FIRE_VERY_LOW: fire_color = TFT_BLUE; break;
        case RoastGuide::FIRE_LOW: fire_color = TFT_CYAN; break;
        case RoastGuide::FIRE_MEDIUM: fire_color = TFT_YELLOW; break;
        case RoastGuide::FIRE_HIGH: fire_color = TFT_ORANGE; break;
        case RoastGuide::FIRE_VERY_HIGH: fire_color = TFT_RED; break;
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
  if (ROAST_GUIDE->isActive()) {
    RoastGuide::RoastStage current_stage = ROAST_GUIDE->getCurrentStage();
    if (needs_full_clear || current_stage != last_displayed_stage) {
      M5.Lcd.fillRect(150, 25, 170, 12, TFT_BLACK);
      M5.Lcd.setCursor(150, 25);
      M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
      M5.Lcd.setTextColor(TFT_WHITE);
      M5.Lcd.printf("Stage: %s", getStageName(current_stage));
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
  } else if (last_roast_guide_state != ROAST_GUIDE->isActive()) {
    // 焙焙ガイドが停止した場合、関連領域をクリア
    M5.Lcd.fillRect(150, 25, 170, 12, TFT_BLACK);
    M5.Lcd.fillRect(0, 35, 120, 12, TFT_BLACK);
    last_roast_guide_state = ROAST_GUIDE->isActive();
  }
}

/**
 * 現在の buf 内容でグラフを再描画
 */
void drawIdealCurve(const ProfilePoint* profile, size_t len) {
  if (!sprite_initialized || !profile || len < 2) return;

  // ドット間隔ピクセル
  constexpr int DOT_STEP = 4;

  for (size_t i = 1; i < len; ++i) {
    // 区間の２端
    float t0 = profile[i - 1].sec;
    float v0 = profile[i - 1].temp;
    float t1 = profile[i].sec;
    float v1 = profile[i].temp;

    // 区間を 1 秒刻みで線形補間しドットを置く
    for (uint16_t s = t0; s <= t1; ++s) {
      // 1 s → 1 px で 15 分グラフ (900 s) に収まる
      float x_ratio = (float)s / (BUF_SIZE - 1);        // 0.0 – 1.0
      int   x = x_ratio * GRAPH_W;

      // 線形補間温度
      float f = (t1 > t0) ? (float)(s - t0) / (t1 - t0) : 0;
      float temp = v0 + f * (v1 - v0);

      // 温度を Y 座標へ
      float y_ratio = (temp - TEMP_MIN) / (TEMP_MAX - TEMP_MIN);
      int   y = GRAPH_H - y_ratio * GRAPH_H;

      // ドット間隔ごとに描画（範囲チェック付き）
      if ((s % DOT_STEP) == 0 && x >= 0 && x < GRAPH_W && y >= 0 && y < GRAPH_H) {
        graph_sprite.drawPixel(x, y, TFT_DARKGREY);
      }
    }
  }
}

void drawGraph() {
  if (count == 0 || !sprite_initialized) return;

  // 画面上の枠線を描画
  M5.Lcd.fillRect(GRAPH_X0-1, GRAPH_Y0-1, GRAPH_W+2, GRAPH_H+2, TFT_BLACK);
  M5.Lcd.drawRect(GRAPH_X0-1, GRAPH_Y0-1, GRAPH_W+2, GRAPH_H+2, TFT_WHITE);
  
  // Sprite内容をクリアして背景を再構築
  graph_sprite.fillSprite(TFT_BLACK);
  
  // ローストレベル依存の危険域表示（Sprite内）
  float current_danger_temp = getDangerTemp(ROAST_GUIDE->getSelectedLevel());
  float current_critical_temp = getCriticalTemp(ROAST_GUIDE->getSelectedLevel());
  
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

  // 理想曲線を描画（背景として）
  switch (ROAST_GUIDE->getSelectedLevel()) {
    case RoastGuide::ROAST_LIGHT:
    case RoastGuide::ROAST_MEDIUM_LIGHT:
      drawIdealCurve(PROFILE_LIGHT, sizeof(PROFILE_LIGHT)/sizeof(ProfilePoint));
      break;
    case RoastGuide::ROAST_MEDIUM:
      drawIdealCurve(PROFILE_MEDIUM, sizeof(PROFILE_MEDIUM)/sizeof(ProfilePoint));
      break;
    case RoastGuide::ROAST_MEDIUM_DARK:
      drawIdealCurve(PROFILE_MEDIUM_DARK, sizeof(PROFILE_MEDIUM_DARK)/sizeof(ProfilePoint));
      break;
    case RoastGuide::ROAST_DARK:
      drawIdealCurve(PROFILE_DARK, sizeof(PROFILE_DARK)/sizeof(ProfilePoint));
      break;
    case RoastGuide::ROAST_FRENCH:
      drawIdealCurve(PROFILE_FRENCH, sizeof(PROFILE_FRENCH)/sizeof(ProfilePoint));
      break;
    default:
      break;
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
        bool newState = !isTickerEnabledWrapper();
        setTickerEnabledWrapper(newState);
        combo_handled = true;
        
        // Visual feedback
        M5.Lcd.fillRect(60, 100, 200, 40, TFT_BLACK);
        M5.Lcd.drawRect(60, 100, 200, 40, newState ? TFT_GREEN : TFT_RED);
        M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
        M5.Lcd.setTextColor(newState ? TFT_GREEN : TFT_RED);
        M5.Lcd.setCursor(70, 115);
        M5.Lcd.printf("TICKER %s", newState ? "ENABLED" : "DISABLED");
        M5.Lcd.setTextColor(TFT_WHITE);
        playBeep(100, newState ? 1000 : 800);
        
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
      if (isEmergencyActive() && isAutoRecoveryAvailable()) {
        // 自動復旧実行
        executeAutoRecovery();
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
      if (ROAST_GUIDE->isActive() && ROAST_GUIDE->getCurrentStage() < RoastGuide::STAGE_FINISH) {
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
      if (ROAST_GUIDE->isFirstCrackConfirmationNeeded()) {
        ROAST_GUIDE->confirmFirstCrack();
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
      } else if (display_mode == MODE_GUIDE && !ROAST_GUIDE->isActive()) {
        // 焙煎レベル変更
        ROAST_GUIDE->cycleRoastLevel();
      } else {
        // Reset statistics
        recalculateStatsFromBuffer();
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
      resetStats();
      current_ror = 0.0f;
      ror_count = 0;
      // Reset roast guide state
      ROAST_GUIDE->stop();
      setEmergencyActive(false);  // Theodore提言：緊急停止状態もリセット
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
        if (display_mode == MODE_GUIDE && !ROAST_GUIDE->isActive()) {
          // 焙煎ガイド開始
          ROAST_GUIDE->start(ROAST_GUIDE->getSelectedLevel());
          stage_start_temp = current_temp;
          roast_start_time = millis();
          stage_start_time = millis();
        } else {
          // Stop monitoring
          system_state = STATE_STANDBY;
          ROAST_GUIDE->stop();
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
    float current_danger_temp = getDangerTemp(ROAST_GUIDE->getSelectedLevel());
    float current_critical_temp = getCriticalTemp(ROAST_GUIDE->getSelectedLevel());
    
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
  M5.Lcd.printf("^ Maximum: %.2f C", getMaxTemp());
  
  y_pos += 25;
  M5.Lcd.setCursor(20, y_pos);
  M5.Lcd.printf("v Minimum: %.2f C", getMinTemp());
  
  y_pos += 25;
  M5.Lcd.setCursor(20, y_pos);
  M5.Lcd.printf("~ Average: %.2f C", getAverageTemp());
  
  y_pos += 25;
  M5.Lcd.setCursor(20, y_pos);
  M5.Lcd.printf("# Data Points: %d", count);
  
  // Button instructions（統一フッターに移動）
  drawFooter("[A]Mode [B]Reset [C]Stop");
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
  if (ROAST_GUIDE->isActive()) {
    RoastGuide::RoastTarget target = ROAST_GUIDE->getRoastTarget(ROAST_GUIDE->getCurrentStage(), ROAST_GUIDE->getSelectedLevel());
    RoastGuide::RoastStage current_stage = ROAST_GUIDE->getCurrentStage();
    if (current_ror > target.ror_max + 3) {
      M5.Lcd.setTextColor(TFT_RED);
      M5.Lcd.printf("[!] RoR: Too High for %s", getStageName(current_stage));
    } else if (current_ror > target.ror_max) {
      M5.Lcd.setTextColor(TFT_ORANGE);
      M5.Lcd.printf("[^] RoR: High for %s", getStageName(current_stage));
    } else if (current_ror >= target.ror_min) {
      M5.Lcd.setTextColor(TFT_GREEN);
      M5.Lcd.printf("[OK] RoR: Good for %s", getStageName(current_stage));
    } else if (current_ror >= target.ror_min - 2) {
      M5.Lcd.setTextColor(TFT_CYAN);
      M5.Lcd.printf("[v] RoR: Low for %s", getStageName(current_stage));
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

// getRoastTarget implementation removed - now delegated to RoastGuide module

// getRoastLevelName and getRoastStageName removed - now delegated to RoastGuide module

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


void drawRoastLevelSelection() {
  M5.Lcd.fillRect(0, GRAPH_Y0, 320, 240 - GRAPH_Y0, TFT_BLACK);
  M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
  
  int y_pos = GRAPH_Y0 + 10;
  M5.Lcd.setCursor(60, y_pos);
  M5.Lcd.printf(">> Select Roast Level <<");
  
  y_pos += 30;
  
  // 2列レイアウトで表示
  for (int i = 0; i < RoastGuide::ROAST_COUNT; i++) {
    int col = i % 2;  // 0 or 1
    int row = i / 2;  // 0, 1, 2
    
    int x_pos = col == 0 ? 20 : 170;  // 左列と右列
    int current_y = y_pos + row * 20;
    
    M5.Lcd.setCursor(x_pos, current_y);
    if (i == ROAST_GUIDE->getSelectedLevel()) {
      M5.Lcd.setTextColor(TFT_YELLOW);
      M5.Lcd.printf("> %s", ROAST_GUIDE->getRoastLevelName((RoastGuide::RoastLevel)i));
      M5.Lcd.setTextColor(TFT_WHITE);
    } else {
      M5.Lcd.printf("  %s", ROAST_GUIDE->getRoastLevelName((RoastGuide::RoastLevel)i));
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
  M5.Lcd.printf("%s - %s", ROAST_GUIDE->getRoastLevelName(ROAST_GUIDE->getSelectedLevel()), getStageName(ROAST_GUIDE->getCurrentStage()));
  
  // 段階プログレスバー
  y_pos += 15;
  int progress_width = 280;
  int progress_x = 20;
  int stage_progress = (ROAST_GUIDE->getCurrentStage() * progress_width) / 7;  // 8段階
  M5.Lcd.drawRect(progress_x, y_pos, progress_width, 8, TFT_WHITE);
  M5.Lcd.fillRect(progress_x + 1, y_pos + 1, stage_progress, 6, TFT_GREEN);
  
  // 現在ステージ内の詳細進行率
  y_pos += 10;
  RoastGuide::RoastTarget stage_target = ROAST_GUIDE->getRoastTarget(ROAST_GUIDE->getCurrentStage(), ROAST_GUIDE->getSelectedLevel());
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
    uint16_t color = (i <= ROAST_GUIDE->getCurrentStage()) ? TFT_YELLOW : TFT_DARKGREY;
    M5.Lcd.fillCircle(dot_x, y_pos + 4, 3, color);
  }
  
  y_pos += 25;  // プログレスバー拡張分のスペースを追加
  M5.Lcd.setCursor(10, y_pos);
  M5.Lcd.printf("Time: %02d:%02d", getRoastElapsedTime() / 60, getRoastElapsedTime() % 60);
  
  // 現在の目標値
  RoastGuide::RoastTarget target = ROAST_GUIDE->getRoastTarget(ROAST_GUIDE->getCurrentStage(), ROAST_GUIDE->getSelectedLevel());
  
  y_pos += 20;
  M5.Lcd.setCursor(10, y_pos);
  // 三層ガイド表示（初心者向け明確化）
  M5.Lcd.setTextColor(TFT_CYAN);
  M5.Lcd.printf("Maintain: %.0f-%.0f C", target.temp_min, target.temp_max);
  M5.Lcd.setTextColor(TFT_WHITE);
  
  y_pos += 15;
  M5.Lcd.setCursor(10, y_pos);
  // 次段階の鍵温度表示
  float next_key_temp = getNextStageKeyTemp(ROAST_GUIDE->getCurrentStage(), ROAST_GUIDE->getSelectedLevel());
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
  M5.Lcd.printf("Fire: %s", getFirePowerName(getRecommendedFire()));
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
  if (ROAST_GUIDE->getCurrentStage() == RoastGuide::STAGE_FINISH && y_pos < max_y - 20) {
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
    RoastGuide::FirePower current_fire = getRecommendedFire();
    M5.Lcd.printf("Pred: %.0f°C | %s", predictor.predictTemperatureIn30s(), 
                  getGasAdjustmentAdvice(last_recommended_fire, current_fire));
  }
  
  // ボタン指示（統一フッターに移動）
  if (first_crack_confirmation_needed) {
    drawFooter("[A]Mode [B]Confirm 1st Crack [C]Stop");
  } else if (ROAST_GUIDE->isActive() && ROAST_GUIDE->getCurrentStage() < RoastGuide::STAGE_FINISH) {
    drawFooter("[A]Mode [B-Hold]Next Stage [C]Stop");
  } else {
    drawFooter("[A]Mode [C]Stop");
  }
}

const char* getFirePowerName(RoastGuide::FirePower fire) {
  switch(fire) {
    case RoastGuide::FIRE_OFF: return "OFF";
    case RoastGuide::FIRE_VERY_LOW: return "極弱火";
    case RoastGuide::FIRE_LOW: return "弱火";
    case RoastGuide::FIRE_MEDIUM: return "中火";
    case RoastGuide::FIRE_HIGH: return "強火";
    case RoastGuide::FIRE_VERY_HIGH: return "極強火";
    default: return "不明";
  }
}

RoastGuide::FirePower calculateRecommendedFire() {
  if (!ROAST_GUIDE->isActive()) return RoastGuide::FIRE_MEDIUM;
  
  RoastGuide::RoastTarget target = ROAST_GUIDE->getRoastTarget(ROAST_GUIDE->getCurrentStage(), ROAST_GUIDE->getSelectedLevel());
  RoastGuide::FirePower base_fire = target.fire;
  
  // Scott Rao原則：「常に下降するRoR」を維持する火力制御
  float ror_trend = 0.0f;
  if (count >= 3) {
    // 直近3点のRoR傾向を計算
    uint16_t idx1 = (head - 3 + BUF_SIZE) % BUF_SIZE;
    uint16_t idx2 = (head - 1 + BUF_SIZE) % BUF_SIZE;
    ror_trend = getTempFromBuffer(idx2) - getTempFromBuffer(idx1);  // 簡易的な傾向
  }
  
  // 段階別の高度な火力制御
  auto current_stage = ROAST_GUIDE->getCurrentStage();
  switch(current_stage) {
    case RoastGuide::STAGE_CHARGE:
      // 投入段階：RoRピーク後の下降開始をサポート
      if (current_ror > 18 && ror_trend > 0) {
        base_fire = RoastGuide::FIRE_LOW;  // 早めに火力を落とす
      } else if (current_ror < 10) {
        base_fire = RoastGuide::FIRE_MEDIUM;  // 十分な熱を与える
      }
      break;
      
    case RoastGuide::STAGE_DRYING:
      // 乾燥段階：10-15°C/minを維持しつつ下降
      if (current_ror > target.ror_max + 2) {
        base_fire = (base_fire > RoastGuide::FIRE_OFF) ? (RoastGuide::FirePower)(base_fire - 1) : RoastGuide::FIRE_OFF;
      } else if (current_ror < target.ror_min - 2) {
        base_fire = (base_fire < RoastGuide::FIRE_VERY_HIGH) ? (RoastGuide::FirePower)(base_fire + 1) : RoastGuide::FIRE_VERY_HIGH;
      }
      break;
      
    case RoastGuide::STAGE_MAILLARD:
      // メイラード段階：滑らかな下降を最優先
      if (ror_trend > 0.5f) {  // RoRが上昇傾向
        base_fire = (base_fire > RoastGuide::FIRE_OFF) ? (RoastGuide::FirePower)(base_fire - 1) : RoastGuide::FIRE_OFF;
      } else if (current_ror > target.ror_max) {
        base_fire = RoastGuide::FIRE_VERY_LOW;  // 1ハゼ前に火力を十分下げる
      } else if (current_ror < target.ror_min && current_temp < 180) {
        base_fire = (base_fire < RoastGuide::FIRE_LOW) ? (RoastGuide::FirePower)(base_fire + 1) : RoastGuide::FIRE_LOW;
      }
      break;
      
    case RoastGuide::STAGE_FIRST_CRACK:
      // 1ハゼ：クラッシュ防止とフィック回避が最優先
      base_fire = RoastGuide::FIRE_VERY_LOW;  // 基本は極低火
      if (current_ror < -1.0f) {  // 急激なクラッシュ
        base_fire = RoastGuide::FIRE_LOW;  // 少し火力を戻す
      } else if (ror_trend > 0.2f) {  // フィックの兆候
        base_fire = RoastGuide::FIRE_OFF;  // 即座に火を切る
      }
      break;
      
    case RoastGuide::STAGE_DEVELOPMENT:
      // 発達段階：RoR 0に向けて緩やかに制御
      if (current_ror > 3) {
        base_fire = RoastGuide::FIRE_OFF;  // 火力を大幅削減
      } else if (current_ror > 1) {
        base_fire = RoastGuide::FIRE_VERY_LOW;
      } else if (current_ror < 0.5f && getStageElapsedTime() < target.time_min) {
        base_fire = RoastGuide::FIRE_VERY_LOW;  // 最低限の熱は維持
      } else {
        base_fire = RoastGuide::FIRE_OFF;  // 基本はオフ
      }
      break;
      
    case RoastGuide::STAGE_SECOND_CRACK:
    case RoastGuide::STAGE_FINISH:
      // 2ハゼ以降：火力オフが基本
      base_fire = RoastGuide::FIRE_OFF;
      break;
      
    default:
      // 予熱段階
      base_fire = RoastGuide::FIRE_HIGH;
      break;
  }
  
  // 温度安全措置（最優先）
  float current_danger_temp = getDangerTemp(ROAST_GUIDE->getSelectedLevel());
  if (current_temp > current_danger_temp - 5) {
    base_fire = RoastGuide::FIRE_OFF;  // 危険温度接近時は火力カット
  } else if (current_temp > target.temp_max + 3) {
    base_fire = (base_fire > RoastGuide::FIRE_OFF) ? (RoastGuide::FirePower)(base_fire - 1) : RoastGuide::FIRE_OFF;
  }
  
  // ヒステリシス適用（振動防止）
  RoastGuide::FirePower current_fire = last_recommended_fire;
  if (abs(base_fire - current_fire) <= 1) {
    // 微小な変更は無視
    if (current_ror >= target.ror_min - 1 && 
        current_ror <= target.ror_max + 1 &&
        abs(ror_trend) < 0.3f) {
      return current_fire;
    }
  }
  
  return base_fire;
}

void playBeep(int duration_ms, int frequency) {
  // M5Stackのスピーカーでビープ音を鳴らす
  MELODY_PLAYER->playBeep(duration_ms, frequency);
}

void playStageChangeBeep() {
  // 段階変更時の3音ビープ（非ブロッキング開始）
  stage_beep_active = true;
  stage_beep_start = millis();
  stage_beep_count = 0;
}

void playCriticalWarningBeep() {
  SAFETY->playCriticalWarning();
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
  if (!ROAST_GUIDE->isActive() || ROAST_GUIDE->getCurrentStage() >= RoastGuide::STAGE_FINISH) return;
  
  RoastGuide::RoastStage prev_stage = ROAST_GUIDE->getCurrentStage();
  
  // 次の段階に強制移行
  // TODO: Implement forceNextStage in RoastGuide module
  // ROAST_GUIDE->forceNextStage();
  
  // Check if stage changed and play beep
  if (prev_stage != ROAST_GUIDE->getCurrentStage()) {
    playStageChangeBeep();
  }
}

void checkEmergencyConditions() {
  // Configure safety thresholds
  SAFETY->setDangerTemp(getDangerTemp(ROAST_GUIDE->getSelectedLevel()));
  SAFETY->setCriticalTemp(getCriticalTemp(ROAST_GUIDE->getSelectedLevel()));
  
  // Set callbacks
  static bool callbacks_set = false;
  if (!callbacks_set) {
    SAFETY->setBeepCallback(playBeep);
    SAFETY->setEmergencyCallback([]() {
      // Emergency stop - handled by RoastGuide
      ROAST_GUIDE->stop();
      M5.Lcd.fillScreen(TFT_RED);
      M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);
      M5.Lcd.setFont(&fonts::lgfxJapanGothic_36);
      M5.Lcd.setCursor(50, 100);
      M5.Lcd.printf("EMERGENCY STOP!");
    });
    callbacks_set = true;
  }
  
  // Check conditions
  bool guide_active = ROAST_GUIDE->isActive();
  SAFETY->checkEmergencyConditions(current_temp, current_ror, ROAST_GUIDE->getCurrentStage(), guide_active);
  
  // Draw recovery dialog if active
  if (SAFETY->getState().recovery_dialog_active) {
    SAFETY->drawRecoveryDialog(current_temp, current_ror);
  }
  
  // Show danger warning
  if (current_temp >= getDangerTemp(ROAST_GUIDE->getSelectedLevel())) {
    M5.Lcd.fillRect(0, 100, 320, 40, TFT_RED);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_RED);
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_24);
    M5.Lcd.setCursor(20, 110);
    M5.Lcd.printf("!!! DANGER TEMP: %.1f°C !!!", current_temp);
    M5.Lcd.setCursor(20, 125);
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
    M5.Lcd.printf("(%s Limit: %.0f°C)", ROAST_GUIDE->getRoastLevelName(ROAST_GUIDE->getSelectedLevel()), getDangerTemp(ROAST_GUIDE->getSelectedLevel()));
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    
    // 三段階音響警告
    playTemperatureWarning(current_temp, getDangerTemp(ROAST_GUIDE->getSelectedLevel()), getCriticalTemp(ROAST_GUIDE->getSelectedLevel()));
  }
}

const char* getGasAdjustmentAdvice(RoastGuide::FirePower current_fire, RoastGuide::FirePower target_fire) {
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
  if (!ROAST_GUIDE->isActive()) return;
  
  RoastGuide::FirePower new_fire = calculateRecommendedFire();
  
  // 火力推奨が変わった場合の通知
  if (new_fire != last_recommended_fire) {
    last_recommended_fire = new_fire;
    
    // 火力変更の音声通知
    uint32_t now = millis();
    if ((now - last_beep_time) > 3000) {  // 3秒間隔制限
      playBeep(500, 800);  // 低音で火力変更を通知
      last_beep_time = now;
    }
  }
  
  // 基本的な警告チェック
  RoastGuide::RoastTarget target = ROAST_GUIDE->getRoastTarget(ROAST_GUIDE->getCurrentStage(), ROAST_GUIDE->getSelectedLevel());
  bool critical_temp = (current_temp > target.temp_max + 10) || 
                      (ROAST_GUIDE->getCurrentStage() == RoastGuide::STAGE_FINISH && current_temp > target.temp_max);
  
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
  // モジュラーBLEManagerが自動的に処理
  BLE_MGR->update();
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
      updateStats(current_temp);

      setTempToBuffer(head, current_temp);
      head = (head + 1) % BUF_SIZE;
      if (count < BUF_SIZE) ++count;

      // Calculate and update RoR (both 15s and 60s)
      current_ror = calculateRoR();
      current_ror_15s = calculateRoR15s();
      updateRoRBuffer();
      
      // Check for stall condition
      ROAST_GUIDE->checkStallCondition(current_temp, current_ror);
      
      // Add temperature to predictor
      predictor.addTemperature(current_temp);
      
      // Check emergency conditions
      checkEmergencyConditions();

      // Update fire power recommendations and audio notifications
      updateFirePowerRecommendation();

      // Send BLE data
      sendBLEData();
      
      // Update ticker system information periodically
      updateTickerSystemInfoWrapper();

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
        if (ROAST_GUIDE->isActive()) {
          ROAST_GUIDE->update(current_temp, current_ror);
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
  updateTickerFooterWrapper();
}

void handleNonBlockingBeeps() {
  // Update safety system beeps (emergency and critical)
  SAFETY->updateBeeps();

  // Handle stage change beeps (kept in main for now)
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
  
  // セオドア提言：メロディの非ブロッキング更新
  updateMelodyWrapper();
}

// 次段階進行に必要な鍵温度を返す（初心者向け明確化）
float getNextStageKeyTemp(RoastGuide::RoastStage stage, RoastGuide::RoastLevel level) {
  switch(stage) {
    case RoastGuide::STAGE_PREHEAT:
      return 150.0f;  // Charge段階へ
    case RoastGuide::STAGE_CHARGE:
      return 0.0f;    // 時間ベース
    case RoastGuide::STAGE_DRYING:
      return 150.0f;  // Maillard殲へ
    case RoastGuide::STAGE_MAILLARD:
      return 180.0f;  // 1st Crack段階への鍵温度
    case RoastGuide::STAGE_FIRST_CRACK:
      return 0.0f;    // 時間ベース
    case RoastGuide::STAGE_DEVELOPMENT:
      // 焙煎レベルに応じた仕上げ温度
      switch(level) {
        case RoastGuide::ROAST_LIGHT: return 200.0f;
        case RoastGuide::ROAST_MEDIUM_LIGHT: return 205.0f;
        case RoastGuide::ROAST_MEDIUM: return 210.0f;
        case RoastGuide::ROAST_MEDIUM_DARK: return 220.0f;
        case RoastGuide::ROAST_DARK: return 225.0f;  // 2nd Crack域へ
        case RoastGuide::ROAST_FRENCH: return 225.0f;
        default: return 210.0f;
      }
    case RoastGuide::STAGE_SECOND_CRACK:
      // 各レベルの最終温度
      switch(level) {
        case RoastGuide::ROAST_DARK: return 250.0f;
        case RoastGuide::ROAST_FRENCH: return 260.0f;
        default: return 235.0f;
      }
    case RoastGuide::STAGE_FINISH:
    default:
      return 0.0f;  // 完了
  }
}

/**
 * セオドア提言：完全非ブロッキングメロディ再生
 */

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

/**
 * ティッカーフッター：システム情報自動収集
 */

/**
 * ティッカーフッター：描画更新
 */
