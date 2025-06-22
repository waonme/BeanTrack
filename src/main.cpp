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
constexpr float    TEMP_MAX    = 280.0f;     // グラフ上限
constexpr int16_t  GRAPH_X0    = 10;
constexpr int16_t  GRAPH_Y0    = 70;
constexpr int16_t  GRAPH_W     = 300;
constexpr int16_t  GRAPH_H     = 160;

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
float ror_buf[BUF_SIZE];
uint16_t ror_count = 0;
constexpr uint16_t ROR_INTERVAL = 60;  // 60 seconds for RoR calculation

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

// Button C long press detection
uint32_t btnC_press_start = 0;
bool btnC_long_press_handled = false;
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
void updateRoRBuffer();
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
void playStageChangeBeep();
void playCriticalWarningBeep();
void updateFirePowerRecommendation();

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
  
  // Draw initial standby screen
  drawStandbyScreen();
  
  next_tick = millis();
}


/**
 * 現在の温度を画面上部に大きく表示
 */
void drawCurrentValue() {
  M5.Lcd.fillRect(0, 30, 320, 30, TFT_BLACK);
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_24);
  M5.Lcd.printf("TEMP: %6.2f C", current_temp);
  
  // 火力推奨インジケーター表示（焙煎ガイド有効時のみ）
  if (roast_guide_active && system_state == STATE_RUNNING) {
    M5.Lcd.setCursor(220, 30);
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
}

/**
 * 現在の buf 内容でグラフを再描画
 */
void drawGraph() {
  if (count == 0) return;

  M5.Lcd.startWrite();
  M5.Lcd.fillRect(GRAPH_X0-1, GRAPH_Y0-1, GRAPH_W+2, GRAPH_H+2, TFT_BLACK);
  M5.Lcd.drawRect(GRAPH_X0-1, GRAPH_Y0-1, GRAPH_W+2, GRAPH_H+2, TFT_WHITE);

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
    
    if (M5.BtnB.wasPressed()) {
      if (display_mode == MODE_GUIDE && !roast_guide_active) {
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
  }
  
  // Handle Button C for start/stop and long press clear
  if (M5.BtnC.isPressed()) {
    if (btnC_press_start == 0) {
      btnC_press_start = millis();
      btnC_long_press_handled = false;
    }
    
    // Check for long press (2 seconds)
    if (!btnC_long_press_handled && (millis() - btnC_press_start) >= LONG_PRESS_DURATION) {
      // Long press: Clear all data
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
      need_full_redraw = true;
      
      // Visual feedback for clear
      M5.Lcd.fillScreen(TFT_BLACK);
      M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_24);
      M5.Lcd.setCursor(80, 120);
      M5.Lcd.println("*** DATA CLEARED ***");
      delay(1000);
      
      if (system_state == STATE_RUNNING) {
        M5.Lcd.fillScreen(TFT_BLACK);
        M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_24);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("Real‑Time Temperature");
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
        M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_24);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("Real‑Time Temperature");
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
  
  // Button instructions
  y_pos += 40;
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_12);
  M5.Lcd.setCursor(10, y_pos);
  M5.Lcd.printf("[A]Mode [B]Reset [C]Stop");
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
  
  // Calculate RoR: temperature change per 60 seconds
  return current_temp_val - old_temp;
}

void updateRoRBuffer() {
  if (count >= ROR_INTERVAL) {
    ror_buf[head] = current_ror;
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
  
  // RoR interpretation
  y_pos += 30;
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
  M5.Lcd.setCursor(20, y_pos);
  if (current_ror > 15) {
    M5.Lcd.setTextColor(TFT_RED);
    M5.Lcd.printf("[!] RoR: Too Fast (>15 C/min)");
  } else if (current_ror > 8) {
    M5.Lcd.setTextColor(TFT_YELLOW);
    M5.Lcd.printf("[^] RoR: Fast (8-15 C/min)");
  } else if (current_ror > 3) {
    M5.Lcd.setTextColor(TFT_GREEN);
    M5.Lcd.printf("[OK] RoR: Good (3-8 C/min)");
  } else if (current_ror > 0) {
    M5.Lcd.setTextColor(TFT_CYAN);
    M5.Lcd.printf("[v] RoR: Slow (0-3 C/min)");
  } else {
    M5.Lcd.setTextColor(TFT_BLUE);
    M5.Lcd.printf("[-] RoR: Cooling (%.1f C/min)", current_ror);
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
  
  // Button instructions
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_12);
  M5.Lcd.setCursor(10, 230);
  M5.Lcd.printf("[A]Mode [B]Reset [C]Stop");
}

RoastTarget getRoastTarget(RoastStage stage, RoastLevel level) {
  // 焙煎プロファイルテーブル
  static const RoastTarget profiles[8][6] = {
    // STAGE_PREHEAT
    {{180, 200, 0, 0, 0, 0, "Warm up roaster to 180-200C", FIRE_HIGH},
     {180, 200, 0, 0, 0, 0, "Warm up roaster to 180-200C", FIRE_HIGH},
     {180, 200, 0, 0, 0, 0, "Warm up roaster to 180-200C", FIRE_HIGH},
     {180, 200, 0, 0, 0, 0, "Warm up roaster to 180-200C", FIRE_HIGH},
     {180, 200, 0, 0, 0, 0, "Warm up roaster to 180-200C", FIRE_HIGH},
     {180, 200, 0, 0, 0, 0, "Warm up roaster to 180-200C", FIRE_HIGH}},
    // STAGE_CHARGE
    {{150, 170, -20, -10, 30, 60, "Add beans! Temperature will drop", FIRE_OFF},
     {150, 170, -20, -10, 30, 60, "Add beans! Temperature will drop", FIRE_OFF},
     {150, 170, -20, -10, 30, 60, "Add beans! Temperature will drop", FIRE_OFF},
     {150, 170, -20, -10, 30, 60, "Add beans! Temperature will drop", FIRE_OFF},
     {150, 170, -20, -10, 30, 60, "Add beans! Temperature will drop", FIRE_OFF},
     {150, 170, -20, -10, 30, 60, "Add beans! Temperature will drop", FIRE_OFF}},
    // STAGE_DRYING
    {{120, 160, 2, 5, 240, 360, "Gentle heat - beans are drying", FIRE_LOW},
     {120, 160, 2, 5, 240, 360, "Gentle heat - beans are drying", FIRE_LOW},
     {120, 160, 2, 5, 240, 360, "Gentle heat - beans are drying", FIRE_LOW},
     {120, 160, 2, 5, 240, 360, "Gentle heat - beans are drying", FIRE_LOW},
     {120, 160, 2, 5, 240, 360, "Gentle heat - beans are drying", FIRE_LOW},
     {120, 160, 2, 5, 240, 360, "Gentle heat - beans are drying", FIRE_LOW}},
    // STAGE_MAILLARD
    {{150, 180, 5, 8, 180, 300, "Browning begins! Keep steady heat", FIRE_MEDIUM},
     {150, 180, 5, 8, 180, 300, "Browning begins! Keep steady heat", FIRE_MEDIUM},
     {150, 180, 5, 8, 180, 300, "Browning begins! Keep steady heat", FIRE_MEDIUM},
     {150, 180, 5, 8, 180, 300, "Browning begins! Keep steady heat", FIRE_MEDIUM},
     {150, 180, 5, 8, 180, 300, "Browning begins! Keep steady heat", FIRE_MEDIUM},
     {150, 180, 5, 8, 180, 300, "Browning begins! Keep steady heat", FIRE_MEDIUM}},
    // STAGE_FIRST_CRACK
    {{185, 205, 1, 3, 60, 120, "Listen for first crack! Reduce heat", FIRE_LOW},
     {188, 208, 1, 3, 60, 120, "Listen for first crack! Reduce heat", FIRE_LOW},
     {190, 210, 1, 3, 60, 120, "Listen for first crack! Reduce heat", FIRE_LOW},
     {193, 213, 1, 3, 60, 120, "Listen for first crack! Reduce heat", FIRE_LOW},
     {195, 215, 1, 3, 60, 120, "Listen for first crack! Reduce heat", FIRE_LOW},
     {198, 218, 1, 3, 60, 120, "Listen for first crack! Reduce heat", FIRE_LOW}},
    // STAGE_DEVELOPMENT
    {{195, 210, 1, 2, 90, 180, "Light roast flavor development", FIRE_VERY_LOW},
     {200, 215, 1, 2, 105, 195, "Medium-light flavor development", FIRE_VERY_LOW},
     {205, 220, 1, 2, 120, 240, "Medium flavor development", FIRE_LOW},
     {210, 230, 1, 2, 150, 300, "Medium-dark flavor development", FIRE_LOW},
     {220, 240, 1, 2, 180, 360, "Dark flavor development", FIRE_MEDIUM},
     {230, 250, 1, 2, 240, 420, "French roast development", FIRE_MEDIUM}},
    // STAGE_SECOND_CRACK
    {{210, 225, 0.5, 1.5, 30, 90, "Light second crack - finish soon", FIRE_VERY_LOW},
     {215, 230, 0.5, 1.5, 45, 105, "Medium-light second crack", FIRE_VERY_LOW},
     {220, 235, 0.5, 1.5, 60, 120, "Medium second crack", FIRE_VERY_LOW},
     {225, 240, 0.5, 1.5, 90, 150, "Medium-dark second crack", FIRE_LOW},
     {230, 250, 0.5, 1.5, 120, 240, "Dark second crack - watch carefully", FIRE_LOW},
     {235, 260, 0.5, 1.5, 180, 300, "French roast - deep second crack", FIRE_MEDIUM}},
    // STAGE_FINISH
    {{200, 240, 0, 0, 0, 0, "Light roast complete! Cool beans", FIRE_OFF},
     {205, 245, 0, 0, 0, 0, "Medium-light complete! Cool beans", FIRE_OFF},
     {210, 250, 0, 0, 0, 0, "Medium roast complete! Cool beans", FIRE_OFF},
     {220, 260, 0, 0, 0, 0, "Medium-dark complete! Cool beans", FIRE_OFF},
     {235, 270, 0, 0, 0, 0, "Dark roast complete! Cool immediately", FIRE_OFF},
     {250, 280, 0, 0, 0, 0, "French roast complete! Cool NOW!", FIRE_OFF}}
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
  
  uint32_t elapsed = getRoastElapsedTime();
  RoastStage prev_stage = current_stage;
  
  // 段階判定ロジック
  switch(current_stage) {
    case STAGE_PREHEAT:
      if (current_temp >= 150 && current_temp <= 200) {
        current_stage = STAGE_CHARGE;
        stage_start_time = millis();
        stage_start_temp = current_temp;
      }
      break;
      
    case STAGE_CHARGE:
      if (elapsed > 60 && current_temp < stage_start_temp) {
        current_stage = STAGE_DRYING;
        stage_start_time = millis();
      }
      break;
      
    case STAGE_DRYING:
      if (elapsed > 240 && current_temp > 150) {
        current_stage = STAGE_MAILLARD;
        stage_start_time = millis();
      }
      break;
      
    case STAGE_MAILLARD:
      if (current_temp > 185 && current_ror < 5) {
        current_stage = STAGE_FIRST_CRACK;
        stage_start_time = millis();
        first_crack_detected = true;
      }
      break;
      
    case STAGE_FIRST_CRACK:
      if (getStageElapsedTime() > 60) {
        current_stage = STAGE_DEVELOPMENT;
        stage_start_time = millis();
      }
      break;
      
    case STAGE_DEVELOPMENT:
      {
        RoastTarget target = getRoastTarget(STAGE_DEVELOPMENT, selected_roast_level);
        if (current_temp >= target.temp_max || 
            (selected_roast_level == ROAST_LIGHT && getStageElapsedTime() > 90) ||
            (selected_roast_level == ROAST_MEDIUM_LIGHT && getStageElapsedTime() > 105) ||
            (selected_roast_level == ROAST_MEDIUM && getStageElapsedTime() > 180) ||
            (selected_roast_level == ROAST_MEDIUM_DARK && getStageElapsedTime() > 210)) {
          if ((selected_roast_level >= ROAST_DARK) && current_temp > 220) {
            current_stage = STAGE_SECOND_CRACK;
            stage_start_time = millis();
            second_crack_detected = true;
          } else {
            current_stage = STAGE_FINISH;
            stage_start_time = millis();
          }
        }
      }
      break;
      
    case STAGE_SECOND_CRACK:
      if (getStageElapsedTime() > 60 || 
          (selected_roast_level == ROAST_LIGHT && current_temp >= 225) ||
          (selected_roast_level == ROAST_MEDIUM_LIGHT && current_temp >= 230) ||
          (selected_roast_level == ROAST_MEDIUM && current_temp >= 235) ||
          (selected_roast_level == ROAST_MEDIUM_DARK && current_temp >= 240) ||
          (selected_roast_level == ROAST_DARK && current_temp >= 250) ||
          (selected_roast_level == ROAST_FRENCH && current_temp >= 260)) {
        current_stage = STAGE_FINISH;
        stage_start_time = millis();
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
  
  int y_pos = GRAPH_Y0 + 20;
  M5.Lcd.setCursor(80, y_pos);
  M5.Lcd.printf(">> Select Roast Level <<");
  
  y_pos += 40;
  for (int i = 0; i < ROAST_COUNT; i++) {
    M5.Lcd.setCursor(40, y_pos);
    if (i == selected_roast_level) {
      M5.Lcd.setTextColor(TFT_YELLOW);
      M5.Lcd.printf("> %s <", getRoastLevelName((RoastLevel)i));
      M5.Lcd.setTextColor(TFT_WHITE);
    } else {
      M5.Lcd.printf("  %s", getRoastLevelName((RoastLevel)i));
    }
    y_pos += 25;
  }
  
  y_pos += 20;
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_12);
  M5.Lcd.setCursor(10, y_pos);
  M5.Lcd.printf("[A]Select [B]Change [C]Start");
}

void drawGuide() {
  M5.Lcd.fillRect(0, GRAPH_Y0, 320, 240 - GRAPH_Y0, TFT_BLACK);
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
  
  int y_pos = GRAPH_Y0 + 10;
  
  // 焙煎レベルと段階表示
  M5.Lcd.setCursor(10, y_pos);
  M5.Lcd.printf("%s - %s", getRoastLevelName(selected_roast_level), getRoastStageName(current_stage));
  
  // 段階プログレスバー
  y_pos += 20;
  int progress_width = 280;
  int progress_x = 20;
  int stage_progress = (current_stage * progress_width) / 7;  // 8段階
  M5.Lcd.drawRect(progress_x, y_pos, progress_width, 8, TFT_WHITE);
  M5.Lcd.fillRect(progress_x + 1, y_pos + 1, stage_progress, 6, TFT_GREEN);
  
  // 段階インジケーター
  for (int i = 0; i < 8; i++) {
    int dot_x = progress_x + (i * progress_width / 7);
    uint16_t color = (i <= current_stage) ? TFT_YELLOW : TFT_DARKGREY;
    M5.Lcd.fillCircle(dot_x, y_pos + 4, 3, color);
  }
  
  y_pos += 35;  // プログレスバー分のスペースを追加
  M5.Lcd.setCursor(10, y_pos);
  M5.Lcd.printf("Time: %02d:%02d", getRoastElapsedTime() / 60, getRoastElapsedTime() % 60);
  
  // 現在の目標値
  RoastTarget target = getRoastTarget(current_stage, selected_roast_level);
  
  y_pos += 25;
  M5.Lcd.setCursor(10, y_pos);
  M5.Lcd.printf("Target: %.0f-%.0f C", target.temp_min, target.temp_max);
  
  y_pos += 20;
  M5.Lcd.setCursor(10, y_pos);
  M5.Lcd.printf("RoR: %.1f-%.1f C/min", target.ror_min, target.ror_max);
  
  // 火力推奨表示
  y_pos += 20;
  M5.Lcd.setCursor(10, y_pos);
  M5.Lcd.setTextColor(TFT_ORANGE);
  M5.Lcd.printf("Fire: %s", getFirePowerName(current_recommended_fire));
  M5.Lcd.setTextColor(TFT_WHITE);
  
  // 現在の状態評価
  y_pos += 25;
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
  
  // アドバイス
  y_pos += 25;
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_12);
  M5.Lcd.setCursor(10, y_pos);
  M5.Lcd.printf("TIP: %s", target.advice);
  
  // 締め切り時闓の警告
  if (current_stage == STAGE_FINISH) {
    y_pos += 25;
    M5.Lcd.setTextColor(TFT_RED);
    M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_16);
    M5.Lcd.setCursor(60, y_pos);
    M5.Lcd.printf("*** DROP BEANS NOW! ***");
    M5.Lcd.setTextColor(TFT_WHITE);
  }
  
  // ボタン指示
  M5.Lcd.setTextFont(&fonts::lgfxJapanGothic_12);
  M5.Lcd.setCursor(10, 230);
  M5.Lcd.printf("[A]Mode [C]Stop");
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
  
  // RoRに基づいて火力を調整
  if (current_ror > target.ror_max + 3) {
    // RoRが高すぎる場合は火力を下げる
    if (base_fire > FIRE_OFF) base_fire = (FirePower)(base_fire - 1);
  } else if (current_ror < target.ror_min - 1) {
    // RoRが低すぎる場合は火力を上げる
    if (base_fire < FIRE_MAX) base_fire = (FirePower)(base_fire + 1);
  }
  
  // 温度に基づいて微調整
  if (current_temp > target.temp_max + 5) {
    // 温度が高すぎる場合は火力を下げる
    if (base_fire > FIRE_OFF) base_fire = (FirePower)(base_fire - 1);
  } else if (current_temp < target.temp_min - 5) {
    // 温度が低すぎる場合は火力を上げる
    if (base_fire < FIRE_MAX) base_fire = (FirePower)(base_fire + 1);
  }
  
  return base_fire;
}

void playBeep(int duration_ms, int frequency) {
  // M5Stackのスピーカーでビープ音を鳴らす
  M5.Speaker.tone(frequency, duration_ms);
}

void playStageChangeBeep() {
  // 段階変更時の3音ビープ
  playBeep(200, 1000);
  delay(100);
  playBeep(200, 1200);
  delay(100);
  playBeep(300, 1500);
}

void playCriticalWarningBeep() {
  // 緊急警告時の断続ビープ
  for (int i = 0; i < 3; i++) {
    playBeep(150, 2000);
    delay(100);
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
  
  // 緊急警告チェック
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
  
  // Create JSON data for Web Bluetooth compatibility
  String json = "{";
  json += "\"timestamp\":" + String(millis()) + ",";
  json += "\"temperature\":" + String(current_temp, 2) + ",";
  json += "\"ror\":" + String(current_ror, 1) + ",";
  json += "\"state\":\"" + String((system_state == STATE_RUNNING) ? "running" : "standby") + "\",";
  json += "\"mode\":\"" + String(display_mode) + "\",";
  json += "\"count\":" + String(count);
  
  if (roast_guide_active) {
    json += ",\"roast\":{";
    json += "\"level\":\"" + String(getRoastLevelName(selected_roast_level)) + "\",";
    json += "\"stage\":\"" + String(getRoastStageName(current_stage)) + "\",";
    json += "\"elapsed\":" + String(getRoastElapsedTime()) + ",";
    json += "\"fire\":\"" + String(getFirePowerName(current_recommended_fire)) + "\"";
    json += "}";
  }
  
  if (count > 0) {
    json += ",\"stats\":{";
    json += "\"min\":" + String(temp_min_recorded, 2) + ",";
    json += "\"max\":" + String(temp_max_recorded, 2) + ",";
    json += "\"avg\":" + String(getAverageTemp(), 2);
    json += "}";
  }
  
  json += "}\n";
  
  pTxCharacteristic->setValue(json.c_str());
  pTxCharacteristic->notify();
}

void loop() {
  M5.update();
  handleButtons();

  // Handle BLE disconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
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

      // Calculate and update RoR
      current_ror = calculateRoR();
      updateRoRBuffer();

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
