#include "RoastGuide.h"
#include <Arduino.h>

// シングルトンインスタンス
RoastGuide* RoastGuide::instance = nullptr;

// 定数定義
constexpr float DANGER_TEMPS[RoastGuide::ROAST_COUNT] = {
  220.0f,  // ROAST_LIGHT
  230.0f,  // ROAST_MEDIUM_LIGHT
  240.0f,  // ROAST_MEDIUM
  250.0f,  // ROAST_MEDIUM_DARK
  270.0f,  // ROAST_DARK（深煎り用）
  270.0f   // ROAST_FRENCH（深煎り用）
};

constexpr float CRITICAL_TEMPS[RoastGuide::ROAST_COUNT] = {
  230.0f,  // ROAST_LIGHT
  240.0f,  // ROAST_MEDIUM_LIGHT
  250.0f,  // ROAST_MEDIUM
  260.0f,  // ROAST_MEDIUM_DARK
  280.0f,  // ROAST_DARK（深煎り用）
  280.0f   // ROAST_FRENCH（深煎り用）
};

// コンストラクタ
RoastGuide::RoastGuide() {
    active = false;
    selected_level = ROAST_MEDIUM;
    current_stage = STAGE_PREHEAT;
    stage_start_time = 0;
    roast_start_time = 0;
    last_stall_check = 0;
    stall_detected = false;
    stall_start_time = 0;
    stall_temp = 0;
    first_crack_detected = false;
    first_crack_confirmation_needed = false;
    first_crack_time = 0;
    adherence_score = 100.0f;
    off_target_count = 0;
}

// デストラクタ
RoastGuide::~RoastGuide() {
}

// 初期化
void RoastGuide::begin() {
    // 必要に応じて初期化処理を追加
}

// ガイド開始
void RoastGuide::start(RoastLevel level) {
    active = true;
    selected_level = level;
    current_stage = STAGE_PREHEAT;
    roast_start_time = millis();
    stage_start_time = millis();
    stall_detected = false;
    first_crack_detected = false;
    first_crack_confirmation_needed = false;
    adherence_score = 100.0f;
    off_target_count = 0;
}

// ガイド停止
void RoastGuide::stop() {
    active = false;
    current_stage = STAGE_PREHEAT;
}

// 状態更新
void RoastGuide::update(float current_temp, float current_ror) {
    if (!active) return;
    
    // ストール検出
    checkStallCondition(current_temp, current_ror);
    
    // ステージ進行更新
    updateStageProgression(current_temp, current_ror);
    
    // 遵守度評価
    evaluateAdherence(current_temp, current_ror);
}

// ストール検出
void RoastGuide::checkStallCondition(float current_temp, float current_ror) {
    uint32_t now = millis();
    if (!active || (now - last_stall_check) < 5000) {
        return; // 5秒ごとにチェック
    }
    
    last_stall_check = now;
    
    // ストール条件：RoR < 1°C/minが60秒以上継続
    if (current_ror < 1.0f && ((now - stage_start_time) / 1000) > 60) {
        if (!stall_detected) {
            stall_detected = true;
            stall_start_time = now;
            stall_temp = current_temp;
        }
    } else {
        stall_detected = false;
    }
}

// 1ハゼ確認
void RoastGuide::confirmFirstCrack() {
    if (first_crack_confirmation_needed) {
        first_crack_detected = true;
        first_crack_confirmation_needed = false;
        first_crack_time = millis();
    }
}

// 焙煎ターゲット取得
RoastGuide::RoastTarget RoastGuide::getRoastTarget(RoastStage stage, RoastLevel level) const {
    // PROGMEM最適化で12KB RAM削減
    static const RoastTarget PROGMEM profiles[8][6] = {
        // STAGE_PREHEAT - 投入前予熱（180-200°C空焼き）
        {{180, 200, 0, 0, 0, 0, FIRE_HIGH, "Preheat roaster to 200C for light roast"},
         {180, 200, 0, 0, 0, 0, FIRE_HIGH, "Preheat roaster to 190-200C"},
         {180, 200, 0, 0, 0, 0, FIRE_HIGH, "Preheat roaster to 190C"},
         {180, 200, 0, 0, 0, 0, FIRE_HIGH, "Preheat roaster to 180-190C"},
         {180, 200, 0, 0, 0, 0, FIRE_HIGH, "Preheat roaster to 180C"},
         {180, 200, 0, 0, 0, 0, FIRE_HIGH, "Preheat roaster to 180C"}},
        
        // STAGE_CHARGE - 投入～転換点（1.5-2分、RoRピーク18-20→下降開始）
        {{80, 120, 15, 20, 90, 120, FIRE_MEDIUM, "Charge at 200C! RoR peak 18-20 C/min"},
         {80, 120, 15, 18, 90, 120, FIRE_MEDIUM, "Charge at 190-200C! RoR peak ~18 C/min"},
         {80, 120, 12, 15, 105, 135, FIRE_MEDIUM, "Charge at 190C! RoR peak ~15 C/min"},
         {80, 120, 12, 15, 105, 150, FIRE_MEDIUM, "Charge at 180-190C! RoR peak 12-15 C/min"},
         {80, 120, 10, 12, 120, 150, FIRE_MEDIUM, "Charge at 180C! RoR peak ~12 C/min"},
         {80, 120, 10, 12, 120, 180, FIRE_MEDIUM, "Charge at 180C! RoR controlled start"}},
        
        // STAGE_DRYING - 乾燥フェーズ（～150°C、240-360秒、RoR 10-15→下降）
        {{120, 150, 10, 15, 240, 300, FIRE_MEDIUM, "Drying phase - high RoR 10-15 C/min"},
         {120, 150, 10, 12, 240, 330, FIRE_MEDIUM, "Drying phase - RoR 10-12 C/min"},
         {120, 150, 8, 10, 270, 360, FIRE_MEDIUM, "Drying phase - RoR ~10 C/min"},
         {120, 150, 8, 10, 270, 390, FIRE_MEDIUM, "Drying phase - RoR 8-10 C/min"},
         {120, 150, 6, 8, 300, 420, FIRE_LOW, "Drying phase - RoR ~8 C/min"},
         {120, 150, 6, 8, 300, 450, FIRE_LOW, "Drying phase - RoR <8 C/min stable"}},
        
        // STAGE_MAILLARD - メイラード反応（150-195°C、180-300秒、RoR 5-8→下降）
        {{150, 195, 5, 8, 180, 240, FIRE_LOW, "Maillard reaction - RoR 5-8 C/min"},
         {150, 195, 5, 8, 180, 270, FIRE_LOW, "Maillard reaction - RoR 5-8 C/min"},
         {150, 200, 4, 5, 210, 300, FIRE_LOW, "Maillard reaction - RoR ~5 C/min"},
         {150, 200, 3, 5, 240, 330, FIRE_LOW, "Maillard reaction - RoR 3-5 C/min"},
         {150, 200, 2, 3, 270, 360, FIRE_LOW, "Maillard reaction - RoR ~3 C/min"},
         {150, 200, 1, 3, 300, 420, FIRE_VERY_LOW, "Maillard reaction - RoR <3 C/min"}},
        
        // STAGE_FIRST_CRACK - 1ハゼ（195-200°C、60-120秒、RoR急降下→谷）
        {{190, 200, 2, 5, 60, 90, FIRE_VERY_LOW, "1st crack! RoR crash then stabilize"},
         {195, 200, 2, 5, 60, 90, FIRE_VERY_LOW, "1st crack! Manage RoR crash"},
         {195, 205, 2, 4, 60, 90, FIRE_VERY_LOW, "1st crack! RoR valley control"},
         {195, 205, 1, 4, 60, 120, FIRE_VERY_LOW, "1st crack! RoR descent control"},
         {195, 205, 1, 3, 60, 120, FIRE_VERY_LOW, "1st crack! Low RoR maintenance"},
         {195, 205, 1, 3, 60, 120, FIRE_VERY_LOW, "1st crack! Gentle RoR control"}},
        
        // STAGE_DEVELOPMENT - 発達段階（DTR 15-25%、RoR 0-5→0へ）
        {{195, 205, 2, 5, 60, 120, FIRE_VERY_LOW, "Light dev: 1-2min, RoR 5→0 C/min"},
         {200, 210, 2, 5, 90, 150, FIRE_VERY_LOW, "Med-light dev: 1.5-2min, RoR <5 C/min"},
         {205, 218, 1, 3, 120, 180, FIRE_VERY_LOW, "Medium dev: 2-3min, RoR 3-5→0 C/min"},
         {210, 225, 1, 3, 180, 210, FIRE_VERY_LOW, "Med-dark dev: ~3min, RoR ≤3 C/min"},
         {220, 235, 1, 2, 180, 240, FIRE_OFF, "Dark dev: 3-4min, RoR 2-3→0 C/min"},
         {230, 245, 0, 1, 240, 300, FIRE_OFF, "French dev: 4min+, RoR ≈0 C/min"}},
        
        // STAGE_SECOND_CRACK - 2ハゼ（深煎りのみ、RoR≈0維持）
        {{225, 235, 0, 1, 30, 90, FIRE_OFF, "Light 2nd crack - finish soon!"},
         {225, 240, 0, 1, 45, 105, FIRE_OFF, "Med-light 2nd crack"},
         {230, 245, 0, 1, 60, 120, FIRE_OFF, "Medium 2nd crack"},
         {230, 250, 0, 1, 90, 150, FIRE_OFF, "Med-dark rolling 2nd crack"},
         {235, 255, 0, 1, 120, 180, FIRE_OFF, "Dark rolling 2nd crack - watch oil"},
         {240, 260, 0, 1, 90, 150, FIRE_OFF, "French intense 2nd crack - risk!"}},
        
        // STAGE_FINISH - 排出（各レベルの最終温度）
        {{200, 205, 0, 0, 0, 0, FIRE_OFF, "Light roast complete at 205C"},
         {205, 210, 0, 0, 0, 0, FIRE_OFF, "Med-light complete at 210C"},
         {210, 218, 0, 0, 0, 0, FIRE_OFF, "Medium complete at 218C"},
         {220, 225, 0, 0, 0, 0, FIRE_OFF, "Med-dark complete at 225C"},
         {230, 235, 0, 0, 0, 0, FIRE_OFF, "Dark complete at 235C"},
         {240, 250, 0, 0, 0, 0, FIRE_OFF, "French complete at 245C"}}
    };
    
    // PROGMEMから読み取り
    RoastTarget target;
    memcpy_P(&target, &profiles[stage][level], sizeof(RoastTarget));
    return target;
}

// 焙煎レベル名取得
const char* RoastGuide::getRoastLevelName(RoastLevel level) const {
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

// 危険温度取得
float RoastGuide::getDangerTemp(RoastLevel level) const {
    return DANGER_TEMPS[level];
}

// 臨界温度取得
float RoastGuide::getCriticalTemp(RoastLevel level) const {
    return CRITICAL_TEMPS[level];
}

// レベル変更
void RoastGuide::cycleRoastLevel() {
    selected_level = (RoastLevel)((selected_level + 1) % ROAST_COUNT);
}

// ステージ名取得
const char* RoastGuide::getStageName(RoastStage stage) const {
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

// 火力名取得
const char* RoastGuide::getFirePowerName(FirePower power) const {
    switch(power) {
        case FIRE_OFF: return "OFF";
        case FIRE_VERY_LOW: return "極弱火";
        case FIRE_LOW: return "弱火";
        case FIRE_MEDIUM: return "中火";
        case FIRE_HIGH: return "強火";
        case FIRE_VERY_HIGH: return "最大火力";
        default: return "不明";
    }
}

// ステージカラー取得
uint32_t RoastGuide::getStageColor(RoastStage stage) const {
    switch(stage) {
        case STAGE_PREHEAT: return 0x666666;  // グレー
        case STAGE_CHARGE: return 0x00FF00;   // 緑
        case STAGE_DRYING: return 0xFFFF00;   // 黄
        case STAGE_MAILLARD: return 0xFFA500; // オレンジ
        case STAGE_FIRST_CRACK: return 0xFF0000; // 赤
        case STAGE_DEVELOPMENT: return 0x8B4513; // 茶
        case STAGE_SECOND_CRACK: return 0x4B0082; // インディゴ
        case STAGE_FINISH: return 0x000000;   // 黒
        default: return 0xFFFFFF;
    }
}

// ステージ進行更新
void RoastGuide::updateStageProgression(float current_temp, float current_ror) {
    if (!active) return;
    
    uint32_t now = millis();
    uint32_t total_elapsed = (now - roast_start_time) / 1000;
    float stage_elapsed = (now - stage_start_time) / 1000.0f;
    RoastStage prev_stage = current_stage;
    RoastTarget current_target = getRoastTarget(current_stage, selected_level);
    
    // Scott Rao氏のガイドラインに基づく段階判定ロジック
    switch(current_stage) {
        case STAGE_PREHEAT:
            // 予熱完了：温度条件またはユーザー操作で投入段階へ
            if ((current_temp >= 180 && current_temp <= 200) || stage_elapsed > 300) {
                current_stage = STAGE_CHARGE;
                stage_start_time = now;
            }
            break;
            
        case STAGE_CHARGE:
            {
                // 転換点検出または時間経過で次段階へ
                bool min_time_met = (stage_elapsed > 90);  // 最低1.5分は投入段階維持
                bool backup_condition = (stage_elapsed > 120 && current_ror > 8);  // 予備条件
                
                if (min_time_met && backup_condition) {
                    current_stage = STAGE_DRYING;
                    stage_start_time = now;
                }
                // 安全措置：3分経過で強制移行
                else if (stage_elapsed > 180) {
                    current_stage = STAGE_DRYING;
                    stage_start_time = now;
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
                    stage_start_time = now;
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
                    if (!first_crack_detected && current_temp >= 195) {
                        first_crack_confirmation_needed = true;
                    }
                    
                    // 自動移行または手動確認
                    if (first_crack_detected || (current_temp >= 200 && current_ror <= 5)) {
                        current_stage = STAGE_FIRST_CRACK;
                        stage_start_time = now;
                        first_crack_detected = true;
                        first_crack_time = now;
                    }
                }
                
                // 最大時間超過で強制移行
                if (stage_elapsed > current_target.time_max) {
                    current_stage = STAGE_FIRST_CRACK;
                    stage_start_time = now;
                }
            }
            break;
            
        case STAGE_FIRST_CRACK:
            {
                // 1ハゼ終了判定：RoR谷を超えて回復開始
                bool min_time_met = (stage_elapsed >= current_target.time_min);
                bool max_time_reached = (stage_elapsed >= 120);
                
                if (min_time_met || max_time_reached) {
                    current_stage = STAGE_DEVELOPMENT;
                    stage_start_time = now;
                }
            }
            break;
            
        case STAGE_DEVELOPMENT:
            {
                RoastTarget dev_target = getRoastTarget(STAGE_DEVELOPMENT, selected_level);
                
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
                    if (selected_level >= ROAST_MEDIUM_DARK && current_temp >= 220) {
                        current_stage = STAGE_SECOND_CRACK;
                        stage_start_time = now;
                    } else {
                        current_stage = STAGE_FINISH;
                        stage_start_time = now;
                    }
                }
                
                // 緊急停止条件
                if (current_temp >= getCriticalTemp(selected_level)) {
                    current_stage = STAGE_FINISH;
                    stage_start_time = now;
                }
            }
            break;
            
        case STAGE_SECOND_CRACK:
            {
                RoastTarget sc_target = getRoastTarget(STAGE_SECOND_CRACK, selected_level);
                
                // 2ハゼ：時間と温度の両方で判定
                bool min_time_met = (stage_elapsed >= sc_target.time_min);
                bool temp_target_reached = (current_temp >= sc_target.temp_max);
                bool critical_temp_reached = (current_temp >= getCriticalTemp(selected_level));
                
                if ((min_time_met && temp_target_reached) || critical_temp_reached) {
                    current_stage = STAGE_FINISH;
                    stage_start_time = now;
                }
            }
            break;
            
        case STAGE_FINISH:
            // 排出段階、手動でリセット
            break;
    }
}

// 遵守度評価
void RoastGuide::evaluateAdherence(float current_temp, float current_ror) {
    if (!active || current_stage == STAGE_PREHEAT || current_stage == STAGE_FINISH) return;
    
    RoastTarget target = getRoastTarget(current_stage, selected_level);
    
    // 温度とRoRの目標範囲からの逸脱をチェック
    bool temp_off = (current_temp < target.temp_min || current_temp > target.temp_max);
    bool ror_off = (current_ror < target.ror_min || current_ror > target.ror_max);
    
    if (temp_off || ror_off) {
        off_target_count++;
        // 逸脱が続くとスコアを減点
        if (off_target_count > 5) {
            adherence_score -= 0.1f;
            if (adherence_score < 0) adherence_score = 0;
        }
    } else {
        off_target_count = 0;
        // 目標範囲内ならスコアを少しずつ回復
        adherence_score += 0.05f;
        if (adherence_score > 100) adherence_score = 100;
    }
}

// メイン描画関数
void RoastGuide::draw(float current_temp, float current_ror, 
                      int graph_x0, int graph_y0, int graph_w, int graph_h) {
    if (!active) return;
    
    // ステージインジケーター
    drawStageIndicator(graph_x0, graph_y0, graph_w, 30);
    
    // ターゲット情報
    drawTargetInfo(current_temp, current_ror);
    
    // 火力推奨
    drawFirePowerRecommendation();
}

// ステージインジケーター描画
void RoastGuide::drawStageIndicator(int x, int y, int width, int height) {
    // プログレスバー背景
    M5.Lcd.fillRect(x, y, width, height, TFT_BLACK);
    M5.Lcd.drawRect(x, y, width, height, TFT_WHITE);
    
    // 各ステージのセグメント
    int segment_width = width / 8;
    for (int i = 0; i < 8; i++) {
        int seg_x = x + i * segment_width;
        uint32_t color = (i <= current_stage) ? getStageColor((RoastStage)i) : TFT_DARKGREY;
        M5.Lcd.fillRect(seg_x + 1, y + 1, segment_width - 2, height - 2, color);
        
        // ステージ区切り線
        if (i > 0) {
            M5.Lcd.drawLine(seg_x, y, seg_x, y + height, TFT_WHITE);
        }
    }
    
    // 現在のステージ名
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
    M5.Lcd.setCursor(x + 5, y + height + 5);
    M5.Lcd.printf("%s - %s", getRoastLevelName(selected_level), getStageName(current_stage));
}

// ターゲット情報描画
void RoastGuide::drawTargetInfo(float current_temp, float current_ror) {
    RoastTarget target = getRoastTarget(current_stage, selected_level);
    
    int y_pos = 100;
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
    
    // 温度ターゲット
    M5.Lcd.setCursor(10, y_pos);
    M5.Lcd.setTextColor(TFT_CYAN);
    M5.Lcd.printf("Target: %.0f-%.0f°C", target.temp_min, target.temp_max);
    
    // 現在の温度評価
    if (current_temp >= target.temp_min && current_temp <= target.temp_max) {
        M5.Lcd.setTextColor(TFT_GREEN);
        M5.Lcd.printf(" [OK]");
    } else {
        M5.Lcd.setTextColor(TFT_RED);
        M5.Lcd.printf(" [OFF]");
    }
    M5.Lcd.setTextColor(TFT_WHITE);
    
    // RoRターゲット
    y_pos += 20;
    M5.Lcd.setCursor(10, y_pos);
    M5.Lcd.printf("RoR: %.1f-%.1f°C/min", target.ror_min, target.ror_max);
    
    // ティップス
    y_pos += 20;
    M5.Lcd.setCursor(10, y_pos);
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
    M5.Lcd.printf("%s", target.tips);
}

// 火力推奨描画
void RoastGuide::drawFirePowerRecommendation() {
    RoastTarget target = getRoastTarget(current_stage, selected_level);
    
    int y_pos = 180;
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_16);
    M5.Lcd.setCursor(10, y_pos);
    M5.Lcd.setTextColor(TFT_ORANGE);
    M5.Lcd.printf("Fire: %s", getFirePowerName(target.fire));
    M5.Lcd.setTextColor(TFT_WHITE);
    
    // ストール警告
    if (stall_detected) {
        y_pos += 25;
        M5.Lcd.setCursor(10, y_pos);
        M5.Lcd.setTextColor(TFT_RED);
        M5.Lcd.printf("!!! STALL DETECTED !!!");
        M5.Lcd.setTextColor(TFT_WHITE);
    }
    
    // 1ハゼ確認プロンプト
    if (first_crack_confirmation_needed) {
        y_pos += 25;
        M5.Lcd.setCursor(10, y_pos);
        M5.Lcd.setTextColor(TFT_YELLOW);
        M5.Lcd.printf("1st Crack? Press [B] to confirm");
        M5.Lcd.setTextColor(TFT_WHITE);
    }
}