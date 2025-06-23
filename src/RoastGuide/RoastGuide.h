#pragma once

#include <Arduino.h>
#include <M5Unified.h>

/**
 * 焙煎ガイドシステム
 * 
 * 機能：
 * - 焙煎レベル別の温度・RoRプロファイル管理
 * - ステージ別ターゲット値提供
 * - ストール（停滞）検出
 * - 焙煎ガイド画面描画
 * - プロファイル遵守度評価
 */
class RoastGuide {
public:
    // 焙煎レベル
    enum RoastLevel {
        ROAST_LIGHT = 0,        // 浅煎り
        ROAST_MEDIUM_LIGHT = 1, // 中浅煎り
        ROAST_MEDIUM = 2,       // 中煎り
        ROAST_MEDIUM_DARK = 3,  // 中深煎り
        ROAST_DARK = 4,         // 深煎り
        ROAST_FRENCH = 5,       // フレンチロースト
        ROAST_COUNT = 6
    };

    // 焙煎ステージ
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

    // 火力レベル
    enum FirePower {
        FIRE_OFF = 0,      // 火力OFF
        FIRE_VERY_LOW = 1, // 極弱火
        FIRE_LOW = 2,      // 弱火
        FIRE_MEDIUM = 3,   // 中火
        FIRE_HIGH = 4,     // 強火
        FIRE_VERY_HIGH = 5 // 極強火
    };

    // 焙煎ターゲット
    struct RoastTarget {
        float temp_min;
        float temp_max;
        float ror_min;
        float ror_max;
        uint16_t time_min;  // 秒
        uint16_t time_max;  // 秒
        FirePower fire;
        const char* tips;
    };

private:
    // 状態管理
    bool active = false;
    RoastLevel selected_level = ROAST_MEDIUM;
    RoastStage current_stage = STAGE_PREHEAT;
    uint32_t stage_start_time = 0;
    uint32_t roast_start_time = 0;
    
    // ストール検出
    uint32_t last_stall_check = 0;
    bool stall_detected = false;
    uint32_t stall_start_time = 0;
    float stall_temp = 0;
    
    // 1ハゼ確認
    bool first_crack_detected = false;
    bool first_crack_confirmation_needed = false;
    uint32_t first_crack_time = 0;
    
    // 評価スコア
    float adherence_score = 100.0f;
    int off_target_count = 0;
    
    // シングルトン
    static RoastGuide* instance;
    
    // プライベートメソッド
    void updateStageProgression(float current_temp, float current_ror);
    void evaluateAdherence(float current_temp, float current_ror);
    const char* getStageName(RoastStage stage) const;
    const char* getFirePowerName(FirePower power) const;
    uint32_t getStageColor(RoastStage stage) const;

public:
    RoastGuide();
    ~RoastGuide();
    
    // 初期化
    void begin();
    
    // ガイド制御
    void start(RoastLevel level);
    void stop();
    bool isActive() const { return active; }
    
    // 状態更新
    void update(float current_temp, float current_ror);
    
    // ストール検出
    void checkStallCondition(float current_temp, float current_ror);
    bool isStalled() const { return stall_detected; }
    
    // 1ハゼ確認
    void confirmFirstCrack();
    bool isFirstCrackConfirmationNeeded() const { return first_crack_confirmation_needed; }
    
    // 情報取得
    RoastLevel getSelectedLevel() const { return selected_level; }
    RoastStage getCurrentStage() const { return current_stage; }
    RoastTarget getRoastTarget(RoastStage stage, RoastLevel level) const;
    const char* getRoastLevelName(RoastLevel level) const;
    float getDangerTemp(RoastLevel level) const;
    float getCriticalTemp(RoastLevel level) const;
    float getAdherenceScore() const { return adherence_score; }
    
    // レベル変更
    void cycleRoastLevel();
    
    // 描画
    void draw(float current_temp, float current_ror, 
              int graph_x0, int graph_y0, int graph_w, int graph_h);
    void drawStageIndicator(int x, int y, int width, int height);
    void drawTargetInfo(float current_temp, float current_ror);
    void drawFirePowerRecommendation();
    
    // シングルトンインスタンス取得
    static RoastGuide* getInstance() {
        if (!instance) {
            instance = new RoastGuide();
        }
        return instance;
    }
};

// 便利なマクロ
#define ROAST_GUIDE RoastGuide::getInstance()