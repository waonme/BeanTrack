#pragma once

#include <Arduino.h>
#include <M5Unified.h>

/**
 * コーヒー焙煎安全管理システム
 * 
 * 機能：
 * - 温度監視と緊急停止
 * - 自動復旧システム
 * - 非ブロッキング警報
 * - 安全ダイアログ表示
 */
class SafetySystem {
public:
    // 安全システムの状態
    struct SafetyState {
        bool emergency_active;
        bool auto_recovery_available;
        bool recovery_dialog_active;
        float current_danger_temp;
        float current_critical_temp;
    };

    // コールバック関数型定義
    typedef void (*EmergencyCallback)();
    typedef void (*RecoveryCallback)();
    typedef void (*BeepCallback)(int duration, int frequency);

private:
    // 緊急警報システム
    bool emergency_active = false;
    uint32_t emergency_beep_start = 0;
    int emergency_beep_count = 0;
    static constexpr int MAX_EMERGENCY_BEEPS = 10;
    static constexpr uint32_t EMERGENCY_BEEP_INTERVAL = 300;

    // 自動復旧システム
    bool auto_recovery_available = false;
    uint32_t recovery_dialog_start = 0;
    bool recovery_dialog_active = false;

    // 臨界警告システム
    bool critical_beep_active = false;
    uint32_t critical_beep_start = 0;
    int critical_beep_count = 0;
    static constexpr int MAX_CRITICAL_BEEPS = 3;
    static constexpr uint32_t CRITICAL_BEEP_INTERVAL = 250;

    // 現在の閾値
    float current_danger_temp = 245.0f;
    float current_critical_temp = 260.0f;

    // コールバック
    EmergencyCallback on_emergency = nullptr;
    RecoveryCallback on_recovery = nullptr;
    BeepCallback beep_func = nullptr;

    // シングルトン
    static SafetySystem* instance;

public:
    SafetySystem();
    ~SafetySystem();

    // 初期化
    void begin();

    // コールバック設定
    void setEmergencyCallback(EmergencyCallback cb) { on_emergency = cb; }
    void setRecoveryCallback(RecoveryCallback cb) { on_recovery = cb; }
    void setBeepCallback(BeepCallback cb) { beep_func = cb; }

    // 閾値設定
    void setDangerTemp(float temp) { current_danger_temp = temp; }
    void setCriticalTemp(float temp) { current_critical_temp = temp; }

    // 状態取得
    SafetyState getState() const {
        return {
            emergency_active,
            auto_recovery_available,
            recovery_dialog_active,
            current_danger_temp,
            current_critical_temp
        };
    }

    // 緊急状態確認と更新
    void checkEmergencyConditions(float current_temp, float current_ror, 
                                  int current_stage, bool& roast_guide_active);

    // 自動復旧実行
    bool executeAutoRecovery();

    // 復旧ダイアログ表示
    void showRecoveryDialog(bool show) { recovery_dialog_active = show; }

    // 緊急警報再生
    void playEmergencyAlert();

    // 臨界警告再生
    void playCriticalWarning();

    // 非ブロッキングビープ更新
    void updateBeeps();

    // 復旧ダイアログ描画
    void drawRecoveryDialog(float current_temp, float current_ror);

    // 緊急状態リセット
    void resetEmergency();

    // シングルトンインスタンス取得
    static SafetySystem* getInstance() {
        if (!instance) {
            instance = new SafetySystem();
        }
        return instance;
    }
};

// 便利なマクロ
#define SAFETY SafetySystem::getInstance()