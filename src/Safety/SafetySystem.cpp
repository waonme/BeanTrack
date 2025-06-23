#include "SafetySystem.h"

// シングルトンインスタンス
SafetySystem* SafetySystem::instance = nullptr;

SafetySystem::SafetySystem() {
    // コンストラクタ
}

SafetySystem::~SafetySystem() {
    // デストラクタ
}

void SafetySystem::begin() {
    emergency_active = false;
    emergency_beep_count = 0;
    auto_recovery_available = false;
    recovery_dialog_active = false;
    critical_beep_active = false;
    critical_beep_count = 0;
}

void SafetySystem::checkEmergencyConditions(float current_temp, float current_ror, 
                                            int current_stage, bool& roast_guide_active) {
    // 緊急停止チェック
    if (current_temp >= current_critical_temp && !emergency_active) {
        // 緊急停止発動
        emergency_active = true;
        emergency_beep_start = millis();
        emergency_beep_count = 0;
        roast_guide_active = false;
        
        // コールバック実行
        if (on_emergency) {
            on_emergency();
        }
    }

    // 自動復旧可能性チェック
    if (emergency_active && !auto_recovery_available) {
        // 複数の安全条件を確認
        bool temp_safe = current_temp < (current_danger_temp - 10.0f);
        bool cooling_active = current_ror < 0;
        bool temp_stable = abs(current_ror) < 5.0f;
        bool sufficient_cooldown = current_temp < (current_danger_temp - 15.0f);

        if (temp_safe && cooling_active && sufficient_cooldown) {
            auto_recovery_available = true;
            recovery_dialog_start = millis();
            recovery_dialog_active = true;
        }
    }

    // 復旧ダイアログの30秒タイムアウト
    if (recovery_dialog_active && (millis() - recovery_dialog_start > 30000)) {
        auto_recovery_available = false;
        recovery_dialog_active = false;
    }

    // 温度が再上昇したら復旧オプションを無効化
    if (auto_recovery_available && current_temp >= current_danger_temp) {
        auto_recovery_available = false;
        recovery_dialog_active = false;
    }
}

bool SafetySystem::executeAutoRecovery() {
    if (!auto_recovery_available || !emergency_active) {
        return false;
    }

    // 緊急状態をリセット
    resetEmergency();
    
    // コールバック実行
    if (on_recovery) {
        on_recovery();
    }

    return true;
}

void SafetySystem::playEmergencyAlert() {
    if (!emergency_active || emergency_beep_count >= MAX_EMERGENCY_BEEPS) {
        return;
    }

    emergency_beep_start = millis();
    emergency_beep_count = 0;
}

void SafetySystem::playCriticalWarning() {
    critical_beep_active = true;
    critical_beep_start = millis();
    critical_beep_count = 0;
}

void SafetySystem::updateBeeps() {
    uint32_t now = millis();

    // 緊急ビープ処理
    if (emergency_active && emergency_beep_count < MAX_EMERGENCY_BEEPS) {
        uint32_t elapsed = now - emergency_beep_start;
        int beep_phase = elapsed / EMERGENCY_BEEP_INTERVAL;
        
        if (beep_phase > emergency_beep_count * 2) {
            if (beep_phase % 2 == 0 && beep_func) {
                // ビープON
                beep_func(EMERGENCY_BEEP_INTERVAL - 50, 3000);
            }
            emergency_beep_count = beep_phase / 2;
        }
    }

    // 臨界警告ビープ処理
    if (critical_beep_active && critical_beep_count < MAX_CRITICAL_BEEPS) {
        uint32_t elapsed = now - critical_beep_start;
        int beep_phase = elapsed / CRITICAL_BEEP_INTERVAL;
        
        if (beep_phase > critical_beep_count * 2) {
            if (beep_phase % 2 == 0 && beep_func) {
                // ビープON
                beep_func(CRITICAL_BEEP_INTERVAL - 50, 2000);
            }
            critical_beep_count = beep_phase / 2;
            
            if (critical_beep_count >= MAX_CRITICAL_BEEPS) {
                critical_beep_active = false;
            }
        }
    }
}

void SafetySystem::drawRecoveryDialog(float current_temp, float current_ror) {
    if (!recovery_dialog_active) return;

    // 詳細な復旧ダイアログ描画
    M5.Lcd.fillRect(25, 105, 270, 125, TFT_DARKGREEN);
    M5.Lcd.drawRect(25, 105, 270, 125, TFT_GREEN);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_DARKGREEN);
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
    
    M5.Lcd.setCursor(35, 115);
    M5.Lcd.printf("INTELLIGENT RECOVERY READY:");
    
    M5.Lcd.setCursor(35, 130);
    M5.Lcd.printf("Temp: %.1f°C (Safe: <%.0f°C)", current_temp, current_danger_temp - 10);
    
    M5.Lcd.setCursor(35, 145);
    M5.Lcd.printf("RoR: %.1f°C/min (Cooling)", current_ror);
    
    M5.Lcd.setCursor(35, 160);
    M5.Lcd.printf("%s", (abs(current_ror) < 5.0f) ? "Temperature Stable" : "Cool Down Active");
    
    M5.Lcd.setCursor(35, 180);
    M5.Lcd.printf("System Ready for Safe Recovery");
    
    M5.Lcd.setCursor(35, 205);
    M5.Lcd.printf("[A] Auto Reset [C] Manual Control");
    
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);

    // 30秒タイムアウト警告
    uint32_t remaining = 30 - ((millis() - recovery_dialog_start) / 1000);
    if (remaining < 10) {
        M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
        M5.Lcd.setCursor(100, 90);
        M5.Lcd.printf("Timeout in %d sec", remaining);
        M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    }
}

void SafetySystem::resetEmergency() {
    emergency_active = false;
    emergency_beep_count = MAX_EMERGENCY_BEEPS;
    auto_recovery_available = false;
    recovery_dialog_active = false;
    critical_beep_active = false;
    critical_beep_count = MAX_CRITICAL_BEEPS;
}