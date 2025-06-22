#pragma once

#include <Arduino.h>
#include <limits>

/**
 * 温度統計管理クラス
 * 
 * 機能：
 * - 最小/最大/平均温度の追跡
 * - 統計のリセット
 * - バッファからの再計算
 */
class TemperatureStatistics {
private:
    float min_temp;
    float max_temp;
    float sum_temp;
    uint32_t count;
    
    // シングルトン
    static TemperatureStatistics* instance;
    
public:
    TemperatureStatistics();
    ~TemperatureStatistics();
    
    // 初期化
    void begin();
    
    // 温度データ追加
    void addTemperature(float temp);
    
    // 統計値取得
    float getMin() const { return (count > 0) ? min_temp : 0.0f; }
    float getMax() const { return (count > 0) ? max_temp : 0.0f; }
    float getAverage() const { return (count > 0) ? (sum_temp / count) : 0.0f; }
    uint32_t getCount() const { return count; }
    
    // リセット
    void reset();
    
    // バッファから再計算
    void recalculateFromBuffer(const float* buffer, uint16_t buffer_size, uint16_t valid_count);
    
    // シングルトンインスタンス取得
    static TemperatureStatistics* getInstance() {
        if (!instance) {
            instance = new TemperatureStatistics();
        }
        return instance;
    }
};

// 便利なマクロ
#define TEMP_STATS TemperatureStatistics::getInstance()