#include "TemperatureStatistics.h"

// シングルトンインスタンス
TemperatureStatistics* TemperatureStatistics::instance = nullptr;

TemperatureStatistics::TemperatureStatistics() {
    reset();
}

TemperatureStatistics::~TemperatureStatistics() {
    // デストラクタ
}

void TemperatureStatistics::begin() {
    reset();
}

void TemperatureStatistics::addTemperature(float temp) {
    if (temp < min_temp) min_temp = temp;
    if (temp > max_temp) max_temp = temp;
    sum_temp += temp;
    count++;
}

void TemperatureStatistics::reset() {
    min_temp = std::numeric_limits<float>::infinity();
    max_temp = -std::numeric_limits<float>::infinity();
    sum_temp = 0.0f;
    count = 0;
}

void TemperatureStatistics::recalculateFromBuffer(const float* buffer, uint16_t buffer_size, uint16_t valid_count) {
    reset();
    
    if (valid_count == 0 || buffer == nullptr) return;
    
    // バッファから統計を再計算
    for (uint16_t i = 0; i < valid_count; i++) {
        float temp = buffer[i];
        if (temp > 0.0f) {  // 有効な温度データのみ
            addTemperature(temp);
        }
    }
}