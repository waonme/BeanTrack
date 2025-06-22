#pragma once

#include <M5Unified.h>
#include <stdarg.h>

/**
 * スクロールティッカー式フッター
 * 
 * 特徴：
 * - リアルタイム情報をスクロール表示
 * - システム情報の自動収集
 * - 非ブロッキングスクロール
 * - 重複メッセージの防止
 */
class TickerFooter {
public:
    struct Message {
        char text[128];
        uint32_t added_time;
    };

private:
    // 設定
    static constexpr int MAX_MESSAGES = 10;
    static constexpr uint32_t MESSAGE_DURATION = 5000; // 各メッセージ5秒表示
    static constexpr uint32_t SCROLL_SPEED = 50; // スクロール速度(ms)
    static constexpr int Y_POSITION = 220; // フッター位置
    
    // メッセージ管理
    Message messages[MAX_MESSAGES];
    int message_count = 0;
    int current_index = 0;
    int scroll_offset = 0;
    uint32_t last_scroll = 0;
    uint32_t message_start = 0;
    bool enabled = false;
    
    // シングルトン
    static TickerFooter* instance;

public:
    TickerFooter();
    ~TickerFooter();
    
    // 初期化
    void begin();
    
    // 有効/無効切り替え
    void setEnabled(bool enable);
    bool isEnabled() const { return enabled; }
    
    // メッセージ追加（可変長引数対応）
    void addMessage(const char* format, ...);
    
    // 更新処理（loop()から呼ぶ）
    void update();
    
    // メッセージクリア
    void clearMessages();
    
    // シングルトンインスタンス取得
    static TickerFooter* getInstance() {
        if (!instance) {
            instance = new TickerFooter();
        }
        return instance;
    }
};

// 便利なマクロ
#define TICKER TickerFooter::getInstance()