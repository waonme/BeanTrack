#include "TickerFooter.h"

// シングルトンインスタンス
TickerFooter* TickerFooter::instance = nullptr;

TickerFooter::TickerFooter() {
    // コンストラクタ
}

TickerFooter::~TickerFooter() {
    // デストラクタ
}

void TickerFooter::begin() {
    enabled = false;
    message_count = 0;
    current_index = 0;
    scroll_offset = 0;
}

void TickerFooter::setEnabled(bool enable) {
    enabled = enable;
    if (!enable) {
        // 無効化時はフッター領域をクリア
        M5.Lcd.fillRect(0, Y_POSITION, 320, 20, TFT_BLACK);
    } else {
        // 有効化時は初期メッセージを追加
        clearMessages();
        addMessage("ティッカーフッター有効化");
    }
}

void TickerFooter::addMessage(const char* format, ...) {
    if (!enabled) return;
    
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // 既存メッセージと重複チェック
    for (int i = 0; i < message_count; i++) {
        if (strcmp(messages[i].text, buffer) == 0) {
            return; // 重複メッセージは追加しない
        }
    }
    
    // メッセージ追加（リングバッファ形式）
    int index = message_count % MAX_MESSAGES;
    strncpy(messages[index].text, buffer, sizeof(messages[index].text) - 1);
    messages[index].text[sizeof(messages[index].text) - 1] = '\0'; // null終端を保証
    messages[index].added_time = millis();
    
    if (message_count < MAX_MESSAGES) {
        message_count++;
    }
}

void TickerFooter::update() {
    if (!enabled || message_count == 0) return;
    
    uint32_t now = millis();
    
    // メッセージ切り替えタイミング
    if (now - message_start > MESSAGE_DURATION) {
        current_index = (current_index + 1) % message_count;
        scroll_offset = 320; // 右端から開始
        message_start = now;
    }
    
    // スクロール更新
    if (now - last_scroll > SCROLL_SPEED) {
        scroll_offset -= 2; // 2ピクセルずつスクロール
        last_scroll = now;
        
        // メッセージが左端を超えたら右端に戻す
        const char* current_msg = messages[current_index].text;
        int msg_width = M5.Lcd.textWidth(current_msg);
        if (scroll_offset < -msg_width) {
            scroll_offset = 320;
        }
    }
    
    // 描画
    M5.Lcd.fillRect(0, Y_POSITION, 320, 20, TFT_BLACK);
    M5.Lcd.setFont(&fonts::lgfxJapanGothic_12);
    M5.Lcd.setTextColor(TFT_CYAN);
    M5.Lcd.setCursor(scroll_offset, Y_POSITION + 4);
    M5.Lcd.printf("%s", messages[current_index].text);
}

void TickerFooter::clearMessages() {
    message_count = 0;
    current_index = 0;
    scroll_offset = 0;
    message_start = millis();
}