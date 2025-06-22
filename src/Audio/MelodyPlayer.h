#pragma once

#include <M5Unified.h>

/**
 * セオドア提言：完全非ブロッキングメロディ再生システム
 * 
 * 特徴：
 * - delay()を一切使用しない
 * - millis()ベースのタイミング制御
 * - 複数メロディの定義と再生
 * - シンプルなAPIで使いやすい
 */
class MelodyPlayer {
public:
    // メロディ定義構造体
    struct Melody {
        int notes[8];      // 8音のメロディ
        int duration_ms;   // 各音の長さ（ミリ秒）
    };

private:
    // 再生状態
    bool melody_active = false;
    uint32_t melody_start = 0;
    int melody_note_index = 0;
    int melody_notes[8];
    int melody_duration = 0;
    uint32_t melody_note_start = 0;

    // シングルトンパターン用
    static MelodyPlayer* instance;

public:
    MelodyPlayer();
    ~MelodyPlayer();

    // 初期化
    void begin();

    // メロディ再生（PROGMEM対応）
    void playMelody(const Melody& melody_pgm);

    // 単音ビープ
    void playBeep(int duration_ms, int frequency_hz);

    // 更新処理（loop()から呼ぶ）
    void update();

    // 再生中チェック
    bool isPlaying() const { return melody_active; }

    // 再生停止
    void stop();

    // シングルトンインスタンス取得
    static MelodyPlayer* getInstance() {
        if (!instance) {
            instance = new MelodyPlayer();
        }
        return instance;
    }
};

// 便利なマクロ
#define MELODY_PLAYER MelodyPlayer::getInstance()