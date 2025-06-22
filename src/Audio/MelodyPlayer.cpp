#include "MelodyPlayer.h"

// シングルトンインスタンス
MelodyPlayer* MelodyPlayer::instance = nullptr;

MelodyPlayer::MelodyPlayer() {
    // コンストラクタ
}

MelodyPlayer::~MelodyPlayer() {
    stop();
}

void MelodyPlayer::begin() {
    // 初期化処理（今のところ特になし）
    melody_active = false;
}

void MelodyPlayer::playMelody(const Melody& melody_pgm) {
    if (melody_active) return;  // 既に再生中の場合は無視
    
    // PROGMEMから読み取り
    Melody melody;
    memcpy_P(&melody, &melody_pgm, sizeof(Melody));
    
    // メロディデータをコピー
    for (int i = 0; i < 8; i++) {
        melody_notes[i] = melody.notes[i];
    }
    melody_duration = melody.duration_ms;
    
    // 非ブロッキング再生開始
    melody_active = true;
    melody_start = millis();
    melody_note_index = 0;
    melody_note_start = millis();
    
    // 最初の音を再生
    if (melody_notes[0] > 0) {
        M5.Speaker.tone(melody_notes[0], melody_duration);
    }
}

void MelodyPlayer::playBeep(int duration_ms, int frequency_hz) {
    // 単音ビープ（シンプル版）
    M5.Speaker.tone(frequency_hz, duration_ms);
}

void MelodyPlayer::update() {
    if (!melody_active) return;
    
    uint32_t now = millis();
    uint32_t note_elapsed = now - melody_note_start;
    
    // 現在の音符の再生時間終了チェック
    if (note_elapsed >= melody_duration + 50) {  // 50msの間隔を設ける
        melody_note_index++;
        
        if (melody_note_index >= 8 || melody_notes[melody_note_index] <= 0) {
            // メロディ終了
            melody_active = false;
            M5.Speaker.stop();
            return;
        }
        
        // 次の音符を再生
        M5.Speaker.tone(melody_notes[melody_note_index], melody_duration);
        melody_note_start = now;
    }
}

void MelodyPlayer::stop() {
    if (melody_active) {
        melody_active = false;
        M5.Speaker.stop();
    }
}