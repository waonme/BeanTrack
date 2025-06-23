#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

/**
 * Bluetooth Low Energy 通信管理クラス
 * 
 * 機能：
 * - Nordic UART Service実装
 * - JSON形式でのデータ送信
 * - 自動再接続
 * - 差分データ送信による帯域最適化
 */
class BLEManager {
public:
    // コールバック関数型定義
    typedef void (*ConnectionCallback)(bool connected);
    typedef void (*DataRequestCallback)(JsonDocument& doc, bool fullData);

    // Nordic UART Service UUIDs
    static constexpr const char* SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
    static constexpr const char* CHARACTERISTIC_UUID_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
    static constexpr const char* CHARACTERISTIC_UUID_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

private:
    // BLEオブジェクト
    BLEServer* server = nullptr;
    BLECharacteristic* txCharacteristic = nullptr;
    
    // 接続状態
    bool deviceConnected = false;
    bool oldDeviceConnected = false;
    
    // 送信タイミング管理
    uint32_t lastDataSend = 0;
    uint32_t lastFullDataSend = 0;
    static constexpr uint32_t DATA_SEND_INTERVAL = 1000;  // 1秒
    static constexpr uint32_t FULL_DATA_INTERVAL = 15000; // 15秒
    
    // 再接続管理
    bool restartPending = false;
    uint32_t restartTimer = 0;
    static constexpr uint32_t RESTART_DELAY = 300;
    
    // コールバック
    ConnectionCallback onConnectionChange = nullptr;
    DataRequestCallback onDataRequest = nullptr;
    
    // シングルトン
    static BLEManager* instance;
    
    // サーバーコールバッククラス
    class ServerCallbacks : public BLEServerCallbacks {
        BLEManager* manager;
    public:
        ServerCallbacks(BLEManager* mgr) : manager(mgr) {}
        void onConnect(BLEServer* pServer);
        void onDisconnect(BLEServer* pServer);
    };

public:
    BLEManager();
    ~BLEManager();
    
    // 初期化
    bool begin(const char* deviceName = "M5Stack-Thermometer");
    
    // コールバック設定
    void setConnectionCallback(ConnectionCallback cb) { onConnectionChange = cb; }
    void setDataRequestCallback(DataRequestCallback cb) { onDataRequest = cb; }
    
    // 接続状態
    bool isConnected() const { return deviceConnected; }
    
    // データ送信（定期的に呼ぶ）
    void update();
    
    // 手動データ送信
    bool sendData(const char* data);
    bool sendJson(const JsonDocument& doc);
    
    // 接続管理
    void handleConnectionChange();
    
    // シングルトンインスタンス取得
    static BLEManager* getInstance() {
        if (!instance) {
            instance = new BLEManager();
        }
        return instance;
    }
};

// 便利なマクロ
#define BLE_MGR BLEManager::getInstance()