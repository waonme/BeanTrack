#include "BLEManager.h"
#include <M5Unified.h>

// シングルトンインスタンス
BLEManager* BLEManager::instance = nullptr;

// ServerCallbacks実装
void BLEManager::ServerCallbacks::onConnect(BLEServer* pServer) {
    manager->deviceConnected = true;
    M5_LOGI("BLE Client connected");
    if (manager->onConnectionChange) {
        manager->onConnectionChange(true);
    }
}

void BLEManager::ServerCallbacks::onDisconnect(BLEServer* pServer) {
    manager->deviceConnected = false;
    M5_LOGI("BLE Client disconnected");
    if (manager->onConnectionChange) {
        manager->onConnectionChange(false);
    }
}

BLEManager::BLEManager() {
    // コンストラクタ
}

BLEManager::~BLEManager() {
    // デストラクタ
    if (server) {
        // BLEDevice::deinit(); // 通常は不要
    }
}

bool BLEManager::begin(const char* deviceName) {
    // BLEデバイス初期化
    BLEDevice::init(deviceName);
    
    // サーバー作成
    server = BLEDevice::createServer();
    if (!server) {
        M5_LOGE("Failed to create BLE server");
        return false;
    }
    
    // コールバック設定
    server->setCallbacks(new ServerCallbacks(this));
    
    // Nordic UART Service作成
    BLEService* service = server->createService(SERVICE_UUID);
    if (!service) {
        M5_LOGE("Failed to create BLE service");
        return false;
    }
    
    // TX Characteristic作成（通知のみ）
    txCharacteristic = service->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    
    if (!txCharacteristic) {
        M5_LOGE("Failed to create TX characteristic");
        return false;
    }
    
    // 通知用ディスクリプタ追加
    txCharacteristic->addDescriptor(new BLE2902());
    
    // サービス開始
    service->start();
    
    // アドバタイジング開始
    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(false);
    advertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
    
    M5_LOGI("BLE advertising started, waiting for connections...");
    
    return true;
}

void BLEManager::update() {
    uint32_t now = millis();
    
    // 再接続処理
    handleConnectionChange();
    
    // データ送信処理
    if (deviceConnected && txCharacteristic) {
        // 送信間隔チェック
        if (now - lastDataSend >= DATA_SEND_INTERVAL) {
            // フルデータかライトデータか判定
            bool sendFullData = (now - lastFullDataSend >= FULL_DATA_INTERVAL);
            
            if (onDataRequest) {
                // コールバックでJSONデータを構築
                JsonDocument doc;
                onDataRequest(doc, sendFullData);
                
                // JSON送信
                if (sendJson(doc)) {
                    lastDataSend = now;
                    if (sendFullData) {
                        lastFullDataSend = now;
                    }
                }
            }
        }
    }
}

bool BLEManager::sendData(const char* data) {
    if (!deviceConnected || !txCharacteristic) {
        return false;
    }
    
    try {
        txCharacteristic->setValue((uint8_t*)data, strlen(data));
        txCharacteristic->notify();
        return true;
    } catch (...) {
        M5_LOGE("Failed to send BLE data");
        return false;
    }
}

bool BLEManager::sendJson(const JsonDocument& doc) {
    if (!deviceConnected || !txCharacteristic) {
        return false;
    }
    
    // JSONをシリアライズ
    String output;
    serializeJson(doc, output);
    
    // 送信
    return sendData(output.c_str());
}

void BLEManager::handleConnectionChange() {
    uint32_t now = millis();
    
    // 切断検出
    if (!deviceConnected && oldDeviceConnected) {
        // 切断された - 再接続タイマー開始
        restartTimer = now;
        restartPending = true;
        oldDeviceConnected = deviceConnected;
    }
    
    // 再接続処理
    if (restartPending && (now - restartTimer >= RESTART_DELAY)) {
        M5_LOGI("Restarting BLE advertising...");
        server->startAdvertising();
        restartPending = false;
    }
    
    // 接続検出
    if (deviceConnected && !oldDeviceConnected) {
        M5_LOGI("BLE connection established");
        oldDeviceConnected = deviceConnected;
    }
}