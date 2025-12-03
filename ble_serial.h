#pragma once
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/*
   BLE Debug UART for ESP32-C3

   - Nordic UART-style BLE service.
   - Use any BLE UART app on phone/PC.
   - Use DBG_PRINT / DBG_PRINTLN to mirror logs to both Serial and BLE.
*/

static bool bleClientConnected = false;
static BLECharacteristic *bleTxChar = nullptr;

// Nordic UART Service UUIDs
static const char *BLE_UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *BLE_UART_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // notify
static const char *BLE_UART_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // write

class BleDebugServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    bleClientConnected = true;
    Serial.println(F("[BLE] Client connected"));
  }
  void onDisconnect(BLEServer *pServer) override {
    bleClientConnected = false;
    Serial.println(F("[BLE] Client disconnected"));
    pServer->startAdvertising();
  }
};

class BleDebugRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    // In this core, getValue() returns String
    String value = pCharacteristic->getValue();
    value.trim();
    if (value.length()) {
      Serial.print(F("[BLE RX] "));
      Serial.println(value);
    }
  }
};

// Call once in setup()
inline void bleDebugBegin(const char *deviceName = "ESP32C3-RFID") {
  BLEDevice::init(deviceName);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BleDebugServerCallbacks());

  BLEService *pService = pServer->createService(BLE_UART_SERVICE_UUID);

  // TX characteristic (ESP32 → phone)
  bleTxChar = pService->createCharacteristic(
      BLE_UART_TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
  );
  bleTxChar->addDescriptor(new BLE2902());

  // RX characteristic (phone → ESP32)
  BLECharacteristic *rxChar = pService->createCharacteristic(
      BLE_UART_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE
  );
  rxChar->setCallbacks(new BleDebugRxCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_UART_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
  Serial.println(F("[BLE] Debug UART advertising started"));
}

// Send raw payload over BLE (no newline)
inline void bleDebugPrint(const String &s) {
  if (!bleClientConnected || bleTxChar == nullptr) return;
  bleTxChar->setValue((uint8_t *)s.c_str(), s.length());
  bleTxChar->notify();
  delay(2); // small gap so phone doesn't choke
}

inline void bleDebugPrintLn(const String &s) {
  bleDebugPrint(s + "\r\n");
}

// Convenience macros: mirror to Serial + BLE
#define DBG_PRINT(x)    do { Serial.print(x);   bleDebugPrint(String(x));    } while (0)
#define DBG_PRINTLN(x)  do { Serial.println(x); bleDebugPrintLn(String(x));  } while (0)
