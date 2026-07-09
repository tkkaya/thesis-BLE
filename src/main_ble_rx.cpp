// main_ble_rx.cpp — env:ble_rx
//
// BLE Central (client) — scans for the TX node via Service UUID,
// connects automatically, requests proper MTU size, subscribes to IMU packet notifications,
// decodes the binary ImuPacket and prints all 6-axis values to Serial.

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include "packet.h"
#include "ble.h"   // reuses UUID defines from ble.h

// ─── State ───────────────────────────────────────────────────────────────────

static BLEClient               *pClient      = nullptr;
static BLERemoteCharacteristic *pRemoteChar  = nullptr;
static BLEAdvertisedDevice     *targetDevice = nullptr;

static volatile bool deviceFound    = false;
static volatile bool connected       = false;
static volatile bool newPacketReady = false;
static ImuPacket       rxPacket;
static int16_t       lastRssi       = 0;

// ─── Notification callback ───────────────────────────────────────────────────

static void notifyCallback(
  BLERemoteCharacteristic *pChar,
  uint8_t                 *pData,
  size_t                   length,
  bool                     isNotify)
{
  if (length != sizeof(ImuPacket)) {
    Serial.print("[BLE RX] Unexpected packet size: ");
    Serial.println(length);
    return;
  }
  memcpy(&rxPacket, pData, sizeof(ImuPacket));
  newPacketReady = true;
}

// ─── Client callbacks ────────────────────────────────────────────────────────

class ClientCB : public BLEClientCallbacks {
  void onConnect(BLEClient *) override {
    connected = true;
    Serial.println("[BLE RX] Connected to TX node");
  }
  void onDisconnect(BLEClient *) override {
    connected    = false;
    deviceFound  = false;
    targetDevice = nullptr;
    Serial.println("[BLE RX] Disconnected — will rescan...");
    BLEDevice::getScan()->clearResults();
    BLEDevice::getScan()->start(0, false);
  }
};

// ─── Scan callback ───────────────────────────────────────────────────────────
// DEBUG VERSION — prints every device seen, with UUID status.

class AdvertisedDeviceCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    Serial.print("[BLE RX] Saw device: ");
    Serial.print(advertisedDevice.getName().c_str());
    Serial.print(" | RSSI: ");
    Serial.print(advertisedDevice.getRSSI());
    Serial.print(" | hasUUID: ");
    Serial.println(advertisedDevice.haveServiceUUID() ? "yes" : "no");

    if (advertisedDevice.haveServiceUUID()) {
      Serial.print("  -> UUID: ");
      Serial.println(advertisedDevice.getServiceUUID().toString().c_str());
    }

if (advertisedDevice.haveServiceUUID() &&
    advertisedDevice.isAdvertisingService(BLEUUID(BLE_SERVICE_UUID))) {
  Serial.println("[BLE RX] MATCH — Found TX node by UUID");
  BLEDevice::getScan()->stop();
  targetDevice = new BLEAdvertisedDevice(advertisedDevice);
  deviceFound  = true;
}

  }
};

// ─── Connect and subscribe ───────────────────────────────────────────────────

static bool connectAndSubscribe() {
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new ClientCB());

  Serial.println("[BLE RX] Connecting...");
  if (!pClient->connect(targetDevice)) {
    Serial.println("[BLE RX] Connection failed");
    return false;
  }

  pClient->setMTU(32);
  Serial.println("[BLE RX] MTU updated to 32");

  BLERemoteService *pService = pClient->getService(BLE_SERVICE_UUID);
  if (!pService) {
    Serial.println("[BLE RX] Service not found");
    pClient->disconnect();
    return false;
  }

  pRemoteChar = pService->getCharacteristic(BLE_CHARACTERISTIC_UUID);
  if (!pRemoteChar) {
    Serial.println("[BLE RX] Characteristic not found");
    pClient->disconnect();
    return false;
  }

  if (pRemoteChar->canNotify()) {
    pRemoteChar->registerForNotify(notifyCallback);
    Serial.println("[BLE RX] Subscribed to notifications");
    return true;
  }

  Serial.println("[BLE RX] Characteristic does not support notify");
  pClient->disconnect();
  return false;
}

// ─── setup() ─────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  uint32_t startTime = millis();
  while (!Serial && (millis() - startTime < 3000)) {
    delay(10);
  }

  Serial.println("[BLE RX] Serial Engine Initialized.");
  BLEDevice::init("ESP32S3-IMU-RX");

  BLEScan *pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCB());
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(99);

  Serial.println("[BLE RX] Scanning for ESP32S3-IMU...");
  pScan->start(0, false);
}

// ─── loop() ──────────────────────────────────────────────────────────────────
void loop() {
  if (deviceFound && !connected) {
    if (!connectAndSubscribe()) {
      deviceFound  = false;
      targetDevice = nullptr;
      Serial.println("[BLE RX] Retrying scan in 2s...");
      delay(2000);
      BLEDevice::getScan()->clearResults();
      BLEDevice::getScan()->start(0, false);
    }
  }

  if (!connected && !deviceFound && targetDevice == nullptr) {
    delay(1000);
    BLEDevice::getScan()->clearResults();
    BLEDevice::getScan()->start(0, false);
  }

  if (newPacketReady) {
    newPacketReady = false;
    if (pClient && connected) lastRssi = pClient->getRssi();
    printPacket(Serial, rxPacket, lastRssi, 0);
  }
}