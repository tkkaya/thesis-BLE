#include "ble.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


static BLECharacteristic *pChar = nullptr;
static bool connected = false;


class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *)     override { connected = true; }
  void onDisconnect(BLEServer *s) override {
    connected = false;
    s->startAdvertising();
  }
};


bool bleServerBegin(const char *deviceName) {
  BLEDevice::init(deviceName);
  BLEServer  *srv = BLEDevice::createServer();
  srv->setCallbacks(new ServerCB());
  BLEService *svc = srv->createService(BLE_SERVICE_UUID);
  pChar = svc->createCharacteristic(
    BLE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pChar->addDescriptor(new BLE2902());
  svc->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  return true;
}


bool bleIsConnected() { return connected; }


void bleSendPacket(const ImuPacket &pkt) {
  if (!pChar || !connected) return;
  pChar->setValue((uint8_t *)&pkt, sizeof(pkt));
  pChar->notify();
}