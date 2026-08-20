#include "ble.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

static BLECharacteristic *pChar     = nullptr;
static BLECharacteristic *pCtrlChar = nullptr;
static bool connected = false;

static ControlPacket pendingCtrl;
static volatile bool ctrlPending = false;

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *)     override { connected = true; }
  void onDisconnect(BLEServer *s) override {
    connected = false;
    s->startAdvertising();
  }
};

class CtrlCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *chr) override {
    String v = chr->getValue();
    if (v.length() != sizeof(ControlPacket)) return;
    memcpy(&pendingCtrl, v.c_str(), sizeof(ControlPacket));
    ctrlPending = true;
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

  pCtrlChar = svc->createCharacteristic(
    BLE_CONTROL_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCtrlChar->setCallbacks(new CtrlCB());

  svc->start();

  // 31-byte PDU can't hold both a complete 128-bit UUID (18B) and a longer
  // name — UUID goes in the primary advertisement, name in the scan
  // response (RX uses active scanning, so it reads both).
  BLEAdvertising *adv = BLEDevice::getAdvertising();

  BLEAdvertisementData advData;
  advData.setFlags(0x06);
  advData.setCompleteServices(BLEUUID(BLE_SERVICE_UUID));
  adv->setAdvertisementData(advData);

  BLEAdvertisementData scanData;
  scanData.setName(deviceName);
  adv->setScanResponseData(scanData);

  BLEDevice::startAdvertising();
  return true;
}

bool bleIsConnected() { return connected; }

void bleSendPacket(const ImuPacket &pkt) {
  if (!pChar || !connected) return;
  pChar->setValue((uint8_t *)&pkt, sizeof(pkt));
  pChar->notify();
}

bool blePollControl(ControlPacket &out) {
  if (!ctrlPending) return false;
  ctrlPending = false;
  memcpy(&out, &pendingCtrl, sizeof(ControlPacket));
  return true;
}
