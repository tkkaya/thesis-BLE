#include "ble.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

static BLECharacteristic *pChar     = nullptr;
static BLECharacteristic *pCtrlChar = nullptr;
static bool connected = false;

static ControlPacket pendingCtrl;
static volatile bool ctrlPending = false;

// BLE callbacks pass connection changes back to the main firmware state.
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *)     override { connected = true; }
  void onDisconnect(BLEServer *s) override {
    connected = false;
    s->startAdvertising();
  }
};

// Save a complete command for the main loop to apply.
class CtrlCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *chr) override {
    String v = chr->getValue();
    if (v.length() != sizeof(ControlPacket)) {
      Serial.printf("[BLE TX] rejected control length=%u expected=%u\n",
        static_cast<unsigned>(v.length()),
        static_cast<unsigned>(sizeof(ControlPacket)));
      return;
    }
    memcpy(&pendingCtrl, v.c_str(), sizeof(ControlPacket));
    ctrlPending = true;
  }
};

// Create the service, data/control characteristics, and discoverable advertisement.
bool bleServerBegin(const char *deviceName) {
  BLEDevice::init(deviceName);
  esp_err_t mtuState = BLEDevice::setMTU(64);
  if (mtuState != ESP_OK) {
    Serial.printf("[BLE TX] failed to set server MTU: %d\n", mtuState);
  }
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

// Send one telemetry packet to the connected BLE receiver.
void bleSendPacket(const ImuPacket &pkt) {
  if (!pChar || !connected) return;
  pChar->setValue((uint8_t *)&pkt, sizeof(pkt));
  pChar->notify();
}

// Hand the saved command to the main loop once.
bool blePollControl(ControlPacket &out) {
  if (!ctrlPending) return false;
  ctrlPending = false;
  memcpy(&out, &pendingCtrl, sizeof(ControlPacket));
  return true;
}
