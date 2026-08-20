// main_ble_rx.cpp — env:ble_rx
//
// Adds live node-count control on top of the working bounded-scan build:
// `~/radio_control.py --protocol ble nodes N` lets you raise/lower how
// many simultaneous TX connections the RX holds, on the go, without
// reflashing. This command is intercepted and handled LOCALLY by the RX
// — it is never relayed to any TX board (TX firmware needs no changes).

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include "packet.h"
#include "control_packet.h"
#include "ble.h"

#define MAX_TX_NODES 3     // hard ceiling — array size, compile-time
#define SCAN_SECONDS 3

struct TxLink {
  bool inUse     = false;
  bool connected = false;
  BLEAddress address = BLEAddress((uint8_t *)"\0\0\0\0\0\0");
  BLEClient               *client      = nullptr;
  BLERemoteCharacteristic *dataChar    = nullptr;
  BLERemoteCharacteristic *controlChar = nullptr;
};

static TxLink links[MAX_TX_NODES];

// Runtime-adjustable limit, 0..MAX_TX_NODES. Starts fully open.
static int activeNodeLimit = MAX_TX_NODES;

// Recorded by scan callback. Callback never calls stop().
static BLEAdvertisedDevice *pendingDevice = nullptr;
static volatile bool matchFound = false;
static bool scanning = false;

static volatile bool newPacketReady = false;
static ImuPacket     rxPacket;
static int           rxLink = -1;

static SemaphoreHandle_t logLock = nullptr;

static void slog(const char *fmt, ...) {
  char buf[192];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  xSemaphoreTake(logLock, portMAX_DELAY);
  for (size_t k = 0; buf[k] != '\0'; k++) {
    if (buf[k] == '\n') Serial.write('\r');
    Serial.write(buf[k]);
  }
  xSemaphoreGive(logLock);
}

static int connectedCount() {
  int n = 0;
  for (int i = 0; i < MAX_TX_NODES; i++) if (links[i].inUse && links[i].connected) n++;
  return n;
}

static int freeSlot() {
  for (int i = 0; i < MAX_TX_NODES; i++) if (!links[i].inUse) return i;
  return -1;
}

// ─── Live node-count control ─────────────────────────────────────────────
// Lowering the limit disconnects the highest-indexed connected link(s)
// until we're at or under the new limit. Raising it just lets loop()'s
// scan-trigger condition (connectedCount() < activeNodeLimit) start
// filling free slots again on the next cycle — no immediate action needed.

static void setNodeCountLimit(int desired) {
  if (desired < 0) desired = 0;
  if (desired > MAX_TX_NODES) desired = MAX_TX_NODES;

  slog("[BLE RX] node count limit: %d -> %d\n", activeNodeLimit, desired);
  activeNodeLimit = desired;

  int excess = connectedCount() - activeNodeLimit;
  for (int i = MAX_TX_NODES - 1; i >= 0 && excess > 0; i--) {
    if (links[i].inUse && links[i].connected) {
      slog("[BLE RX] link=%d disconnecting (over new limit)\n", i);
      links[i].client->disconnect();  // onDisconnect() callback frees the slot
      excess--;
    }
  }
}

// ─── Telemetry notify (BLE host task) ────────────────────────────────────────

static void onNotify(BLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool) {
  if (len != sizeof(ImuPacket)) return;
  for (int i = 0; i < MAX_TX_NODES; i++) {
    if (links[i].inUse && links[i].dataChar == chr) {
      memcpy(&rxPacket, data, sizeof(ImuPacket));
      rxLink = i;
      newPacketReady = true;
      return;
    }
  }
}

// ─── Serial control input from the Pi ────────────────────────────────────────

static void pollSerialForControl() {
  static uint8_t buf[sizeof(ControlPacket)];
  static size_t len = 0;

  while (Serial.available() > 0) {
    uint8_t b = Serial.read();
    if (len == 0 && b != CONTROL_MAGIC) continue;
    buf[len++] = b;
    if (len < sizeof(buf)) continue;
    len = 0;

    ControlPacket cmd;
    memcpy(&cmd, buf, sizeof(cmd));
    if (!controlPacketValid(cmd)) continue;

    // Intercepted locally — never relayed to any TX board.
    if (cmd.cmd == CMD_SET_NODE_COUNT) {
      setNodeCountLimit((int)cmd.intervalMs);
      continue;
    }

    for (int i = 0; i < MAX_TX_NODES; i++) {
      if (!links[i].inUse || !links[i].connected || !links[i].controlChar) continue;
      bool ok = links[i].controlChar->writeValue((uint8_t *)&cmd, sizeof(cmd), true);
      slog("[BLE RX] link=%d cmd=%u target=%u -> %s\n",
           i, cmd.cmd, cmd.targetNodeId, ok ? "OK" : "FAIL");
    }
  }
}

// ─── Per-link client callbacks ───────────────────────────────────────────────

class LinkCB : public BLEClientCallbacks {
public:
  explicit LinkCB(int idx) : i(idx) {}
  void onConnect(BLEClient *) override {
    links[i].connected = true;
    slog("[BLE RX] link=%d connected\n", i);
  }
  void onDisconnect(BLEClient *) override {
    slog("[BLE RX] link=%d disconnected\n", i);
    links[i] = TxLink();
  }
private:
  int i;
};

// ─── Scan callback: records ONE candidate, never calls stop() ──────────────

class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    if (matchFound || pendingDevice) return;

    bool match =
      (dev.haveServiceUUID() && dev.isAdvertisingService(BLEUUID(BLE_SERVICE_UUID))) ||
      (dev.haveName() && strncmp(dev.getName().c_str(), "ESP32S3-IMU", 11) == 0);
    if (!match) return;

    BLEAddress addr = dev.getAddress();
    for (int i = 0; i < MAX_TX_NODES; i++)
      if (links[i].inUse && links[i].address.equals(addr)) return;

    pendingDevice = new BLEAdvertisedDevice(dev);
    matchFound = true;
    slog("[BLE RX] found candidate %s (scan will finish naturally)\n", addr.toString().c_str());
  }
};

// ─── Bounded scan cycle + pending connect (loop task) ───────────────────────

static void runBoundedScanCycle() {
  if (scanning) return;
  if (freeSlot() < 0) return;
  if (connectedCount() >= activeNodeLimit) return;   // node-count gate
  if (pendingDevice) return;

  BLEScan *scan = BLEDevice::getScan();
  matchFound = false;
  scan->clearResults();

  scanning = true;
  slog("[BLE RX] scanning for %ds...\n", SCAN_SECONDS);
  scan->start(SCAN_SECONDS, false);   // bounded — returns on its own
  scanning = false;
  slog("[BLE RX] scan cycle finished, matchFound=%d\n", matchFound ? 1 : 0);
}

static void connectPending() {
  if (!pendingDevice) return;

  // Re-check the gate: the limit may have been lowered while this
  // candidate was pending.
  if (connectedCount() >= activeNodeLimit) {
    slog("[BLE RX] dropping candidate — node count limit reached\n");
    delete pendingDevice;
    pendingDevice = nullptr;
    matchFound = false;
    return;
  }

  int i = freeSlot();
  if (i < 0) {
    delete pendingDevice;
    pendingDevice = nullptr;
    matchFound = false;
    return;
  }

  TxLink &link = links[i];
  link = TxLink();
  link.address = pendingDevice->getAddress();
  link.client  = BLEDevice::createClient();
  link.client->setClientCallbacks(new LinkCB(i));

  slog("[BLE RX] link=%d connecting to %s...\n", i, link.address.toString().c_str());
  bool ok = link.client->connect(pendingDevice);
  slog("[BLE RX] link=%d connect() returned %s\n", i, ok ? "true" : "false");

  delete pendingDevice;
  pendingDevice = nullptr;
  matchFound = false;

  if (!ok) { link = TxLink(); return; }

  link.client->setMTU(32);

  BLERemoteService *svc = link.client->getService(BLE_SERVICE_UUID);
  if (!svc) {
    slog("[BLE RX] link=%d service not found\n", i);
    link.client->disconnect(); link = TxLink(); return;
  }

  link.dataChar = svc->getCharacteristic(BLE_CHARACTERISTIC_UUID);
  if (!link.dataChar || !link.dataChar->canNotify()) {
    slog("[BLE RX] link=%d telemetry characteristic unusable\n", i);
    link.client->disconnect(); link = TxLink(); return;
  }
  link.dataChar->registerForNotify(onNotify);

  link.controlChar = svc->getCharacteristic(BLE_CONTROL_CHAR_UUID);
  if (!(link.controlChar && link.controlChar->canWrite())) link.controlChar = nullptr;

  link.inUse = true;
  slog("[BLE RX] link=%d ready (control %s)\n", i, link.controlChar ? "yes" : "no");
}

// ─── setup() / loop() ────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

  logLock = xSemaphoreCreateMutex();
  slog("[BLE RX] boot, MAX_TX_NODES=%d, initial limit=%d, bounded %ds scans\n",
    MAX_TX_NODES, activeNodeLimit, SCAN_SECONDS);

  BLEDevice::init("ESP32S3-IMU-RX");
  BLEScan *scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new ScanCB());
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
}

void loop() {
  pollSerialForControl();

  if (!pendingDevice) runBoundedScanCycle();
  connectPending();

  static uint32_t lastBeat = 0;
  if (millis() - lastBeat > 3000) {
    lastBeat = millis();
    slog("[BLE RX] alive links=%d/%d limit=%d\n",
      connectedCount(), MAX_TX_NODES, activeNodeLimit);
  }

  if (newPacketReady) {
    newPacketReady = false;
    int16_t rssi = (rxLink >= 0 && links[rxLink].client) ? links[rxLink].client->getRssi() : 0;
    xSemaphoreTake(logLock, portMAX_DELAY);
    Serial.printf("[BLE RX] link=%d  ", rxLink);
    printPacket(Serial, rxPacket, rssi, 0);
    xSemaphoreGive(logLock);
  }
}