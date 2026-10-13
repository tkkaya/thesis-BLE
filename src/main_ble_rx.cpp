// main_ble_rx.cpp — env:ble_rx
//
// The RX connection limit can be changed without reflashing with
// `radio_control.py --protocol ble nodes N`. This command is handled locally
// and is never relayed to a TX board.

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "packet.h"
#include "control_packet.h"
#include "ble.h"

#define MAX_TX_NODES 3     // hard ceiling — array size, compile-time
#define SCAN_SECONDS 3

// Keep the BLE handles and state for one connected transmitter together.
struct TxLink {
  bool inUse     = false;
  bool connected = false;
  BLEAddress address = BLEAddress((uint8_t *)"\0\0\0\0\0\0");
  BLEClient               *client      = nullptr;
  BLERemoteCharacteristic *dataChar    = nullptr;
  BLERemoteCharacteristic *controlChar = nullptr;
  bool controlWithResponse = true;
};

static TxLink links[MAX_TX_NODES];

// Runtime-adjustable limit, 0..MAX_TX_NODES. Starts fully open.
static int activeNodeLimit = MAX_TX_NODES;

// Recorded by scan callback. Callback never calls stop().
static BLEAdvertisedDevice *pendingDevice = nullptr;
static volatile bool matchFound = false;
static bool scanning = false;

// Save the packet and arrival time so the BLE callback can return quickly.
struct ReceivedTelemetry {
  ImuPacket packet;
  int link;
  uint32_t rxTimestampMs;
};

static QueueHandle_t telemetryQueue = nullptr;
static uint32_t telemetryQueueOverflow = 0;
static PacketTracker packetTrackers[MAX_TX_NODES];

static SemaphoreHandle_t logLock = nullptr;

// Keep messages from the BLE task and main loop from mixing together.
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

// Copy each packet to the queue; the main loop handles tracking and printing.
static void onNotify(BLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool) {
  if (len != sizeof(ImuPacket)) return;
  for (int i = 0; i < MAX_TX_NODES; i++) {
    if (links[i].inUse && links[i].dataChar == chr) {
      ReceivedTelemetry event;
      memcpy(&event.packet, data, sizeof(event.packet));
      event.link = i;
      event.rxTimestampMs = millis();
      if (xQueueSend(telemetryQueue, &event, 0) != pdTRUE) {
        __atomic_fetch_add(&telemetryQueueOverflow, 1, __ATOMIC_RELAXED);
      }
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
      bool ok = links[i].controlChar->writeValue(
        (uint8_t *)&cmd, sizeof(cmd), links[i].controlWithResponse);
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

  if (!link.client->setMTU(64)) {
    slog("[BLE RX] link=%d failed to request MTU 64\n", i);
  }
  slog("[BLE RX] link=%d negotiated MTU=%u control_bytes=%u\n", i,
       static_cast<unsigned>(link.client->getMTU()),
       static_cast<unsigned>(sizeof(ControlPacket)));

  // Find both characteristics before marking this link ready.
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
  if (link.client->getMTU() < sizeof(ControlPacket) + 3) {
    slog("[BLE RX] link=%d MTU too small for control packet (%u < %u)\n",
      i,
      static_cast<unsigned>(link.client->getMTU()),
      static_cast<unsigned>(sizeof(ControlPacket) + 3));
    link.controlChar = nullptr;
  } else if (!link.controlChar ||
             (!link.controlChar->canWrite() &&
              !link.controlChar->canWriteNoResponse())) {
    link.controlChar = nullptr;
  } else {
    link.controlWithResponse = link.controlChar->canWrite();
  }

  link.inUse = true;
  slog("[BLE RX] link=%d ready (control %s)\n", i, link.controlChar ? "yes" : "no");
}

// ─── setup() / loop() ────────────────────────────────────────────────────────

// Start serial logging, create the packet queue, and configure BLE scanning.
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

  logLock = xSemaphoreCreateMutex();
  slog("[BLE RX] boot, MAX_TX_NODES=%d, initial limit=%d, bounded %ds scans\n",
    MAX_TX_NODES, activeNodeLimit, SCAN_SECONDS);

  telemetryQueue = xQueueCreate(32, sizeof(ReceivedTelemetry));
  if (!telemetryQueue) {
    slog("[BLE RX] failed to create telemetry queue\n");
    while (true) delay(1000);
  }

  BLEDevice::init("ESP32S3-IMU-RX");
  BLEScan *scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new ScanCB());
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
}

// Handle host commands and connections, then print every queued notification.
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

  ReceivedTelemetry event;
  // Drain every saved packet so one slow log write doesn't replace another.
  while (xQueueReceive(telemetryQueue, &event, 0) == pdTRUE) {
    int link = event.link;
    if (link < 0 || link >= MAX_TX_NODES) continue;
    trackPacket(packetTrackers[link], event.packet);
    int16_t rssi = links[link].client ? links[link].client->getRssi() : 0;
    xSemaphoreTake(logLock, portMAX_DELAY);
    Serial.printf("[BLE RX] link=%d  ", link);
    printPacket(Serial, event.packet, rssi, 0);
    Serial.printf(" rx_ms=%lu received=%lu lost=%lu dup=%lu ooo=%lu\n",
      static_cast<unsigned long>(event.rxTimestampMs),
      static_cast<unsigned long>(packetTrackers[link].received),
      static_cast<unsigned long>(packetTrackers[link].lost),
      static_cast<unsigned long>(packetTrackers[link].duplicates),
      static_cast<unsigned long>(packetTrackers[link].outOfOrder));
    xSemaphoreGive(logLock);
  }

  static uint32_t reportedQueueOverflow = 0;
  uint32_t overflow = __atomic_load_n(&telemetryQueueOverflow, __ATOMIC_RELAXED);
  if (overflow != reportedQueueOverflow) {
    reportedQueueOverflow = overflow;
    slog("[BLE RX] telemetry queue overflow total=%lu\n",
      static_cast<unsigned long>(overflow));
  }
}