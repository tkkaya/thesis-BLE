// main_espnow_rx.cpp

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include "packet.h"
#include "frame.h"
#include "control_packet.h"

#define ESPNOW_CHANNEL 1

static volatile bool packetReady = false;
// This single slot keeps only the latest packet until loop() prints it.
static ImuPacket rxPacket;
static uint8_t lastSender[6] = {0};
static uint32_t rxTimestampMs = 0;

// Store one sender's identity alongside its packet counters.
struct NodeTracker {
  bool inUse = false;
  uint8_t nodeId = 0;
  PacketTracker packets;
};

static NodeTracker nodeTrackers[3];

// Find this sender's counters, or reserve an empty tracker for it.
static PacketTracker *trackerForNode(uint8_t nodeId) {
  for (NodeTracker &entry : nodeTrackers) {
    if (entry.inUse && entry.nodeId == nodeId) return &entry.packets;
  }

  for (NodeTracker &entry : nodeTrackers) {
    if (!entry.inUse) {
      entry.inUse = true;
      entry.nodeId = nodeId;
      entry.packets = {};
      return &entry.packets;
    }
  }

  return nullptr;
}

uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Validate and copy each incoming telemetry frame into the shared slot.
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 1) return;

  if (data[0] == FRAME_TELEMETRY) {
    if (len - 1 != sizeof(ImuPacket)) {
      Serial.printf("[ESP-NOW RX] Wrong telemetry size: %d\n", len);
      return;
    }
    memcpy(&rxPacket, data + 1, sizeof(ImuPacket));
    memcpy(lastSender, info->src_addr, 6);
    rxTimestampMs = millis();
    packetReady = true;
  }
  // The current ESP-NOW TX firmware does not send FRAME_CONTROL responses.
  // Add response handling here if a TX-to-RX control or ACK path is introduced.
}

// Put Wi-Fi and ESP-NOW on one channel, then register the receiver.
bool espNowBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("[ESP-NOW RX] Local MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW RX] esp_now_init failed");
    return false;
  }

  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESP-NOW RX] Failed to add broadcast peer");
    return false;
  }

  return true;
}

// Reads exactly sizeof(ControlPacket) bytes from Serial (sent by the Pi),
// validates it, and re-broadcasts it to every TX node over ESP-NOW.
void pollSerialForControl() {
  static uint8_t buf[sizeof(ControlPacket)];
  static size_t bufLen = 0;

  while (Serial.available() > 0) {
    uint8_t b = Serial.read();

    if (bufLen == 0 && b != CONTROL_MAGIC) {
      // Discard anything that isn't the start of a real frame —
      // this is what lets the parser recover from boot banners,
      // noise, or a previous misaligned read.
      continue;
    }

    buf[bufLen++] = b;

    if (bufLen == sizeof(buf)) {
      ControlPacket cmd;
      memcpy(&cmd, buf, sizeof(ControlPacket));
      bufLen = 0;

      if (!controlPacketValid(cmd)) {
        Serial.println("[ESP-NOW RX] Invalid control packet from Pi (magic/crc)");
        continue; // keep scanning — don't get stuck
      }

      uint8_t frame[1 + sizeof(ControlPacket)];
      frame[0] = FRAME_CONTROL;
      memcpy(frame + 1, &cmd, sizeof(ControlPacket));

      esp_err_t err = esp_now_send(broadcastMac, frame, sizeof(frame));
      Serial.printf("[ESP-NOW RX] Relayed cmd=%d target=%d interval=%lu -> %s\n",
        cmd.cmd, cmd.targetNodeId, cmd.intervalMs,
        err == ESP_OK ? "OK" : "FAIL");
    }
  }
}// Start serial logging and prepare ESP-NOW reception.
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(1000);


  if (!espNowBegin()) { while (true) delay(1000); }

  Serial.println("[ESP-NOW RX] Ready");
}

// Relay host commands, then process the latest received packet.
void loop() {
  pollSerialForControl();

  // Print the pending packet and update its sender's delivery counters.
  if (packetReady) {
    packetReady = false;

    Serial.printf("[ESP-NOW RX] From %02X:%02X:%02X:%02X:%02X:%02X  id=%d  ",
      lastSender[0], lastSender[1], lastSender[2],
      lastSender[3], lastSender[4], lastSender[5],
      rxPacket.node_id);

    PacketTracker *tracker = trackerForNode(rxPacket.node_id);
    if (tracker != nullptr) trackPacket(*tracker, rxPacket);
    printPacket(Serial, rxPacket);
    Serial.printf(" rx_ms=%lu node_received=%lu node_lost=%lu node_dup=%lu node_ooo=%lu\n\t",
      static_cast<unsigned long>(rxTimestampMs),
      tracker != nullptr ? static_cast<unsigned long>(tracker->received) : 0UL,
      tracker != nullptr ? static_cast<unsigned long>(tracker->lost) : 0UL,
      tracker != nullptr ? static_cast<unsigned long>(tracker->duplicates) : 0UL,
      tracker != nullptr ? static_cast<unsigned long>(tracker->outOfOrder) : 0UL);
  }
}