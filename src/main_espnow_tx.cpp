// main_espnow_tx.cpp

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include "payload_config.h"
#include "imu.h"
#include "packet.h"
#include "frame.h"
#include "control_packet.h"
#include "runtime_config.h"
#include "apply_config.h"
#include "identity.h"

static uint16_t seq = 0;
static uint32_t lastSend = 0;

#define ESPNOW_CHANNEL 1

uint8_t rxMac[] = {0x84, 0xFC, 0xE6, 0x6B, 0x7F, 0x3C};
uint8_t g_myNodeId = 0; // derived at boot in setup()

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("[ESP-NOW TX] Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 1) return;

  if (data[0] == FRAME_CONTROL) {
    if (len - 1 != sizeof(ControlPacket)) {
      Serial.printf("[ESP-NOW TX] Bad control frame size: %d\n", len);
      return;
    }
    ControlPacket cmd;
    memcpy(&cmd, data + 1, sizeof(ControlPacket));
    if (!controlPacketValid(cmd)) {
      Serial.println("[ESP-NOW TX] Invalid control packet (magic/crc)");
      return;
    }
    bool ok = applyConfig(cmd);
    Serial.printf("[ESP-NOW TX] id=%d cmd=%d target=%d applied=%s interval=%lu streaming=%d\n",
      g_myNodeId, cmd.cmd, cmd.targetNodeId, ok ? "yes" : "no",
      g_config.sendIntervalMs, g_config.streamingEnabled);
  }
}

bool espNowBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("[ESP-NOW TX] id=%d Local MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    g_myNodeId, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW TX] esp_now_init failed");
    return false;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, rxMac, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESP-NOW TX] add_peer failed");
    return false;
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(1000);

  g_myNodeId = deriveNodeId();

#if USE_IMU_PAYLOAD
  if (!imuBegin()) { Serial.println("MPU6050 init failed"); while (true) delay(1000); }
  Serial.println("IMU ready");
#else
  Serial.println("Dummy mode");
#endif

  if (!espNowBegin()) { while (true) delay(1000); }

  Serial.printf("[ESP-NOW TX] id=%d Ready\n", g_myNodeId);
}

void loop() {
  if (!g_config.streamingEnabled) return;
  if (millis() - lastSend < g_config.sendIntervalMs) return;
  lastSend = millis();

  uint8_t buf[1 + sizeof(ImuPacket)];
  buf[0] = FRAME_TELEMETRY;

  ImuPacket pkt;
#if USE_IMU_PAYLOAD
  ImuData d;
  imuRead(d);
  packImuData(d, pkt, g_myNodeId, seq++);
#else
  packDummy(pkt, g_myNodeId, seq++);
#endif

  memcpy(buf + 1, &pkt, sizeof(ImuPacket));

  esp_err_t err = esp_now_send(rxMac, buf, sizeof(buf));

  Serial.print(err == ESP_OK ? "[ESP-NOW TX] Queued  " : "[ESP-NOW TX] Error   ");
  printPacket(Serial, pkt);
}