// main_espnow_tx.cpp

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include "payload_config.h"
#include "imu.h"
#include "packet.h"

static uint16_t seq = 0;
static uint32_t lastSend = 0;

#define ESPNOW_CHANNEL 1

// Replace with RX board MAC
uint8_t rxMac[] = {0x84, 0xFC, 0xE6, 0x6B, 0x7F, 0x3C};

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("[ESP-NOW TX] Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

bool espNowBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("[ESP-NOW TX] Local MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("[ESP-NOW TX] WiFi channel: %d\n", ESPNOW_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW TX] esp_now_init failed");
    return false;
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, rxMac, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  esp_err_t addResult = esp_now_add_peer(&peerInfo);
  if (addResult != ESP_OK) {
    Serial.printf("[ESP-NOW TX] add_peer failed: %d\n", addResult);
    return false;
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(1000);

#if USE_IMU_PAYLOAD
  if (!imuBegin()) {
    Serial.println("MPU6050 init failed");
    while (true) delay(1000);
  }
  Serial.println("IMU ready");
#else
  Serial.println("Dummy mode");
#endif

  if (!espNowBegin()) {
    while (true) delay(1000);
  }

  Serial.println("[ESP-NOW TX] Ready");
}

void loop() {
  if (millis() - lastSend < 100) return;   // 10 Hz example
  lastSend = millis();

  ImuPacket pkt;

#if USE_IMU_PAYLOAD
  ImuData d;
  imuRead(d);
  packImuData(d, pkt, NODE_ID, seq++);
#else
  packDummy(pkt, NODE_ID, seq++);
#endif

  esp_err_t err = esp_now_send(rxMac, (uint8_t*)&pkt, sizeof(pkt));

  Serial.print(err == ESP_OK ? "[ESP-NOW TX] Queued  " : "[ESP-NOW TX] Error   ");
  printPacket(Serial, pkt);
}