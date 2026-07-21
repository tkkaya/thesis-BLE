// main_espnow_tx.cpp

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "payload_config.h"
#include "imu.h"
#include "packet.h"

static uint16_t seq = 0;
static uint32_t lastSend = 0;

// Replace with RX board MAC
uint8_t rxMac[] = {0x84, 0xFC, 0xE6, 0x6B, 0x7F, 0x3C};

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("[ESP-NOW TX] Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

bool espNowBegin() {
  WiFi.mode(WIFI_STA);
Serial.printf("[ESP-NOW] WiFi channel: %d\n", WiFi.channel());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW TX] esp_now_init failed");
    return false;
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, rxMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESP-NOW TX] add_peer failed");
    return false;
  }

  Serial.print("[ESP-NOW TX] Local MAC: ");
  Serial.println(WiFi.macAddress());
  return true;
}

void setup() {
  Serial.begin(115200);
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