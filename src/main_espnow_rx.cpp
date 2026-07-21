// main_espnow_rx.cpp

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "packet.h"

static volatile bool packetReady = false;
static ImuPacket rxPacket;
static uint8_t lastSender[6] = {0};

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(ImuPacket)) {
    Serial.printf("[ESP-NOW RX] Wrong packet size: %d\n", len);
    return;
  }
  memcpy(&rxPacket, data, sizeof(ImuPacket));
  memcpy(lastSender, info->src_addr, 6);
  packetReady = true;
}

bool espNowBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.STA.begin();   // required on core 3.x to populate the MAC address
Serial.printf("[ESP-NOW] WiFi channel: %d\n", WiFi.channel());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW RX] esp_now_init failed");
    return false;
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.print("[ESP-NOW RX] Local MAC: ");
  Serial.println(WiFi.macAddress());
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!espNowBegin()) {
    while (true) delay(1000);
  }

  Serial.println("[ESP-NOW RX] Ready");
}

void loop() {
  if (packetReady) {
    packetReady = false;

    Serial.printf("[ESP-NOW RX] From %02X:%02X:%02X:%02X:%02X:%02X  ",
      lastSender[0], lastSender[1], lastSender[2],
      lastSender[3], lastSender[4], lastSender[5]);

    printPacket(Serial, rxPacket);
  }
}