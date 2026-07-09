#include <Arduino.h>
#include "payload_config.h"
#include "imu.h"
#include "packet.h"
#include "ble.h"

static uint16_t seq = 0;
static uint32_t lastSend = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
#if USE_IMU_PAYLOAD
  if (!imuBegin()) { Serial.println("MPU6050 init failed"); while(true) delay(1000); }
  Serial.println("IMU ready");
#else
  Serial.println("Dummy mode");
#endif
  bleServerBegin("ESP32S3-IMU");
  Serial.println("BLE advertising — waiting for central...");
}

void loop() {
  if (!bleIsConnected() || millis() - lastSend < 50) return;
  lastSend = millis();

  ImuPacket pkt;
#if USE_IMU_PAYLOAD
  ImuData d; imuRead(d); packImuData(d, pkt, NODE_ID, seq++);
#else
  packDummy(pkt, NODE_ID, seq++);
#endif
  bleSendPacket(pkt);
  //printPacket(Serial, pkt);
}