#include <Arduino.h>
#include "payload_config.h"
#include "imu.h"
#include "packet.h"
#include "radio_sx1262.h"

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
  if (!radioBeginGFSK(868.0)) { Serial.println("GFSK init failed"); while(true) delay(1000); }
  Serial.println("GFSK TX ready — 868 MHz");
}

void loop() {
  if (millis() - lastSend < 500) return;
  lastSend = millis();

  ImuPacket pkt;
#if USE_IMU_PAYLOAD
  ImuData d; imuRead(d); packImuData(d, pkt, NODE_ID, seq++);
#else
  packDummy(pkt, NODE_ID, seq++);
#endif
  Serial.print(radioSendPacket(pkt) ? "TX OK  " : "TX FAIL ");
  printPacket(Serial, pkt);
}