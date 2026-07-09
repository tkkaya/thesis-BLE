#include <Arduino.h>
#include "payload_config.h"
#include "imu.h"
#include "packet.h"

static uint16_t seq = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
#if USE_IMU_PAYLOAD
  if (!imuBegin()) { Serial.println("MPU6050 init failed"); while(true) delay(1000); }
  Serial.println("IMU ready");
#else
  Serial.println("Dummy mode — no IMU needed");
#endif
}

void loop() {
  ImuPacket pkt;
#if USE_IMU_PAYLOAD
  ImuData d; imuRead(d); packImuData(d, pkt, NODE_ID, seq++);
#else
  packDummy(pkt, NODE_ID, seq++);
#endif
  printPacket(Serial, pkt);
  delay(200);
}