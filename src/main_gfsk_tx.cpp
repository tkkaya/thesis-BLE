#include <Arduino.h>
#include "payload_config.h"
#include "imu.h"
#include "packet.h"
#include "radio_sx1262.h"

static uint16_t seq = 0;
static uint32_t lastSend = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0); // Prevent blocking on battery power
  delay(1000);
  
#if USE_IMU_PAYLOAD
  if (!imuBegin()) { 
    if (Serial) Serial.println("MPU6050 init failed"); 
    while(true) delay(1000); 
  }
  if (Serial) Serial.println("IMU ready");
#else
  if (Serial) Serial.println("Dummy mode");
#endif
  
  if (!radioBeginGFSK(868.0)) { 
    if (Serial) Serial.println("GFSK init failed"); 
    while(true) delay(1000); 
  }
  
  if (Serial) Serial.println("GFSK TX ready — 868 MHz");
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

  bool txSuccess = radioSendPacket(pkt);
  
  if (Serial) {
    Serial.print(txSuccess ? "TX OK  " : "TX FAIL ");
    printPacket(Serial, pkt);
  }
}