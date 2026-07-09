#include <Arduino.h>
#include "packet.h"
#include "radio_sx1262.h"

void setup() {
  Serial.begin(115200);
  delay(1000);
  if (!radioBeginGFSK(868.0)) { Serial.println("GFSK init failed"); while(true) delay(1000); }
  Serial.println("GFSK RX listening — 868 MHz");
}

void loop() {
  ImuPacket pkt;
  int16_t rssi = 0, snr = 0;
  if (radioReceivePacket(pkt, rssi, snr)) {
    Serial.print("RX OK  ");
    printPacket(Serial, pkt, rssi, snr);
  }
}