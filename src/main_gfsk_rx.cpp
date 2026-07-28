#include <Arduino.h>
#include "packet.h"
#include "radio_sx1262.h"

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0); // Prevent blocking on battery power
  delay(1000);
  
  if (!radioBeginGFSK(868.0)) { 
    if (Serial) Serial.println("GFSK init failed"); 
    while(true) delay(1000); 
  }
  
  if (Serial) Serial.println("GFSK RX listening — 868 MHz");
}

void loop() {
  ImuPacket pkt;
  int16_t rssi = 0, snr = 0;
  
  if (radioReceivePacket(pkt, rssi, snr)) {
    if (Serial) {
      Serial.print("RX OK  ");
      printPacket(Serial, pkt, rssi, snr);
    }
  }
}