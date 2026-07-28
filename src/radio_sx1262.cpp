#include "radio_sx1262.h"
#include <RadioLib.h>
#include <SPI.h>

static SX1262 radio = new Module(SX1262_NSS, SX1262_DIO1, SX1262_NRST, SX1262_BUSY);
static bool loraMode = true;

// --- Interrupt Flag ---
static volatile bool rxFlag = false;

// Direct IRAM_ATTR since we know this is compiling for ESP32S3
IRAM_ATTR static void setRxFlag() {
  rxFlag = true;
}

static void radioHardwareInit() {
  // Antenna switch must be HIGH or the radio transmits/receives nothing
  pinMode(SX1262_ANT_SW, OUTPUT);
  digitalWrite(SX1262_ANT_SW, HIGH);

  // B2B connector uses non-default SPI pins — must set before radio.begin()
  SPI.begin(SX1262_SCK, SX1262_MISO, SX1262_MOSI, SX1262_NSS);
}

bool radioBeginLoRa(float freqMHz) {
  loraMode = true;
  radioHardwareInit();
  int s = radio.begin(freqMHz);
  if (s != RADIOLIB_ERR_NONE) {
    if (Serial) { Serial.print("LoRa init err: "); Serial.println(s); }
    return false;
  }
  
  // Attach interrupt and arm continuous RX
  radio.setPacketReceivedAction(setRxFlag);
  radio.startReceive(); 
  return true;
}

bool radioBeginGFSK(float freqMHz) {
  loraMode = false;
  radioHardwareInit();
  int s = radio.beginFSK(freqMHz);
  if (s != RADIOLIB_ERR_NONE) {
    if (Serial) { Serial.print("GFSK init err: "); Serial.println(s); }
    return false;
  }
  
  // Attach interrupt and arm continuous RX
  radio.setPacketReceivedAction(setRxFlag);
  radio.startReceive(); 
  return true;
}

bool radioSendPacket(const ImuPacket &pkt) {
  bool success = (radio.transmit((uint8_t *)&pkt, sizeof(pkt)) == RADIOLIB_ERR_NONE);
  // Return to background RX mode immediately after transmitting
  radio.startReceive(); 
  return success;
}

bool radioReceivePacket(ImuPacket &pkt, int16_t &rssi, int16_t &snr) {
  // Check the hardware interrupt flag
  if (rxFlag) {
    rxFlag = false; // Clear the flag immediately
    
    int s = radio.readData((uint8_t *)&pkt, sizeof(pkt));
    
    // Re-arm background RX immediately so we don't miss the next packet
    radio.startReceive(); 
    
    if (s == RADIOLIB_ERR_NONE) {
      rssi = (int16_t)radio.getRSSI();
      snr  = loraMode ? (int16_t)radio.getSNR() : 0;
      return true;
    } else {
      // If we caught an interrupt but failed to read, print the error code.
      // E.g., -7 is RADIOLIB_ERR_CRC_MISMATCH
      if (Serial) { 
        Serial.print("RX DROP (Error code: "); 
        Serial.print(s); 
        Serial.println(")"); 
      }
    }
  }
  return false;
}