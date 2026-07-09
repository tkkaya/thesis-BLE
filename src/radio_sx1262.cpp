#include "radio_sx1262.h"
#include <RadioLib.h>
#include <SPI.h>

static SX1262 radio = new Module(SX1262_NSS, SX1262_DIO1, SX1262_NRST, SX1262_BUSY);
static bool loraMode = true;

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
    Serial.print("LoRa init err: "); Serial.println(s);
    return false;
  }
  return true;
}

bool radioBeginGFSK(float freqMHz) {
  loraMode = false;
  radioHardwareInit();
  int s = radio.beginFSK(freqMHz);
  if (s != RADIOLIB_ERR_NONE) {
    Serial.print("GFSK init err: "); Serial.println(s);
    return false;
  }
  return true;
}

bool radioSendPacket(const ImuPacket &pkt) {
  return radio.transmit((uint8_t *)&pkt, sizeof(pkt)) == RADIOLIB_ERR_NONE;
}

bool radioReceivePacket(ImuPacket &pkt, int16_t &rssi, int16_t &snr) {
  int s = radio.receive((uint8_t *)&pkt, sizeof(pkt));
  if (s == RADIOLIB_ERR_NONE) {
    rssi = (int16_t)radio.getRSSI();
    snr  = loraMode ? (int16_t)radio.getSNR() : 0;
    return true;
  }
  return false;
}