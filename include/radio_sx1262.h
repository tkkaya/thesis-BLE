#ifndef RADIO_SX1262_H
#define RADIO_SX1262_H

#include <Arduino.h>
#include "packet.h"
#include "control_packet.h"

// Wio-SX1262 B2B connector — XIAO ESP32-S3
#define SX1262_NSS    41
#define SX1262_DIO1   39
#define SX1262_NRST   42
#define SX1262_BUSY   40
#define SX1262_SCK    7
#define SX1262_MOSI   9
#define SX1262_MISO   8
#define SX1262_ANT_SW 38  // antenna switch, must be HIGH to TX/RX

bool radioBeginLoRa(float freqMHz);
bool radioBeginGFSK(float freqMHz);

// TX-side: send telemetry, RX-side: send control (role-specific by design)
bool radioSendPacket(const ImuPacket &pkt);
bool radioSendControl(const ControlPacket &cmd);

// RX-side: poll for telemetry, TX-side: poll for control (role-specific by design)
bool radioReceivePacket(ImuPacket &pkt, int16_t &rssi, int16_t &snr);
bool radioPollControlFrame(ControlPacket &out);

uint32_t radioGetIsrFireCount();
uint32_t radioGetAirtimeMs(bool controlFrame);

#endif
