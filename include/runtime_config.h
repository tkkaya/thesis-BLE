// runtime_config.h — included by every node firmware
#ifndef RUNTIME_CONFIG_H
#define RUNTIME_CONFIG_H

#include "control_packet.h"

struct RuntimeConfig {
  uint32_t sendIntervalMs   = 100;
  float    freqMHz          = 868.0f;

  // Bench-proven GFSK defaults:
  // 50 kbps, 25 kHz deviation, 128-bit preamble, RX bandwidth index 15.
  GfskParams gfsk = {
    50000,  // bitrateBps
    128,    // preambleBits
    25,     // deviationKhz
    15,     // rxBwIndex: 156.2 kHz in radio_sx1262.cpp table
    0
  };

  // Existing/default LoRa parameters.
  LoraParams lora = {
  7,      // sf
  7,      // bwIndex → 125 kHz (NOT 0 = 7.8 kHz)
  0,      // crIndex → 4/5
  0x12,   // syncWord
  14,     // txPowerDbm
  8,      // preambleSymbols
  {0, 0}
};


  bool streamingEnabled = false; // boot silent until Pi says start
};

extern RuntimeConfig g_config;

#endif