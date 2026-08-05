// runtime_config.h — included by every node firmware
#ifndef RUNTIME_CONFIG_H
#define RUNTIME_CONFIG_H

#include "control_packet.h"

struct RuntimeConfig {
  uint32_t sendIntervalMs   = 100;
  float    freqMHz          = 868.0f;
  uint8_t  sf               = 7;
  uint8_t  bw               = 0;
  uint8_t  cr               = 5;
  bool     streamingEnabled = false; // boot silent until Pi says start
};

extern RuntimeConfig g_config;

#endif