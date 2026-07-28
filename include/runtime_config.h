// runtime_config.h — included by every node firmware
#ifndef RUNTIME_CONFIG_H
#define RUNTIME_CONFIG_H

#include "control_packet.h"

struct RuntimeConfig {
  uint32_t sendIntervalMs = 100;
  uint8_t  topologyMode   = TOPO_ONE_TO_ONE;
  uint32_t targetMask     = 0x01;
  float    freqMHz        = 868.0f;
  uint8_t  sf             = 7;
  uint8_t  bw             = 0;   // index into a lookup table, e.g. 0=125kHz,1=250kHz,2=500kHz
  uint8_t  cr             = 5;
  bool     streamingEnabled = true;
};

extern RuntimeConfig g_config;

#endif