// src/apply_config.cpp — single shared implementation for all protocols
#include "apply_config.h"
#include "runtime_config.h"
#include "identity.h"

RuntimeConfig g_config;
extern uint8_t g_myNodeId;

#if defined(PROTOCOL_ESPNOW)

bool applyConfig(const ControlPacket &cmd) {
  bool addressedToMe = (cmd.targetNodeId == 0xFF || cmd.targetNodeId == g_myNodeId);
  if (!addressedToMe) return true;

  switch (cmd.cmd) {
    case CMD_SET_RATE:
      g_config.sendIntervalMs = cmd.intervalMs;
      return true;
    case CMD_START:
      g_config.streamingEnabled = true;
      return true;
    case CMD_STOP:
      g_config.streamingEnabled = false;
      return true;
    default:
      return false;
  }
}

#elif defined(PROTOCOL_BLE)
bool applyConfig(const ControlPacket &cmd) { (void)cmd; return false; }
#elif defined(PROTOCOL_FSK)
bool applyConfig(const ControlPacket &cmd) { (void)cmd; return false; }
#elif defined(PROTOCOL_LORA)
bool applyConfig(const ControlPacket &cmd) { (void)cmd; return false; }
#else
#error "No PROTOCOL_* build flag defined — set one in platformio.ini (-D PROTOCOL_ESPNOW, etc.)"
#endif