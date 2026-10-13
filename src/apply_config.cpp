// src/apply_config.cpp — single shared implementation for all protocols
#include "apply_config.h"
#include "runtime_config.h"
#include "identity.h"
#include "radio_sx1262.h"

RuntimeConfig g_config;
extern uint8_t g_myNodeId;

static bool isAddressedToMe(uint8_t targetNodeId) {
  return targetNodeId == CONTROL_BROADCAST_NODE_ID ||
         targetNodeId == g_myNodeId;
}

// Each build includes only the command handler for its radio protocol.
#if defined(PROTOCOL_ESPNOW)

bool applyConfig(const ControlPacket &cmd) {
  if (!isAddressedToMe(cmd.targetNodeId)) return true;

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

bool applyConfig(const ControlPacket &cmd) {
  if (!isAddressedToMe(cmd.targetNodeId)) return true;

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

#elif defined(PROTOCOL_FSK)

bool applyConfig(const ControlPacket &cmd) {
  // Both link endpoints must use the same physical-layer parameters.
  // Apply link-wide GFSK parameter updates even when a specific TX node
  // was addressed, because the attached RX node must follow as well.
  if (cmd.cmd == CMD_SET_GFSK_PARAMS) {
    g_config.freqMHz = cmd.freqMHz;
    g_config.gfsk = cmd.gfsk;
    return radioBeginGFSK(g_config.freqMHz);
  }

  if (!isAddressedToMe(cmd.targetNodeId)) return true;

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

#elif defined(PROTOCOL_LORA)

bool applyConfig(const ControlPacket &cmd) {
  if (cmd.cmd == CMD_SET_LORA_PARAMS ||
      cmd.cmd == CMD_SET_RADIO_PARAMS) {
    g_config.freqMHz = cmd.freqMHz;
    g_config.lora = cmd.lora;
    return radioBeginLoRa(g_config.freqMHz);
  }

  if (!isAddressedToMe(cmd.targetNodeId)) return true;

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

#else
#error "No PROTOCOL_* build flag defined — set one in platformio.ini"
#endif