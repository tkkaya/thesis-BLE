// control_packet.h — shared by BLE, ESP-NOW, FSK, and LoRa firmwares
#ifndef CONTROL_PACKET_H
#define CONTROL_PACKET_H

#include <cstdint>
#include <cstddef>

#define CONTROL_MAGIC 0xC0
#define CONTROL_BROADCAST_NODE_ID 0xFF

enum CommandType : uint8_t {
  CMD_SET_RATE         = 1,
  CMD_SET_RADIO_PARAMS = 4,   // Legacy/generic radio command; keep for compatibility
  CMD_START            = 5,
  CMD_STOP             = 6,
  CMD_STATUS           = 7,
  CMD_ACK              = 8,
  CMD_SET_GFSK_PARAMS  = 9,
  CMD_SET_LORA_PARAMS  = 10,
  CMD_SET_NODE_COUNT     = 11 // ble only

};

// GFSK-specific radio parameters.
struct __attribute__((packed)) GfskParams {
  uint32_t bitrateBps;      // e.g. 50000
  uint16_t preambleBits;    // e.g. 128
  uint8_t  deviationKhz;    // e.g. 25
  uint8_t  rxBwIndex;       // index into firmware RX bandwidth table
  uint8_t  reserved;
};

// LoRa-specific radio parameters.
struct __attribute__((packed)) LoraParams {
  uint8_t sf;               // spreading factor
  uint8_t bwIndex;          // encoded bandwidth index
  uint8_t crIndex;          // coding-rate index
  uint8_t syncWord;
  int8_t  txPowerDbm;
  uint8_t preambleSymbols;
  uint8_t reserved[2];
};

// packed = no compiler padding, so this struct is byte-identical
// on the wire regardless of platform.
struct __attribute__((packed)) ControlPacket {
  uint8_t  magic;           // always CONTROL_MAGIC
  uint8_t  cmd;             // one of CommandType
  uint8_t  targetNodeId;    // MAC-derived target ID, or CONTROL_BROADCAST_NODE_ID
  uint8_t  reserved0;

  uint32_t intervalMs;      // desired telemetry send interval
  float    freqMHz;         // carrier frequency for GFSK/LoRa

  GfskParams gfsk;          // used by CMD_SET_GFSK_PARAMS
  LoraParams lora;          // used by CMD_SET_LORA_PARAMS

  uint16_t crc;             // checksum over all preceding bytes
};

inline uint16_t controlPacketCrc(const ControlPacket &p) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&p);
  uint16_t sum = 0;

  for (size_t i = 0; i < sizeof(ControlPacket) - sizeof(p.crc); i++) {
    sum += bytes[i];
  }

  return sum;
}

inline void controlPacketFinalize(ControlPacket &p) {
  p.magic = CONTROL_MAGIC;
  p.crc = controlPacketCrc(p);
}

inline bool controlPacketValid(const ControlPacket &p) {
  return p.magic == CONTROL_MAGIC && p.crc == controlPacketCrc(p);
}

#endif