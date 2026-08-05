// control_packet.h — shared by BLE, ESP-NOW, FSK, and LoRa firmwares
#ifndef CONTROL_PACKET_H
#define CONTROL_PACKET_H

#include <cstdint>
#include <cstddef>

#define CONTROL_MAGIC 0xC0

enum CommandType : uint8_t {
  CMD_SET_RATE         = 1,
  CMD_SET_RADIO_PARAMS = 4,
  CMD_START            = 5,
  CMD_STOP             = 6,
  CMD_STATUS           = 7,
  CMD_ACK              = 8
};

// packed = no compiler padding, so this struct is byte-identical
// on the wire regardless of platform.
struct __attribute__((packed)) ControlPacket {
  uint8_t  magic;        // always CONTROL_MAGIC — quick sanity check on receipt
  uint8_t  cmd;          // one of CommandType
  uint8_t  targetNodeId; // MAC-derived target ID, or 0xFF for "all nodes"
  uint8_t  reserved0;    // padding/future use
  uint32_t intervalMs;   // desired send interval
  float    freqMHz;      // radio frequency (FSK/LoRa only, ignored otherwise)
  uint8_t  sf;           // LoRa spreading factor (ignored otherwise)
  uint8_t  bw;           // encoded bandwidth index (ignored otherwise)
  uint8_t  cr;           // LoRa coding rate (ignored otherwise)
  uint8_t  reserved1;    // padding/future use
  uint16_t crc;          // simple checksum over the preceding bytes
};

// Simple additive checksum — good enough for a control channel,
// not meant to replace transport-level integrity (BLE/LoRa already
// have their own CRC at the radio layer).
inline uint16_t controlPacketCrc(const ControlPacket &p) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t*>(&p);
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