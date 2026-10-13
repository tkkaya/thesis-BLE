  #ifndef PACKET_H
  #define PACKET_H

  #include <Arduino.h>
  #include "imu.h"

  #pragma pack(push, 1)
  // Compact, fixed-size sensor report sent by every protocol.
  struct ImuPacket {
    uint8_t  version;       // 1 = real IMU, 0xAA = dummy
    uint8_t  node_id;
    uint16_t seq;
    int16_t  ax_mg;         // milli-g
    int16_t  ay_mg;
    int16_t  az_mg;
    int16_t  gx_dps10;      // 0.1 deg/s
    int16_t  gy_dps10;
    int16_t  gz_dps10;
    int16_t  temp_c10;      // 0.1 °C
    uint32_t timestamp_ms;
  };
  #pragma pack(pop)

  // Keep delivery counts separate for each sender.
  struct PacketTracker {
    bool initialized = false;
    uint16_t lastSeq = 0;
    uint32_t received = 0;
    uint32_t lost = 0;
    uint32_t duplicates = 0;
    uint32_t outOfOrder = 0;
  };

  void packImuData(const ImuData &in, ImuPacket &out, uint8_t nodeId, uint16_t seq);
  void packDummy   (ImuPacket &out, uint8_t nodeId, uint16_t seq);
  void printPacket (Stream &s, const ImuPacket &pkt, int16_t rssi = 0, int16_t snr = 0);
  void trackPacket(PacketTracker &tracker, const ImuPacket &pkt);

  #endif