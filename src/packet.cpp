#include "packet.h"

// Keep scaled readings inside the packet's signed 16-bit fields.
static int16_t c16(long v) {
  if (v >  32767) return  32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

// Convert sensor units to compact integers for the radio packet.
void packImuData(const ImuData &in, ImuPacket &out, uint8_t nodeId, uint16_t seq) {
  out.version      = 1;
  out.node_id      = nodeId;
  out.seq          = seq;
  out.ax_mg        = c16((long)(in.ax * 1000.0f / 9.80665f));
  out.ay_mg        = c16((long)(in.ay * 1000.0f / 9.80665f));
  out.az_mg        = c16((long)(in.az * 1000.0f / 9.80665f));
  out.gx_dps10     = c16((long)(in.gx * 57.29578f * 10.0f));
  out.gy_dps10     = c16((long)(in.gy * 57.29578f * 10.0f));
  out.gz_dps10     = c16((long)(in.gz * 57.29578f * 10.0f));
  out.temp_c10     = c16((long)(in.temperature * 10.0f));
  out.timestamp_ms = in.timestamp_ms;
}

// Make a recognizable test packet when no IMU is available.
void packDummy(ImuPacket &out, uint8_t nodeId, uint16_t seq) {
  out.version      = 0xAA;
  out.node_id      = nodeId;
  out.seq          = seq;
  out.ax_mg        = 1000;
  out.ay_mg        = 2000;
  out.az_mg        = 3000;
  out.gx_dps10     = 100;
  out.gy_dps10     = 200;
  out.gz_dps10     = 300;
  out.temp_c10     = 250;
  out.timestamp_ms = millis();
}

// Print the packet in the same readable field format for every radio.
void printPacket(Stream &s, const ImuPacket &pkt, int16_t rssi, int16_t snr) {
  s.print(pkt.version == 0xAA ? "[DUMMY]" : "[IMU]");
  s.print(" node=");  s.print(pkt.node_id);
  s.print(" seq=");   s.print(pkt.seq);
  s.print(" ax=");    s.print(pkt.ax_mg);
  s.print(" ay=");    s.print(pkt.ay_mg);
  s.print(" az=");    s.print(pkt.az_mg);
  s.print(" gx=");    s.print(pkt.gx_dps10);
  s.print(" gy=");    s.print(pkt.gy_dps10);
  s.print(" gz=");    s.print(pkt.gz_dps10);
  s.print(" t=");     s.print(pkt.temp_c10);
  s.print(" ts=");    s.print(pkt.timestamp_ms);
  if (rssi != 0) { s.print(" RSSI="); s.print(rssi); }
  if (snr  != 0) { s.print(" SNR=");  s.print(snr);  }
  s.println();
}

// Use sequence numbers to count missing, repeated, and late packets.
void trackPacket(PacketTracker &tracker, const ImuPacket &pkt) {
  tracker.received++;

  if (!tracker.initialized) {
    tracker.initialized = true;
    tracker.lastSeq = pkt.seq;
    return;
  }

  uint16_t delta = static_cast<uint16_t>(pkt.seq - tracker.lastSeq);
  if (delta == 0) {
    tracker.duplicates++;
  } else if (delta < 0x8000) {
    tracker.lost += static_cast<uint32_t>(delta - 1);
    tracker.lastSeq = pkt.seq;
  } else {
    tracker.outOfOrder++;
  }
}
