// include/identity.h — shared by every TX and RX firmware
#ifndef IDENTITY_H
#define IDENTITY_H

#include <esp_mac.h>
#include <cstdint>

// Derives a stable, unique 1-byte identity from this board's own
// factory-burned MAC address. No manual assignment, no build flags —
// the same compiled firmware produces a different, consistent ID
// on every physical board it runs on.
inline uint8_t deriveNodeId() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  return mac[5];
}

#endif