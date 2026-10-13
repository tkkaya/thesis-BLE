// Node identity helper for firmware roles that use MAC-derived addresses.
#ifndef IDENTITY_H
#define IDENTITY_H

#include <esp_mac.h>
#include <cstdint>

// Derives a stable 1-byte identity from this board's factory-burned
// station MAC address. Using only the final byte is convenient for
// experiment logs, but does not guarantee uniqueness across boards.
inline uint8_t deriveNodeId() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  return mac[5];
}

#endif