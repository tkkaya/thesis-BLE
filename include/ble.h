#ifndef BLE_H
#define BLE_H

#include <Arduino.h>
#include "packet.h"

#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

bool bleServerBegin(const char *deviceName = "ESP32S3-IMU");
bool bleIsConnected();
void bleSendPacket(const ImuPacket &pkt);

#endif