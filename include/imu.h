#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

struct ImuData {
  float    ax, ay, az;        // m/s²
  float    gx, gy, gz;        // rad/s
  float    temperature;       // °C
  uint32_t timestamp_ms;
};

bool imuBegin(int sdaPin = 5, int sclPin = 6);
bool imuRead(ImuData &data);

#endif