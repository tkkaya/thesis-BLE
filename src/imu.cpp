#include "imu.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

static Adafruit_MPU6050 mpu;

// Start the sensor bus and set ranges used by the packet format.
bool imuBegin(int sdaPin, int sclPin) {
  Wire.begin(sdaPin, sclPin);
  if (!mpu.begin()) return false;
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  return true;
}

// Copy one sensor sample and the time it was read.
void imuRead(ImuData &data) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  data.ax           = a.acceleration.x;
  data.ay           = a.acceleration.y;
  data.az           = a.acceleration.z;
  data.gx           = g.gyro.x;
  data.gy           = g.gyro.y;
  data.gz           = g.gyro.z;
  data.temperature  = temp.temperature;
  data.timestamp_ms = millis();
}