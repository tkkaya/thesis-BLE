#include <Arduino.h>
#include "payload_config.h"
#include "imu.h"
#include "packet.h"
#include "ble.h"
#include "control_packet.h"
#include "runtime_config.h"
#include "apply_config.h"
#include "identity.h"

static uint16_t seq = 0;
static uint32_t lastSend = 0;
uint8_t g_myNodeId = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  g_myNodeId = deriveNodeId();

#if USE_IMU_PAYLOAD
  if (!imuBegin()) { Serial.println("MPU6050 init failed"); while (true) delay(1000); }
  Serial.println("IMU ready");
#else
  Serial.println("Dummy mode");
#endif

  char name[24];
  snprintf(name, sizeof(name), "ESP32S3-IMU-%u", g_myNodeId);
  bleServerBegin(name);
  Serial.printf("[BLE TX] id=%u advertising as '%s'\n", g_myNodeId, name);
}

void loop() {
  ControlPacket cmd;
  if (blePollControl(cmd)) {
    if (controlPacketValid(cmd)) {
      bool ok = applyConfig(cmd);
      Serial.printf("[BLE TX] id=%u cmd=%u target=%u applied=%s streaming=%u interval=%lu\n",
        g_myNodeId, cmd.cmd, cmd.targetNodeId, ok ? "yes" : "no",
        g_config.streamingEnabled ? 1 : 0, (unsigned long)g_config.sendIntervalMs);
    }
  }

  if (!bleIsConnected() || !g_config.streamingEnabled) return;
  if (millis() - lastSend < g_config.sendIntervalMs) return;
  lastSend = millis();

  ImuPacket pkt;
#if USE_IMU_PAYLOAD
  ImuData d; imuRead(d); packImuData(d, pkt, g_myNodeId, seq++);
#else
  packDummy(pkt, g_myNodeId, seq++);
#endif
  bleSendPacket(pkt);
}
