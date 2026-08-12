#include <Arduino.h>
#include "payload_config.h"
#include "imu.h"
#include "packet.h"
#include "radio_sx1262.h"
#include "control_packet.h"
#include "apply_config.h"
#include "runtime_config.h"
#include "identity.h"
#include "esp_random.h"

static uint16_t seq = 0;
static uint32_t lastSend = 0;
uint8_t g_myNodeId = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(1000);

  g_myNodeId = deriveNodeId();

#if USE_IMU_PAYLOAD
  if (!imuBegin()) { Serial.println("MPU6050 init failed"); while (true) delay(1000); }
  Serial.println("IMU ready");
#else
  Serial.println("Dummy mode");
#endif

  if (!radioBeginLoRa(g_config.freqMHz)) {
    if (Serial) Serial.println("LoRa init failed");
    while (true) delay(1000);
  }

  if (Serial) {
    Serial.printf("[LoRa TX] id=%u ready — %.3f MHz\n", g_myNodeId, g_config.freqMHz);
    Serial.printf("[LoRa TX] airtime: telemetry=%lums control=%lums\n",
      (unsigned long)radioGetAirtimeMs(false),
      (unsigned long)radioGetAirtimeMs(true));
  }
}


void loop() {
  ControlPacket cmd;
  if (radioPollControlFrame(cmd)) {
    bool ok = applyConfig(cmd);
    bool forMe = (cmd.targetNodeId == CONTROL_BROADCAST_NODE_ID ||
                  cmd.targetNodeId == g_myNodeId ||
                  cmd.cmd == CMD_SET_LORA_PARAMS ||
                  cmd.cmd == CMD_SET_RADIO_PARAMS);
    if (Serial) {
      Serial.printf("[LoRa TX] id=%u cmd=%u target=%u %s interval=%lu streaming=%u\n",
        g_myNodeId, cmd.cmd, cmd.targetNodeId,
        forMe ? (ok ? "applied=yes" : "applied=FAILED") : "ignored(not-for-me)",
        (unsigned long)g_config.sendIntervalMs,
        g_config.streamingEnabled ? 1 : 0);
    }
  }

  if (!g_config.streamingEnabled) return;

  uint32_t interval = g_config.sendIntervalMs;
  uint32_t minInterval = radioGetAirtimeMs(false) + radioGetAirtimeMs(true) + 50;
  if (interval < minInterval) interval = minInterval;

  // Multi-node decorrelation: identical periods phase-lock nodes into
  // permanent collisions, and the capture effect lets the stronger node
  // win every time. Random per-cycle jitter breaks the lock while keeping
  // the average rate equal across nodes (fair for comparison).
  uint32_t jitter = esp_random() % (interval / 2);

  if (millis() - lastSend < interval + jitter) return;
  lastSend = millis();

  ImuPacket pkt;
#if USE_IMU_PAYLOAD
  ImuData d;
  imuRead(d);
  packImuData(d, pkt, g_myNodeId, seq++);
#else
  packDummy(pkt, g_myNodeId, seq++);
#endif

  bool txSuccess = radioSendPacket(pkt);
  if (Serial) {
    Serial.print(txSuccess ? "TX OK  " : "TX FAIL ");
    printPacket(Serial, pkt);
  }
}