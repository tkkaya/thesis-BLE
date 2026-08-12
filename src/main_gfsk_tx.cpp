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

static uint32_t telemetrySlotMs() {
  uint32_t interval = g_config.sendIntervalMs;
  if (interval == 0) interval = 1;

  // Stable per-node phase, useful when comparing synchronized nodes.
  uint32_t nodePhase = (g_myNodeId * 37UL) % interval;

  // Small non-deterministic component to avoid two same-phase nodes
  // staying permanently locked together.
  uint32_t randomPhase = esp_random() % (interval < 20 ? interval : 20);

  return (nodePhase + randomPhase) % interval;
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(1000);

  g_myNodeId = deriveNodeId();

#if USE_IMU_PAYLOAD
  if (!imuBegin()) {
    if (Serial) Serial.println("MPU6050 init failed");
    while (true) delay(1000);
  }
  if (Serial) Serial.println("IMU ready");
#else
  if (Serial) Serial.println("Dummy mode");
#endif

  if (!radioBeginGFSK(g_config.freqMHz)) {
    if (Serial) Serial.println("GFSK init failed");
    while (true) delay(1000);
  }

  if (Serial) {
    Serial.printf("[GFSK TX] id=%u ready — %.3f MHz\n",
      g_myNodeId, g_config.freqMHz);
  }
}

void loop() {
  static uint32_t lastIsrPrint = 0;

  if (millis() - lastIsrPrint > 2000) {
    lastIsrPrint = millis();
    Serial.printf("[TX DEBUG] id=%u isrFireCount=%lu streaming=%u interval=%lu\n",
      g_myNodeId,
      radioGetIsrFireCount(),
      g_config.streamingEnabled ? 1 : 0,
      static_cast<unsigned long>(g_config.sendIntervalMs));
  }

  ControlPacket cmd;
  if (radioPollControlFrame(cmd)) {
    bool ok = applyConfig(cmd);

    if (Serial) {
      Serial.printf(
        "[GFSK TX] id=%u cmd=%u target=%u addressed=%s applied=%s "
        "interval=%lu streaming=%u freq=%.3f bitrate=%lu\n",
        g_myNodeId,
        cmd.cmd,
        cmd.targetNodeId,
        (cmd.targetNodeId == CONTROL_BROADCAST_NODE_ID ||
         cmd.targetNodeId == g_myNodeId ||
         cmd.cmd == CMD_SET_GFSK_PARAMS) ? "yes" : "no",
        ok ? "yes" : "no",
        static_cast<unsigned long>(g_config.sendIntervalMs),
        g_config.streamingEnabled ? 1 : 0,
        g_config.freqMHz,
        static_cast<unsigned long>(g_config.gfsk.bitrateBps)
      );
    }
  }

  if (!g_config.streamingEnabled) return;
  if (millis() - lastSend < g_config.sendIntervalMs + telemetrySlotMs()) return;
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