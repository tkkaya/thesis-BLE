#include <Arduino.h>
#include "payload_config.h"
#include "imu.h"
#include "packet.h"
#include "radio_sx1262.h"
#include "control_packet.h"
#include "apply_config.h"
#include "runtime_config.h"
#include "identity.h"

static uint16_t seq = 0;
static uint32_t nextSendAt = 0;
static uint32_t phaseOffsetMs = 0;
static bool scheduleInitialized = false;
static bool imuReady = false;

uint8_t g_myNodeId = 0;

// Give each known test node its own first-packet phase.
static uint8_t nodeSlot(uint8_t nodeId) {
  if (nodeId == 24) return 0;
  if (nodeId == 88) return 1;
  if (nodeId == 180) return 2;
  return static_cast<uint8_t>((nodeId ^ (nodeId >> 3)) % 3);
}

// Offset only the first packet; later sends stay on the requested rate.
static uint32_t telemetrySlotMs() {
  uint32_t interval = g_config.sendIntervalMs;
  if (interval == 0) interval = 1;

  if (!scheduleInitialized) {
    // Spread the first transmission without changing the requested
    // reporting period. The phase must not be added to every period.
    phaseOffsetMs = (interval / 3) * nodeSlot(g_myNodeId);
    nextSendAt = millis() + phaseOffsetMs;
    scheduleInitialized = true;
  }

  return phaseOffsetMs;
}

// Set this node's ID and start the sensor and GFSK radio.
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(1000);

  g_myNodeId = deriveNodeId();

#if USE_IMU_PAYLOAD
  imuReady = imuBegin();
  if (imuReady) {
    if (Serial) Serial.println("IMU ready");
  } else {
    if (Serial) Serial.println("MPU6050 init failed; GFSK control remains available");
  }
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

// Apply controls, then send telemetry on this node's assigned phase.
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
    if (cmd.cmd == CMD_SET_RATE || cmd.cmd == CMD_START ||
        cmd.cmd == CMD_STOP) {
      // Start a fresh first-packet phase after rate or stream changes.
      scheduleInitialized = false;
      nextSendAt = 0;
    }

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
  telemetrySlotMs();
  uint32_t interval = g_config.sendIntervalMs;
  if (interval == 0) interval = 1;
  uint32_t now = millis();
  uint32_t wait = static_cast<uint32_t>(now - nextSendAt);
  if (static_cast<int32_t>(wait) < 0) return;
  nextSendAt += interval;

  ImuPacket pkt;

#if USE_IMU_PAYLOAD
  if (imuReady) {
    ImuData d;
    imuRead(d);
    packImuData(d, pkt, g_myNodeId, seq++);
  } else {
    packDummy(pkt, g_myNodeId, seq++);
  }
#else
  packDummy(pkt, g_myNodeId, seq++);
#endif

  bool txSuccess = radioSendPacket(pkt);

  if (Serial) {
    Serial.print(txSuccess ? "TX OK  " : "TX FAIL ");
    printPacket(Serial, pkt);
  }
}