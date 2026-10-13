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
static bool scheduleInitialized = false;
static uint32_t startHoldUntil = 0;
uint8_t g_myNodeId = 0;
static bool imuReady = false;

static uint8_t nodeSlot(uint8_t nodeId) {
  // Stable assignments for the three boards used in the experiment.
  if (nodeId == 48) return 0;
  if (nodeId == 88) return 1;
  if (nodeId == 180) return 2;
  return static_cast<uint8_t>((nodeId ^ (nodeId >> 3)) % 3);
}

static void initializeSchedule() {
  uint32_t interval = g_config.sendIntervalMs;
  if (interval == 0) interval = 1;

  // Spread the first packet across the frame. Subsequent packets use
  // exactly the requested interval and never add this phase again.
  uint32_t slotWidth = interval / 3;
  uint32_t phase = slotWidth * nodeSlot(g_myNodeId);
  nextSendAt = millis() + phase;
  scheduleInitialized = true;
}

// Set this node's ID and prepare its sensor and LoRa radio.
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(1000);

  g_myNodeId = deriveNodeId();

#if USE_IMU_PAYLOAD
  imuReady = imuBegin();
  if (imuReady) {
    Serial.println("IMU ready");
  } else {
    Serial.println("MPU6050 init failed; LoRa control remains available");
  }
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


// Apply radio commands first, then send telemetry on this node's schedule.
void loop() {
  ControlPacket cmd;
  if (radioPollControlFrame(cmd)) {
    bool ok = applyConfig(cmd);
    bool forMe = (cmd.targetNodeId == CONTROL_BROADCAST_NODE_ID ||
                  cmd.targetNodeId == g_myNodeId ||
                  cmd.cmd == CMD_SET_LORA_PARAMS ||
                  cmd.cmd == CMD_SET_RADIO_PARAMS);
    if (cmd.cmd == CMD_SET_RATE || cmd.cmd == CMD_START ||
        cmd.cmd == CMD_STOP) {
      // Start a fresh first-packet phase after rate or stream changes.
      scheduleInitialized = false;
      nextSendAt = 0;
      if (cmd.cmd == CMD_START && forMe) {
        // Keep the radio listening while the RX delivers START to the
        // remaining nodes in the broadcast sequence.
        startHoldUntil = millis() + 1000;
      } else if (cmd.cmd == CMD_STOP && forMe) {
        startHoldUntil = 0;
      }
    }
    if (Serial) {
      Serial.printf("[LoRa TX] id=%u cmd=%u target=%u %s interval=%lu streaming=%u\n",
        g_myNodeId, cmd.cmd, cmd.targetNodeId,
        forMe ? (ok ? "applied=yes" : "applied=FAILED") : "ignored(not-for-me)",
        (unsigned long)g_config.sendIntervalMs,
        g_config.streamingEnabled ? 1 : 0);
    }

    // A short ACK makes command delivery observable without changing the
    // telemetry packet used by the benchmark.
    if (forMe && ok && cmd.targetNodeId != CONTROL_BROADCAST_NODE_ID &&
        (cmd.cmd == CMD_SET_RATE || cmd.cmd == CMD_START ||
         cmd.cmd == CMD_STOP)) {
      ControlPacket ack = {};
      ack.cmd = CMD_ACK;
      ack.targetNodeId = g_myNodeId;
      // Tell the RX which command this ACK confirms.
      ack.reserved0 = cmd.cmd;
      controlPacketFinalize(ack);
      radioSendControl(ack);
    }
  }

  if (!g_config.streamingEnabled) return;
  if (static_cast<int32_t>(millis() - startHoldUntil) < 0) return;
  if (!scheduleInitialized) initializeSchedule();

  uint32_t interval = g_config.sendIntervalMs;
  if (interval == 0) interval = 1;
  uint32_t now = millis();
  if (static_cast<int32_t>(now - nextSendAt) < 0) return;
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