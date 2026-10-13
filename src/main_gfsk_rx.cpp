#include <Arduino.h>
#include "packet.h"
#include "radio_sx1262.h"
#include "control_packet.h"
#include "runtime_config.h"
#include "apply_config.h"
#include "identity.h"

// The RX identity is used for addressed commands and logging.
uint8_t g_myNodeId = 0;

// Keep packet counts for each transmitter separate.
struct NodeTracker {
  bool inUse = false;
  uint8_t nodeId = 0;
  PacketTracker packets;
};

static NodeTracker nodeTrackers[3];

static PacketTracker *trackerForNode(uint8_t nodeId) {
  for (NodeTracker &entry : nodeTrackers) {
    if (entry.inUse && entry.nodeId == nodeId) return &entry.packets;
  }

  for (NodeTracker &entry : nodeTrackers) {
    if (!entry.inUse) {
      entry.inUse = true;
      entry.nodeId = nodeId;
      entry.packets = {};
      return &entry.packets;
    }
  }

  return nullptr;
}

static ControlPacket pendingCmd;
static bool hasPendingCmd = false;
static uint8_t pendingRetries = 0;
static uint32_t lastTelemetryTime = 0;

static bool pendingLocalGfskReconfig = false;
static ControlPacket localGfskReconfigCmd;
static uint32_t localGfskReconfigAtMs = 0;

// Send radio changes several times on the old settings before switching this RX.
static void queueControlFromSerial(const ControlPacket &cmd) {
  pendingCmd = cmd;
  hasPendingCmd = true;

  if (cmd.cmd == CMD_SET_GFSK_PARAMS) {
    // Send the update repeatedly while still using the old radio settings.
    pendingRetries = 5;

    localGfskReconfigCmd = cmd;
    pendingLocalGfskReconfig = true;
    localGfskReconfigAtMs = 0;

    Serial.printf(
      "[GFSK RX] Queued GFSK reconfig: freq=%.3f bitrate=%lu dev=%u "
      "rxBwIdx=%u preamble=%u\n",
      cmd.freqMHz,
      static_cast<unsigned long>(cmd.gfsk.bitrateBps),
      cmd.gfsk.deviationKhz,
      cmd.gfsk.rxBwIndex,
      cmd.gfsk.preambleBits
    );
  } else {
    pendingRetries = 3;
  }
}

void pollSerialForControl() {
  static uint8_t buf[sizeof(ControlPacket)];
  static size_t bufLen = 0;

  while (Serial.available() > 0) {
    uint8_t b = Serial.read();

    // Recover from banners/noise by requiring the magic byte first.
    if (bufLen == 0 && b != CONTROL_MAGIC) {
      continue;
    }

    buf[bufLen++] = b;

    if (bufLen == sizeof(buf)) {
      ControlPacket cmd;
      memcpy(&cmd, buf, sizeof(cmd));
      bufLen = 0;

      if (!controlPacketValid(cmd)) {
        Serial.println("[GFSK RX] Invalid control packet from Pi (magic/crc)");
        continue;
      }

      queueControlFromSerial(cmd);
    }
  }
}

// Send a queued command when a transmitter is likely listening for it.
void dispatchPendingControl() {
  if (!hasPendingCmd) return;

  bool ok = radioSendControl(pendingCmd);
  bool finalAttempt = pendingRetries == 1;

  Serial.printf(
    "[GFSK RX] Relayed cmd=%u target=%u interval=%lu -> %s (retries left=%u)\n",
    pendingCmd.cmd,
    pendingCmd.targetNodeId,
    static_cast<unsigned long>(pendingCmd.intervalMs),
    ok ? "OK" : "FAIL",
    pendingRetries > 0 ? pendingRetries - 1 : 0
  );
  if (finalAttempt && !ok) {
    Serial.println("[GFSK RX] CONTROL DELIVERY FAILED after retries");
  }

  if (--pendingRetries == 0) {
    hasPendingCmd = false;

    if (pendingLocalGfskReconfig && localGfskReconfigAtMs == 0) {
      // Give the final relayed frame time to leave the air and be processed.
      localGfskReconfigAtMs = millis() + 300;
    }
  }
}

// Change the RX settings only after the final old-settings command can arrive.
static void applyPendingLocalGfskReconfig() {
  if (!pendingLocalGfskReconfig) return;
  if (localGfskReconfigAtMs == 0) return;
  if (millis() < localGfskReconfigAtMs) return;

  pendingLocalGfskReconfig = false;
  localGfskReconfigAtMs = 0;

  bool ok = applyConfig(localGfskReconfigCmd);

  Serial.printf("[GFSK RX] Local GFSK reconfig -> %s\n",
    ok ? "OK" : "FAIL");
}

// Set this node's ID and start the radio in GFSK receive mode.
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(1000);

  g_myNodeId = deriveNodeId();

  if (!radioBeginGFSK(g_config.freqMHz)) {
    if (Serial) Serial.println("GFSK init failed");
    while (true) delay(1000);
  }

  if (Serial) {
    Serial.printf("[GFSK RX] id=%u listening — %.3f MHz\n",
      g_myNodeId, g_config.freqMHz);
  }
}

// Handle controls, apply delayed radio changes, and receive telemetry.
void loop() {
  pollSerialForControl();
  applyPendingLocalGfskReconfig();

  ImuPacket pkt;
  int16_t rssi = 0;
  int16_t snr = 0;

  if (radioReceivePacket(pkt, rssi, snr)) {
    lastTelemetryTime = millis();
    PacketTracker *tracker = trackerForNode(pkt.node_id);
    if (tracker != nullptr) trackPacket(*tracker, pkt);

    // A TX node normally returns to RX immediately after telemetry.
    dispatchPendingControl();

    if (Serial) {
      Serial.print("[GFSK RX] RX OK  ");
      printPacket(Serial, pkt, rssi, snr);
      Serial.printf(" rx_ms=%lu node_received=%lu node_lost=%lu node_dup=%lu node_ooo=%lu\n",
        static_cast<unsigned long>(millis()),
        tracker != nullptr ? static_cast<unsigned long>(tracker->received) : 0UL,
        tracker != nullptr ? static_cast<unsigned long>(tracker->lost) : 0UL,
        tracker != nullptr ? static_cast<unsigned long>(tracker->duplicates) : 0UL,
        tracker != nullptr ? static_cast<unsigned long>(tracker->outOfOrder) : 0UL);
    }
  } else if (millis() - lastTelemetryTime > 400) {
    // No recent telemetry: nodes may be stopped or idle.
    static uint32_t lastIdleDispatch = 0;

    if (millis() - lastIdleDispatch > 150) {
      lastIdleDispatch = millis();
      dispatchPendingControl();
    }
  }
}