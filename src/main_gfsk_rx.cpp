#include <Arduino.h>
#include "packet.h"
#include "radio_sx1262.h"
#include "control_packet.h"
#include "runtime_config.h"
#include "apply_config.h"
#include "identity.h"

// RX node also has an identity, mainly for logging/consistency.
uint8_t g_myNodeId = 0;

static ControlPacket pendingCmd;
static bool hasPendingCmd = false;
static uint8_t pendingRetries = 0;
static uint32_t lastTelemetryTime = 0;

static bool pendingLocalGfskReconfig = false;
static ControlPacket localGfskReconfigCmd;
static uint32_t localGfskReconfigAtMs = 0;

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

void dispatchPendingControl() {
  if (!hasPendingCmd) return;

  bool ok = radioSendControl(pendingCmd);

  Serial.printf(
    "[GFSK RX] Relayed cmd=%u target=%u interval=%lu -> %s (retries left=%u)\n",
    pendingCmd.cmd,
    pendingCmd.targetNodeId,
    static_cast<unsigned long>(pendingCmd.intervalMs),
    ok ? "OK" : "FAIL",
    pendingRetries > 0 ? pendingRetries - 1 : 0
  );

  if (--pendingRetries == 0) {
    hasPendingCmd = false;

    if (pendingLocalGfskReconfig && localGfskReconfigAtMs == 0) {
      // Give the final relayed frame time to leave the air and be processed.
      localGfskReconfigAtMs = millis() + 300;
    }
  }
}

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

void loop() {
  pollSerialForControl();
  applyPendingLocalGfskReconfig();

  ImuPacket pkt;
  int16_t rssi = 0;
  int16_t snr = 0;

  if (radioReceivePacket(pkt, rssi, snr)) {
    lastTelemetryTime = millis();

    // A TX node normally returns to RX immediately after telemetry.
    dispatchPendingControl();

    if (Serial) {
      Serial.print("[GFSK RX] RX OK  ");
      printPacket(Serial, pkt, rssi, snr);
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