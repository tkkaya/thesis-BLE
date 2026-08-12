#include <Arduino.h>
#include "packet.h"
#include "radio_sx1262.h"
#include "control_packet.h"
#include "runtime_config.h"
#include "apply_config.h"
#include "identity.h"

uint8_t g_myNodeId = 0;

static ControlPacket pendingCmd;
static bool hasPendingCmd = false;
static uint8_t pendingRetries = 0;
static uint32_t lastTelemetryTime = 0;

static bool pendingLocalReconfig = false;
static ControlPacket reconfigCmd;
static uint32_t reconfigAtMs = 0;

void pollSerialForControl() {
  static uint8_t buf[sizeof(ControlPacket)];
  static size_t bufLen = 0;

  while (Serial.available() > 0) {
    uint8_t b = Serial.read();
    if (bufLen == 0 && b != CONTROL_MAGIC) continue;
    buf[bufLen++] = b;

    if (bufLen == sizeof(buf)) {
      ControlPacket cmd;
      memcpy(&cmd, buf, sizeof(cmd));
      bufLen = 0;

      if (!controlPacketValid(cmd)) {
        Serial.println("[LoRa RX] Invalid control packet from Pi (magic/crc)");
        continue;
      }

      if (cmd.cmd == CMD_SET_LORA_PARAMS || cmd.cmd == CMD_SET_RADIO_PARAMS) {
        reconfigCmd = cmd;
        pendingLocalReconfig = true;
        reconfigAtMs = 0;
        pendingRetries = 5;
      } else {
        pendingRetries = 3;
      }

      pendingCmd = cmd;
      hasPendingCmd = true;
    }
  }
}

void dispatchPendingControl() {
  if (!hasPendingCmd) return;

  bool ok = radioSendControl(pendingCmd);
  Serial.printf("[LoRa RX] Relayed cmd=%u target=%u -> %s (retries left=%u)\n",
    pendingCmd.cmd, pendingCmd.targetNodeId,
    ok ? "OK" : "FAIL", pendingRetries > 0 ? pendingRetries - 1 : 0);

  if (--pendingRetries == 0) {
    hasPendingCmd = false;
    if (pendingLocalReconfig && reconfigAtMs == 0) {
      reconfigAtMs = millis() + 500; // let the last relay land before switching
    }
  }
}

static void applyPendingLocalReconfig() {
  if (!pendingLocalReconfig) return;
  if (reconfigAtMs == 0 || millis() < reconfigAtMs) return;

  pendingLocalReconfig = false;
  reconfigAtMs = 0;
  bool ok = applyConfig(reconfigCmd);
  Serial.printf("[LoRa RX] Local reconfig -> %s\n", ok ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(1000);

  g_myNodeId = deriveNodeId();

  if (!radioBeginLoRa(g_config.freqMHz)) {
    if (Serial) Serial.println("LoRa init failed");
    while (true) delay(1000);
  }

  if (Serial) {
    Serial.printf("[LoRa RX] id=%u listening — %.3f MHz\n", g_myNodeId, g_config.freqMHz);
  }
}

void loop() {
  pollSerialForControl();
  applyPendingLocalReconfig();

  ImuPacket pkt;
  int16_t rssi = 0, snr = 0;

  if (radioReceivePacket(pkt, rssi, snr)) {
    lastTelemetryTime = millis();

    // TX just finished a transmission and is inside its guaranteed
    // listening window — the best possible moment to relay control.
    dispatchPendingControl();

    if (Serial) {
      Serial.print("[LoRa RX] RX OK  ");
      printPacket(Serial, pkt, rssi, snr);
    }
  } else if (millis() - lastTelemetryTime > 400) {
    static uint32_t lastIdleCheck = 0;
    if (millis() - lastIdleCheck > 150) {
      lastIdleCheck = millis();
      dispatchPendingControl();
    }
  }
}