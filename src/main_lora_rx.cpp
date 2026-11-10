#include <Arduino.h>
#include "packet.h"
#include "radio_sx1262.h"
#include "control_packet.h"
#include "runtime_config.h"
#include "apply_config.h"
#include "identity.h"
#include <cstring>

uint8_t g_myNodeId = 0;

// Keep packet counters separate for each transmitter.
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

static constexpr uint8_t CONTROL_QUEUE_SIZE = 12;
// Send queued controls one at a time so each node has a chance to receive them.
static ControlPacket controlQueue[CONTROL_QUEUE_SIZE];
static uint8_t controlQueueHead = 0;
static uint8_t controlQueueCount = 0;
static uint8_t pendingRetries = 0;
static uint32_t nextControlAtMs = 0;
static uint32_t lastTelemetryTime = 0;
static ControlPacket lastSerialControl = {};
static uint32_t lastSerialControlAtMs = 0;
static bool haveLastSerialControl = false;

static constexpr uint32_t CONTROL_DUPLICATE_WINDOW_MS = 3000;

static void clearControlQueue() {
  controlQueueHead = 0;
  controlQueueCount = 0;
  pendingRetries = 0;
  nextControlAtMs = 0;
}

// Ignore repeated copies of the same command from the host.
static bool isRecentSerialDuplicate(const ControlPacket &cmd) {
  if (!haveLastSerialControl ||
      memcmp(&lastSerialControl, &cmd, sizeof(cmd)) != 0) {
    return false;
  }

  return millis() - lastSerialControlAtMs < CONTROL_DUPLICATE_WINDOW_MS;
}

static void rememberSerialControl(const ControlPacket &cmd) {
  lastSerialControl = cmd;
  lastSerialControlAtMs = millis();
  haveLastSerialControl = true;
}

// Add a command once; report when the fixed queue has no room.
static bool enqueueControl(const ControlPacket &cmd) {
  for (uint8_t i = 0; i < controlQueueCount; ++i) {
    uint8_t index = static_cast<uint8_t>(
      (controlQueueHead + i) % CONTROL_QUEUE_SIZE);
    if (memcmp(&controlQueue[index], &cmd, sizeof(cmd)) == 0) {
      return true;
    }
  }

  if (controlQueueCount >= CONTROL_QUEUE_SIZE) {
    Serial.printf("[LoRa RX] control_queue_full cmd=%u target=%u\n",
      cmd.cmd, cmd.targetNodeId);
    return false;
  }

  uint8_t index = static_cast<uint8_t>(
    (controlQueueHead + controlQueueCount) % CONTROL_QUEUE_SIZE);
  controlQueue[index] = cmd;
  controlQueueCount++;
  return true;
}

// Turn broadcast rate/start/stop commands into one addressed send per node.
static void enqueueReceivedControl(const ControlPacket &cmd) {
  if (cmd.targetNodeId != CONTROL_BROADCAST_NODE_ID ||
      (cmd.cmd != CMD_SET_RATE && cmd.cmd != CMD_START &&
       cmd.cmd != CMD_STOP)) {
    enqueueControl(cmd);
    return;
  }

  for (uint8_t nodeId : EXPERIMENT_NODE_IDS) {
    ControlPacket addressed = cmd;
    addressed.targetNodeId = nodeId;
    controlPacketFinalize(addressed);
    enqueueControl(addressed);
  }
}

// Clear older work so STOP reaches the nodes before anything else.
static void enqueuePriorityStop(const ControlPacket &cmd) {
  clearControlQueue();

  if (cmd.targetNodeId == CONTROL_BROADCAST_NODE_ID) {
    for (uint8_t nodeId : EXPERIMENT_NODE_IDS) {
      ControlPacket addressed = cmd;
      addressed.targetNodeId = nodeId;
      controlPacketFinalize(addressed);
      enqueueControl(addressed);
    }
  } else {
    enqueueControl(cmd);
  }
}

// Read complete host commands and queue them, giving STOP priority.
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

      if (isRecentSerialDuplicate(cmd)) {
        continue;
      }
      rememberSerialControl(cmd);

      uint8_t previousDepth = controlQueueCount;
      if (cmd.cmd == CMD_STOP) {
        enqueuePriorityStop(cmd);
      } else {
        enqueueReceivedControl(cmd);
      }
      bool accepted = cmd.cmd == CMD_STOP ||
                      controlQueueCount > previousDepth;
      if (accepted) {
        Serial.printf("[LoRa RX] control_queued cmd=%u target=%u depth=%u\n",
          cmd.cmd, cmd.targetNodeId, controlQueueCount);
        if (controlQueueCount == 1 && pendingRetries == 0) {
          nextControlAtMs = 0;
        }
      }
    }
  }
}

// Retry the oldest command, then apply radio changes locally after relaying.
void dispatchPendingControl() {
  if (controlQueueCount == 0) return;
  if (nextControlAtMs != 0 && millis() < nextControlAtMs) return;

  ControlPacket &cmd = controlQueue[controlQueueHead];
  if (pendingRetries == 0) pendingRetries = 4;

  bool ok = radioSendControl(cmd);
  Serial.printf("[LoRa RX] control_tx_complete cmd=%u target=%u result=%s (retries left=%u)\n",
    cmd.cmd, cmd.targetNodeId,
    ok ? "OK" : "FAIL", pendingRetries > 0 ? pendingRetries - 1 : 0);
  if (pendingRetries == 1 && !ok) {
    Serial.println("[LoRa RX] CONTROL DELIVERY FAILED after retries");
  }

  if (--pendingRetries == 0) {
    if (cmd.cmd == CMD_SET_LORA_PARAMS || cmd.cmd == CMD_SET_RADIO_PARAMS) {
      ControlPacket completed = cmd;
      controlQueueHead = static_cast<uint8_t>(
        (controlQueueHead + 1) % CONTROL_QUEUE_SIZE);
      controlQueueCount--;
      bool reconfigOk = applyConfig(completed);
      Serial.printf("[LoRa RX] Local reconfig -> %s\n",
        reconfigOk ? "OK" : "FAIL");
    } else {
      controlQueueHead = static_cast<uint8_t>(
        (controlQueueHead + 1) % CONTROL_QUEUE_SIZE);
      controlQueueCount--;
    }
    nextControlAtMs = millis() + 500;
  } else {
    nextControlAtMs = millis() + 150;
  }
}

// Remove a queued command only when its target and command match the ACK.
static bool completeControlFromAck(const ControlPacket &ack) {
  if (controlQueueCount == 0 || pendingRetries == 0) return false;

  ControlPacket &pending = controlQueue[controlQueueHead];
  if (ack.targetNodeId != pending.targetNodeId ||
      ack.reserved0 != pending.cmd) {
    return false;
  }

  Serial.printf("[LoRa RX] control_delivered cmd=%u target=%u\n",
    pending.cmd, pending.targetNodeId);
  ControlPacket completed = pending;
  controlQueueHead = static_cast<uint8_t>(
    (controlQueueHead + 1) % CONTROL_QUEUE_SIZE);
  controlQueueCount--;
  pendingRetries = 0;
  nextControlAtMs = millis() + 100;
  if (completed.cmd == CMD_SET_LORA_PARAMS ||
      completed.cmd == CMD_SET_RADIO_PARAMS) {
    bool reconfigOk = applyConfig(completed);
    Serial.printf("[LoRa RX] Local reconfig -> %s\n",
      reconfigOk ? "OK" : "FAIL");
  }
  return true;
}

// Set this node's ID and start the LoRa receiver.
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

// Handle ACKs and telemetry while steadily draining the control queue.
void loop() {
  pollSerialForControl();

  ImuPacket pkt;
  int16_t rssi = 0, snr = 0;

  ControlPacket ack;
  // Check for command ACKs before telemetry; both arrive on the same radio.
  if (radioReceiveControlFrame(ack)) {
    if (ack.cmd == CMD_ACK) {
      Serial.printf("[LoRa RX] control_ack node=%u cmd=%u\n",
        ack.targetNodeId, ack.reserved0);
      completeControlFromAck(ack);
    }
  } else if (radioReceivePacket(pkt, rssi, snr)) {
    lastTelemetryTime = millis();
    PacketTracker *tracker = trackerForNode(pkt.node_id);
    if (tracker != nullptr) trackPacket(*tracker, pkt);

    dispatchPendingControl();

    if (Serial) {
      Serial.print("[LoRa RX] RX OK  ");
      printPacket(Serial, pkt, rssi, snr);
      Serial.printf(" rx_ms=%lu node_received=%lu node_lost=%lu node_dup=%lu node_ooo=%lu\n",
        static_cast<unsigned long>(millis()),
        tracker != nullptr ? static_cast<unsigned long>(tracker->received) : 0UL,
        tracker != nullptr ? static_cast<unsigned long>(tracker->lost) : 0UL,
        tracker != nullptr ? static_cast<unsigned long>(tracker->duplicates) : 0UL,
        tracker != nullptr ? static_cast<unsigned long>(tracker->outOfOrder) : 0UL);
    }
  } else if (millis() - lastTelemetryTime > 400) {
    static uint32_t lastIdleCheck = 0;
    if (millis() - lastIdleCheck > 100) {
      lastIdleCheck = millis();
      dispatchPendingControl();
    }
  }
}