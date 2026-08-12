#include "radio_sx1262.h"
#include <RadioLib.h>
#include <SPI.h>
#include "frame.h"
#include "runtime_config.h"

static SX1262 radio =
  new Module(SX1262_NSS, SX1262_DIO1, SX1262_NRST, SX1262_BUSY);

static bool loraMode = true;
static float activeFreqMHz = 868.0;

static volatile bool rxFlag = false;
static volatile uint32_t isrFireCount = 0;

IRAM_ATTR static void setRxFlag() {
  rxFlag = true;
  isrFireCount++;
}

uint32_t radioGetIsrFireCount() {
  return isrFireCount;
}

static void radioHardwareInit() {
  pinMode(SX1262_ANT_SW, OUTPUT);

  // Wio-SX1262 RF switch:
  // HIGH enables the receive path; LOW is used during transmission.
  digitalWrite(SX1262_ANT_SW, HIGH);

  SPI.begin(SX1262_SCK, SX1262_MISO, SX1262_MOSI, SX1262_NSS);
}

// ─── Bandwidth / coding-rate lookup tables ──────────────────────────────────

// SX1262 discrete GFSK RX bandwidths in kHz.
static const float GFSK_RX_BW_TABLE_KHZ[] = {
  4.8, 5.8, 7.3, 9.7, 11.7, 14.6, 19.5, 23.4,
  29.3, 39.0, 46.9, 58.6, 78.2, 93.8, 117.3,
  156.2, 187.2, 234.3, 312.0, 373.6, 467.0
};
static constexpr size_t GFSK_RX_BW_TABLE_SIZE =
  sizeof(GFSK_RX_BW_TABLE_KHZ) / sizeof(GFSK_RX_BW_TABLE_KHZ[0]);

static float gfskRxBandwidthFromIndex(uint8_t index) {
  if (index >= GFSK_RX_BW_TABLE_SIZE) {
    index = 15; // safe default: 156.2 kHz
  }
  return GFSK_RX_BW_TABLE_KHZ[index];
}

// SX1262 LoRa bandwidths in kHz, indexed 0..9.
static const float LORA_BW_TABLE_KHZ[] = {
  7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, 500.0
};
static constexpr size_t LORA_BW_TABLE_SIZE =
  sizeof(LORA_BW_TABLE_KHZ) / sizeof(LORA_BW_TABLE_KHZ[0]);

static float loraBandwidthFromIndex(uint8_t index) {
  if (index >= LORA_BW_TABLE_SIZE) index = 7; // default: 125 kHz
  return LORA_BW_TABLE_KHZ[index];
}

// RadioLib setCodingRate() expects the raw denominator (5..8 = 4/5..4/8).
// crIndex 0..3 maps onto that range.
static uint8_t loraCodingRateFromIndex(uint8_t index) {
  if (index > 3) index = 0; // default: 4/5
  return 5 + index;
}

static void enterReceiveMode() {
  digitalWrite(SX1262_ANT_SW, HIGH);
  rxFlag = false;
  radio.setPacketReceivedAction(setRxFlag);
  radio.startReceive();
}

// ─── LoRa ───────────────────────────────────────────────────────────────────

bool radioBeginLoRa(float freqMHz) {
  loraMode = true;
  activeFreqMHz = freqMHz;
  radioHardwareInit();

  int state = radio.begin(freqMHz);
  if (state != RADIOLIB_ERR_NONE) {
    if (Serial) {
      Serial.print("LoRa init err: ");
      Serial.println(state);
    }
    return false;
  }

  state = radio.setSpreadingFactor(g_config.lora.sf);
  if (state != RADIOLIB_ERR_NONE && Serial) {
    Serial.printf("[LoRa] setSpreadingFactor(%u) error: %d\n",
      g_config.lora.sf, state);
  }

  state = radio.setBandwidth(loraBandwidthFromIndex(g_config.lora.bwIndex));
  if (state != RADIOLIB_ERR_NONE && Serial) {
    Serial.printf("[LoRa] setBandwidth(%.2f kHz) error: %d\n",
      loraBandwidthFromIndex(g_config.lora.bwIndex), state);
  }

  state = radio.setCodingRate(loraCodingRateFromIndex(g_config.lora.crIndex));
  if (state != RADIOLIB_ERR_NONE && Serial) {
    Serial.printf("[LoRa] setCodingRate(4/%u) error: %d\n",
      loraCodingRateFromIndex(g_config.lora.crIndex), state);
  }

  state = radio.setSyncWord(g_config.lora.syncWord);
  if (state != RADIOLIB_ERR_NONE && Serial) {
    Serial.printf("[LoRa] setSyncWord(0x%02X) error: %d\n",
      g_config.lora.syncWord, state);
  }

  state = radio.setOutputPower(g_config.lora.txPowerDbm);
  if (state != RADIOLIB_ERR_NONE && Serial) {
    Serial.printf("[LoRa] setOutputPower(%d dBm) error: %d\n",
      g_config.lora.txPowerDbm, state);
  }

  state = radio.setPreambleLength(g_config.lora.preambleSymbols);
  if (state != RADIOLIB_ERR_NONE && Serial) {
    Serial.printf("[LoRa] setPreambleLength(%u) error: %d\n",
      g_config.lora.preambleSymbols, state);
  }

  int swState = radio.setDio2AsRfSwitch(true);
  if (Serial) {
    Serial.printf("[RADIO] LoRa setDio2AsRfSwitch: %d\n", swState);
    Serial.printf(
      "[LoRa] freq=%.3f MHz sf=%u bw=%.2f kHz cr=4/%u sync=0x%02X "
      "power=%d dBm preamble=%u\n",
      activeFreqMHz, g_config.lora.sf,
      loraBandwidthFromIndex(g_config.lora.bwIndex),
      loraCodingRateFromIndex(g_config.lora.crIndex),
      g_config.lora.syncWord, g_config.lora.txPowerDbm,
      g_config.lora.preambleSymbols
    );
  }

  enterReceiveMode();
  return true;
}

// ─── GFSK ───────────────────────────────────────────────────────────────────

bool radioBeginGFSK(float freqMHz) {
  loraMode = false;
  activeFreqMHz = freqMHz;
  radioHardwareInit();

  int state = radio.beginFSK(freqMHz);
  if (state != RADIOLIB_ERR_NONE) {
    if (Serial) {
      Serial.print("GFSK init err: ");
      Serial.println(state);
    }
    return false;
  }

  state = radio.setBitRate(g_config.gfsk.bitrateBps / 1000.0f);
  if (state != RADIOLIB_ERR_NONE && Serial) {
    Serial.printf("[GFSK] setBitRate error: %d\n", state);
  }

  state = radio.setFrequencyDeviation(g_config.gfsk.deviationKhz);
  if (state != RADIOLIB_ERR_NONE && Serial) {
    Serial.printf("[GFSK] setFrequencyDeviation error: %d\n", state);
  }

  state = radio.setRxBandwidth(
    gfskRxBandwidthFromIndex(g_config.gfsk.rxBwIndex)
  );
  if (state != RADIOLIB_ERR_NONE && Serial) {
    Serial.printf("[GFSK] setRxBandwidth error: %d\n", state);
  }

  state = radio.setPreambleLength(g_config.gfsk.preambleBits);
  if (state != RADIOLIB_ERR_NONE && Serial) {
    Serial.printf("[GFSK] setPreambleLength error: %d\n", state);
  }

  int swState = radio.setDio2AsRfSwitch(true);
  if (Serial) {
    Serial.printf("[RADIO] GFSK setDio2AsRfSwitch: %d\n", swState);
    Serial.printf(
      "[GFSK] freq=%.3f MHz bitrate=%lu bps deviation=%u kHz "
      "rxBw=%.1f kHz preamble=%u bits\n",
      activeFreqMHz,
      static_cast<unsigned long>(g_config.gfsk.bitrateBps),
      g_config.gfsk.deviationKhz,
      gfskRxBandwidthFromIndex(g_config.gfsk.rxBwIndex),
      g_config.gfsk.preambleBits
    );
  }

  enterReceiveMode();
  return true;
}

// ─── Airtime helper (used by the TX airtime guard) ──────────────────────────

uint32_t radioGetAirtimeMs(bool controlFrame) {
  size_t len = controlFrame ? (1 + sizeof(ControlPacket))
                            : (1 + sizeof(ImuPacket));
  return radio.getTimeOnAir((uint8_t)len) / 1000UL; // µs → ms
}

// ─── Sending ────────────────────────────────────────────────────────────────

bool radioSendPacket(const ImuPacket &pkt) {
  uint8_t buf[1 + sizeof(ImuPacket)];
  buf[0] = FRAME_TELEMETRY;
  memcpy(buf + 1, &pkt, sizeof(pkt));

  digitalWrite(SX1262_ANT_SW, LOW);
  bool ok = radio.transmit(buf, sizeof(buf)) == RADIOLIB_ERR_NONE;
  digitalWrite(SX1262_ANT_SW, HIGH);

  // Lightweight turnaround: no radioBegin*() here.
  // The echo test showed that RX works immediately after TX when the modem
  // is simply returned to receive mode.
  rxFlag = false;
  radio.setPacketReceivedAction(setRxFlag);
  radio.startReceive();

  return ok;
}

bool radioSendControl(const ControlPacket &cmd) {
  uint8_t buf[1 + sizeof(ControlPacket)];
  buf[0] = FRAME_CONTROL;
  memcpy(buf + 1, &cmd, sizeof(cmd));

  digitalWrite(SX1262_ANT_SW, LOW);
  bool ok = radio.transmit(buf, sizeof(buf)) == RADIOLIB_ERR_NONE;
  digitalWrite(SX1262_ANT_SW, HIGH);

  rxFlag = false;
  radio.setPacketReceivedAction(setRxFlag);
  radio.startReceive();

  return ok;
}

// ─── Control reception on TX nodes ──────────────────────────────────────────

bool radioPollControlFrame(ControlPacket &out) {
  if (!rxFlag) return false;
  rxFlag = false;

  const size_t expectedLen = 1 + sizeof(ControlPacket);
  size_t len = radio.getPacketLength();

  if (Serial) {
    Serial.printf(
      "[TX DEBUG] rxFlag set, len=%u expected=%u\n",
      static_cast<unsigned int>(len),
      static_cast<unsigned int>(expectedLen)
    );
  }

  uint8_t raw[1 + sizeof(ControlPacket)];

  if (len != sizeof(raw)) {
    radio.startReceive();
    return false;
  }

  int state = radio.readData(raw, len);
  radio.startReceive();

  if (state != RADIOLIB_ERR_NONE) {
    if (Serial) Serial.printf("[TX DEBUG] readData failed: %d\n", state);
    return false;
  }

  if (raw[0] != FRAME_CONTROL) {
    if (Serial) Serial.printf("[TX DEBUG] wrong frame type: 0x%02X\n", raw[0]);
    return false;
  }

  memcpy(&out, raw + 1, sizeof(out));

  if (!controlPacketValid(out)) {
    if (Serial) {
      Serial.printf("[TX DEBUG] bad magic/crc: magic=0x%02X crc=0x%04X calc=0x%04X\n",
        out.magic, out.crc, controlPacketCrc(out));
    }
    return false;
  }

  return true;
}

// ─── Telemetry reception on the RX node ─────────────────────────────────────

bool radioReceivePacket(ImuPacket &pkt, int16_t &rssi, int16_t &snr) {
  if (!rxFlag) return false;
  rxFlag = false;

  size_t len = radio.getPacketLength();
  uint8_t raw[1 + sizeof(ImuPacket)];

  if (len != sizeof(raw)) {
    radio.startReceive();
    return false;
  }

  int state = radio.readData(raw, len);
  radio.startReceive();

  if (state != RADIOLIB_ERR_NONE) {
    if (Serial) {
      Serial.print("RX DROP (Error code: ");
      Serial.print(state);
      Serial.println(")");
    }
    return false;
  }

  if (raw[0] != FRAME_TELEMETRY) {
    return false;
  }

  memcpy(&pkt, raw + 1, sizeof(pkt));
  rssi = static_cast<int16_t>(radio.getRSSI());
  snr = loraMode ? static_cast<int16_t>(radio.getSNR()) : 0;
  return true;
}