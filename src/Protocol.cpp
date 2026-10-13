#include "Protocol.h"

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "Config.h"

GaugeData sgGauge;

static HardwareSerial KLINE(2);

static SemaphoreHandle_t sgBusMutex;

void sgBusLock() {
  xSemaphoreTake(sgBusMutex, portMAX_DELAY);
}

void sgBusUnlock() {
  xSemaphoreGive(sgBusMutex);
}

// ============================================================================
// K-LINE COMMANDS & LIVE-DATA TABLE LAYOUTS
// ============================================================================

static const uint8_t SG_WAKEUP_FE[] = {
  0xFE, 0x04, 0x72, 0x8C
};

static const uint8_t SG_CONNECT_TN[] = {
  0x2C, 0x54, 0x4E, 0x23
};

// Diagnostic init sent after FE (ECU replies 02 04 00 FA), verified on a bench ECU.
static const uint8_t SG_DIAG_INIT[] = {
  0x72, 0x05, 0x00, 0xF0, 0x99
};

// Live-data tables polled besides 17 (main sensors) and 20 (O2 bank 1): 00 (ECM ID)
// and D1 (digital status). A raw scan ('s' over Serial) of a real ECU showed these
// answer; 10/11/13/60/61/63 (IAT/MAP) and 21/71 (O2 bank 2) returned "no data", and
// 67/70 just repeat 17/20 -- so none of those are polled or shown.
static const uint8_t SG_EXT_TABLE_IDS[] = { 0x00, 0xD1 };

static const uint8_t SG_EXT_TABLE_COUNT = sizeof(SG_EXT_TABLE_IDS);

static uint8_t sgExtIndex = 0;

static uint8_t sgMainPayload[SG_MAIN_PAYLOAD_MAX];
static uint8_t sgAuxPayload[SG_AUX_PAYLOAD_MAX];

static uint8_t sgMainPayloadLen = 0;
static uint8_t sgAuxPayloadLen = 0;

static uint32_t sgConsecutiveFail = 0;
static uint32_t sgLastConnectTryMs = 0;
static uint32_t sgLastKLineCmdDoneMs = 0;
static uint32_t sgReconnectNotBeforeMs = 0;

static uint8_t sgPollStep = 0;
static bool sgReconnectPending = false;

// ============================================================================
// K-LINE FRAME PRIMITIVES
// ============================================================================

uint8_t sgChecksum(const uint8_t *data, uint8_t count) {
  if (data == nullptr || count == 0) {
    return 0;
  }

  uint16_t sum = 0;

  for (uint8_t i = 0; i < count; ++i) {
    sum += data[i];
  }

  return (uint8_t)(0x100U - (sum & 0xFFU));
}

void sgKLineFlushRx() {
  while (KLINE.available()) {
    (void)KLINE.read();
  }
}

void sgKLineBegin() {
  KLINE.end();

  pinMode(KLINE_TX_PIN, OUTPUT);
  digitalWrite(KLINE_TX_PIN, HIGH);

  pinMode(KLINE_RX_PIN, INPUT);

  KLINE.begin(
    KLINE_BAUD,
    SERIAL_8N1,
    KLINE_RX_PIN,
    KLINE_TX_PIN
  );

  delay(10);
}

static uint8_t sgKLineReadUntilQuiet(
  uint8_t *out,
  uint8_t maxLen,
  uint16_t timeoutMs
) {
  if (out == nullptr || maxLen == 0) {
    return 0;
  }

  uint8_t n = 0;

  const uint32_t beginMs = millis();
  uint32_t lastByteMs = beginMs;

  while ((millis() - beginMs) < timeoutMs && n < maxLen) {

    while (KLINE.available() && n < maxLen) {
      out[n++] = (uint8_t)KLINE.read();
      lastByteMs = millis();
    }

    if (n > 0 &&
        (millis() - lastByteMs) >= SG_RESPONSE_GAP_MS) {
      break;
    }

    delay(1);
  }

  return n;
}

bool sgValidateReply(
  const uint8_t *full,
  uint8_t fullLen,
  uint8_t echoLen
) {
  if (full == nullptr) {
    return false;
  }

  if (fullLen <= echoLen + 1) {
    return false;
  }

  const uint8_t replyLen = fullLen - echoLen;

  if (replyLen < 2) {
    return false;
  }

  const uint8_t *reply = full + echoLen;

  return sgChecksum(
    reply,
    replyLen - 1
  ) == reply[replyLen - 1];
}

uint8_t sgKLineSendCapture(
  const uint8_t *tx,
  uint8_t txLen,
  uint8_t *rx,
  uint8_t rxMax,
  uint16_t timeoutMs
) {
  if (tx == nullptr || txLen == 0 ||
      rx == nullptr || rxMax == 0) {
    return 0;
  }

  sgKLineFlushRx();

  KLINE.write(tx, txLen);
  KLINE.flush();

  return sgKLineReadUntilQuiet(
    rx,
    rxMax,
    timeoutMs
  );
}

// Generic "read live data table" request: 72 05 71 <tableId> <checksum>.
// Shared by table 17/20 and the tables in SG_EXT_TABLE_IDS.
static uint8_t sgReadLiveTable(
  uint8_t tableId,
  uint8_t *payload,
  uint8_t payloadMax,
  uint16_t timeoutMs
) {
  uint8_t cmd[5] = { 0x72, 0x05, 0x71, tableId, 0x00 };
  cmd[4] = sgChecksum(cmd, 4);

  uint8_t frame[SG_T17_FRAME_MAX];

  const uint8_t n = sgKLineSendCapture(
    cmd,
    sizeof(cmd),
    frame,
    sizeof(frame),
    timeoutMs
  );

  const uint8_t echoLen = sizeof(cmd);

  if (n < echoLen + 2 ||
      !sgValidateReply(frame, n, echoLen)) {
    return 0;
  }

  const uint8_t lengthIndex = echoLen + 1;

  if (lengthIndex >= n) {
    return 0;
  }

  const uint8_t declaredLen = frame[lengthIndex];

  if (declaredLen < 2) {
    return 0;
  }

  const uint8_t replyLen = declaredLen - 1;

  if (replyLen > payloadMax ||
      (uint16_t)echoLen + replyLen > n) {
    return 0;
  }

  memcpy(payload, frame + echoLen, replyLen);

  return replyLen;
}

// ============================================================================
// WAKEUP / RECONNECT
// ============================================================================

bool sgKLineWakeup() {
  KLINE.end();

  // Wake pulse is bit-banged on the TX pin itself (UART is detached above).
  pinMode(KLINE_TX_PIN, OUTPUT);
  digitalWrite(KLINE_TX_PIN, LOW);
  delay(70);

  // ponytail: PGMFI HONDA.pdf specifies 120ms here; this sketch uses 130ms and
  // is real-vehicle-tested on the Wave ECU, so left as-is rather than "corrected"
  // against a spec written for a different ECU family (Beat/Scoopy/Spacy, see
  // PROTOCOL.md). Revisit if wakeup ever needs retuning.
  digitalWrite(KLINE_TX_PIN, HIGH); // idle high
  delay(130);

  sgKLineBegin();

  uint8_t rx[32];

  uint8_t n = sgKLineSendCapture(
    SG_WAKEUP_FE,
    sizeof(SG_WAKEUP_FE),
    rx,
    sizeof(rx),
    200
  );

  if (n == 0) {
    n = sgKLineSendCapture(
      SG_CONNECT_TN,
      sizeof(SG_CONNECT_TN),
      rx,
      sizeof(rx),
      200
    );
  }

  sgGauge.ecuConnected = (n > 0);

  if (sgGauge.ecuConnected) {
    // Best-effort: its reply isn't checked, a dead ECU already fails the first table read.
    delay(SG_KLINE_GAP_MS);
    sgKLineSendCapture(SG_DIAG_INIT, sizeof(SG_DIAG_INIT), rx, sizeof(rx), 200);
    delay(SG_KLINE_GAP_MS);
  }

  return sgGauge.ecuConnected;
}

void sgScheduleReconnect(const char *reason) {
  Serial.printf(
    "[K-LINE] Reconnecting... (%s)\n",
    reason ? reason : "UNKNOWN"
  );

  sgGauge.ecuConnected = false;
  sgGauge.t17Ok = false;
  sgGauge.t20Ok = false;

  sgConsecutiveFail = 0;
  sgPollStep = 0;

  sgReconnectPending = true;

  sgReconnectNotBeforeMs =
    millis() + SG_RECONNECT_PAUSE_MS;

  sgLastConnectTryMs = millis();
}

static void sgEnsureConnected() {
  const uint32_t now = millis();

  if (sgGauge.ecuConnected) {
    return;
  }

  if (sgReconnectPending) {
    if ((int32_t)(now - sgReconnectNotBeforeMs) < 0) {
      return;
    }

    sgReconnectPending = false;
  }

  if (now - sgLastConnectTryMs < SG_CONNECT_RETRY_MS) {
    return;
  }

  sgLastConnectTryMs = now;

  if (sgKLineWakeup()) {
    Serial.println(
      "[ECU CONNECTED] Wave 110i/125i ECU Link Established"
    );
  }
}

// ============================================================================
// WAVE DECODER
// ============================================================================

// Table 17 follows the user-supplied PGM-FI layout: ECT=D14-40,
// INJ=(D20*256+D21)/200. Earlier Wave code used offset 0x0B and /250;
// validate these updated conversions separately before relying on Wave readings.
static bool sgDecodeWaveMain() {
  if (sgMainPayloadLen < 0x12) {
    Serial.printf(
      "[DECODE] T17 payload too short: %u\n",
      sgMainPayloadLen
    );

    return false;
  }

  sgGauge.rpm =
    ((uint16_t)sgMainPayload[0x04] << 8) |
    sgMainPayload[0x05];

  sgGauge.tpsVolt =
    (float)sgMainPayload[0x06] *
    (5.0f / 256.0f);

  sgGauge.tps =
    (float)sgMainPayload[0x07] * 0.5f;

  sgGauge.tempC =
    (int16_t)sgMainPayload[0x09] - 40; // D14: ECT (D16 is not ECT)

  sgGauge.batteryV =
    (float)sgMainPayload[0x0E] / 10.0f;

  const uint16_t rawInj =
    ((uint16_t)sgMainPayload[0x0F] << 8) |
    sgMainPayload[0x10];

  sgGauge.injectorMs =
    (float)rawInj / 200.0f; // Table 17: (D20 * 256 + D21) / 200

  sgGauge.ignitionDeg =
    ((float)sgMainPayload[0x11] / 2.0f) -
    64.0f;

  if (sgMainPayloadLen > 0x13) {
    sgGauge.speedKmh = sgMainPayload[0x13]; // D24, PGMFI HONDA.pdf table 17

    sgGauge.ext.eotTempC = (int16_t)sgMainPayload[0x0D] - 40; // D18: EOT
    sgGauge.ext.fastIdleOn = (sgMainPayload[0x12] < 0x80);    // D23 < 128 = fast idle ON
    sgGauge.ext.mode = (sgMainPayload[0x12] < 0x80)
      ? sgMainPayload[0x12]
      : sgMainPayload[0x12] - 0x80;
    sgGauge.ext.ok = true;
  }

  return true;
}

// Table 0x00: ECM identification code = D9..D13 (PROTOCOL.md / PGMFI HONDA.pdf).
static bool sgDecodeEcmCode(const uint8_t *payload, uint8_t len) {
  if (len < 9) {
    sgGauge.ecmCodeOk = false;
    return false;
  }

  memcpy(sgGauge.ecmCode, payload + 4, sizeof(sgGauge.ecmCode));
  sgGauge.ecmCodeOk = true;

  return true;
}

// Table 0xD1: gear/bank/side-stand (D9), fuel pump (D13), cooling fan (D14).
static bool sgDecodeDigitalStatus(const uint8_t *payload, uint8_t len) {
  if (len < 10) {
    sgGauge.digital.ok = false;
    return false;
  }

  sgGauge.digital.gearBankStand = payload[4];
  sgGauge.digital.fuelPumpOn = (payload[8] != 0x00);
  sgGauge.digital.coolingFanOn = (payload[9] != 0x00);
  sgGauge.digital.ok = true;

  return true;
}

// O2 bank decode (tables 0x20/0x21 share this layout, see PROTOCOL.md).
static bool sgDecodeO2Bank(const uint8_t *payload, uint8_t len, uint8_t bank) {
  if (len <= 0x06 || bank >= 2) {
    sgGauge.o2[bank].ok = false;
    return false;
  }

  sgGauge.o2[bank].voltV    = (float)payload[0x04] * (5.0f / 256.0f);
  sgGauge.o2[bank].stft     = ((float)payload[0x05] - 128.0f) * (100.0f / 128.0f); // percent, 128 = 0%
  sgGauge.o2[bank].heaterV  = (float)payload[0x06] * (5.0f / 256.0f);
  sgGauge.o2[bank].ok       = true;

  return true;
}

static bool sgDecodeWaveAux() {
  if (sgAuxPayloadLen <= 0x04) {
    Serial.printf(
      "[DECODE] T20 payload too short: %u\n",
      sgAuxPayloadLen
    );

    sgGauge.o2[0].ok = false;

    return false;
  }

  return sgDecodeO2Bank(sgAuxPayload, sgAuxPayloadLen, 0);
}

// ============================================================================
// TABLE 17 / TABLE 20 / EXTENDED TABLES
// ============================================================================

static bool sgReadTable17() {
  sgMainPayloadLen = sgReadLiveTable(
    0x17,
    sgMainPayload,
    SG_MAIN_PAYLOAD_MAX,
    SG_T17_TIMEOUT_MS
  );

  if (sgMainPayloadLen == 0) {
    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T17 FAIL");
    }

    return false;
  }

  if (!sgDecodeWaveMain()) {
    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T17 DECODE FAIL");
    }

    return false;
  }

  sgGauge.t17Ok = true;
  sgConsecutiveFail = 0;

  return true;
}

static bool sgReadTable20() {
  sgAuxPayloadLen = sgReadLiveTable(
    0x20,
    sgAuxPayload,
    SG_AUX_PAYLOAD_MAX,
    SG_T20_TIMEOUT_MS
  );

  if (sgAuxPayloadLen == 0) {
    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T20 FAIL");
    }

    return false;
  }

  if (!sgDecodeWaveAux()) {
    if (++sgConsecutiveFail >= 5) {
      sgScheduleReconnect("T20 DECODE FAIL");
    }

    return false;
  }

  sgGauge.t20Ok = true;
  sgConsecutiveFail = 0;

  return true;
}

// Extended live-data tables (SG_EXT_TABLE_IDS).
static bool sgReadExtTable(uint8_t tableId) {
  uint8_t payload[SG_MAIN_PAYLOAD_MAX];

  const uint8_t len = sgReadLiveTable(
    tableId,
    payload,
    sizeof(payload),
    SG_T17_TIMEOUT_MS
  );

  if (len == 0) {
    return false;
  }

  return (tableId == 0x00)
    ? sgDecodeEcmCode(payload, len)
    : sgDecodeDigitalStatus(payload, len);
}

static void sgPollExtTable() {
  sgReadExtTable(SG_EXT_TABLE_IDS[sgExtIndex]);

  sgExtIndex = (sgExtIndex + 1) % SG_EXT_TABLE_COUNT;
}

// Serial 's': ask the ECU for every known live table once and print the raw
// payload, so you can see which tables your ECU actually answers.
void sgScanLiveTables() {
  static const uint8_t ids[] = {
    0x00, 0x10, 0x11, 0x13, 0x17, 0x20, 0x21, 0x60, 0x61, 0x63, 0x67, 0x70, 0x71, 0xD1
  };

  if (!sgGauge.ecuConnected) {
    Serial.println("[SCAN] ECU not connected yet");
    return;
  }

  sgBusLock();

  for (uint8_t i = 0; i < sizeof(ids); ++i) {
    uint8_t payload[SG_MAIN_PAYLOAD_MAX];

    const uint8_t len = sgReadLiveTable(ids[i], payload, sizeof(payload), SG_T17_TIMEOUT_MS);

    Serial.printf("[SCAN] T%02X ", ids[i]);

    if (len == 0) {
      Serial.println("no valid reply");
    } else if (len <= 4) {
      Serial.println("no data (table not supported)");
    } else {
      Serial.printf("%u data bytes:", len - 4);

      for (uint8_t k = 4; k < len; ++k) {
        Serial.printf(" %02X", payload[k]);
      }

      Serial.println();
    }

    delay(SG_KLINE_GAP_MS);
  }

  sgBusUnlock();
}

// ============================================================================
// ENGINE TASK -- called once per sgKLineTaskLoop() iteration, below
// ============================================================================

void sgEngineTask() {
  const uint32_t now = millis();

  sgEnsureConnected();

  if (!sgGauge.ecuConnected) {
    return;
  }

  if (now - sgLastKLineCmdDoneMs <
      SG_KLINE_GAP_MS) {
    return;
  }

  switch (sgPollStep) {
    case 0:  sgReadTable17(); break;
    case 1:  sgReadTable20(); break;
    default: sgPollExtTable(); break;
  }

  sgPollStep =
    (sgPollStep + 1) % 3;

  sgLastKLineCmdDoneMs =
    millis();
}

// ============================================================================
// DEDICATED K-LINE TASK
// ============================================================================

static void sgKLineTaskLoop(void *) {
  for (;;) {
    sgBusLock();
    sgEngineTask();
    sgBusUnlock();

    // sgEngineTask() self-throttles against SG_KLINE_GAP_MS/SG_CONNECT_RETRY_MS
    // and returns immediately outside those windows, so this just needs to
    // yield the core rather than busy-spin -- 1 tick is plenty.
    vTaskDelay(1);
  }
}

void sgKLineTaskStart() {
  sgBusMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    sgKLineTaskLoop,
    "kline",
    4096,
    nullptr,
    1,
    nullptr,
    0 // core 0 -- loop() (GUI/touch/Serial) stays on core 1
  );
}
