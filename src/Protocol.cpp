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

// Live-data tables beyond 17/20 -- exact per-table byte offsets from
// "PGMFI HONDA.pdf" (section E.6, tables 00/10/11/13/17/20/21/60/61/63/67/70/71/D1).
// RPM/TPS/ECT (D9-D14) are at the same offset in every main-set table; only the
// tail (IAT+MAP vs MAP-only vs EOT-only, and where BAT/INJ/IGT/SPEED land) differs,
// so one decoder (sgDecodeMainLayout) is parameterized by SG_MAIN_LAYOUTS instead of
// duplicating it per table. Table 17 keeps its own dedicated sgDecodeWaveMain() below
// (real-vehicle-tested on the Wave ECU already, left untouched) even though its shape
// matches layout C -- see the note on sgDecodeWaveMain for how it differs from the PDF.
enum SgTableKind : uint8_t {
  SG_KIND_WAVE17,
  SG_KIND_MAIN,
  SG_KIND_O2,
  SG_KIND_ECMCODE,
  SG_KIND_DIGITAL
};

#define SG_OFF_NONE 0xFF

struct SgMainLayout {
  uint8_t iatVOff, iatCOff;   // SG_OFF_NONE if this table has no IAT
  uint8_t mapVOff, mapKpaOff; // SG_OFF_NONE if this table has no MAP
  uint8_t eotVOff, eotCOff;   // SG_OFF_NONE if this table has no EOT
  uint8_t batOff;
  uint8_t injOff;             // 2 bytes: injOff, injOff+1
  uint8_t igtOff;
  uint8_t modeOff;            // MODE / FAST IDLE flag byte, SG_OFF_NONE if absent
  uint8_t speedOff;
};

// Payload index = 4 + (D-number - 9), since payload[0..3] = response header/
// length/query/table and payload[4] = D9 (see sgReadLiveTable).
static const SgMainLayout SG_MAIN_LAYOUTS[] = {
  // Layout A: tables 10, 11, 60, 61 (IAT + MAP, BAT/MODE/INJ/IGT/SPEED at D21-D26)
  { 10, 11, 12, 13, SG_OFF_NONE, SG_OFF_NONE, 16, 18, 20, 17, 21 },
  // Layout B: tables 13, 63 (IAT + MAP, no MODE, BAT/INJ/IGT/SPEED at D19-D25)
  { 10, 11, 12, 13, SG_OFF_NONE, SG_OFF_NONE, 14, 15, 17, SG_OFF_NONE, 20 },
  // Layout C: table 67 (EOT instead of IAT/MAP, FAST IDLE/MODE share one byte)
  { SG_OFF_NONE, SG_OFF_NONE, SG_OFF_NONE, SG_OFF_NONE, 12, 13, 14, 15, 17, 18, 19 }
};

// What a working table lets the screen show. Which tables actually answer
// depends on the ECU, so sgProbeTables() tests them all once per connect and
// only the ones that answer are polled and only their fields are displayed.
#define SG_CAP_MAIN    0x001 // RPM, TPS, ECT, battery, injector, ignition, speed
#define SG_CAP_IAT     0x002
#define SG_CAP_MAP     0x004
#define SG_CAP_EOT     0x008
#define SG_CAP_MODE    0x010
#define SG_CAP_O2B1    0x020
#define SG_CAP_O2B2    0x040
#define SG_CAP_DIGITAL 0x080
#define SG_CAP_ECMID   0x100

struct SgLiveTable {
  uint8_t id;
  SgTableKind kind;
  uint8_t variant; // O2: sgGauge.o2[] index. MAIN: SG_MAIN_LAYOUTS[] index.
  uint16_t caps;
};

// Order matters: the first working MAIN table / first working O2B1 table is the
// one polled most often.
static const SgLiveTable SG_TABLES[] = {
  { 0x17, SG_KIND_WAVE17,  0, SG_CAP_MAIN | SG_CAP_EOT | SG_CAP_MODE },
  { 0x67, SG_KIND_MAIN,    2, SG_CAP_MAIN | SG_CAP_EOT | SG_CAP_MODE },
  { 0x10, SG_KIND_MAIN,    0, SG_CAP_MAIN | SG_CAP_IAT | SG_CAP_MAP | SG_CAP_MODE },
  { 0x11, SG_KIND_MAIN,    0, SG_CAP_MAIN | SG_CAP_IAT | SG_CAP_MAP | SG_CAP_MODE },
  { 0x13, SG_KIND_MAIN,    1, SG_CAP_MAIN | SG_CAP_IAT | SG_CAP_MAP },
  { 0x60, SG_KIND_MAIN,    0, SG_CAP_MAIN | SG_CAP_IAT | SG_CAP_MAP | SG_CAP_MODE },
  { 0x61, SG_KIND_MAIN,    0, SG_CAP_MAIN | SG_CAP_IAT | SG_CAP_MAP | SG_CAP_MODE },
  { 0x63, SG_KIND_MAIN,    1, SG_CAP_MAIN | SG_CAP_IAT | SG_CAP_MAP },
  { 0x20, SG_KIND_O2,      0, SG_CAP_O2B1 },
  { 0x70, SG_KIND_O2,      0, SG_CAP_O2B1 }, // alt table id for the same bank-1 O2 sensor
  { 0x21, SG_KIND_O2,      1, SG_CAP_O2B2 },
  { 0x71, SG_KIND_O2,      1, SG_CAP_O2B2 }, // alt table id for the same bank-2 O2 sensor
  { 0xD1, SG_KIND_DIGITAL, 0, SG_CAP_DIGITAL },
  { 0x00, SG_KIND_ECMCODE, 0, SG_CAP_ECMID }
};

#define SG_TABLE_COUNT ((uint8_t)(sizeof(SG_TABLES) / sizeof(SG_TABLES[0])))

// Filled by sgProbeTables(): the tables that answered, and how they are polled.
static int8_t sgPrimaryMain = -1; // SG_TABLES index polled most often (main sensors)
static int8_t sgPrimaryO2 = -1;   // same, for the O2 bank 1 sensor
static uint8_t sgRest[SG_TABLE_COUNT]; // every other working table, polled round-robin
static uint8_t sgRestCount = 0;
static uint8_t sgRestIdx = 0;

static uint32_t sgConsecutiveFail = 0;
static uint32_t sgLastConnectTryMs = 0;
static uint32_t sgLastKLineCmdDoneMs = 0;
static uint32_t sgReconnectNotBeforeMs = 0;

static void sgProbeTables();

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
// Shared by every table in SG_TABLES.
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

    sgProbeTables();
  }
}

// ============================================================================
// WAVE DECODER
// ============================================================================

// FAST IDLE and MODE share one byte in tables 17/67: bit 7 set = fast idle off.
static void sgSetSharedMode(uint8_t raw) {
  sgGauge.ext.fastIdleOn = (raw < 0x80);
  sgGauge.ext.mode = (raw < 0x80) ? raw : raw - 0x80;
}

// Table 17 follows the user-supplied PGM-FI layout: ECT=D14-40,
// INJ=(D20*256+D21)/200. Earlier Wave code used offset 0x0B and /250;
// validate these updated conversions separately before relying on Wave readings.
static bool sgDecodeWaveMain(const uint8_t *p, uint8_t len) {
  if (len < 0x14) {
    return false;
  }

  sgGauge.rpm = ((uint16_t)p[0x04] << 8) | p[0x05];
  sgGauge.tpsVolt = (float)p[0x06] * (5.0f / 256.0f);
  sgGauge.tps = (float)p[0x07] * 0.5f;
  sgGauge.tempC = (int16_t)p[0x09] - 40; // D14: ECT (D16 is not ECT)
  sgGauge.batteryV = (float)p[0x0E] / 10.0f;
  sgGauge.injectorMs = (float)(((uint16_t)p[0x0F] << 8) | p[0x10]) / 200.0f; // (D20*256+D21)/200
  sgGauge.ignitionDeg = ((float)p[0x11] / 2.0f) - 64.0f;
  sgGauge.speedKmh = p[0x13]; // D24

  sgGauge.ext.eotVolt = (float)p[0x0C] * (5.0f / 256.0f); // D17
  sgGauge.ext.eotTempC = (int16_t)p[0x0D] - 40;           // D18
  sgSetSharedMode(p[0x12]);                               // D23
  sgGauge.ext.ok = true;

  return true;
}

// PDF-accurate decode for tables 10/11/13/60/61/63/67 (SG_MAIN_LAYOUTS).
static bool sgDecodeMainLayout(const uint8_t *payload, uint8_t len, uint8_t layoutIdx) {
  if (len < 10) {
    sgGauge.ext.ok = false;
    return false;
  }

  const SgMainLayout &lay = SG_MAIN_LAYOUTS[layoutIdx];

  sgGauge.rpm = ((uint16_t)payload[4] << 8) | payload[5];
  sgGauge.tpsVolt = (float)payload[6] * (5.0f / 256.0f);
  sgGauge.tps = (float)payload[7] * 0.5f;
  sgGauge.tempC = (int16_t)payload[9] - 40;

  if (lay.iatVOff != SG_OFF_NONE && lay.iatCOff < len) {
    sgGauge.ext.iatVolt = (float)payload[lay.iatVOff] * (5.0f / 256.0f);
    sgGauge.ext.iatTempC = (int16_t)payload[lay.iatCOff] - 40;
  }

  if (lay.mapVOff != SG_OFF_NONE && lay.mapKpaOff < len) {
    sgGauge.ext.mapVolt = (float)payload[lay.mapVOff] * (5.0f / 256.0f);
    sgGauge.ext.mapKpa = payload[lay.mapKpaOff];
  }

  if (lay.eotVOff != SG_OFF_NONE && lay.eotCOff < len) {
    sgGauge.ext.eotVolt = (float)payload[lay.eotVOff] * (5.0f / 256.0f);
    sgGauge.ext.eotTempC = (int16_t)payload[lay.eotCOff] - 40;
  }

  if (lay.batOff < len) {
    sgGauge.batteryV = (float)payload[lay.batOff] * 0.1f;
  }

  if ((uint16_t)lay.injOff + 1 < len) {
    const uint16_t rawInj =
      ((uint16_t)payload[lay.injOff] << 8) | payload[lay.injOff + 1];
    sgGauge.injectorMs = (float)rawInj / 200.0f;
  }

  if (lay.igtOff < len) {
    sgGauge.ignitionDeg = -64.0f + ((float)payload[lay.igtOff] * 0.5f);
  }

  if (lay.modeOff != SG_OFF_NONE && lay.modeOff < len) {
    if (layoutIdx == 2) {
      sgSetSharedMode(payload[lay.modeOff]);
    } else {
      sgGauge.ext.mode = payload[lay.modeOff]; // layout A: plain MODE byte, no fast-idle bit
    }
  }

  if (lay.speedOff < len) {
    sgGauge.speedKmh = payload[lay.speedOff];
  }

  sgGauge.ext.ok = true;

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

// ============================================================================
// LIVE-DATA TABLES: read, probe, poll
// ============================================================================

// Reads one table and decodes it into sgGauge. False = no reply, a "no data"
// reply (table not supported by this ECU) or a payload that doesn't decode.
static bool sgReadTable(const SgLiveTable &table) {
  uint8_t payload[SG_MAIN_PAYLOAD_MAX];

  const uint8_t len = sgReadLiveTable(
    table.id,
    payload,
    sizeof(payload),
    SG_T17_TIMEOUT_MS
  );

  if (len <= 4) {
    return false;
  }

  switch (table.kind) {
    case SG_KIND_WAVE17:
      return sgDecodeWaveMain(payload, len);

    case SG_KIND_MAIN:
      return sgDecodeMainLayout(payload, len, table.variant);

    case SG_KIND_O2:
      return sgDecodeO2Bank(payload, len, table.variant);

    case SG_KIND_ECMCODE:
      return sgDecodeEcmCode(payload, len);

    case SG_KIND_DIGITAL:
    default:
      return sgDecodeDigitalStatus(payload, len);
  }
}

// Called right after a successful wakeup (bus lock held by the K-line task):
// ask the ECU for every known table once, keep the ones that answer, and publish
// what they provide in sgGauge.support so the screen only shows real data.
static void sgProbeTables() {
  sgGauge.ext = ExtSensors();
  sgGauge.o2[0] = O2Bank();
  sgGauge.o2[1] = O2Bank();
  sgGauge.digital = DigitalStatus();
  sgGauge.ecmCodeOk = false;

  uint16_t caps = 0;
  bool answered[SG_TABLE_COUNT];

  sgPrimaryMain = -1;
  sgPrimaryO2 = -1;
  sgRestCount = 0;
  sgRestIdx = 0;

  for (uint8_t i = 0; i < SG_TABLE_COUNT; ++i) {
    answered[i] = sgReadTable(SG_TABLES[i]);

    delay(SG_KLINE_GAP_MS);

    if (!answered[i]) {
      continue;
    }

    caps |= SG_TABLES[i].caps;

    if (sgPrimaryMain < 0 && (SG_TABLES[i].caps & SG_CAP_MAIN)) {
      sgPrimaryMain = i;
    } else if (sgPrimaryO2 < 0 && (SG_TABLES[i].caps & SG_CAP_O2B1)) {
      sgPrimaryO2 = i;
    } else {
      sgRest[sgRestCount++] = i;
    }
  }

  Serial.print("[PROBE] tables answering:");

  for (uint8_t i = 0; i < SG_TABLE_COUNT; ++i) {
    if (answered[i]) {
      Serial.printf(" %02X", SG_TABLES[i].id);
    }
  }

  Serial.println(caps == 0 ? " none" : "");

  if (caps == 0) {
    // Probably a bad link rather than an ECU with no tables: retry from the wakeup.
    sgScheduleReconnect("PROBE: NO TABLE ANSWERED");
    return;
  }

  TableSupport &sup = sgGauge.support;

  sup.main    = (caps & SG_CAP_MAIN) != 0;
  sup.iat     = (caps & SG_CAP_IAT) != 0;
  sup.map     = (caps & SG_CAP_MAP) != 0;
  sup.eot     = (caps & SG_CAP_EOT) != 0;
  sup.mode    = (caps & SG_CAP_MODE) != 0;
  sup.o2b1    = (caps & SG_CAP_O2B1) != 0;
  sup.o2b2    = (caps & SG_CAP_O2B2) != 0;
  sup.digital = (caps & SG_CAP_DIGITAL) != 0;
  sup.ecmId   = (caps & SG_CAP_ECMID) != 0;
  sup.probed  = true;
}

static void sgPollNext() {
  int8_t pick = -1;

  if (sgPollStep == 0) {
    pick = sgPrimaryMain;
  } else if (sgPollStep == 1) {
    pick = sgPrimaryO2;
  }

  if (pick < 0 && sgRestCount > 0) {
    pick = sgRest[sgRestIdx];
    sgRestIdx = (sgRestIdx + 1) % sgRestCount;
  }

  if (pick < 0) {
    pick = (sgPrimaryMain >= 0) ? sgPrimaryMain : sgPrimaryO2;
  }

  if (pick < 0) {
    return;
  }

  if (sgReadTable(SG_TABLES[pick])) {
    sgConsecutiveFail = 0;
  } else if (++sgConsecutiveFail >= 5) {
    sgScheduleReconnect("TABLE READ FAIL");
  }
}

// Serial 's': ask the ECU for every known table once and print the raw
// payload, so you can see which tables your ECU actually answers.
void sgScanLiveTables() {
  if (!sgGauge.ecuConnected) {
    Serial.println("[SCAN] ECU not connected yet");
    return;
  }

  sgBusLock();

  for (uint8_t i = 0; i < SG_TABLE_COUNT; ++i) {
    uint8_t payload[SG_MAIN_PAYLOAD_MAX];

    const uint8_t len = sgReadLiveTable(SG_TABLES[i].id, payload, sizeof(payload), SG_T17_TIMEOUT_MS);

    Serial.printf("[SCAN] T%02X ", SG_TABLES[i].id);

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

  sgPollNext();

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
