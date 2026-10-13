#include "Dtc.h"

#include <Arduino.h>
#include <string.h>

#include "Config.h"
#include "Protocol.h"

DtcData sgDtc;

struct SgDtcDefinition {
  uint8_t mainCode;
  uint8_t subCode;
  const char *sensorTH;
  const char *faultTH;
};

// Full catalog from PGMFI HONDA.pdf section F (main codes 001-113, Indonesian
// source translated to Thai here). Main code 022-1 is transcribed "bank sensor 1"
// in the source PDF, which looks like a typo for "bank sensor 2" (it would
// otherwise duplicate 021-1 and leave 022-2's bank-2 unpaired) -- kept as bank 2
// here to match the pairing with 022-2, flag if a real ECU capture disagrees.
static const SgDtcDefinition sgDtcDatabase[] = {
  { 1,   1, "MAP Sensor",              "วงจรเซนเซอร์ MAP แรงดันต่ำ" },
  { 1,   2, "MAP Sensor",              "วงจรเซนเซอร์ MAP แรงดันสูง" },
  { 2,   1, "MAP Sensor",              "ปัญหาประสิทธิภาพเซนเซอร์ MAP" },
  { 6,   1, "Barometric Sensor",       "วงจรเซนเซอร์ความกดอากาศแรงดันต่ำ" },
  { 6,   2, "Barometric Sensor",       "วงจรเซนเซอร์ความกดอากาศแรงดันสูง" },
  { 7,   1, "ECT Sensor",              "วงจรเซนเซอร์ ECT แรงดันต่ำ" },
  { 7,   2, "ECT Sensor",              "วงจรเซนเซอร์ ECT แรงดันสูง" },
  { 8,   1, "TPS Sensor",              "วงจรเซนเซอร์ TP แรงดันต่ำ" },
  { 8,   2, "TPS Sensor",              "วงจรเซนเซอร์ TP แรงดันสูง" },
  { 9,   1, "IAT Sensor",              "วงจรเซนเซอร์ IAT แรงดันต่ำ" },
  { 9,   2, "IAT Sensor",              "วงจรเซนเซอร์ IAT แรงดันสูง" },
  { 11,  1, "VS Sensor",               "เซนเซอร์ VS ไม่มีสัญญาณ" },
  { 12,  1, "Injector 1",              "วงจรหัวฉีดหลักตัวที่ 1 ขัดข้อง" },
  { 13,  1, "Injector 2",              "วงจรหัวฉีดหลักตัวที่ 2 ขัดข้อง" },
  { 14,  1, "Injector 3",              "วงจรหัวฉีดหลักตัวที่ 3 ขัดข้อง" },
  { 15,  1, "Injector 4",              "วงจรหัวฉีดหลักตัวที่ 4 ขัดข้อง" },
  { 16,  1, "Injector 1 (Secondary)",  "วงจรหัวฉีดรองตัวที่ 1 ขัดข้อง" },
  { 17,  1, "Injector 2 (Secondary)",  "วงจรหัวฉีดรองตัวที่ 2 ขัดข้อง" },
  { 18,  1, "CMP Sensor",              "เซนเซอร์ CMP ไม่มีสัญญาณ" },
  { 19,  1, "CKP Sensor",              "เซนเซอร์ CKP ไม่มีสัญญาณ" },
  { 21,  1, "O2 Sensor Bank1",         "เซนเซอร์ O2 แบงค์ 1 แรงดันต่ำ" },
  { 21,  2, "O2 Sensor Bank1",         "เซนเซอร์ O2 แบงค์ 1 แรงดันสูง" },
  { 22,  1, "O2 Sensor Bank2",         "เซนเซอร์ O2 แบงค์ 2 แรงดันต่ำ" },
  { 22,  2, "O2 Sensor Bank2",         "เซนเซอร์ O2 แบงค์ 2 แรงดันสูง" },
  { 23,  1, "O2 Sensor Bank1 Heater",  "ฮีตเตอร์เซนเซอร์ O2 แบงค์ 1 ขัดข้อง" },
  { 24,  1, "O2 Sensor Bank2 Heater",  "ฮีตเตอร์เซนเซอร์ O2 แบงค์ 2 ขัดข้อง" },
  { 25,  2, "Knock Sensor",            "วงจรเซนเซอร์น็อคขัดข้อง" },
  { 25,  3, "Knock Sensor",            "วงจรเซนเซอร์น็อคขัดข้อง" },
  { 29,  1, "IACV",                    "วงจร IACV ขัดข้อง" },
  { 33,  2, "ECM EEPROM",              "ECM EEPROM ขัดข้อง" },
  { 34,  1, "ECV POT",                 "ECV POT แรงดันต่ำขัดข้อง" },
  { 34,  2, "ECV POT",                 "ECV POT แรงดันสูงขัดข้อง" },
  { 35,  1, "EGCA",                    "EGCA ขัดข้อง" },
  { 36,  1, "A/F Sensor",              "เซนเซอร์ A/F ขัดข้อง" },
  { 38,  1, "A/F Sensor Heater",       "ฮีตเตอร์เซนเซอร์ A/F ขัดข้อง" },
  { 41,  1, "Shiftdrum Angle Sensor",  "วงจรเซนเซอร์มุม Shiftdrum แรงดันต่ำ" },
  { 41,  2, "Shiftdrum Angle Sensor",  "วงจรเซนเซอร์มุม Shiftdrum แรงดันสูง" },
  { 48,  1, "Injector 3 (Secondary)",  "วงจรหัวฉีดรองตัวที่ 3 ขัดข้อง" },
  { 49,  1, "Injector 4 (Secondary)",  "วงจรหัวฉีดรองตัวที่ 4 ขัดข้อง" },
  { 51,  1, "HESD Linear Solenoid",    "โซลินอยด์เชิงเส้น HESD ขัดข้อง" },
  { 54,  1, "Bank Angle Sensor",       "วงจรเซนเซอร์มุมเอียงแรงดันต่ำ" },
  { 54,  2, "Bank Angle Sensor",       "วงจรเซนเซอร์มุมเอียงแรงดันสูง" },
  { 56,  1, "Knock Sensor IC",         "IC เซนเซอร์น็อคขัดข้อง" },
  { 66,  1, "Rear Wheel Speed Sensor", "เซนเซอร์ความเร็วล้อหลังไม่มีสัญญาณ" },
  { 67,  1, "Front Wheel Speed Sensor","เซนเซอร์ความเร็วล้อหน้าไม่มีสัญญาณ" },
  { 71,  1, "TP Sensor 1",             "เซนเซอร์ TP 1 แรงดันต่ำ" },
  { 71,  2, "TP Sensor 1",             "เซนเซอร์ TP 1 แรงดันสูง" },
  { 72,  1, "TP Sensor 2",             "เซนเซอร์ TP 2 แรงดันต่ำ" },
  { 72,  2, "TP Sensor 2",             "เซนเซอร์ TP 2 แรงดันสูง" },
  { 73,  1, "TP Sensor 1/2",           "ความสัมพันธ์แรงดันเซนเซอร์ TP 1 และ 2 ผิดปกติ" },
  { 73,  2, "TP Sensor 1/2",           "เซนเซอร์ TP 1 และ 2 ลัดวงจร" },
  { 74,  1, "APS 1 (TCP)",             "APS 1 (TCP) แรงดันต่ำ" },
  { 74,  2, "APS 1 (TCP)",             "APS 1 (TCP) แรงดันสูง" },
  { 75,  1, "APS 2 (TCP)",             "APS 2 (TCP) แรงดันต่ำ" },
  { 75,  2, "APS 2 (TCP)",             "APS 2 (TCP) แรงดันสูง" },
  { 76,  1, "APS 1-2 (TCP)",           "ความสัมพันธ์แรงดัน APS 1-2 (TCP) ผิดปกติ" },
  { 77,  1, "TBW Return Spring",       "สปริงคืนตัว TBW ขัดข้อง" },
  { 78,  1, "TBW Motor",               "มอเตอร์ TBW ขัดข้อง" },
  { 79,  1, "TBW Control System",      "ความสัมพันธ์การควบคุมระบบ TBW ผิดปกติ" },
  { 82,  1, "Fast Idle Solenoid Valve","วาล์วโซลินอยด์รอบเดินเบาเร็วขัดข้อง" },
  { 84,  1, "ECM CPU",                 "CPU ภายใน ECM ขัดข้อง" },
  { 85,  1, "TBW Relay",               "รีเลย์ TBW ขัดข้อง (ฝั่ง ON)" },
  { 85,  2, "TBW Relay",               "รีเลย์ TBW ขัดข้อง (ฝั่ง OFF)" },
  { 86,  1, "Serial Communication",    "การสื่อสารแบบอนุกรมขัดข้อง" },
  { 88,  1, "EVAP Purge Solenoid",     "วาล์วโซลินอยด์ควบคุมการล้าง EVAP ขัดข้อง" },
  { 89,  1, "PAIR Solenoid",           "วาล์วโซลินอยด์ควบคุม PAIR ขัดข้อง" },
  { 91,  1, "Ignition Coil 1",         "วงจรคอยล์จุดระเบิดตัวที่ 1 ขัดข้อง" },
  { 92,  2, "Ignition Coil 2",         "วงจรคอยล์จุดระเบิดตัวที่ 2 ขัดข้อง" },
  { 93,  1, "Ignition Coil 3",         "วงจรคอยล์จุดระเบิดตัวที่ 3 ขัดข้อง" },
  { 94,  1, "Ignition Coil 4",         "วงจรคอยล์จุดระเบิดตัวที่ 4 ขัดข้อง" },
  { 99,  1, "IMU",                     "วงจร IMU ขัดข้อง" },
  { 103, 1, "CAN Communication",       "การสื่อสาร CAN ขัดข้อง" },
  { 103, 2, "CAN Communication",       "การสื่อสาร CAN ขัดข้อง" },
  { 103, 3, "CAN Communication",       "การสื่อสาร CAN ขัดข้อง" },
  { 103, 4, "CAN Communication",       "การสื่อสาร CAN ขัดข้อง" },
  { 107, 1, "Shift Stroke Sensor",     "วงจรเซนเซอร์ shift stroke แรงดันต่ำ" },
  { 107, 2, "Shift Stroke Sensor",     "วงจรเซนเซอร์ shift stroke แรงดันสูง" },
  { 108, 1, "Shift Spindle Switch",    "สวิตช์สปินเดิลเปลี่ยนเกียร์แรงดันต่ำ" },
  { 108, 2, "Shift Spindle Switch",    "สวิตช์สปินเดิลเปลี่ยนเกียร์แรงดันสูง" },
  { 113, 1, "Clutch Switch",           "สวิตช์คลัตช์แรงดันต่ำ" }
};

static const uint8_t SG_DTC_DB_COUNT =
  sizeof(sgDtcDatabase) / sizeof(sgDtcDatabase[0]);

static const SgDtcDefinition *sgDtcLookup(uint8_t mainCode, uint8_t subCode) {
  for (uint8_t i = 0; i < SG_DTC_DB_COUNT; ++i) {
    if (sgDtcDatabase[i].mainCode == mainCode &&
        sgDtcDatabase[i].subCode == subCode) {
      return &sgDtcDatabase[i];
    }
  }

  return nullptr;
}

// Frames and reply formats here were verified on a KVB-S53 bench ECU: current
// faults = query 0x74, stored = 0x73, pages 1..11 (see PROTOCOL.md).
static const uint8_t SG_DTC_PAGES = 11;

// One "72 05 <query> <arg> <cs>" request. On success copies the reply frame
// (header, length, query, arg, ..., checksum) into reply[] and returns its length.
static uint8_t sgDtcQuery(uint8_t query, uint8_t arg, uint8_t *reply, uint8_t replyMax) {
  uint8_t cmd[5] = { 0x72, 0x05, query, arg, 0x00 };
  cmd[4] = sgChecksum(cmd, 4);

  uint8_t rx[SG_DTC_FRAME_MAX];

  const uint8_t n = sgKLineSendCapture(cmd, sizeof(cmd), rx, sizeof(rx), SG_DTC_TIMEOUT_MS);

  delay(SG_KLINE_GAP_MS); // tested tool pauses 20 ms after every command

  if (n < sizeof(cmd) + 5 || !sgValidateReply(rx, n, sizeof(cmd))) {
    return 0;
  }

  const uint8_t replyLen = n - sizeof(cmd);

  if (replyLen > replyMax || rx[sizeof(cmd) + 1] != replyLen) {
    return 0;
  }

  memcpy(reply, rx + sizeof(cmd), replyLen);

  return replyLen;
}

static void sgDtcPushLine(const char *tag, uint8_t mainCode, uint8_t subCode) {
  if (sgDtc.count >= 12) {
    return; // shortcut: GUI/Serial list holds 12 codes, extra codes are dropped
  }

  char *shortLine = sgDtc.shortLines[sgDtc.count];
  char *line = sgDtc.lines[sgDtc.count++];
  const SgDtcDefinition *def = sgDtcLookup(mainCode, subCode);

  snprintf(shortLine, sizeof(sgDtc.shortLines[0]), "%c %02u-%02u %.18s",
           tag[0], mainCode, subCode, def != nullptr ? def->sensorTH : "unknown");

  if (def != nullptr) {
    snprintf(line, sizeof(sgDtc.lines[0]), "%s %02u-%02u %s - %s",
             tag, mainCode, subCode, def->sensorTH, def->faultTH);
  } else {
    snprintf(line, sizeof(sgDtc.lines[0]), "%s %02u-%02u (unknown code, not in sgDtcDatabase)",
             tag, mainCode, subCode);
  }
}

static void sgDtcPushError(uint8_t table, uint8_t page) {
  const uint8_t slot = sgDtc.count < 12 ? sgDtc.count++ : 11; // always visible, even when full

  snprintf(sgDtc.lines[slot], sizeof(sgDtc.lines[0]),
           "ERR no valid reply: table 0x%02X page %u", table, page);
  snprintf(sgDtc.shortLines[slot], sizeof(sgDtc.shortLines[0]),
           "ERR T%02X page %u", table, page);
}

// Fills sgDtc.lines[] from both fault tables. A failed page aborts with an ERR
// line instead of reporting a false "no codes".
static bool sgDtcReadTables() {
  sgDtc.count = 0;

  const uint8_t tables[2] = { 0x74, 0x73 };

  for (uint8_t t = 0; t < 2; ++t) {
    const uint8_t table = tables[t];
    const char *tag = (table == 0x74) ? "NOW" : "MEM";

    uint8_t seen[SG_DTC_PAGES * 3][2];
    uint8_t seenCount = 0;

    for (uint8_t page = 1; page <= SG_DTC_PAGES; ++page) {
      uint8_t reply[SG_DTC_FRAME_MAX];

      const uint8_t len = sgDtcQuery(table, page, reply, sizeof(reply));

      // Reply: 02 len table page status main sub main sub main sub cs
      if (len < 12 || reply[2] != table || reply[3] != page) {
        sgDtcPushError(table, page);
        return false;
      }

      for (uint8_t off = 5; off <= 9; off += 2) {
        const uint8_t dtcMain = reply[off];
        const uint8_t dtcSub = reply[off + 1];

        if (dtcMain == 0 || dtcMain == 0xFF) {
          continue;
        }

        bool dup = false;

        for (uint8_t i = 0; i < seenCount; ++i) {
          dup = dup || (seen[i][0] == dtcMain && seen[i][1] == dtcSub);
        }

        if (dup) {
          continue;
        }

        seen[seenCount][0] = dtcMain;
        seen[seenCount][1] = dtcSub;
        ++seenCount;

        sgDtcPushLine(tag, dtcMain, dtcSub);
      }
    }
  }

  return true;
}

static void sgDtcPrintResults() {
  Serial.printf("[DTC] Total Found: %u\n", sgDtc.count);

  for (uint8_t i = 0; i < sgDtc.count; ++i) {
    Serial.println(sgDtc.lines[i]);
  }
}

// Query 0x60 per memory 1..3. The ECU answers with a STATUS, not an echo:
// 60 00 = done, 60 01 = still pending (poll up to 5 s).
static bool sgDtcClearMemories() {
  for (uint8_t memory = 1; memory <= 3; ++memory) {
    const uint32_t deadline = millis() + 5000;

    for (;;) {
      uint8_t reply[SG_DTC_FRAME_MAX];

      const uint8_t len = sgDtcQuery(0x60, memory, reply, sizeof(reply));

      const bool valid = (len == 5 && reply[2] == 0x60);

      if (valid && reply[3] == 0x00) {
        break;
      }

      if (!valid || reply[3] != 0x01 || (int32_t)(millis() - deadline) >= 0) {
        Serial.printf("[DTC] Clear failed at memory %u\n", memory);
        return false;
      }

      delay(100);
    }
  }

  return true;
}

bool sgDtcRead() {
  if (sgGauge.rpm > 0) {
    Serial.println(
      "[DTC] Cannot read while engine is running!"
    );
    return false;
  }

  Serial.println(
    "[DTC] Reading DTC Fault Codes..."
  );

  sgBusLock();

  sgDtc.busy = true;
  sgGauge.ecuConnected = false;
  sgDtc.count = 0;

  sgKLineFlushRx();

  const bool ok = sgKLineWakeup() && sgDtcReadTables();

  sgDtcPrintResults();

  sgDtc.busy = false;

  sgScheduleReconnect(ok ? "DTC DONE" : "DTC FAIL");

  sgBusUnlock();

  return ok;
}

bool sgDtcClear() {
  if (sgGauge.rpm > 0) {
    Serial.println(
      "[DTC] Cannot clear while engine is running!"
    );
    return false;
  }

  Serial.println(
    "[DTC] Clearing DTC codes..."
  );

  sgBusLock();

  sgDtc.busy = true;
  sgGauge.ecuConnected = false;
  sgDtc.count = 0;

  sgKLineFlushRx();

  const bool cleared = sgKLineWakeup() && sgDtcClearMemories();

  Serial.println(
    cleared
      ? "[DTC] Clear SUCCESS! Rereading (active faults may remain or return)..."
      : "[DTC] Clear FAILED."
  );

  if (cleared) {
    sgDtcReadTables();
    sgDtcPrintResults();
  }

  sgDtc.busy = false;

  sgScheduleReconnect(cleared ? "CLEAR DONE" : "CLEAR FAIL");

  sgBusUnlock();

  return cleared;
}
