// Honda K-line protocol engine: wakeup/reconnect and live-data table polling.
// Decoded results land in the shared sgGauge snapshot other modules read.

#pragma once

#include <Arduino.h>

struct O2Bank {
  float voltV = 0.0f;
  float stft = 0.0f; // percent, negative = leaner correction
  float heaterV = 0.0f;
  bool ok = false;
};

// Only eotTempC/mode/fastIdleOn are filled (from table 17). The IAT/MAP fields
// belong to tables 10/11/13/60/61/63, which the tested ECU does not answer, so
// they stay 0 (kept so the 'L' log packet layout doesn't change).
struct ExtSensors {
  float iatVolt = 0.0f;
  int16_t iatTempC = 0;
  float mapVolt = 0.0f;
  uint8_t mapKpa = 0;
  float eotVolt = 0.0f;
  int16_t eotTempC = 0;
  uint8_t mode = 0;
  bool fastIdleOn = false;
  bool ok = false;
};

// Table 0xD1 (PROTOCOL.md "Table D1").
struct DigitalStatus {
  uint8_t gearBankStand = 0; // raw D9: gear/bank-angle/side-stand status
  bool fuelPumpOn = false;
  bool coolingFanOn = false;
  bool ok = false;
};

struct GaugeData {
  uint16_t rpm = 0;
  float tps = 0.0f;
  float tpsVolt = 0.0f;
  int16_t tempC = 0;
  float batteryV = 0.0f;
  float injectorMs = 0.0f;
  float ignitionDeg = 0.0f;
  uint8_t speedKmh = 0;
  O2Bank o2[2];          // [0]=tables 0x20/0x70, [1]=tables 0x21/0x71
  ExtSensors ext;        // last extended main-set table read (IAT/MAP/EOT/mode)
  DigitalStatus digital; // table 0xD1
  uint8_t ecmCode[5];    // table 0x00 (ECM identification bytes)
  bool ecmCodeOk = false;
  bool ecuConnected = false;
  bool t17Ok = false;
  bool t20Ok = false;
};

// Mutated only by whichever side holds sgBusLock() (the K-line task, or a
// Dtc.cpp operation running from the GUI/Serial loop() task).
// Read elsewhere (GUI, Serial print) without a lock -- display-only scalars,
// so a torn read is at worst one stale/half-updated ~200ms GUI frame, not a
// correctness issue. Take sgBusLock() too if a future reader ever needs two
// fields to be consistent with each other.
extern GaugeData sgGauge;

// Generic 0x72 request/checksum frame primitives -- shared with the DTC
// (Dtc.cpp) commands, which speak the same framing as the live-data tables
// handled internally by Protocol.cpp.
uint8_t sgChecksum(const uint8_t *data, uint8_t count);
void sgKLineFlushRx();
uint8_t sgKLineSendCapture(
  const uint8_t *tx, uint8_t txLen,
  uint8_t *rx, uint8_t rxMax,
  uint16_t timeoutMs
);
bool sgValidateReply(const uint8_t *full, uint8_t fullLen, uint8_t echoLen);

// Wakeup / connection lifecycle -- also called directly by Dtc.cpp before its
// own request/response exchanges, since those need a fresh wakeup and lose the
// connection for the duration of the transfer.
bool sgKLineWakeup();
void sgScheduleReconnect(const char *reason);

// Guards every K-line bus transaction, and the sgGauge/sgDtc fields written
// as its result, between the dedicated K-line task (sgKLineTaskStart) and
// the DTC operations Dtc.cpp runs synchronously from the GUI/Serial loop()
// task on a touch tap or Serial command.
void sgBusLock();
void sgBusUnlock();

// Called once from setup(), after sgKLineBegin().
void sgKLineBegin();

// Starts the dedicated FreeRTOS task (pinned to core 0, away from the
// GUI/touch SPI traffic on loop()'s core) that polls sgEngineTask() --
// K-line's SG_RESPONSE_GAP_MS response window is tight enough that sharing
// a core with GUI redraws risked missing bytes. Call once from setup().
void sgKLineTaskStart();

// Runs one poll step (wakeup/reconnect or a live-data table read). Called
// only from the task sgKLineTaskStart() creates -- not from loop() anymore.
void sgEngineTask();

// Prints the raw reply of every known live-data table to Serial (command 's').
void sgScanLiveTables();
