// ============================================================================
// PROJECT: HONDA K-LINE CORE ENGINE - WAVE 110i / 125i ONLY
// Hardware: ESP32-3248S035R (Sunton "CYD" 3.5" 480x320 ST7796 + XPT2046 touch)
// Serial  : Diagnostic data via Serial Monitor (115200 Baud)
// GUI     : Onboard TFT, lvgl widgets over TFT_eSPI -- see Gui.cpp for setup notes.
// ECU Bus : Honda K-Line (UART2), see Protocol.cpp/Protocol.h
// Threads : K-line polling runs on its own FreeRTOS task/core (started by
//           sgKLineTaskStart() in setup()) so it's never blocked by GUI
//           redraws, and vice versa -- see Protocol.h's sgBusLock() note.
// NOTE    : Standalone backup / real-vehicle-tested baseline.
//           NOT part of SUN_DIAG_GAUGE main firmware.
//           GUI/pin changes for this board are UNTESTED on real hardware --
//           no ESP32-3248S035R was available to flash/verify this pass.
//
// Files (all under src/ -- Arduino IDE compiles that folder but doesn't show
// its files as tabs, see https://arduino.github.io/arduino-cli/latest/sketch-specification/#src-subfolder):
//   src/Config.h          hardware pins, protocol timings, buffer sizes
//   src/Protocol.h/.cpp   K-line wakeup/reconnect + live-data table polling
//   src/Dtc.h/.cpp        DTC fault code read/clear + Thai code database
//   src/Gui.h/.cpp        onboard TFT/touch GUI (lvgl over TFT_eSPI)
// ============================================================================

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "src/Protocol.h"
#include "src/Dtc.h"
#include "src/Gui.h"

static void sgLogPutU8(uint8_t *buf, uint8_t &i, uint8_t v) {
  buf[i++] = v;
}

static void sgLogPutI16(uint8_t *buf, uint8_t &i, int16_t v) {
  buf[i++] = (uint8_t)((uint16_t)v >> 8);
  buf[i++] = (uint8_t)((uint16_t)v & 0xFF);
}

static int16_t sgLogScale100(float v) {
  return (int16_t)lroundf(v * 100.0f);
}

// Fixed 45-byte big-endian telemetry packet for external loggers (TunerPro
// RT via HondaSmartGauge.adx at the repo root -- field order/offsets there
// must stay in sync with this function). Sent once per 'L' Serial command.
void sgSendLogPacket() {
  uint8_t buf[45];
  uint8_t i = 0;

  sgLogPutU8(buf, i, 'S');
  sgLogPutU8(buf, i, 'G');
  sgLogPutI16(buf, i, (int16_t)sgGauge.rpm);
  sgLogPutI16(buf, i, sgLogScale100(sgGauge.tps));
  sgLogPutI16(buf, i, sgLogScale100(sgGauge.tpsVolt));
  sgLogPutI16(buf, i, sgGauge.tempC);
  sgLogPutI16(buf, i, sgLogScale100(sgGauge.batteryV));
  sgLogPutI16(buf, i, sgLogScale100(sgGauge.injectorMs));
  sgLogPutI16(buf, i, sgLogScale100(sgGauge.ignitionDeg));
  sgLogPutU8(buf, i, sgGauge.speedKmh);
  sgLogPutI16(buf, i, sgLogScale100(sgGauge.o2[0].voltV));
  sgLogPutI16(buf, i, sgLogScale100(sgGauge.o2[0].stft));
  sgLogPutI16(buf, i, sgLogScale100(sgGauge.o2[0].heaterV));
  sgLogPutU8(buf, i, sgGauge.o2[0].ok ? 1 : 0);
  sgLogPutI16(buf, i, sgLogScale100(sgGauge.o2[1].voltV));
  sgLogPutI16(buf, i, sgLogScale100(sgGauge.o2[1].stft));
  sgLogPutI16(buf, i, sgLogScale100(sgGauge.o2[1].heaterV));
  sgLogPutU8(buf, i, sgGauge.o2[1].ok ? 1 : 0);
  sgLogPutI16(buf, i, sgGauge.ext.iatTempC);
  sgLogPutU8(buf, i, sgGauge.ext.mapKpa);
  sgLogPutI16(buf, i, sgGauge.ext.eotTempC);
  sgLogPutU8(buf, i, sgGauge.ext.mode);
  sgLogPutU8(buf, i, sgGauge.ext.fastIdleOn ? 1 : 0);
  sgLogPutU8(buf, i, sgGauge.ext.ok ? 1 : 0);
  sgLogPutU8(buf, i, sgGauge.digital.gearBankStand);
  sgLogPutU8(buf, i, sgGauge.digital.fuelPumpOn ? 1 : 0);
  sgLogPutU8(buf, i, sgGauge.digital.coolingFanOn ? 1 : 0);
  sgLogPutU8(buf, i, sgGauge.digital.ok ? 1 : 0);
  sgLogPutU8(buf, i, sgGauge.ecuConnected ? 1 : 0);

  buf[i] = sgChecksum(buf, i);
  Serial.write(buf, sizeof(buf));
}

void sgPrintRealtimeData() {
  static uint32_t lastPrintMs = 0;

  if (millis() - lastPrintMs < 200) {
    return;
  }

  lastPrintMs = millis();

  if (!sgGauge.ecuConnected) {
    Serial.println(
      "[STATUS] Connecting to Wave ECU...");
    return;
  }

  Serial.printf(
    "RPM:%4u | TPS:%5.1f%% (%3.2fV) | "
    "TEMP:%3dC | BAT:%4.1fV | INJ:%4.2fms | "
    "IGN:%4.1f deg | SPEED:%3u km/h | O2:%3.2fV STFT:%4.1f%% HTR:%3.2fV\n",
    sgGauge.rpm,
    sgGauge.tps,
    sgGauge.tpsVolt,
    sgGauge.tempC,
    sgGauge.batteryV,
    sgGauge.injectorMs,
    sgGauge.ignitionDeg,
    sgGauge.speedKmh,
    sgGauge.o2[0].voltV,
    sgGauge.o2[0].stft,
    sgGauge.o2[0].heaterV);

  if (sgGauge.ext.ok) {
    Serial.printf(
      "  [EXT] EOT:%3dC MODE:%u FASTIDLE:%s\n",
      sgGauge.ext.eotTempC,
      sgGauge.ext.mode,
      sgGauge.ext.fastIdleOn ? "ON" : "OFF");
  }

  if (sgGauge.digital.ok) {
    Serial.printf(
      "  [T D1] GEAR/BANK/STAND:%u FUEL_PUMP:%s COOLING_FAN:%s\n",
      sgGauge.digital.gearBankStand,
      sgGauge.digital.fuelPumpOn ? "ON" : "OFF",
      sgGauge.digital.coolingFanOn ? "ON" : "OFF");
  }

  if (sgGauge.ecmCodeOk) {
    Serial.printf(
      "  [ECM CODE] %02X%02X%02X%02X%02X\n",
      sgGauge.ecmCode[0], sgGauge.ecmCode[1], sgGauge.ecmCode[2],
      sgGauge.ecmCode[3], sgGauge.ecmCode[4]);
  }
}

void setup() {
  Serial.begin(115200);

  delay(1000);

  Serial.println(
    "=================================================");

  Serial.println(
    "   WAVE 110i / 125i K-LINE ENGINE INITIALIZED   ");

  Serial.println(
    "=================================================");

  memset(
    &sgGauge,
    0,
    sizeof(sgGauge));

  sgKLineBegin();
  sgKLineTaskStart();  // polls K-line on its own core from here on -- see Protocol.h
  sgGuiInit();
}

// Runs on core 1 (Arduino's default loop() core): GUI/touch + Serial. The
// K-line engine polls independently on core 0 (sgKLineTaskStart()), so this
// loop is never blocked waiting on ECU response timeouts.
void loop() {
  sgPrintRealtimeData();
  sgGuiPollTouch();
  sgGuiUpdate();

  if (Serial.available()) {
    const char cmd = (char)Serial.read();

    if (cmd == 'r' || cmd == 'R') {
      sgDtcRead();
    } else if (cmd == 'c' || cmd == 'C') {
      sgDtcClear();
    } else if (cmd == 'l' || cmd == 'L') {
      sgSendLogPacket();
    } else if (cmd == 's' || cmd == 'S') {
      sgScanLiveTables();
    } else if (cmd == 't' || cmd == 'T') {
      sgGuiCalibrateTouch();
    }
  }

  delay(5);
}
