// DTC (fault code) read/clear over K-line.

#pragma once

#include <stdint.h>

struct DtcData {
  char lines[12][200]; // Thai descriptions run long in UTF-8 (up to ~140 bytes) -- Serial only
  char shortLines[12][32]; // "N 07-01 ECT Sensor": fits the TFT (Montserrat has no Thai glyphs)
  uint8_t count = 0;
  bool busy = false;
};

extern DtcData sgDtc;

// Both return true only if every ECU exchange succeeded. sgDtcClear() rereads
// the fault tables afterwards, so sgDtc.lines[] shows whatever remains.
bool sgDtcRead();
bool sgDtcClear();
