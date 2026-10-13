// Hardware pins, protocol timings, and buffer sizes shared across modules.

#pragma once

// ESP32-3248S035R P3 header: GND/IO22/IO21 are used for K-line, every other GPIO
// is committed to the onboard TFT/touch/SD/speaker/RGB LED (see
// Setup_CYD_3248S035.h). The wake pulse is generated on TX itself (no separate
// wakeup pin), so the transceiver's TX input must be able to pull the bus low.
#define KLINE_RX_PIN                  21
#define KLINE_TX_PIN                  22
#define KLINE_BAUD                    10400

#define SG_RESPONSE_GAP_MS             15
#define SG_KLINE_GAP_MS                20
#define SG_T17_TIMEOUT_MS             120
#define SG_T20_TIMEOUT_MS             120
#define SG_DTC_TIMEOUT_MS             300
#define SG_CONNECT_RETRY_MS           2500
#define SG_RECONNECT_PAUSE_MS         3000

#define SG_MAIN_PAYLOAD_MAX             32
#define SG_AUX_PAYLOAD_MAX              16
#define SG_T17_FRAME_MAX                32
#define SG_DTC_FRAME_MAX                32
