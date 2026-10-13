// TFT_eSPI board config for the Sunton ESP32-3248S035R ("CYD" 3.5").
//
// Included from HondaKlineGauge.ino via USER_SETUP_LOADED -- but that only
// configures the .ino's OWN compilation. TFT_eSPI.cpp (and Extensions/Touch.cpp)
// are separate translation units and never see a sketch-level #define, so this
// same file also has to be copied into the TFT_eSPI library itself:
//   1. Copy this file to <Arduino/libraries>/TFT_eSPI/User_Setups/
//   2. In that library's User_Setup_Select.h, comment out the default
//      `#include <User_Setup.h>` line and add
//      `#include <User_Setups/Setup_CYD_3248S035.h>` instead.
// Skipping this step compiles the .ino fine but fails at link time with
// "undefined reference to TFT_eSPI::getTouch/calibrateTouch/setTouch" (and,
// worse, if it linked at all the display would silently use the library's
// default driver/pins instead of this board's). See README "Setup".
//
// Pins confirmed from board documentation (esp3d.io / espboards.dev), not from
// a physical unit in this session -- verify against your board's silkscreen
// before flashing if the display stays blank.

#define USER_SETUP_ID 9001

#define ST7796_DRIVER
// Tested TFT_RGB here on a hunch (an R/B channel swap would explain why only
// non-white/black colors looked wrong) -- disproved: pure red rendered as a
// completely different color under TFT_RGB, the opposite of what a fix would
// do. Reverted to TFT_BGR, confirmed correct for this panel.
#define TFT_RGB_ORDER TFT_BGR

#define TFT_WIDTH  320
#define TFT_HEIGHT 480

#define TFT_MISO 12
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST  -1  // tied to EN (hardware reset), no dedicated GPIO
#define TFT_BL   27
#define TFT_BACKLIGHT_ON HIGH

// XPT2046 resistive touch shares the TFT SPI bus (CLK/DIN/DOUT), only CS differs.
#define TOUCH_CS 33

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define SMOOTH_FONT

#define SPI_FREQUENCY       40000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000
