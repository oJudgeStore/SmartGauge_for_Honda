#include "Gui.h"

#include <stdio.h>
#include <string.h>
#include <Arduino.h>

// GUI : LVGL widgets rendered onto the onboard TFT via TFT_eSPI (install
//       "TFT_eSPI" by Bodmer AND "lvgl" from the Arduino Library Manager --
//       neither is vendored in this repo). TFT_eSPI still owns the actual
//       pixel push (LVGL's flush callback below just calls pushPixels() --
//       LV_COLOR_DEPTH 16 in lv_conf.h, see that file's note on why 8-bit
//       RGB332 was tried and reverted) and the XPT2046 touch read (LVGL's
//       indev read callback calls getTouch());
//       LVGL only owns layout/widgets/redraw scheduling on top of that.
//       The #define USER_SETUP_LOADED below only configures THIS .cpp's own
//       compilation; TFT_eSPI.cpp is a separate translation unit and does
//       NOT see it, so Setup_CYD_3248S035.h must ALSO be installed into the
//       library itself (User_Setups/ + User_Setup_Select.h) -- see README
//       "Setup". lv_conf.h needs the equivalent one-time step for lvgl --
//       see that file's header comment.
#define USER_SETUP_LOADED 1
#include "Setup_CYD_3248S035.h"
#include <TFT_eSPI.h>
#include <lvgl.h>
#include <Preferences.h> // bundled with the ESP32 core -- stores touch calibration in NVS
#include "BgImage_oJudge.h" // generated RGB332 (8-bit) watermark, see that file's header comment

#include "Protocol.h"
#include "Dtc.h"

TFT_eSPI sgTft = TFT_eSPI();
Preferences sgPrefs;

// Root cause of every "colors look wrong" report found and fixed: a 16-bit
// byte-swap in the pixel data reaching the panel, NOT a channel-order issue
// -- see sgGuiInit()'s setSwapBytes(true) call and its comment. Color's back.
#define SG_GUI_BG      lv_color_black()
#define SG_GUI_LABEL   lv_color_hex(0x808080)
#define SG_GUI_VALUE   lv_color_white()
#define SG_GUI_OK      lv_color_hex(0x00FF00)
#define SG_GUI_WARN    lv_color_hex(0xFF0000)
#define SG_GUI_BTN     lv_color_hex(0x000080)

// Page 0: table 17/20 fields (the fast-polled primary gauges), plus MAP
// (moved here from page 1, though it's still sourced from the slow-polled
// extended tables like the rest of page 1 -- see the "--" caveat below).
// Page 1: everything else that was previously Serial-only -- tables
// 10/11/13/60/61/63/67 (IAT/EOT/MODE), 21/71 (O2 bank2), D1 (gear/fuel
// pump/cooling fan), 00 (ECM ID).
// Page 2: DTC read/clear, driven by two on-screen touch buttons.
// Pages are switched by double-tapping anywhere on the screen (no auto-cycle),
// detected in the LVGL touchpad read callback so it still fires even when the
// tap also lands on a button -- see sgGuiTouchReadCb().
#define SG_GUI_PAGE_COUNT     3
#define SG_GUI_PAGE_DTC       2
#define SG_GUI_DOUBLE_TAP_MS  400

// Button/row hit-boxes (x, y, w, h) -- unchanged from the direct-draw layout,
// now just lv_obj position/size instead of fillRoundRect() coordinates.
static const int16_t SG_GUI_BTN_READ[4]  = { 10, 280, 300, 40 };
static const int16_t SG_GUI_BTN_CLEAR[4] = { 10, 330, 300, 40 };

static uint8_t sgGuiPage = 0;
static lv_obj_t *sgGuiPageObj[SG_GUI_PAGE_COUNT];

// A caption + value pair. Which cells are shown, and where, depends on which
// live-data tables the ECU answered at connect (sgGauge.support): cells with no
// working table are hidden and the rest are repacked, two per row.
struct SgCell {
  lv_obj_t *cap;
  lv_obj_t *val;
  bool wide; // takes a whole row
};

enum { P0_TPS, P0_TEMP, P0_BAT, P0_INJ, P0_IGN, P0_SPEED, P0_O2, P0_MAP, P0_COUNT };
enum { P1_IAT, P1_EOT, P1_MODE, P1_O2B2, P1_GEAR, P1_PUMP, P1_FAN, P1_ECM, P1_COUNT };

// Page 0/1 widgets that sgGuiUpdate() refreshes from sgGauge every tick.
static struct {
  lv_obj_t *status;
  lv_obj_t *rpmCap, *rpm;
  SgCell c[P0_COUNT];
} sgP0;

static struct {
  lv_obj_t *status;
  SgCell c[P1_COUNT];
} sgP1;

// 8 rows x 26 px fit between the caption (y=35) and the status line (y=250);
// the status text shows the full count, Serial lists all 12.
#define SG_GUI_DTC_ROWS 8

static struct {
  lv_obj_t *lines[SG_GUI_DTC_ROWS];
  lv_obj_t *status;
} sgDtcUi;

// ---------------------------------------------------------------------------
// LVGL <-> TFT_eSPI glue
// ---------------------------------------------------------------------------

static void sgLvglFlushCb(lv_display_t *disp, const lv_area_t *area, uint8_t *pxMap) {
  const uint32_t w = area->x2 - area->x1 + 1;
  const uint32_t h = area->y2 - area->y1 + 1;

  sgTft.startWrite();
  sgTft.setAddrWindow(area->x1, area->y1, w, h);
  sgTft.pushPixels((uint16_t *)pxMap, w * h);
  sgTft.endWrite();

  lv_display_flush_ready(disp);
}

static bool sgGuiTouchWasDown = false;
static uint32_t sgGuiLastTapMs = 0;

static void sgGuiGotoPage(uint8_t page) {
  lv_obj_add_flag(sgGuiPageObj[sgGuiPage], LV_OBJ_FLAG_HIDDEN);
  sgGuiPage = page;
  lv_obj_clear_flag(sgGuiPageObj[sgGuiPage], LV_OBJ_FLAG_HIDDEN);
}

// Feeds real touch data to LVGL (so widgets/buttons on the current page still
// get their own click events) AND separately detects the double-tap-anywhere
// page switch -- both look at the same tap-down edge, so a double-tap landing
// on a button both fires the button (first tap) and advances the page
// (second), same accepted quirk as the old direct-draw version.
static void sgGuiTouchReadCb(lv_indev_t *indev, lv_indev_data_t *data) {
  uint16_t tx, ty;
  const bool touched = sgTft.getTouch(&tx, &ty);

  data->state = touched ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
  if (touched) {
    data->point.x = tx;
    data->point.y = ty;
  }

  const bool tapEdge = touched && !sgGuiTouchWasDown;
  sgGuiTouchWasDown = touched;

  if (!tapEdge) {
    return;
  }

  const uint32_t now = millis();

  if (now - sgGuiLastTapMs <= SG_GUI_DOUBLE_TAP_MS) {
    sgGuiGotoPage((sgGuiPage + 1) % SG_GUI_PAGE_COUNT);
    sgGuiLastTapMs = 0; // consumed -- a 3rd tap starts a fresh window, not a chain
  } else {
    sgGuiLastTapMs = now;
  }
}

// Touch calibration data persists in NVS (ESP32 Preferences) so it only needs
// doing once per physical unit, not once per power-on. Runs before lv_init(),
// straight against TFT_eSPI -- no LVGL widgets needed for a one-off screen.
static void sgGuiTouchInit(bool force) {
  uint16_t calData[5];

  sgPrefs.begin("kline", false);

  const size_t got = sgPrefs.getBytes("tcal", calData, sizeof(calData));

  if (force || got != sizeof(calData)) {
    sgTft.fillScreen(TFT_BLACK);
    sgTft.setTextColor(TFT_WHITE, TFT_BLACK);
    sgTft.setTextFont(2);
    sgTft.drawString("Touch each crosshair to calibrate", 30, 150);
    sgTft.calibrateTouch(calData, TFT_WHITE, TFT_BLACK, 20);
    sgPrefs.putBytes("tcal", calData, sizeof(calData));
  }

  sgTft.setTouch(calData);
}

// Serial 't' -- redo the calibration on a unit that already has one stored
// (drifted panel, replaced screen). calibrateTouch() blocks and draws straight
// to the TFT behind LVGL's back, so invalidate the whole screen afterwards to
// let the next lv_timer_handler() repaint over its crosshairs.
void sgGuiCalibrateTouch() {
  sgGuiTouchInit(true);
  sgGuiTouchWasDown = false; // don't let the last crosshair tap read as a page-switch edge
  lv_obj_invalidate(lv_screen_active());
}

// Watermark, same on every page, flush to the bottom-right corner -- drawn
// straight via TFT_eSPI rather than as an LVGL image object. ponytail: LVGL's
// image descriptor layout isn't stable across v8/v9, and this corner never
// overlaps a widget, so redrawing it directly each throttled tick (matching
// the old cadence) sidesteps that entirely instead of chasing whichever LVGL
// major version actually lands on top of `lvgl` from the Library Manager.
static void sgGuiDrawBackground() {
  // sgBgImage is 8-bit RGB332 (see BgImage_oJudge.h) -- the `true` selects
  // TFT_eSPI's bpp8 pushImage() overload instead of its 16-bit RGB565 one.
  sgTft.pushImage(SG_BG_IMG_X, SG_BG_IMG_Y, SG_BG_IMG_W, SG_BG_IMG_H, sgBgImage, true);
}

// ---------------------------------------------------------------------------
// Widget builders
// ---------------------------------------------------------------------------

static lv_obj_t *sgGuiMakePage() {
  lv_obj_t *page = lv_obj_create(lv_screen_active());
  lv_obj_remove_style_all(page);
  lv_obj_set_size(page, LV_PCT(100), LV_PCT(100));
  lv_obj_clear_flag(page, LV_OBJ_FLAG_SCROLLABLE);
  return page;
}

// ponytail: tried faking a bold weight here (lvgl ships no bold Montserrat)
// by stacking a twin label 1px to the right -- reverted, it read as a
// horizontal smear/blur rather than bold, and cost DRAM we don't have to
// spare (see lv_conf.h's LV_MEM_SIZE note). sgGuiSetText() is now a plain
// pass-through, kept only so call sites don't all need renaming back.
static lv_obj_t *sgGuiAddText(lv_obj_t *parent, int16_t x, int16_t y, const lv_font_t *font, lv_color_t color, const char *text) {
  lv_obj_t *lbl = lv_label_create(parent);
  lv_label_set_text(lbl, text);
  lv_obj_set_style_text_font(lbl, font, 0);
  lv_obj_set_style_text_color(lbl, color, 0);
  lv_obj_set_pos(lbl, x, y);
  return lbl;
}

static void sgGuiSetText(lv_obj_t *label, const char *text) {
  lv_label_set_text(label, text);
}

static lv_obj_t *sgGuiAddCaption(lv_obj_t *parent, int16_t x, int16_t y, const char *text) {
  return sgGuiAddText(parent, x, y, &lv_font_montserrat_22, SG_GUI_LABEL, text);
}

static lv_obj_t *sgGuiAddValue(lv_obj_t *parent, int16_t x, int16_t y, const lv_font_t *font) {
  return sgGuiAddText(parent, x, y, font, SG_GUI_VALUE, "--");
}

static lv_obj_t *sgGuiAddButton(lv_obj_t *parent, const int16_t box[4], const char *text, lv_event_cb_t cb) {
  lv_obj_t *btn = lv_button_create(parent);
  lv_obj_set_pos(btn, box[0], box[1]);
  lv_obj_set_size(btn, box[2], box[3]);
  lv_obj_set_style_bg_color(btn, SG_GUI_BTN, 0);
  lv_obj_set_style_border_color(btn, lv_color_white(), 0);
  lv_obj_set_style_border_width(btn, 2, 0);
  lv_obj_set_style_radius(btn, 6, 0);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *label = lv_label_create(btn);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_color(label, lv_color_white(), 0);
  lv_obj_center(label);

  return btn;
}

static void sgGuiAddPageNumber(lv_obj_t *parent, uint8_t page) {
  char buf[4];
  snprintf(buf, sizeof(buf), "%u/%u", page + 1, SG_GUI_PAGE_COUNT);
  sgGuiAddCaption(parent, 280, 10, buf);
}

// ---------------------------------------------------------------------------
// Page 0/1: gauge/sensor field labels
// ---------------------------------------------------------------------------

static void sgGuiAddCell(lv_obj_t *page, SgCell &cell, const char *caption, bool wide = false) {
  cell.cap = sgGuiAddCaption(page, 0, 0, caption);
  cell.val = sgGuiAddValue(page, 0, 0, &lv_font_montserrat_34);
  cell.wide = wide;
}

static void sgGuiBuildPage0(lv_obj_t *page) {
  sgP0.rpmCap = sgGuiAddCaption(page, 10, 10, "RPM");
  sgP0.rpm    = sgGuiAddValue(page, 10, 30, &lv_font_montserrat_48);
  sgGuiAddPageNumber(page, 0);

  sgP0.status = sgGuiAddText(page, 10, 390, &lv_font_montserrat_22, SG_GUI_WARN, "CONNECTING...");

  sgGuiAddCell(page, sgP0.c[P0_TPS], "TPS");
  sgGuiAddCell(page, sgP0.c[P0_TEMP], "TEMP");
  sgGuiAddCell(page, sgP0.c[P0_BAT], "BAT");
  sgGuiAddCell(page, sgP0.c[P0_INJ], "INJ");
  sgGuiAddCell(page, sgP0.c[P0_IGN], "IGN");
  sgGuiAddCell(page, sgP0.c[P0_SPEED], "SPEED");
  sgGuiAddCell(page, sgP0.c[P0_O2], "O2");
  sgGuiAddCell(page, sgP0.c[P0_MAP], "MAP");
}

static void sgGuiBuildPage1(lv_obj_t *page) {
  sgGuiAddPageNumber(page, 1);

  sgP1.status = sgGuiAddText(page, 10, 390, &lv_font_montserrat_22, SG_GUI_WARN, "CONNECTING...");

  sgGuiAddCell(page, sgP1.c[P1_IAT], "IAT");
  sgGuiAddCell(page, sgP1.c[P1_EOT], "EOT");
  sgGuiAddCell(page, sgP1.c[P1_MODE], "MODE");
  sgGuiAddCell(page, sgP1.c[P1_O2B2], "O2 BANK2");
  sgGuiAddCell(page, sgP1.c[P1_GEAR], "GEAR/BANK");
  sgGuiAddCell(page, sgP1.c[P1_PUMP], "FUEL PUMP");
  sgGuiAddCell(page, sgP1.c[P1_FAN], "COOL FAN");
  sgGuiAddCell(page, sgP1.c[P1_ECM], "ECM ID", true);
}

static void sgGuiShow(lv_obj_t *obj, bool show) {
  if (show) {
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
  }
}

// Shows the cells whose flag is set and packs them from y0 (caption row), two
// per row, so a missing table leaves no gap on screen.
static void sgGuiLayoutCells(SgCell *cells, const bool *show, uint8_t count, int16_t y0) {
  int16_t y = y0;
  uint8_t col = 0;

  for (uint8_t i = 0; i < count; ++i) {
    sgGuiShow(cells[i].cap, show[i]);
    sgGuiShow(cells[i].val, show[i]);

    if (!show[i]) {
      continue;
    }

    if (cells[i].wide && col == 1) {
      col = 0;
      y += 75;
    }

    const int16_t x = (col == 0) ? 10 : 170;

    lv_obj_set_pos(cells[i].cap, x, y);
    lv_obj_set_pos(cells[i].val, x, y + 22);

    if (cells[i].wide || col == 1) {
      col = 0;
      y += 75;
    } else {
      col = 1;
    }
  }
}

static uint16_t sgGuiSupportMask(const TableSupport &s) {
  return (s.probed ? 0x100 : 0) | (s.main ? 1 : 0) | (s.iat ? 2 : 0) | (s.map ? 4 : 0) |
         (s.eot ? 8 : 0) | (s.mode ? 16 : 0) | (s.o2b1 ? 32 : 0) | (s.o2b2 ? 64 : 0) |
         (s.digital ? 128 : 0) | (s.ecmId ? 0x200 : 0);
}

// Re-lays out pages 0/1 when the set of working tables changes (a new ECU
// connect). Before the probe finishes `s` is all-false, so nothing is shown.
static void sgGuiApplySupport(const TableSupport &s) {
  const bool p0[P0_COUNT] = { s.main, s.main, s.main, s.main, s.main, s.main, s.o2b1, s.map };
  const bool p1[P1_COUNT] = { s.iat, s.eot, s.mode, s.o2b2, s.digital, s.digital, s.digital, s.ecmId };

  sgGuiShow(sgP0.rpmCap, s.main);
  sgGuiShow(sgP0.rpm, s.main);
  sgGuiLayoutCells(sgP0.c, p0, P0_COUNT, 105);
  sgGuiLayoutCells(sgP1.c, p1, P1_COUNT, 20);
}

// ---------------------------------------------------------------------------
// Page 2: DTC read/clear
// ---------------------------------------------------------------------------

static char sgDtcStatusMsg[40] = "Tap READ to scan";

static void sgGuiRedrawDtcStatus() {
  sgGuiSetText(sgDtcUi.status, sgDtcStatusMsg);
  lv_refr_now(NULL); // force immediate paint -- sgDtcRead()/sgDtcClear() below block for ~1s
}

static void sgGuiOnReadDtc(lv_event_t *e) {
  if (sgGauge.rpm > 0) {
    strcpy(sgDtcStatusMsg, "Engine running - refused");
    sgGuiRedrawDtcStatus();
    return;
  }

  strcpy(sgDtcStatusMsg, "Reading...");
  sgGuiRedrawDtcStatus();
  if (sgDtcRead()) {
    snprintf(sgDtcStatusMsg, sizeof(sgDtcStatusMsg), "%u DTC found", sgDtc.count);
  } else {
    strcpy(sgDtcStatusMsg, "Read FAILED");
  }
  sgGuiRedrawDtcStatus();
}

static void sgGuiOnClearDtc(lv_event_t *e) {
  if (sgGauge.rpm > 0) {
    strcpy(sgDtcStatusMsg, "Engine running - refused");
    sgGuiRedrawDtcStatus();
    return;
  }

  strcpy(sgDtcStatusMsg, "Clearing...");
  sgGuiRedrawDtcStatus();
  strcpy(sgDtcStatusMsg, sgDtcClear() ? "Cleared" : "Clear FAILED");
  sgGuiRedrawDtcStatus();
}

static void sgGuiBuildPageDtc(lv_obj_t *page) {
  sgGuiAddCaption(page, 10, 10, "DTC READ / CLEAR");
  sgGuiAddPageNumber(page, SG_GUI_PAGE_DTC);
  sgGuiAddButton(page, SG_GUI_BTN_READ, "READ DTC", sgGuiOnReadDtc);
  sgGuiAddButton(page, SG_GUI_BTN_CLEAR, "CLEAR DTC", sgGuiOnClearDtc);

  for (uint8_t i = 0; i < SG_GUI_DTC_ROWS; ++i) {
    sgDtcUi.lines[i] = sgGuiAddText(page, 10, 35 + i * 26, &lv_font_montserrat_22, SG_GUI_VALUE, "");
    lv_obj_set_width(sgDtcUi.lines[i], 300);
    lv_label_set_long_mode(sgDtcUi.lines[i], LV_LABEL_LONG_CLIP);
  }

  sgDtcUi.status = sgGuiAddText(page, 10, 250, &lv_font_montserrat_22, SG_GUI_VALUE, sgDtcStatusMsg);
}

// Called from sgGuiUpdate() so sgDtc.lines[] (populated by sgDtcRead(), from
// either a tap or the Serial 'r'/'c' commands) gets picked up either way.
static void sgGuiRefreshDtcPage() {
  for (uint8_t i = 0; i < SG_GUI_DTC_ROWS; ++i) {
    sgGuiSetText(sgDtcUi.lines[i], i < sgDtc.count ? sgDtc.shortLines[i] : "");
  }
}

// ---------------------------------------------------------------------------
// Support QR code: a shop link shown for SG_QR_SHOW_MS every SG_QR_PERIOD_MS on
// top of every page (README "Support"). It is not clickable, so touch/page
// switching keep working underneath. NOTICE: this advertisement is part of the
// project -- keep it in copies and modified versions you distribute.
// ---------------------------------------------------------------------------

#define SG_QR_URL        "https://s.shopee.co.th/2LZ2cn2Vgh?share_channel_code=6"
#define SG_QR_PERIOD_MS  60000UL  // gauge pages
#define SG_QR_PERIOD_DTC_MS 300000UL // while the DTC page is open
#define SG_QR_SHOW_MS     5000UL

static lv_obj_t *sgQrBox;
static uint32_t sgQrShownAtMs = 0; // last show (or end of the boot splash); the next one is a period later

static void sgGuiBuildQr() {
  sgQrBox = lv_obj_create(lv_layer_top());
  lv_obj_set_size(sgQrBox, 300, 310);
  lv_obj_center(sgQrBox);
  lv_obj_set_style_bg_color(sgQrBox, lv_color_white(), 0);
  lv_obj_set_style_bg_opa(sgQrBox, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(sgQrBox, 0, 0);
  lv_obj_set_style_pad_all(sgQrBox, 0, 0);
  lv_obj_remove_flag(sgQrBox, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));

  lv_obj_t *title = lv_label_create(sgQrBox);
  lv_label_set_text(title, "SCAN TO SUPPORT");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
  lv_obj_set_style_text_color(title, lv_color_black(), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

  lv_obj_t *qr = lv_qrcode_create(sgQrBox);
  lv_qrcode_set_size(qr, 250);
  lv_qrcode_set_quiet_zone(qr, true);
  lv_qrcode_set_dark_color(qr, lv_color_black());
  lv_qrcode_set_light_color(qr, lv_color_white());
  lv_qrcode_update(qr, SG_QR_URL, strlen(SG_QR_URL));
  lv_obj_align(qr, LV_ALIGN_BOTTOM_MID, 0, -10);

  lv_obj_add_flag(sgQrBox, LV_OBJ_FLAG_HIDDEN);
}

static void sgGuiTickQr(uint32_t now) {
  static bool shown = false;

  const uint32_t period = (sgGuiPage == SG_GUI_PAGE_DTC) ? SG_QR_PERIOD_DTC_MS : SG_QR_PERIOD_MS;

  if (!shown && now - sgQrShownAtMs >= period) {
    shown = true;
    sgQrShownAtMs = now;
    lv_obj_remove_flag(sgQrBox, LV_OBJ_FLAG_HIDDEN);
  } else if (shown && now - sgQrShownAtMs >= SG_QR_SHOW_MS) {
    shown = false;
    lv_obj_add_flag(sgQrBox, LV_OBJ_FLAG_HIDDEN);
  }
}

// Boot splash: the QR code with a progress bar for SG_BOOT_SPLASH_MS. Blocks on
// purpose -- setup() starts the K-line task (wakeup + table probe) only after this.
#define SG_BOOT_SPLASH_MS 8000UL

void sgGuiBootSplash() {
  lv_obj_t *bar = lv_bar_create(lv_layer_top());
  lv_obj_set_size(bar, 280, 16);
  lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, -30);
  lv_bar_set_range(bar, 0, 100);

  lv_obj_t *label = lv_label_create(lv_layer_top());
  lv_label_set_text(label, "STARTING...");
  lv_obj_set_style_text_font(label, &lv_font_montserrat_22, 0);
  lv_obj_align_to(label, bar, LV_ALIGN_OUT_TOP_MID, 0, -8);

  lv_obj_remove_flag(sgQrBox, LV_OBJ_FLAG_HIDDEN);

  const uint32_t startMs = millis();
  uint32_t elapsed = 0;

  while (elapsed < SG_BOOT_SPLASH_MS) {
    lv_bar_set_value(bar, (int32_t)(elapsed * 100UL / SG_BOOT_SPLASH_MS), LV_ANIM_OFF);
    sgGuiPollTouch();
    delay(20);
    elapsed = millis() - startMs;
  }

  lv_obj_delete(bar);
  lv_obj_delete(label);
  lv_obj_add_flag(sgQrBox, LV_OBJ_FLAG_HIDDEN);

  sgQrShownAtMs = millis();
  lv_refr_now(NULL);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void sgGuiInit() {
  sgTft.init();
  sgTft.setRotation(0); // portrait, 320x480 -- flip to setRotation(2) if this
                        // comes up upside-down on your physical unit
  // The actual root cause of every "colors look wrong" report so far: LVGL's
  // raw RGB565 pixel buffer (sgLvglFlushCb's pxMap) is byte-swapped relative
  // to what pushPixels() expects by default (_swapBytes = false). It's a
  // 16-bit endianness bug, not a channel-order one -- confirmed by matching
  // a byte-swap simulation exactly against reported swatch colors (red->blue,
  // green->pink, blue->green, yellow->magenta, white unaffected since 0x0000/
  // 0xFFFF survive a byte swap). TFT_RGB_ORDER was the wrong thing to chase.
  sgTft.setSwapBytes(true);
  sgGuiTouchInit(false);

  lv_init();

  static lv_color_t buf1[TFT_WIDTH * 40];
  lv_display_t *disp = lv_display_create(TFT_WIDTH, TFT_HEIGHT);
  lv_display_set_flush_cb(disp, sgLvglFlushCb);
  lv_display_set_buffers(disp, buf1, NULL, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);
  // Hard pixel edges, no blended/grey fringing on text or shapes -- lvgl
  // defaults this ON at LV_COLOR_DEPTH 16 (our setting), so this explicit
  // call is what's actually turning it off. (Font
  // horizontal-subpixel rendering, lv_font_subpx_t's LV_FONT_SUBPX_HOR, is a
  // separate, per-font thing baked in at font-generation time -- the built-in
  // Montserrat fonts we use were never built with it, so there's nothing
  // active to disable there.)
  lv_display_set_antialiasing(disp, false);

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, sgGuiTouchReadCb);

  lv_obj_set_style_bg_color(lv_screen_active(), SG_GUI_BG, 0);
  lv_obj_set_style_bg_opa(lv_screen_active(), LV_OPA_COVER, 0);

  sgGuiPageObj[0] = sgGuiMakePage();
  sgGuiBuildPage0(sgGuiPageObj[0]);

  sgGuiPageObj[1] = sgGuiMakePage();
  sgGuiBuildPage1(sgGuiPageObj[1]);
  lv_obj_add_flag(sgGuiPageObj[1], LV_OBJ_FLAG_HIDDEN);

  sgGuiPageObj[SG_GUI_PAGE_DTC] = sgGuiMakePage();
  sgGuiBuildPageDtc(sgGuiPageObj[SG_GUI_PAGE_DTC]);
  lv_obj_add_flag(sgGuiPageObj[SG_GUI_PAGE_DTC], LV_OBJ_FLAG_HIDDEN);

  sgGuiApplySupport(TableSupport());

  sgGuiBuildQr();

  lv_mem_monitor_t mon;
  lv_mem_monitor(&mon);
  Serial.printf("[GUI] LVGL heap: %u of %u bytes free\n", (unsigned)mon.free_size, (unsigned)LV_MEM_SIZE);
}

// Drives LVGL's tick + timer/render/touch handling. Unthrottled (call every
// loop() iteration, every ~5-10ms) -- a full tap-down/up/tap-down double-tap
// cycle often finishes inside 200-300ms, so gating this behind the ~200ms
// value-refresh throttle in sgGuiUpdate() (as the old direct-draw touch
// polling used to) made it easy to miss one or both press edges entirely.
void sgGuiPollTouch() {
  static uint32_t lastMs = 0;
  const uint32_t now = millis();

  lv_tick_inc(now - lastMs);
  lastMs = now;

  lv_timer_handler();
}

void sgGuiUpdate() {
  static uint32_t lastMs = 0;
  const uint32_t now = millis();

  if (now - lastMs < 200) {
    return;
  }

  lastMs = now;

  sgGuiTickQr(now);
  sgGuiDrawBackground();

  if (sgGuiPage == SG_GUI_PAGE_DTC) {
    sgGuiRefreshDtcPage();
    return;
  }

  const TableSupport &sup = sgGauge.support;

  static uint16_t appliedMask = 0xFFFF;
  const uint16_t mask = sgGuiSupportMask(sup);

  if (mask != appliedMask) {
    appliedMask = mask;
    sgGuiApplySupport(sup);
  }

  lv_obj_t *statusLbl = (sgGuiPage == 0) ? sgP0.status : sgP1.status;
  const char *statusText = !sgGauge.ecuConnected ? "CONNECTING..."
                         : !sup.probed           ? "TESTING TABLES..."
                                                 : "ECU CONNECTED";
  sgGuiSetText(statusLbl, statusText);
  lv_obj_set_style_text_color(statusLbl, (sgGauge.ecuConnected && sup.probed) ? SG_GUI_OK : SG_GUI_WARN, 0);

  if (!sgGauge.ecuConnected || !sup.probed) {
    return;
  }

  char buf[24];

  if (sgGuiPage == 0) {
    if (sup.main) {
      snprintf(buf, sizeof(buf), "%4u", sgGauge.rpm);
      sgGuiSetText(sgP0.rpm, buf);

      snprintf(buf, sizeof(buf), "%.1f%%", sgGauge.tps);
      sgGuiSetText(sgP0.c[P0_TPS].val, buf);

      snprintf(buf, sizeof(buf), "%dC", sgGauge.tempC);
      sgGuiSetText(sgP0.c[P0_TEMP].val, buf);

      snprintf(buf, sizeof(buf), "%.1fV", sgGauge.batteryV);
      sgGuiSetText(sgP0.c[P0_BAT].val, buf);

      snprintf(buf, sizeof(buf), "%.2fms", sgGauge.injectorMs);
      sgGuiSetText(sgP0.c[P0_INJ].val, buf);

      snprintf(buf, sizeof(buf), "%.1f", sgGauge.ignitionDeg);
      sgGuiSetText(sgP0.c[P0_IGN].val, buf);

      snprintf(buf, sizeof(buf), "%u km/h", sgGauge.speedKmh);
      sgGuiSetText(sgP0.c[P0_SPEED].val, buf);
    }

    if (sup.o2b1) {
      snprintf(buf, sizeof(buf), "%.2fV", sgGauge.o2[0].voltV);
      sgGuiSetText(sgP0.c[P0_O2].val, buf);
    }

    if (sup.map) {
      snprintf(buf, sizeof(buf), "%ukPa", sgGauge.ext.mapKpa);
      sgGuiSetText(sgP0.c[P0_MAP].val, buf);
    }

    return;
  }

  if (sup.iat) {
    snprintf(buf, sizeof(buf), "%dC", sgGauge.ext.iatTempC);
    sgGuiSetText(sgP1.c[P1_IAT].val, buf);
  }

  if (sup.eot) {
    snprintf(buf, sizeof(buf), "%dC", sgGauge.ext.eotTempC);
    sgGuiSetText(sgP1.c[P1_EOT].val, buf);
  }

  if (sup.mode) {
    snprintf(buf, sizeof(buf), "%u %s", sgGauge.ext.mode, sgGauge.ext.fastIdleOn ? "FI" : "-");
    sgGuiSetText(sgP1.c[P1_MODE].val, buf);
  }

  if (sup.o2b2) {
    snprintf(buf, sizeof(buf), "%.2fV", sgGauge.o2[1].voltV);
    sgGuiSetText(sgP1.c[P1_O2B2].val, buf);
  }

  if (sup.digital) {
    snprintf(buf, sizeof(buf), "%u", sgGauge.digital.gearBankStand);
    sgGuiSetText(sgP1.c[P1_GEAR].val, buf);

    sgGuiSetText(sgP1.c[P1_PUMP].val, sgGauge.digital.fuelPumpOn ? "ON" : "OFF");
    sgGuiSetText(sgP1.c[P1_FAN].val, sgGauge.digital.coolingFanOn ? "ON" : "OFF");
  }

  if (sup.ecmId && sgGauge.ecmCodeOk) {
    snprintf(
      buf, sizeof(buf), "%02X%02X%02X%02X%02X",
      sgGauge.ecmCode[0], sgGauge.ecmCode[1], sgGauge.ecmCode[2],
      sgGauge.ecmCode[3], sgGauge.ecmCode[4]
    );
    sgGuiSetText(sgP1.c[P1_ECM].val, buf);
  }
}
