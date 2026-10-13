// LVGL config for this project.
//
// Same one-time, machine-wide setup step as Setup_CYD_3248S035.h for
// TFT_eSPI (see Gui.cpp / README "Setup"): lvgl.h includes "../../lv_conf.h"
// by default, which resolves to <Arduino/libraries>/lv_conf.h -- a SIBLING
// of the lvgl library folder, not inside it. Copy this file there.
//
// Only overrides what this project needs; every option left unset falls
// back to LVGL's own internal default (lv_conf_internal.h), so this stays
// short instead of carrying the full ~2000-line template.

#if 1 // set to 1 to use this file

#ifndef LV_CONF_H
#define LV_CONF_H

// RGB332 (LV_COLOR_DEPTH 8) was tried here and reverted: its uneven 3/3/2-bit
// R/G/B split renders pure white/black/red/green exactly (they sit on every
// channel's endpoint at any depth) but visibly tints any mid-tone with equal
// R=G=B -- e.g. grey 0x808080 comes out ~(146,146,170), shifted toward blue,
// because R/G round to 1-of-8 levels while B only gets 1-of-4. RGB565 gives
// every channel enough levels (32/64/32) that grey stays close to neutral.
#define LV_COLOR_DEPTH 16

// This is a static buffer sized at compile time, not a runtime pool, so it
// competes directly with every other global for the ESP32's fixed DRAM
// budget -- was bumped to 56-64K for a since-reverted faux-bold experiment
// that doubled the label count; back to plain single labels, back to 48K.
#define LV_MEM_SIZE (48U * 1024U)

// No hardware tick source wired up -- Gui.cpp calls lv_tick_inc() from
// sgGuiPollTouch() instead, using millis().
#define LV_TICK_CUSTOM 0

#define LV_USE_LABEL 1
#define LV_USE_BUTTON 1
#define LV_USE_BTN 1 // same widget, old (v8) name -- harmless if unused on v9
#define LV_USE_THEME_DEFAULT 1
#define LV_USE_QRCODE 1 // support QR code overlay, see Gui.cpp

#define LV_FONT_MONTSERRAT_22 1
#define LV_FONT_MONTSERRAT_34 1
#define LV_FONT_MONTSERRAT_48 1
#define LV_FONT_DEFAULT &lv_font_montserrat_22

#endif // LV_CONF_H
#endif // 1
