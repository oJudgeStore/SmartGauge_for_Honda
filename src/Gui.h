// Onboard TFT/touch GUI (ESP32-3248S035R 320x480 portrait), built with lvgl.

#pragma once

void sgGuiInit();
void sgGuiPollTouch();
void sgGuiUpdate();

// Re-runs the crosshair calibration and overwrites the one stored in NVS.
void sgGuiCalibrateTouch();

// Shows the support QR code with a progress bar for a few seconds. Call once after sgGuiInit().
void sgGuiBootSplash();
