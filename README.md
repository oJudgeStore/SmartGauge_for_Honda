# SmartGauge for Honda

[ภาษาไทย](README.th.md)

A small digital gauge for Honda motorcycles with a PGM-FI ECU (built for the Wave 110i / 125i). It plugs into the
bike's diagnostic connector and shows live engine data on a 3.5" touch screen, and can read and clear fault codes
(DTCs).

- **Live gauges:** RPM, throttle, coolant temp, battery voltage, injector time, ignition timing, speed, O2 sensor.
- **More sensors:** intake temp, oil temp, MAP, second O2 sensor, ECU mode, gear/side-stand status, fuel pump,
  cooling fan, ECU ID.
- **Only real data is shown:** when it connects, the gauge asks the ECU which data tables it supports and shows only the
  fields that have a working table. Different ECUs support different tables.
- **Fault codes:** read current and stored DTCs with names, and clear them, from the touch screen.

This is an independent project and is not affiliated with, or endorsed by, Honda. "Honda" and "PGM-FI" are
used only to describe the bikes it works with.

It only reads from the ECU. The one thing it writes is "clear DTC" when you tap that button.

## What you need

- **ESP32-3248S035R** board (the Sunton "CYD" 3.5" 320x480 ESP32 touch display).
- A **K-line transceiver** (L9637D, MC33660, Si8621 or an optocoupler circuit) between the bike's K-line wire and
  the ESP32. Never connect the 12 V K-line directly to an ESP32 pin.
- The bike's **4-pin diagnostic connector**: pin 1 GND, pin 2 K-line, pin 4 +12 V (pin 3 is not used).
- A USB cable and a computer with the Arduino IDE.

## Wiring

| ESP32 pin | Goes to |
| --- | --- |
| IO22 | transceiver TX input (also sends the wake-up pulse) |
| IO21 | transceiver RX output |
| GND | transceiver GND (and the bike's GND) |

The transceiver's K-line pin goes to the bike's K-line (connector pin 2). These pins can be changed in
[`src/Config.h`](src/Config.h) (`KLINE_RX_PIN`, `KLINE_TX_PIN`).

## Install

1. Install the **Arduino IDE**, then add the ESP32 board package (Boards Manager, search for "esp32" by Espressif).
2. Install two libraries (Library Manager): **TFT_eSPI** and **lvgl** (version **9.x**; the code does not compile
   with lvgl 8).
3. Two files must be copied into your Arduino libraries folder (usually `Documents/Arduino/libraries`). This is a
   one-time step, and it affects every sketch on this computer that uses these libraries:
   - Copy [`src/lv_conf.h`](src/lv_conf.h) to `libraries/lv_conf.h` (next to the `lvgl` folder, not inside it).
   - Copy [`src/Setup_CYD_3248S035.h`](src/Setup_CYD_3248S035.h) to `libraries/TFT_eSPI/User_Setups/`. Then open
     `libraries/TFT_eSPI/User_Setup_Select.h`, comment out the line `#include <User_Setup.h>`, and add
     `#include <User_Setups/Setup_CYD_3248S035.h>` below it.
4. Open `SmartGauge_for_Honda.ino` (keep the folder name the same as the file name).
5. Choose **Tools > Board > ESP32 Dev Module**, pick your COM port, and click **Upload**.

## Using it

1. Wire it up, turn the key on, power the board from the bike's 12 V (or USB for a bench test).
2. On first boot the screen asks you to touch the crosshairs once. This calibrates the touch and is remembered.
3. **Double-tap anywhere** on the screen to switch pages:
   1. main gauges,
   2. extra sensors,
   3. DTC page with **READ DTC** and **CLEAR DTC** buttons.
4. Reading and clearing DTCs only works with the **engine stopped**. If the engine is running, the buttons refuse.
5. After clearing, active faults can come straight back. Fix the cause too.

Serial Monitor (115200 baud) also prints live values and which tables the ECU answered. Type `r` to read DTCs, `c` to
clear, `t` to redo the touch calibration, and `s` to print the raw reply of every table.

## Troubleshooting

- **Blank or wrong-colored screen:** check that the `Setup_CYD_3248S035.h` step above was done.
- **Compile error about `getTouch`, `calibrateTouch` or `setTouch`:** the TFT_eSPI setup step was skipped.
- **Compile errors in `Gui.cpp`:** you have lvgl 8; install 9.x.
- **Stuck on "CONNECTING...":** check the transceiver, wiring and ground, the key being on, and that the bike's ECU
  is a PGM-FI model.
- **Screen upside-down:** change `setRotation(0)` to `setRotation(2)` in `sgGuiInit()` in `src/Gui.cpp`.

## Limits

- Built around the Wave 110i/125i ECU. Other PGM-FI ECUs use the same protocol but may answer different tables or
  scale some values differently. On the bench ECU tested, tables 17, 20, 70, 67, D1 and 00 answer, so intake temp,
  MAP and the second O2 sensor are not shown for it.
- Live data, DTC read and DTC clear are confirmed working on the board against an ECU on the bench. Use on a
  running bike has not been verified yet.
- Use at your own risk. Do not rely on it for safety-critical decisions.

## Support (advertisement)

The gauge shows a QR code with a progress bar for about 8 seconds at power-on, before it connects to the ECU. After
that it shows the QR code for 5 seconds every minute.
While the DTC page is open it shows only every 5 minutes.
It links to a Shopee page. It is an advertisement: buying through it may earn the author a commission, and it costs
you nothing extra. The touch controls keep working while it is up (double-tap still switches pages), but the live
values are covered for those 5 seconds.

**Please do not remove it.** The QR code is part of this project, so keep it, unchanged, in copies and modified
versions that you share or sell. The link is `SG_QR_URL` in [`src/Gui.cpp`](src/Gui.cpp).

## Credits

Started from the "Educational Core" sketch in
[Artid1994/Honda-K-Line-smart-guage-esp32-project-v1](https://github.com/Artid1994/Honda-K-Line-smart-guage-esp32-project-v1),
then split into modules and extended. The table layouts and fault code list follow the public PGM-FI protocol
reference.
