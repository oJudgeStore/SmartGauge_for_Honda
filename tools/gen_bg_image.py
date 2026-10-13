#!/usr/bin/env python3
"""Regenerate BgImage_oJudge.h from ojudge.png.

The source PNG is black ink at <=20% alpha on transparent (a watermark meant
for a light background) -- composited straight onto our black TFT screen that
would be invisible (black on black), so this treats the alpha channel as a
stencil and recolors it as a full-opacity white watermark instead (the dim
grey an earlier pass used didn't read well on the real RGB565 panel).

Pixels are packed as 8-bit RGB332 (uint8_t, TFT_eSPI's pushImage(..., bpp8=true)
path) rather than 16-bit RGB565 -- half the flash footprint, and since this is
a pure grey/white watermark (never a saturated hue) RGB332's coarser 3/3/2-bit
channels cost only a slightly steppier antialiased edge, not banding anyone
would notice against black.

Usage: put ojudge.png next to this script, then: python gen_bg_image.py
Requires: pip install pillow
"""

from PIL import Image

SRC = "ojudge.png"
DST = "../src/BgImage_oJudge.h"

TARGET_W = 300          # bigger -- was 240 (before that 160, before that 340 centered)
PEAK_GRAY = 255          # brightness at the ink's own max alpha -- full opacity (was 34/72,
                         # too faint to read on the real RGB565 panel)
SRC_MAX_ALPHA = 51       # this PNG's actual alpha ceiling (out of 255)
SCREEN_W, SCREEN_H = 320, 480  # portrait (setRotation(0)); was 480x320 landscape
CORNER_MARGIN = 8              # flush to the bottom-right corner, not centered


def to_rgb332_gray(gray):
    r3 = gray >> 5  # 3 bits
    g3 = gray >> 5  # 3 bits
    b2 = gray >> 6  # 2 bits -- always the shortest channel, so keep grey balanced
    return (r3 << 5) | (g3 << 2) | b2


def main():
    im = Image.open(SRC).convert("RGBA")
    w0, h0 = im.size
    target_h = round(TARGET_W * h0 / w0)

    alpha = im.split()[3].resize((TARGET_W, target_h), Image.LANCZOS)
    pixels = alpha.load()

    out = []
    for y in range(target_h):
        for x in range(TARGET_W):
            gray = min(255, round(pixels[x, y] * (PEAK_GRAY / SRC_MAX_ALPHA)))
            out.append(to_rgb332_gray(gray))

    off_x = SCREEN_W - TARGET_W - CORNER_MARGIN
    off_y = SCREEN_H - target_h - CORNER_MARGIN

    with open(DST, "w") as f:
        f.write("// Background watermark for the onboard GUI, generated from ojudge.png.\n")
        f.write("// Source is black ink at <=20% alpha on transparent, meant for a light\n")
        f.write("// background -- composited straight onto our black screen it would be\n")
        f.write("// invisible (black on black), so the alpha channel is used as a stencil\n")
        f.write("// and recolored as a full-opacity white watermark instead. Packed as\n")
        f.write("// 8-bit RGB332 -- push with sgTft.pushImage(x, y, w, h, sgBgImage, true).\n")
        f.write("// Regenerate with tools/gen_bg_image.py if ojudge.png changes.\n\n")
        f.write("#pragma once\n#include <Arduino.h>\n\n")
        f.write(f"#define SG_BG_IMG_W {TARGET_W}\n")
        f.write(f"#define SG_BG_IMG_H {target_h}\n")
        f.write(f"#define SG_BG_IMG_X {off_x}\n")
        f.write(f"#define SG_BG_IMG_Y {off_y}\n\n")
        f.write(f"const uint8_t sgBgImage[SG_BG_IMG_W * SG_BG_IMG_H] PROGMEM = {{\n")
        for i in range(0, len(out), 16):
            f.write("  " + ",".join(str(v) for v in out[i:i + 16]) + ",\n")
        f.write("};\n")

    print(f"wrote {DST}: {TARGET_W}x{target_h}, offset ({off_x},{off_y}), {len(out)} bytes (8bpp)")


if __name__ == "__main__":
    main()
