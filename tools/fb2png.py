#!/usr/bin/env python3
"""Convert raw RGB565 framebuffer dumps (lcd_buffer_1/2) to PNG.

  fb2png.py [DUMP ...] [--width W] [--height H] [--swap] [-o OUTDIR]

With no DUMP arguments, converts fb1.bin and fb2.bin from the current
directory. Each DUMP.bin is written as DUMP.png (in OUTDIR if given).
Pixels are little-endian RGB565 as the CDC200 scans them out; pass --swap
for byte-swapped (big-endian) dumps.

Standard library only, no Pillow needed.
"""

import argparse
import os
import struct
import sys
import zlib
from array import array


def build_lut():
    """RGB565 value -> 3-byte RGB888, low bits filled by bit replication."""
    lut = []
    for v in range(0x10000):
        r = (v >> 11) & 0x1F
        g = (v >> 5) & 0x3F
        b = v & 0x1F
        lut.append(bytes(((r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2))))
    return lut


def png_chunk(tag, data):
    body = tag + data
    return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)


def write_png(path, width, height, rgb_rows):
    # Each scanline is prefixed with filter type 0 (none).
    raw = b"".join(b"\x00" + row for row in rgb_rows)
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)  # 8-bit RGB
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(png_chunk(b"IHDR", ihdr))
        f.write(png_chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(png_chunk(b"IEND", b""))


def convert(src, dst, width, height, swap, lut):
    expected = width * height * 2
    with open(src, "rb") as f:
        data = f.read()
    if len(data) < expected:
        sys.exit(f"{src}: {len(data)} bytes, need {expected} for {width}x{height} RGB565")
    if len(data) > expected:
        print(f"{src}: {len(data) - expected} trailing bytes ignored")

    px = array("H")
    px.frombytes(data[:expected])
    # array uses host byte order; the dump is little-endian unless --swap.
    if (sys.byteorder == "little") == swap:
        px.byteswap()

    rows = []
    for y in range(height):
        start = y * width
        rows.append(b"".join(lut[p] for p in px[start:start + width]))
    write_png(dst, width, height, rows)
    print(f"{src} -> {dst}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dumps", nargs="*", default=["fb1.bin", "fb2.bin"])
    ap.add_argument("--width", type=int, default=480)
    ap.add_argument("--height", type=int, default=800)
    ap.add_argument("--swap", action="store_true", help="dump is big-endian RGB565")
    ap.add_argument("-o", "--outdir", help="output directory (default: next to each dump)")
    args = ap.parse_args()

    lut = build_lut()
    for src in args.dumps:
        base = os.path.splitext(os.path.basename(src))[0] + ".png"
        dst = os.path.join(args.outdir or os.path.dirname(src), base)
        convert(src, dst, args.width, args.height, args.swap, lut)


if __name__ == "__main__":
    main()
