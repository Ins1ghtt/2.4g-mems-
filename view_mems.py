#!/usr/bin/env python3
"""Quick viewer for mems_frames.bin"""
import struct
import sys

bin_file = sys.argv[1] if len(sys.argv) > 1 else 'mems_frames.bin'

with open(bin_file, 'rb') as f:
    data = f.read()

frame_size = 204
n = len(data) // frame_size
print(f"File size: {len(data)} bytes")
print(f"Total frames: {n}")
print(f"{'Frame':>6} {'Seq':>6} {'Time(ms)':>10} {'X':>7} {'Y':>7} {'Z':>7}")
print("-" * 50)

for i in range(min(n, 30)):
    offset = i * frame_size
    magic, seq, sample_count, flags, timestamp = struct.unpack_from('<HHHHI', data, offset)
    samples = struct.unpack_from('<96h', data, offset + 12)
    x, y, z = samples[0], samples[1], samples[2]
    
    marker = ""
    if magic != 0xA55A:
        marker = " [BAD MAGIC]"
    elif x == 0x7777:  # 0xDDDD as signed = weird
        marker = " [SPI NOT READY]"
    elif x == -21846:  # 0xAAAA signed
        marker = " [INIT OK]"
    elif x == -4370:   # 0xEEEE signed
        marker = " [READ FAIL]"
    
    print(f"{i:>6} {seq:>6} {timestamp:>10} {x:>+7} {y:>+7} {z:>+7}{marker}")

if n > 30:
    print(f"... ({n - 30} more frames)")
