#!/usr/bin/env python3
"""
save_mems_binary.py - Receive binary MEMS frames from nRF54L15 HP core via serial.

Usage:
    python save_mems_binary.py [COM_PORT] [OUTPUT_FILE]

Default: COM7, output to mems_frames.bin

Binary protocol:
    Sync: 0xAA 0x55
    Length: 2 bytes little-endian (204)
    Payload: raw rf_frame (204 bytes)
"""

import sys
import struct
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("Please install pyserial: pip install pyserial")
    sys.exit(1)

SYNC_BYTES = b'\xAA\x55'
FRAME_SIZE = 204

def parse_frame(data):
    """Parse rf_frame binary data into dict."""
    if len(data) < FRAME_SIZE:
        return None
    magic, seq, sample_count, flags, timestamp_ms = struct.unpack_from('<HHHHI', data, 0)
    samples = struct.unpack_from(f'<{96}h', data, 12)
    return {
        'magic': magic,
        'seq': seq,
        'sample_count': sample_count,
        'flags': flags,
        'timestamp_ms': timestamp_ms,
        'samples': samples,
    }

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM7'
    outfile = sys.argv[2] if len(sys.argv) > 2 else 'mems_frames.bin'

    print(f"Opening {port} at 115200 baud...")
    ser = serial.Serial(port, 115200, timeout=1)

    bin_path = Path(outfile)
    hex_path = bin_path.with_suffix('.hex')

    frame_count = 0
    print(f"Saving binary frames to: {bin_path}")
    print(f"Saving hex dump to: {hex_path}")
    print("Press Ctrl+C to stop.\n")

    try:
        with open(bin_path, 'wb') as bf, open(hex_path, 'w') as hf:
            buf = b''
            while True:
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue
                buf += chunk

                while True:
                    # Search for sync bytes
                    idx = buf.find(SYNC_BYTES)
                    if idx == -1:
                        # Keep last byte in case it's partial sync
                        buf = buf[-1:] if len(buf) > 0 else b''
                        break

                    # Discard data before sync
                    buf = buf[idx:]

                    # Need sync(2) + length(2) + payload
                    if len(buf) < 4:
                        break

                    length = struct.unpack_from('<H', buf, 2)[0]

                    if length != FRAME_SIZE:
                        # Invalid length, skip this sync
                        buf = buf[2:]
                        continue

                    # Wait for full frame
                    total = 4 + length
                    if len(buf) < total:
                        break

                    # Extract payload
                    payload = buf[4:total]
                    buf = buf[total:]

                    # Save binary
                    bf.write(payload)
                    bf.flush()

                    # Save hex
                    hf.write(payload.hex() + '\n')
                    hf.flush()

                    frame_count += 1

                    # Parse and display
                    frame = parse_frame(payload)
                    if frame and frame_count % 10 == 0:
                        xyz = frame['samples'][:3]
                        print(f"\rFrames: {frame_count} | seq={frame['seq']} "
                              f"| X={xyz[0]:+6d} Y={xyz[1]:+6d} Z={xyz[2]:+6d} "
                              f"| t={frame['timestamp_ms']}ms", end='')

    except KeyboardInterrupt:
        print(f"\n\nStopped. Total frames captured: {frame_count}")
        print(f"Binary saved to: {bin_path}")
        print(f"Hex saved to: {hex_path}")
    finally:
        ser.close()

if __name__ == '__main__':
    main()
