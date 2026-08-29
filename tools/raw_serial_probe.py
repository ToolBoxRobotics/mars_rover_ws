#!/usr/bin/env python3
"""Standalone, ROS-free probe for exactly one question: does this
board reply to a raw protocol frame at all, once it's had time to
finish any auto-reset triggered by opening the port?

Sends a real, correctly-checksummed frame (via the same encode_frame()
the ROS bridge nodes themselves use), not a hand-typed one - correct
either way, but this also means the exact bytes sent get printed, so
you can compare them directly against a logic analyzer/scope capture
if you have one.

Defaults to base's own 'D' (drive) frame with 10 zero fields, but
every board's frame format differs - a board that doesn't handle 'D'
at all (the arm, for one - it only understands 'A'/'Z'/'P'/'X') will
correctly show "no reply" against the default, which tells you
nothing about whether ITS firmware is actually healthy. Pass
--frame-type/--fields to send something that board actually
recognizes.

Usage:
    python3 tools/raw_serial_probe.py /dev/rover/base
    python3 tools/raw_serial_probe.py /dev/rover/arm --frame-type Z --fields -1
    python3 tools/raw_serial_probe.py /dev/ttyACM0 --baud 115200
"""
import argparse
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "rover_protocol"))
from rover_protocol.framing import encode_frame


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--grace", type=float, default=3.0, help="seconds to wait after opening before sending")
    parser.add_argument(
        "--frame-type",
        default="D",
        help="frame type char to send - 'D' (base drive) by default; use 'Z' for arm's own home request, or whatever the board you're testing actually understands",
    )
    parser.add_argument(
        "--fields",
        default="0,0,0,0,0,0,0,0,0,0",
        help="comma-separated integer fields for the frame - default matches 'D''s own 10-field, all-zero drive command; e.g. -1 for arm's own 'Z' (home all 5 joints)",
    )
    args = parser.parse_args()

    fields = [int(f) for f in args.fields.split(",")]

    print(f"Opening {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=2.0)

    print(f"Port open. Waiting {args.grace}s for any auto-reset to finish rebooting...")
    time.sleep(args.grace)

    frame = encode_frame(args.frame_type, fields).encode("ascii")
    print(f"Sending: {frame}")
    ser.write(frame)

    print("Listening for a reply (up to 5 reads, 2s each)...")
    got_reply = False
    for i in range(5):
        line = ser.readline()
        print(f"  read {i}: {line!r}")
        if line:
            got_reply = True
            break

    print()
    print("GOT A REPLY" if got_reply else "NO REPLY - board never responded even with the grace period")
    ser.close()


if __name__ == "__main__":
    main()

