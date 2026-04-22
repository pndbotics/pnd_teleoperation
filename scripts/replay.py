#!/usr/bin/env python3
"""Replay UDP packets captured by scripts/record.py to a destination host:port.

Usage:
  scripts/replay.py <file.udpcap> [--host 127.0.0.1] [--port 12070]
                    [--speed 1.0] [--loop 1] [--src-port 0]

Examples:
  scripts/replay.py recordings/udp_20260421_160638.udpcap
  scripts/replay.py recordings/x.udpcap --host 127.0.0.1 --port 12070
  scripts/replay.py recordings/x.udpcap --speed 2.0 --loop 0
  scripts/replay.py recordings/x.udpcap --speed 0        # topspeed
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from pathlib import Path

MAGIC = b"UDPREC01"
HEADER_FMT = "<QI"
HEADER_SIZE = struct.calcsize(HEADER_FMT)


def load_records(path: Path) -> list[tuple[int, bytes]]:
    """Load all (ts_ns, payload) records from a .udpcap file."""
    records: list[tuple[int, bytes]] = []
    with open(path, "rb") as f:
        magic = f.read(len(MAGIC))
        if magic != MAGIC:
            raise ValueError(
                f"Bad magic in {path}: expected {MAGIC!r}, got {magic!r}"
            )
        while True:
            header = f.read(HEADER_SIZE)
            if not header:
                break
            if len(header) != HEADER_SIZE:
                raise ValueError(f"Truncated header at EOF in {path}")
            ts_ns, length = struct.unpack(HEADER_FMT, header)
            payload = f.read(length)
            if len(payload) != length:
                raise ValueError(
                    f"Truncated payload in {path}: want {length}, got {len(payload)}"
                )
            records.append((ts_ns, payload))
    return records


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Replay a .udpcap recording to a UDP destination."
    )
    parser.add_argument("file", type=Path, help=".udpcap file")
    parser.add_argument("--host", default="127.0.0.1",
                        help="destination host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=12070,
                        help="destination port (default: 12070)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="playback speed multiplier; 0 = topspeed "
                             "(default: 1.0)")
    parser.add_argument("--loop", type=int, default=1,
                        help="loop count, 0 = infinite (default: 1)")
    parser.add_argument("--src-port", type=int, default=0,
                        help="bind source port (default: 0 = ephemeral)")
    args = parser.parse_args()

    if not args.file.is_file():
        print(f"Error: file not found: {args.file}", file=sys.stderr)
        return 1

    try:
        records = load_records(args.file)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    if not records:
        print("Error: no records in file.", file=sys.stderr)
        return 1

    duration_s = (records[-1][0] - records[0][0]) / 1e9 if len(records) > 1 else 0.0
    total_bytes = sum(len(p) for _, p in records)
    print(f"Loaded {len(records)} packets, {total_bytes / 1024:.1f} kB, "
          f"original duration {duration_s:.2f}s")
    print(f"  dst:   {args.host}:{args.port}")
    print(f"  loop:  {args.loop} ({'infinite' if args.loop == 0 else args.loop})")
    print(f"  speed: {'topspeed' if args.speed == 0 else f'{args.speed}x'}")
    print("Press Ctrl+C to stop.")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if args.src_port:
        sock.bind(("0.0.0.0", args.src_port))

    t0_capture = records[0][0]
    loop_idx = 0
    try:
        while args.loop == 0 or loop_idx < args.loop:
            loop_idx += 1
            wall_start = time.monotonic()
            sent = 0
            for ts_ns, payload in records:
                if args.speed > 0:
                    target = wall_start + (ts_ns - t0_capture) / 1e9 / args.speed
                    delay = target - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                sock.sendto(payload, (args.host, args.port))
                sent += 1
            print(f"[loop {loop_idx}] sent {sent} packets")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
