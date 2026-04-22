#!/usr/bin/env python3
"""Record incoming UDP datagrams on a port into a compact binary capture file.

Unlike pcap captures, this records *application-layer* UDP payloads directly
via a socket, so it is immune to IP fragmentation, MTU, snaplen, GRO, or DLT
issues. Replay with scripts/replay.py.

File format (little-endian):
  magic   : 8 bytes  = b"UDPREC01"
  record* : repeated
      ts_ns  : u64  (wall-clock nanoseconds since epoch)
      length : u32
      payload: <length> bytes

Usage:
  scripts/record.py [-p PORT] [-o OUTPUT] [-b BIND]

Examples:
  scripts/record.py                           # port 12070 -> recordings/udp_<ts>.udpcap
  scripts/record.py -p 12070 -o sess01.udpcap
  scripts/record.py -p 12070 -b 0.0.0.0
"""
from __future__ import annotations

import argparse
import signal
import socket
import struct
import sys
import time
from pathlib import Path

MAGIC = b"UDPREC01"
HEADER_FMT = "<QI"  # ts_ns (u64), length (u32)
HEADER_SIZE = struct.calcsize(HEADER_FMT)
MAX_UDP = 65535


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Record UDP packets on a port to a .udpcap file."
    )
    parser.add_argument("-p", "--port", type=int, default=12070,
                        help="UDP port to listen on (default: 12070)")
    parser.add_argument("-b", "--bind", default="0.0.0.0",
                        help="bind address (default: 0.0.0.0)")
    parser.add_argument("-o", "--output", type=Path, default=None,
                        help="output file (default: recordings/udp_<ts>.udpcap)")
    args = parser.parse_args()

    if args.output is None:
        ts = time.strftime("%Y%m%d_%H%M%S")
        args.output = Path("src/driver/pico_mocap/test") / f"udp_{ts}.udpcap"
    args.output.parent.mkdir(parents=True, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind((args.bind, args.port))

    print(f"Recording UDP on {args.bind}:{args.port}")
    print(f"Writing to: {args.output}")
    print("Press Ctrl+C to stop.")

    stop = False

    def _handle_sigint(signum, frame):  # noqa: ARG001
        nonlocal stop
        stop = True
        try:
            sock.close()
        except OSError:
            pass

    signal.signal(signal.SIGINT, _handle_sigint)
    signal.signal(signal.SIGTERM, _handle_sigint)

    total_pkts = 0
    total_bytes = 0
    t_start = time.monotonic()
    last_report = t_start

    with open(args.output, "wb") as f:
        f.write(MAGIC)
        while not stop:
            try:
                data, _addr = sock.recvfrom(MAX_UDP)
            except OSError:
                break
            if not data:
                continue
            ts_ns = time.time_ns()
            f.write(struct.pack(HEADER_FMT, ts_ns, len(data)))
            f.write(data)
            total_pkts += 1
            total_bytes += len(data)

            now = time.monotonic()
            if now - last_report >= 1.0:
                rate = total_pkts / (now - t_start)
                print(f"  recv {total_pkts} pkts, "
                      f"{total_bytes / 1024:.1f} kB, "
                      f"{rate:.1f} pps")
                last_report = now

    elapsed = time.monotonic() - t_start
    print(f"\nDone. {total_pkts} packets, {total_bytes / 1024:.1f} kB "
          f"in {elapsed:.2f}s -> {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
