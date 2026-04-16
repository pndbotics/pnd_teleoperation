#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/replay.sh <pcap_file> [interface] [loop] [rate_mode] [rate_value]

Args:
  pcap_file  recorded pcap file path (required)
  interface  replay network interface, default lo
  loop       replay count, default 1 (0 means infinite loop)
  rate_mode  timing mode: pps | mbps | multiplier | topspeed (default multiplier)
  rate_value mode value:
             - multiplier: float, default 1.0 (2.0 = 2x faster, 0.5 = 2x slower)
             - pps: packets per second, e.g. 120
             - mbps: Mbps, e.g. 10
             - topspeed: ignored

Notes:
  - Default is multiplier 1.0 (original timing)
  - Ensure vive_mocap is listening on the target UDP port before replay

Examples:
  scripts/replay.sh recordings/session01.pcap
  scripts/replay.sh recordings/session01.pcap lo 5
  scripts/replay.sh recordings/session01.pcap lo 0
  scripts/replay.sh recordings/session01.pcap lo 1 multiplier 2.0
  scripts/replay.sh recordings/session01.pcap lo 1 pps 120
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || $# -lt 1 ]]; then
  usage
  exit 0
fi

PCAP_FILE="$1"
IFACE="${2:-lo}"
LOOP="${3:-1}"
RATE_MODE="${4:-multiplier}"
RATE_VALUE="${5:-1.0}"

if ! command -v tcpreplay >/dev/null 2>&1; then
  echo "Error: tcpreplay not found. Install it with: sudo apt install tcpreplay" >&2
  exit 1
fi

if [[ ! -f "$PCAP_FILE" ]]; then
  echo "Error: file does not exist: $PCAP_FILE" >&2
  exit 1
fi

echo "Start replaying UDP packets..."
echo "  pcap:      $PCAP_FILE"
echo "  interface: $IFACE"
echo "  loop:      $LOOP"
echo "  rate_mode: $RATE_MODE"
echo "  rate_val:  $RATE_VALUE"

RATE_ARGS=()
case "$RATE_MODE" in
  multiplier)
    RATE_ARGS=(--multiplier="$RATE_VALUE")
    ;;
  pps)
    RATE_ARGS=(--pps="$RATE_VALUE")
    ;;
  mbps)
    RATE_ARGS=(--mbps="$RATE_VALUE")
    ;;
  topspeed)
    RATE_ARGS=(--topspeed)
    ;;
  *)
    echo "Error: invalid rate_mode '$RATE_MODE' (use: multiplier|pps|mbps|topspeed)" >&2
    exit 1
    ;;
esac

if [[ "$LOOP" == "0" ]]; then
  exec sudo tcpreplay --intf1="$IFACE" --loop=0 "${RATE_ARGS[@]}" "$PCAP_FILE"
else
  exec sudo tcpreplay --intf1="$IFACE" --loop="$LOOP" "${RATE_ARGS[@]}" "$PCAP_FILE"
fi
