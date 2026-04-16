#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/record.sh [port] [output_pcap] [interface]

Args:
  port         UDP port, default 12070
  output_pcap  output file, default recordings/vive_udp_<timestamp>.pcap
  interface    network interface, default any

Examples:
  scripts/record.sh
  scripts/record.sh 12070 recordings/session01.pcap
  scripts/record.sh 12070 recordings/session01.pcap eno1
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

PORT="${1:-12070}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="${2:-recordings/vive_udp_${TIMESTAMP}.pcap}"
IFACE="${3:-any}"

if ! command -v tcpdump >/dev/null 2>&1; then
  echo "Error: tcpdump not found. Install it with: sudo apt install tcpdump" >&2
  exit 1
fi

mkdir -p "$(dirname "$OUTPUT")"

echo "Start recording UDP packets..."
echo "  interface: $IFACE"
echo "  port:      $PORT"
echo "  output:    $OUTPUT"
echo "Press Ctrl+C to stop."

exec sudo tcpdump -i "$IFACE" "udp port $PORT" -w "$OUTPUT"
