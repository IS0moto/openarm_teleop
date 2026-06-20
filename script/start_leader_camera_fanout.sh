#!/usr/bin/env bash
set -euo pipefail

BIND_IP="${BIND_IP:-0.0.0.0}"
LISTEN_PORT="${LISTEN_PORT:-52000}"
GUI_PORT="${GUI_PORT:-52001}"
AI_PORT="${AI_PORT:-52002}"

cd "$(dirname "$0")/.."

exec ./build/udp_fanout \
  --bind-ip "${BIND_IP}" \
  --listen-port "${LISTEN_PORT}" \
  --target "127.0.0.1:${GUI_PORT}" \
  --target "127.0.0.1:${AI_PORT}"
