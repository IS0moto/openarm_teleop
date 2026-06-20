#!/usr/bin/env bash
set -euo pipefail

BIND_IP="${BIND_IP:-0.0.0.0}"
LISTEN_PORT="${LISTEN_PORT:-51011}"
AI_PORT="${AI_PORT:-51000}"
TOOLS_PORT="${TOOLS_PORT:-51002}"

cd "$(dirname "$0")/.."

exec ./build/telemetry_fanout \
  --bind-ip "${BIND_IP}" \
  --listen-port "${LISTEN_PORT}" \
  --target "127.0.0.1:${AI_PORT}" \
  --target "127.0.0.1:${TOOLS_PORT}"
