#!/usr/bin/env bash
set -euo pipefail

LEADER_PC_IP="${LEADER_PC_IP:?Set LEADER_PC_IP to the Leader PC address}"
BIND_IP="${BIND_IP:-127.0.0.1}"
LISTEN_PORT="${LISTEN_PORT:-51010}"
LOCAL_RECORDER_PORT="${LOCAL_RECORDER_PORT:-51000}"
LEADER_TRUNK_PORT="${LEADER_TRUNK_PORT:-51011}"

cd "$(dirname "$0")/.."

exec ./build/telemetry_fanout \
  --bind-ip "${BIND_IP}" \
  --listen-port "${LISTEN_PORT}" \
  --target "127.0.0.1:${LOCAL_RECORDER_PORT}" \
  --target "${LEADER_PC_IP}:${LEADER_TRUNK_PORT}"
