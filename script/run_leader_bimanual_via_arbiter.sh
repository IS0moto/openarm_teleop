#!/usr/bin/env bash
set -euo pipefail

ARBITER_IP="${ARBITER_IP:-127.0.0.1}"
RIGHT_CAN="${RIGHT_CAN:-can0}"
LEFT_CAN="${LEFT_CAN:-can1}"
RIGHT_PORT="${RIGHT_PORT:-50100}"
LEFT_PORT="${LEFT_PORT:-50101}"
RATE_HZ="${RATE_HZ:-500}"
CONTROL_BIND_IP="${CONTROL_BIND_IP:-0.0.0.0}"
CONTROL_PORT="${CONTROL_PORT:-53201}"

cd "$(dirname "$0")/.."

exec ./build/wifi_bimanual_leader \
  --follower-ip "${ARBITER_IP}" \
  --right-can "${RIGHT_CAN}" \
  --left-can "${LEFT_CAN}" \
  --right-port "${RIGHT_PORT}" \
  --left-port "${LEFT_PORT}" \
  --rate-hz "${RATE_HZ}" \
  --right-urdf urdf/openarm_right.urdf \
  --left-urdf urdf/openarm_left.urdf \
  --control-bind-ip "${CONTROL_BIND_IP}" \
  --control-port "${CONTROL_PORT}"
