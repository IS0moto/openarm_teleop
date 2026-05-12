#!/usr/bin/env bash
# Initialize CAN interfaces in CAN FD mode (required by OpenArm)
# See: https://docs.openarm.dev/teleop/leader-follower/setup-guide

openarm-can-configure-socketcan can0 -fd
openarm-can-configure-socketcan can1 -fd

# Follower PC only (uncomment if can2/can3 exist):
# openarm-can-configure-socketcan can2 -fd
# openarm-can-configure-socketcan can3 -fd
