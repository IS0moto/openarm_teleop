#!/usr/bin/env bash
./build/wifi_bimanual_follower \
  --right-can can2 \
  --left-can can3 \
  --right-port 50000 \
  --left-port 50001 \
  --control-rate-hz 500 \
  --right-urdf urdf/openarm_right.urdf \
  --left-urdf urdf/openarm_left.urdf
