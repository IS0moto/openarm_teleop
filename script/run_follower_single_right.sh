#!/usr/bin/env bash
./build/wifi_single_follower \
  --can can2 \
  --port 50000 \
  --control-rate-hz 500 \
  --urdf urdf/openarm_right.urdf
