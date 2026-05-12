#!/usr/bin/env bash
./build/wifi_single_leader \
  --follower-ip 172.30.21.146 \
  --can can0 \
  --port 50000 \
  --rate-hz 500 \
  --urdf urdf/openarm_right.urdf \
  --enable
