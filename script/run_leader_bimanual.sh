#!/usr/bin/env bash
./build/wifi_bimanual_leader \
  --follower-ip 172.30.21.146 \
  --right-can can0 \
  --left-can can1 \
  --right-port 50000 \
  --left-port 50001 \
  --rate-hz 500 \
  --right-urdf urdf/openarm_right.urdf \
  --left-urdf urdf/openarm_left.urdf \
  --enable
