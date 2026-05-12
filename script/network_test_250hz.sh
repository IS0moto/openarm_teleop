#!/usr/bin/env bash
set -e

FOLLOWER_IP=${1:-172.30.21.146}
iperf3 -c "$FOLLOWER_IP" -u -b 500K -l 250 -t 60 --get-server-output
