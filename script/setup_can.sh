#!/usr/bin/env bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can2 up
sudo ip link set can3 type can bitrate 1000000
sudo ip link set can3 up
