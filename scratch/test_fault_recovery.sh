#!/bin/bash
# Fault recovery test script

pkill -f wifi_bimanual_leader
pkill -f wifi_bimanual_follower

echo "Starting Follower..."
./build/wifi_bimanual_follower --mock --control-rate-hz 500 --watchdog-disable-ms 1000 > follower_recovery.log 2>&1 &
FOLLOWER_PID=$!
sleep 1

echo "Starting Leader 1..."
./build/wifi_bimanual_leader --mock --follower-ip 127.0.0.1 --rate-hz 500 --enable > leader1.log 2>&1 &
LEADER1_PID=$!
sleep 2

echo "Killing Leader 1..."
kill $LEADER1_PID
sleep 2 # Let it time out

echo "Starting Leader 2 (Restart)..."
./build/wifi_bimanual_leader --mock --follower-ip 127.0.0.1 --rate-hz 500 --enable > leader2.log 2>&1 &
LEADER2_PID=$!
sleep 3

kill $FOLLOWER_PID $LEADER2_PID
wait $FOLLOWER_PID $LEADER2_PID 2>/dev/null

echo "Logs check:"
echo "--- Follower Log (Look for Resetting buffer and READY) ---"
grep -E "Resetting buffer|Transitioning to READY" follower_recovery.log
echo "--- Done ---"
