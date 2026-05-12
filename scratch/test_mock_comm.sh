#!/bin/bash
# Mock test script for Bimanual Teleop

# Kill any existing processes
pkill -f wifi_bimanual_leader
pkill -f wifi_bimanual_follower

# Start Follower in mock mode
./build/wifi_bimanual_follower --mock --control-rate-hz 500 --watchdog-disable-ms 5000 > follower_mock.log 2>&1 &
FOLLOWER_PID=$!

# Wait for follower to start
sleep 2

# Start Leader in mock mode
./build/wifi_bimanual_leader --mock --follower-ip 127.0.0.1 --rate-hz 500 --enable > leader_mock.log 2>&1 &
LEADER_PID=$!

# Let them run for a few seconds
sleep 5

# Check if they are still running
if ps -p $FOLLOWER_PID > /dev/null; then
    echo "Follower is running."
else
    echo "Follower crashed!"
    cat follower_mock.log
fi

if ps -p $LEADER_PID > /dev/null; then
    echo "Leader is running."
else
    echo "Leader crashed!"
    cat leader_mock.log
fi

# Kill them
kill $FOLLOWER_PID $LEADER_PID
wait $FOLLOWER_PID $LEADER_PID 2>/dev/null

echo "Logs check:"
echo "--- Follower Log ---"
grep "Transitioning to READY" follower_mock.log
grep "Watchdog timeout" follower_mock.log
echo "--- Leader Log ---"
grep "Sent" leader_mock.log
