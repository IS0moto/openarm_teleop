# Telemetry Integration Guide

This guide explains how to enable and consume the telemetry stream provided by the `openarm_wifi_bimanual_teleop` follower node.

## Why is it decoupled from LeRobot?

The OpenArm bimanual teleoperation runs a strict, soft real-time 500 Hz control loop in C++. Integrating heavy Python runtimes, video capture, or data serialization inside this control loop causes jitter, watchdog timeouts, and instability.

By decoupling the telemetry:
- The 500 Hz control loop remains deterministic and safe.
- Data logging can crash or restart without dropping the robot.
- The telemetry stream can be consumed by *any* recorder (not just LeRobot).

## Enabling the Telemetry Publisher

To enable the telemetry publisher, run `wifi_bimanual_follower` with the following CLI arguments:

```bash
./build/wifi_bimanual_follower \
  --interface wlp46s0 \
  --right-can can0 \
  --left-can can1 \
  --right-port 50000 \
  --left-port 50001 \
  --control-rate-hz 500 \
  --watchdog-disable-ms 1000 \
  --publish-telemetry \
  --telemetry-ip 127.0.0.1 \
  --telemetry-port 51000 \
  --telemetry-rate-hz 100
```

- `--publish-telemetry`: Turns on the UDP broadcast.
- `--telemetry-ip`: The destination IP (can be `127.0.0.1` for local recording, or a remote IP).
- `--telemetry-port`: The destination UDP port.
- `--telemetry-rate-hz`: How often to emit packets. Default is 100 Hz.

## Consuming the Telemetry

You can consume the telemetry using any language that supports UDP sockets. A C++ example is provided in `tools/telemetry_receiver_sample.cpp`.

For Python integration (e.g., LeRobot), see the `lerobot_teleop_recorder` repository which implements a robust `TelemetryClient` that validates magic numbers, sequencing, and packet recency.

## Verifying Telemetry Broadcast

To verify that the telemetry data is being correctly broadcasted over the network when `--publish-telemetry` is enabled, you can run the provided receiver sample tool from Repo A:

```bash
cd openarm_wifi_bimanual_teleop
./build/openarm_telemetry_receiver_sample --bind-ip 127.0.0.1 --port 51000
```

If the data is being successfully received, you will see a continuous stream of packet information:

```text
Listening for telemetry on 127.0.0.1:51000
Received packet seq: 1 state_dim: 16 [R1 pos: 0.123]
Received packet seq: 2 state_dim: 16 [R1 pos: 0.124]
```

You can also use standard command-line tools like `tcpdump` to verify UDP packets on the port:
```bash
sudo tcpdump -i lo udp port 51000
```
