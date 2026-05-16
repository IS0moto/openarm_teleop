# Component Tests and Verification

## Build Verification

```bash
cmake --build build -j"$(nproc)"
```

Individual targets:

```bash
cmake --build build --target wifi_bimanual_leader -j2
cmake --build build --target wifi_bimanual_follower -j2
cmake --build build --target wifi_vr_bimanual_leader -j2
```

Quest sender:

```bash
cd ../quest2_openxr_logger
cmake --build build --target openxr_relative_sender -j2
```

## Unit Tests

The `tests/` directory contains gtest tests for:

- packet encode/decode CRC validation
- sequence loss detection
- state buffer latest-packet behavior
- watchdog transitions
- rate limiting

At the moment, these test sources are present but not wired into `CMakeLists.txt`. To run them, add gtest and test executable targets, or compile them in your local test harness.

Files:

```text
tests/test_packet_codec.cpp
tests/test_seq_loss_detection.cpp
tests/test_state_buffer.cpp
tests/test_watchdog.cpp
tests/test_rate_limiter.cpp
```

## Network Tests

```bash
# Receiver side
iperf3 -s

# Sender side
./script/network_test_250hz.sh <Receiver_IP>
./script/network_test_500hz.sh <Receiver_IP>
```

Interpretation:

- 0% packet loss is the target for real robot operation.
- Jitter should stay well below the watchdog thresholds.
- Test while cameras/VR streaming are active if they share the same network.

## Mock Leader/Follower Test

```bash
# Terminal A
./build/wifi_bimanual_follower --mock --bind-ip 127.0.0.1

# Terminal B
./build/wifi_bimanual_leader --mock --follower-ip 127.0.0.1 --enable
```

Expected:

- leader prints sent packet counts
- follower stays out of FAULT while packets arrive
- stopping leader should move follower safety state toward timeout/HOLD/FAULT

## Telemetry Test

Follower telemetry:

```bash
./build/wifi_bimanual_follower \
  --mock \
  --publish-telemetry \
  --telemetry-ip 127.0.0.1 \
  --telemetry-port 51000
```

Receiver sample:

```bash
./build/openarm_telemetry_receiver_sample --bind-ip 127.0.0.1 --port 51000
```

Note: `tools/telemetry_receiver_sample.cpp` and `tools/telemetry_mock_publisher.cpp` are source files in this repo, but the current CMake file does not build them by default.

## VR Mapping Dry Run

```bash
./build/wifi_vr_bimanual_leader \
  --right-urdf urdf/openarm_right.urdf \
  --left-urdf urdf/openarm_left.urdf \
  --config config/vr_teleop_config.yaml \
  --dry-run \
  --vr-mapping-test \
  --ik-debug \
  --max-test-sec 60 \
  --log-csv /tmp/vr_ik_debug.csv
```

This validates:

- OpenXR-to-robot basis printed at startup
- `invert_rotation_delta`
- `rotation_compose_order`
- IK residuals and rate limiting in CSV

## Quest Sender Pose Source Test

Compare:

```bash
./build/openxr_relative_sender --dest-ip <Leader_IP> --position-pose grip --rotation-pose aim
./build/openxr_relative_sender --dest-ip <Leader_IP> --position-pose aim --rotation-pose aim
./build/openxr_relative_sender --dest-ip <Leader_IP> --position-pose grip --rotation-pose grip
```

Current preferred default is:

```text
position=grip, rotation=aim
```
