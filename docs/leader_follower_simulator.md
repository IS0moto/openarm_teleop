# Simulator and Mock-Mode Leader/Follower

There are two simulator-like workflows:

1. **Repo-local mock mode**: exercise UDP, packet loss detection, watchdogs, and process wiring without CAN hardware.
2. **ROS 2 simulator bridge**: send this repo's UDP commands into a simulator through the adjacent `openarm_udp_bridge` package.

## 1. Repo-Local Mock Mode

Mock mode does not simulate robot physics. It is useful for validating ports, IPs, packet flow, and safety behavior.

### Terminal A: Mock Follower

```bash
cd openarm_wifi_bimanual_teleop
./build/wifi_bimanual_follower \
  --mock \
  --bind-ip 127.0.0.1 \
  --right-port 50000 \
  --left-port 50001 \
  --control-rate-hz 500 \
  --watchdog-disable-ms 1000 \
  --publish-telemetry
```

### Terminal B: Mock Leader

```bash
cd openarm_wifi_bimanual_teleop
./build/wifi_bimanual_leader \
  --mock \
  --follower-ip 127.0.0.1 \
  --right-port 50000 \
  --left-port 50001 \
  --rate-hz 500 \
  --enable
```

The mock leader sends synthetic sine/cosine joint targets. The mock follower receives packets and runs the state buffer/safety loop without CAN output.

## 2. ROS 2 Simulator Through UDP Bridge

Use this when a ROS 2 simulator accepts joint commands through `openarm_udp_bridge`.

### Terminal A: Simulator

Start your OpenArm simulator according to that package's instructions.

### Terminal B: UDP Bridge

From the ROS 2 workspace containing `openarm_udp_bridge`:

```bash
source install/setup.bash
ros2 run openarm_udp_bridge udp_to_ros2_bridge \
  --ros-args \
  -p right_port:=50000 \
  -p left_port:=50001 \
  -p telemetry_port:=51000 \
  -p leader_ip:=127.0.0.1
```

Parameter names may differ if the bridge has changed; check `../openarm_udp_bridge/README.md`.

### Terminal C: Leader

Physical leader:

```bash
./build/wifi_bimanual_leader \
  --follower-ip 127.0.0.1 \
  --right-can can0 \
  --left-can can1 \
  --right-urdf urdf/openarm_right.urdf \
  --left-urdf urdf/openarm_left.urdf \
  --rate-hz 250 \
  --enable
```

Mock leader:

```bash
./build/wifi_bimanual_leader \
  --mock \
  --follower-ip 127.0.0.1 \
  --rate-hz 250 \
  --enable
```

## VR With Simulator

For VR IK into the simulator, run the simulator and UDP bridge, then:

```bash
./build/wifi_vr_bimanual_leader \
  --follower-ip 127.0.0.1 \
  --telemetry-port 51000 \
  --config config/vr_teleop_config.yaml
```

Then start the Quest sender:

```bash
cd ../quest2_openxr_logger
./build/openxr_relative_sender --dest-ip 127.0.0.1 --dest-port 54002
```

## Useful Checks

```bash
ss -ulnp | grep -E '50000|50001|51000|54002'
ros2 topic list
ros2 topic echo /joint_states
```
