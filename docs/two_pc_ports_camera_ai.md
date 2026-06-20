# Two-PC Ports With Camera, Recording, GUI, and AI

This note organizes the network ports when running physical leader/follower
teleop on two PCs while also using the follower-side camera, LeRobot recording,
GUI preview, and AI controller trials.

## Assumed PCs

```text
Leader PC / Controller PC: <LEADER_PC_IP>
Follower PC / Robot PC:    <FOLLOWER_PC_IP>
```

Use fixed IP addresses or DHCP reservations. Avoid changing ports during robot
bring-up unless there is a clear conflict.

## Recommended Port Map

| Port | Proto | Direction | Producer | Consumer | Purpose |
| --- | --- | --- | --- | --- | --- |
| `50000` | UDP | Leader or AI -> Follower | `wifi_bimanual_leader` or `ai_controller.py` | `wifi_bimanual_follower` | Right arm command stream |
| `50001` | UDP | Leader or AI -> Follower | `wifi_bimanual_leader` or `ai_controller.py` | `wifi_bimanual_follower` | Left arm command stream |
| `51000` | UDP | local or routed | `wifi_bimanual_follower --publish-telemetry` or fanout | recorder, AI, VR leader, tools | OpenArm follower telemetry |
| `51010` | UDP | Follower local fanout input | `wifi_bimanual_follower` | `telemetry_fanout` | Optional telemetry fanout input on Follower PC |
| `51011` | UDP | Follower -> Leader | Follower-side `telemetry_fanout` | Leader-side `telemetry_fanout` | Optional telemetry trunk to Leader PC |
| `52000` | UDP | Follower -> Leader | `lerobot_teleop_recorder` preview streamer | GUI, AI, viewer, or video fanout | H.264/RTP camera preview |
| `52001` | UDP | Leader local fanout output | `udp_fanout` | GUI | Optional GUI camera input |
| `52002` | UDP | Leader local fanout output | `udp_fanout` | AI controller | Optional AI camera input |
| `52100` | TCP/HTTP | Leader -> Follower | `lerobot_teleop_recorder` HTTP preview | browser | Optional browser preview |
| `53100` | TCP | Leader GUI -> Follower | `lerobot_teleop_recorder` control server | GUI or CLI client | Start/stop recording, status, shutdown |
| `53200` | TCP | Leader GUI/AI -> Follower | `wifi_bimanual_follower` runtime control server | GUI or AI | Follower status, init, recover, runtime commands |
| `53201` | TCP | GUI -> Leader | `wifi_bimanual_leader` runtime control server | GUI | Leader status, enable/disable, init |
| `54002` | UDP | Quest -> VR Leader | `openxr_relative_sender` | `wifi_vr_bimanual_leader` | VR relative pose input |
| `50100` / `50101` | UDP | Leader -> Arbiter | `wifi_bimanual_leader` | Control Arbiter | Right/left physical leader command candidates |
| `50200` / `50201` | UDP | AI -> Arbiter | `ai_controller.py` | Control Arbiter | Right/left AI command candidates |
| `50300` / `50301` | UDP | VR -> Arbiter | `wifi_vr_bimanual_leader` | Control Arbiter | Right/left VR command candidates |
| `53300` | TCP JSON | GUI/CLI -> Session Manager | GUI or CLI | `openarm_session_manager` | Mode, recording, status operations |
| `53301` | TCP JSON | Session Manager -> Arbiter | `openarm_session_manager` | Control Arbiter | Arbiter mode/status control |

`50000` and `50001` are actuator command ports. Only one controller should send
enabled commands to them at a time. With the Control Arbiter enabled, only the
Arbiter sends to `50000/50001`; physical leader, AI, and VR send to their own
candidate input ports.

## Control Arbiter Mode

Run the Control Arbiter on the Leader PC:

```bash
cd ~/openarm_isolate_teleop/openarm_session_manager
cp config/arbiter.example.json config/arbiter.json
# edit target.ip to <FOLLOWER_PC_IP>
./.venv/bin/openarm-control-arbiter --config config/arbiter.json
```

Run the Session Manager and use it to choose the active owner:

```bash
cp config/session_manager.example.json config/session_manager.json
# edit follower/recorder hosts and keep arbiter enabled at 127.0.0.1:53301
./.venv/bin/openarm-session-manager --config config/session_manager.json

./.venv/bin/openarm-session-control set-mode disabled
./.venv/bin/openarm-session-control set-mode leader
./.venv/bin/openarm-session-control set-mode ai
```

With this layout, start the physical leader through the Arbiter input ports:

```bash
cd ~/openarm_isolate_teleop/openarm_wifi_bimanual_teleop
./script/run_leader_bimanual_via_arbiter.sh
```

Start AI through its Arbiter input ports:

```bash
cd ~/openarm_isolate_teleop/openarm_ai_teleop
MODEL_PATH=<path-or-hf-repo> \
FOLLOWER_IP=<FOLLOWER_PC_IP> \
CONTROL_DEST_IP=127.0.0.1 \
./scripts/run_ai_standard_ports.sh
```

## Baseline Physical Leader/Follower

Start the Follower PC first:

```bash
cd ~/openarm_isolate_teleop/openarm_wifi_bimanual_teleop
./build/wifi_bimanual_follower \
  --interface <FOLLOWER_NET_IFACE> \
  --right-can can0 \
  --left-can can1 \
  --right-urdf urdf/openarm_right.urdf \
  --left-urdf urdf/openarm_left.urdf \
  --right-port 50000 \
  --left-port 50001 \
  --control-rate-hz 500 \
  --watchdog-hold-ms 50 \
  --watchdog-disable-ms 1000 \
  --control-bind-ip 0.0.0.0 \
  --control-port 53200
```

Start the Leader PC:

```bash
cd ~/openarm_isolate_teleop/openarm_wifi_bimanual_teleop
./build/wifi_bimanual_leader \
  --interface <LEADER_NET_IFACE> \
  --follower-ip <FOLLOWER_PC_IP> \
  --right-can can0 \
  --left-can can1 \
  --right-urdf urdf/openarm_right.urdf \
  --left-urdf urdf/openarm_left.urdf \
  --right-port 50000 \
  --left-port 50001 \
  --rate-hz 500 \
  --enable \
  --control-bind-ip 0.0.0.0 \
  --control-port 53201
```

## Recording on the Follower PC

For simple recording only, publish follower telemetry to the local recorder on
the Follower PC:

```bash
./build/wifi_bimanual_follower \
  ... \
  --publish-telemetry \
  --telemetry-ip 127.0.0.1 \
  --telemetry-port 51000 \
  --telemetry-rate-hz 100 \
  --control-bind-ip 0.0.0.0 \
  --control-port 53200
```

Then start the resident recorder/camera session on the Follower PC:

```bash
cd ~/openarm_isolate_teleop/lerobot_teleop_recorder
LEADER_PC_IP=<LEADER_PC_IP> \
TELEMETRY_PORT=51000 \
PREVIEW_PORT=52000 \
PREVIEW_HTTP_PORT=52100 \
CONTROL_PORT=53100 \
CAMERA_TYPE=realsense \
./scripts/session/start_openarm_session.sh
```

The recorder consumes telemetry locally on `127.0.0.1:51000`, records camera
frames locally, and streams low-latency H.264/RTP preview to
`<LEADER_PC_IP>:52000`.

## GUI on the Leader PC

The GUI sends TCP runtime commands to both C++ processes and recording commands
to the recorder:

```json
{
  "leader": {
    "host": "127.0.0.1",
    "control_port": 53201
  },
  "follower": {
    "host": "<FOLLOWER_PC_IP>",
    "control_port": 53200
  },
  "recorder": {
    "host": "<FOLLOWER_PC_IP>",
    "control_port": 53100
  },
  "preview": {
    "host": "<FOLLOWER_PC_IP>",
    "port": 52000
  }
}
```

The GUI video receiver binds a local UDP port. The `preview.host` value is kept
for operator context; the important receiver setting is `preview.port`.

## AI Controller Trial

The AI controller can replace the physical leader as the command source:

```bash
cd ~/openarm_isolate_teleop/openarm_ai_teleop
uv run ai_controller.py \
  --model-path <path-or-hf-repo> \
  --follower-ip <FOLLOWER_PC_IP> \
  --control-port-r 50000 \
  --control-port-l 50001 \
  --telemetry-port 51000 \
  --tcp-port 53200 \
  --gst-port 52000 \
  --init-arms
```

Do not run an enabled physical leader and AI controller against the same
Follower at the same time. If you keep `wifi_bimanual_leader` open for GUI
status or initialization, disable its command stream before starting AI control.

## Telemetry Fanout

Use fanout when telemetry is needed by more than one process, for example:

- local recorder on the Follower PC
- AI or VR leader on the Leader PC
- telemetry logging/debug tools

Recommended pattern:

```text
wifi_bimanual_follower
  -> 127.0.0.1:51010
  -> Follower telemetry_fanout
     -> 127.0.0.1:51000       recorder on Follower PC
     -> <LEADER_PC_IP>:51011  telemetry trunk to Leader PC
  -> Leader telemetry_fanout
     -> 127.0.0.1:51000       AI or VR leader
     -> 127.0.0.1:51002       optional telemetry tool
```

Follower PC:

```bash
cd ~/openarm_isolate_teleop/openarm_wifi_bimanual_teleop
LEADER_PC_IP=<LEADER_PC_IP> ./script/start_follower_telemetry_fanout.sh
```

Run the follower telemetry publisher toward the fanout input:

```bash
./build/wifi_bimanual_follower \
  ... \
  --publish-telemetry \
  --telemetry-ip 127.0.0.1 \
  --telemetry-port 51010 \
  --telemetry-rate-hz 100
```

Leader PC:

```bash
cd ~/openarm_isolate_teleop/openarm_wifi_bimanual_teleop
./script/start_leader_telemetry_fanout.sh
```

Configure each telemetry consumer to a unique local port. For example, AI uses
`--telemetry-port 51000`, while a debug tool can use `51002`.

## Camera Fanout

Use camera fanout when both GUI rendering and AI inference need the same
Follower camera stream on the Leader PC.

```text
lerobot_teleop_recorder preview streamer
  -> <LEADER_PC_IP>:52000
  -> Leader udp_fanout
     -> 127.0.0.1:52001  GUI
     -> 127.0.0.1:52002  AI
```

Leader PC:

```bash
cd ~/openarm_isolate_teleop/openarm_wifi_bimanual_teleop
./script/start_leader_camera_fanout.sh
```

Then set:

```text
GUI config.json preview.port = 52001
AI controller --gst-port 52002
```

If only one process needs live video, skip the fanout and use `52000` directly.

## Quick Checks

On the Follower PC:

```bash
ss -ulnp | grep -E '50000|50001|51000|51010'
ss -tlnp | grep -E '53100|53200|52100'
```

On the Leader PC:

```bash
ss -ulnp | grep -E '51000|51011|52000|52001|52002|54002'
ss -tlnp | grep -E '53201'
```

Network bandwidth tests should include camera streaming because video shares
the same Wi-Fi path as control:

```bash
cd ~/openarm_isolate_teleop/lerobot_teleop_recorder
./scripts/network/iperf_video_only.sh <LEADER_PC_IP> 4M
```
