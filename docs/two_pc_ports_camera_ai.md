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
| `52000` | UDP | Follower -> Leader | `lerobot_teleop_recorder` preview streamer | GUI or viewer | H.264/RTP preview, one stream, camera selectable at runtime |
| `52200`+i | UDP | Follower -> Leader | `lerobot_teleop_recorder` AI streamer | AI controller | One stream per camera index (front=52200, top=52201) |
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

This direct layout is for low-level bring-up and troubleshooting only. Normal
GUI/session operation should use the Control Arbiter layout above so command
ownership is selected by `set_mode`.

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
./scripts/session/start_openarm_session.sh      # reads config/session.yaml
```

Cameras, preview, AI streams, and ports are all described by
`config/session.yaml`; the session takes no other arguments. The recorder
consumes telemetry locally on `127.0.0.1:51000`, records every camera locally,
and streams low-latency H.264/RTP preview to `<LEADER_PC_IP>:52000`.

## GUI on the Leader PC

The GUI sends TCP runtime commands to both C++ processes and recording commands
to the recorder:

```yaml
leader:   { host: 127.0.0.1, control_port: 53201 }
follower: { host: <FOLLOWER_PC_IP>, control_port: 53200 }
recorder: { host: <FOLLOWER_PC_IP>, control_port: 53100 }
preview:
  host: <FOLLOWER_PC_IP>
  port: 52000
  width: 640
  height: 480
```

The GUI video receiver binds a local UDP port. `preview.host` is operator
context; `preview.port` is the receiver setting, and `width`/`height` must match
the recorder's `preview` block because the receiver reads fixed-size raw frames.

## AI Controller Trial

The AI controller can replace the physical leader as the command source:

```bash
cd ~/openarm_isolate_teleop/openarm_ai_teleop
uv run ai_controller.py --config config/ai.yaml
```

The controller reads the loaded policy's image keys and asks the recorder to
stream exactly those cameras on `52200 + index`.

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

## Camera Streams

The preview and the AI streams are separate, and each has exactly one reader, so
no fanout process is involved.

```text
lerobot_teleop_recorder
  preview streamer  -> <LEADER_PC_IP>:52000           GUI (one stream)
  ai streamers      -> <LEADER_PC_IP>:52200 + index   AI controller (per camera)
```

The preview always exists while the session runs; which camera it shows is
changed with `select_preview` from the GUI or the CLI. The AI streams are
started only when the AI controller asks for them and stopped when it exits or
control mode leaves AI.

A UDP port can only be read by one process, so do not point the GUI and a
standalone viewer at `52000` at the same time.

## Quick Checks

On the Follower PC:

```bash
ss -ulnp | grep -E '50000|50001|51000|51010'
ss -tlnp | grep -E '53100|53200'
```

On the Leader PC:

```bash
ss -ulnp | grep -E '51000|51011|52000|52200|52201|54002'
ss -tlnp | grep -E '53201'
```

Network bandwidth tests should include camera streaming because video shares
the same Wi-Fi path as control:

```bash
cd ~/openarm_isolate_teleop/lerobot_teleop_recorder
./scripts/network/iperf_video_only.sh <LEADER_PC_IP> 4M
```
