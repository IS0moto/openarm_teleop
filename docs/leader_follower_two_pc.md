# Two-PC Physical Leader/Follower Teleop

This workflow uses one PC connected to physical leader arms and another PC connected to physical follower arms.

## Network

Default UDP ports:

| Stream | Port |
| --- | --- |
| Right arm command | `50000` |
| Left arm command | `50001` |
| Telemetry | `51000` |

Example:

```text
Leader PC:   192.168.10.20
Follower PC: 192.168.10.30
```

Run a quick network test before real motion:

```bash
# Follower PC
iperf3 -s

# Leader PC
./script/network_test_250hz.sh 192.168.10.30
./script/network_test_500hz.sh 192.168.10.30
```

Aim for low jitter and no packet loss.

## Follower PC

```bash
cd openarm_wifi_bimanual_teleop
sudo openarm-can-configure-socketcan can0 -fd
sudo openarm-can-configure-socketcan can1 -fd

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
  --watchdog-disable-ms 1000
```

Add telemetry when using VR, recording, or remote monitoring:

```bash
  --publish-telemetry \
  --telemetry-ip <LEADER_PC_IP> \
  --telemetry-port 51000 \
  --telemetry-rate-hz 100
```

Expected startup state before leader enable:

```text
Status [R: WAIT_EN, Loss: 0 | L: WAIT_EN, Loss: 0]
```

## Leader PC

```bash
cd openarm_wifi_bimanual_teleop
sudo openarm-can-configure-socketcan can0 -fd
sudo openarm-can-configure-socketcan can1 -fd

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
  --enable
```

Use `--rate-hz 250` for first tests. Increase to `500` after network and watchdog behavior are stable.

## Useful Options

Follower:

```bash
--bind-ip <ip>
--interface <iface>
--watchdog-hold-ms 50
--watchdog-disable-ms 1000
--telemetry-log-stats
```

Leader:

```bash
--bind-ip <ip>
--interface <iface>
--local-port-r <port>
--local-port-l <port>
--publish-telemetry
--mock
```

## Shutdown

Stop leader first, then follower:

```bash
Ctrl+C
./script/kill_teleop.sh
```
