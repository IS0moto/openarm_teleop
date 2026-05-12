# openarm_wifi_bimanual_teleop

## 1. Overview
このリポジトリはOpenArmのWi-Fi経由bi-manual unilateral teleoperation用です。
ROS 2は使わず、C++ネイティブ実装による軽量なUDP通信で2台のPC間の状態交換を行います。
`openarm_teleop` の既存の1台PC構成（AdminThreadでの状態交換）をネットワークに拡張したものです。

## 2. Architecture
```text
Leader PC                               Follower PC
----------                              ------------
right leader CAN ─┐                 ┌─ right follower CAN
left  leader CAN ─┤                 ├─ left  follower CAN
                  │                 │
wifi_bimanual_leader  === UDP ===  wifi_bimanual_follower
```

## 3. Build
```bash
git clone <this_repo>
cd openarm_wifi_bimanual_teleop
mkdir -p build
cd build
cmake ..
make -j
```

## 4. Network test
```bash
# Follower PC
iperf3 -s

# Leader PC: 250 Hz相当
./script/network_test_250hz.sh 172.30.21.146

# Leader PC: 500 Hz相当
./script/network_test_500hz.sh 172.30.21.146
```

## 5. Run

Follower PC:
```bash
./build/wifi_bimanual_follower \
  --right-can can0 \
  --left-can can1 \
  --right-port 50000 \
  --left-port 50001 \
  --control-rate-hz 500 \
  --right-urdf urdf/openarm_right.urdf \
  --left-urdf urdf/openarm_left.urdf
```

Leader PC:
```bash
./build/wifi_bimanual_leader \
  --follower-ip 172.30.21.146 \
  --right-can can0 \
  --left-can can1 \
  --right-port 50000 \
  --left-port 50001 \
  --rate-hz 250 \
  --right-urdf urdf/openarm_right.urdf \
  --left-urdf urdf/openarm_left.urdf \
  --enable
```
初回は `--rate-hz 250` で確認し、問題なければ `--rate-hz 500` に上げてください。

## 6. Safety
- 起動直後は必ずdisabled
- `--enable` を渡さないと動かない
- watchdogあり
- estopあり
- Ctrl+C時にdisable_all
- 初期姿勢差チェック
- target rate limit
