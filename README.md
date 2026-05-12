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

### Follower PC (Robot Side)
```bash
# wlp46s0 インターフェースを使用して起動
./build/wifi_bimanual_follower \
  --interface wlp46s0 \
  --right-can can0 \
  --left-can can1 \
  --right-port 50000 \
  --left-port 50001 \
  --control-rate-hz 500 \
  --watchdog-disable-ms 1000
```

### Leader PC (Operator Side)
```bash
# 相手側(Follower)のIPを指定して起動
./build/wifi_bimanual_leader \
  --interface wlp46s0 \
  --follower-ip 172.30.21.146 \
  --right-can can0 \
  --left-can can1 \
  --rate-hz 250 \
  --enable
```

初回は `--rate-hz 250` で確認し、通信が安定していれば `--rate-hz 500` に上げてください。

## 6. Options

| Option | Description | Default |
|--------|-------------|---------|
| `--interface` | 使用するネットワークインターフェース名 (例: `wlp46s0`) | (空) |
| `--bind-ip` | 待ち受け/送信元のIPアドレスを直接指定する場合 | `0.0.0.0` |
| `--follower-ip` | (Leaderのみ) 送信先のIPアドレス | - |
| `--rate-hz` | 通信・制御レート | 500 (Follower) / 250 (Leader) |
| `--enable` | (Leaderのみ) 送信開始フラグ | false |
| `--watchdog-disable-ms` | 通信切断とみなす許容時間 (Wi-Fi環境では 500-1000 推奨) | 100 |

## 7. Troubleshooting

### URDF Error / Segmentation Fault
`urdf/` フォルダに必要な URDF ファイルが配置されているか確認してください。
```bash
mkdir -p urdf
cp ~/openarm-ws/openarm_bimanual_control.urdf urdf/openarm_right.urdf
cp ~/openarm-ws/openarm_bimanual_control.urdf urdf/openarm_left.urdf
```

### Watchdog Timeout
Wi-Fi環境で遅延が発生する場合、Follower側で `--watchdog-disable-ms 1000` のように閾値を広げてください。

### Permission Denied (CAN)
CANインターフェースへのアクセス権限がない場合は、`sudo` をつけるか `udev` ルールを設定してください。
- 起動直後は必ずdisabled
- `--enable` を渡さないと動かない
- watchdogあり
- estopあり
- Ctrl+C時にdisable_all
- 初期姿勢差チェック
- target rate limit
