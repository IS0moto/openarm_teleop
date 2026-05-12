# Wi-Fi Bimanual Teleop 構築ガイド

このドキュメントは、OpenArm Wi-Fi Bimanual Teleoperation システムのセットアップ手順をまとめたものです。

## 前提条件

- 2台のPC（Leader PC / Follower PC）が同一Wi-Fiネットワークに接続済み
- 各PCに PEAK USB-CAN Pro FD アダプタが接続済み
- 各PCに2本のOpenArmが CAN 経由で接続済み（右腕: can0, 左腕: can1）
- `libopenarm-can-dev` および `openarm-can-utils` がインストール済み

## Step 1: 依存パッケージのインストール

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:openarm/main
sudo apt update
sudo apt install -y \
  libeigen3-dev \
  libopenarm-can-dev \
  liborocos-kdl-dev \
  liburdfdom-dev \
  liburdfdom-headers-dev \
  libyaml-cpp-dev \
  openarm-can-utils
```

> 参照: https://docs.openarm.dev/teleop/leader-follower/setup-guide

## Step 2: ビルド

```bash
git clone <repository_url>
cd openarm_wifi_bimanual_teleop
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

## Step 3: CAN FD インターフェースの初期化

**⚠️ 重要: OpenArmは CAN FD モードが必須です。通常のCAN設定では通信できません。**

各PCで以下のコマンドを実行します：

```bash
# 公式ツールを使用してCAN FDモードで初期化
sudo openarm-can-configure-socketcan can0 -fd
sudo openarm-can-configure-socketcan can1 -fd
```

成功すると以下のように表示されます：
```
Configuring can0...
can0 is now set to CAN FD mode (1000000 bps / 5000000 bps)
✓ can0 is active
```

### CAN状態の確認

```bash
# UPになっていることを確認
ip link show can0
# → state UP であること

# 統計情報の確認
ip -details -statistics link show can0
```

## Step 4: URDF ファイルの配置

```bash
mkdir -p urdf
cp ~/openarm-ws/openarm_bimanual_control.urdf urdf/openarm_right.urdf
cp ~/openarm-ws/openarm_bimanual_control.urdf urdf/openarm_left.urdf
```

## Step 5: ネットワーク疎通確認

```bash
# Follower PC 側で iperf3 サーバー起動
iperf3 -s

# Leader PC 側からテスト（250Hz相当のUDPパケットサイズ）
./script/network_test_250hz.sh <Follower_IP>
```

## Step 6: 起動

### 6-1. Follower PC（Robot側）を先に起動

```bash
./build/wifi_bimanual_follower \
  --interface wlp46s0 \
  --right-can can0 \
  --left-can can1 \
  --right-port 50000 \
  --left-port 50001 \
  --control-rate-hz 500 \
  --watchdog-disable-ms 1000
```

起動時にモーター検証が行われます：
```
[RIGHT] All motors verified OK on can0
[LEFT] All motors verified OK on can1
Adjust complete.
Status [R: WAIT_EN, Loss: 0 | L: WAIT_EN, Loss: 0]
```

`WAIT_EN` が表示されたらLeader側を起動できます。

### 6-2. Leader PC（Operator側）を起動

```bash
./build/wifi_bimanual_leader \
  --interface wlp46s0 \
  --follower-ip <Follower_IP> \
  --right-can can0 \
  --left-can can1 \
  --rate-hz 250 \
  --enable
```

正常に接続されると、Follower側のステータスが以下に変わります：
```
[RIGHT] Enable signal received. Transitioning to READY.
[LEFT] Enable signal received. Transitioning to READY.
Status [R: ACTIVE, Loss: 0 | L: ACTIVE, Loss: 0]
```

### 6-3. レート調整

初回は `--rate-hz 250` で確認し、通信が安定していれば `--rate-hz 500` に上げてください。

## ネットワークインターフェース名の確認

```bash
# Wi-Fiインターフェース名を確認
ip addr show | grep -E "^[0-9]+:" | grep -v lo
# 例: wlp46s0, wlan0 など
```