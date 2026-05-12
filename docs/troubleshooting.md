# トラブルシューティングガイド

このドキュメントは、Wi-Fi Bimanual Teleoperation システムの構築・運用中に発生した問題と解決策をまとめたものです。

---

## 1. CAN インターフェースが DOWN のまま

### 症状
- Leaderを起動しても全モーターが `pos=0 vel=0` で応答なし
- `candump can0` で何も表示されない

### 診断

```bash
ip link show can0
# → state DOWN の場合、CANが起動していない
```

### 原因
CAN インターフェースが初期化されていない。PC再起動後はCANが自動的にDOWN状態に戻る。

### 解決策

```bash
sudo openarm-can-configure-socketcan can0 -fd
sudo openarm-can-configure-socketcan can1 -fd
```

---

## 2. CAN通信は UP だがモーターが応答しない（TX: 0 packets）

### 症状
- CAN は `state UP` だが `ip -statistics link show can0` で TX/RX ともに 0
- `candump` にフレームが表示されない
- モーターの LED が点灯しているのにソフトウェアから通信できない

### 原因
`setup_can.sh` が **通常CANモード** で初期化していたが、OpenArm は **CAN FD モード** が必須。
通常CAN ソケットで CAN FD フレームを送信しようとするとサイレントに失敗する。

### 診断

```bash
# CAN統計を確認
ip -details -statistics link show can0
# TX packets が 0 の場合、フレーム送信がブロックされている

# 手動で通常CANフレームを送信してみる
cansend can0 001#DEADBEEF
# → 成功する場合、ドライバは正常だがFDモードの問題
```

### 解決策
公式ツールで CAN FD モードに再設定：

```bash
sudo ip link set can0 down
sudo openarm-can-configure-socketcan can0 -fd
```

### 根本対策
`script/setup_can.sh` を公式ツールを使う形に更新済み。

---

## 3. Follower が Watchdog Timeout で即座に FAULT になる

### 症状
```
[RIGHT] Enable signal received. Transitioning to READY.
[RIGHT] Watchdog hold triggered.
[RIGHT] Watchdog timeout! Disabling. Gap: 1001.25ms
Status [R: FAULT, Loss: 16653 | L: FAULT, Loss: 16653]
```

Leader は正常に送信しているのに、Follower の Loss カウントが異常に大きい。

### 原因
`TeleopStateBuffer::update()` のバグ。パケット受信時に `latest_packet_`、`last_seq_`、
`last_receive_time_ns_` が更新されておらず、最初のパケットのタイムスタンプのまま固定されていた。

結果、`gap = now - first_packet_time` が常に増加し続け、Watchdog が必ずタイムアウトした。

### 解決策
`teleop_state_buffer.cpp` の `update()` 関数で、正常パケット処理パスでも
パケットデータ・シーケンス番号・タイムスタンプを毎回更新するよう修正。

---

## 4. 左手だけが Watchdog Timeout する（Gap: 1.84467e+13ms）

### 症状
```
Status [R: ACTIVE, Loss: 0 | L: ACTIVE, Loss: 0]
[LEFT] Watchdog timeout! Disabling. Gap: 1.84467e+13ms
```

右手は正常に `ACTIVE` だが、左手だけ巨大な Gap 値で FAULT になる。

### 原因
`uint64_t` のアンダーフロー。Follower のメインループで `now_ns` を冒頭で1回だけ取得していた。
右手の CAN 制御処理中に UDP 受信スレッドが左手のタイムスタンプを更新すると、
`now_ns < last_receive_time_ns` となり、`uint64_t` の減算がアンダーフローして約 2^64 の値になった。

### 解決策
各アームの gap 計算時にタイムスタンプを個別に取得し、アンダーフロー防止のガード条件を追加：

```cpp
uint64_t now_l = utils::now_ns();
uint64_t last_l = state_buffer_l.get_last_receive_time_ns();
gap_l = (now_l >= last_l) ? (now_l - last_l) / 1e6 : 0.0;
```

---

## 5. Leader が起動するがモーターの LED が緑にならない

### 症状
- Leader 起動時に `Arm motor count: 7` と表示されるが、モーターの LED が赤のまま
- `enabled=NO` / `responding=NO` と表示される

### 診断

```bash
# モーターの応答を確認
./build/wifi_bimanual_leader \
  --interface wlp46s0 \
  --follower-ip <IP> \
  --right-can can0 \
  --left-can can1 \
  --rate-hz 250 \
  --enable
```

起動時のモーター検証ログを確認：
- `responding=NO, pos=0, vel=0` → **CAN通信不能**（→ 問題 1 or 2 を参照）
- `responding=YES, pos=非ゼロ` → **CAN通信OK、電源またはenableの問題**

### チェックリスト
1. ロボットアームの電源は入っているか？
2. CAN ケーブルは正しく接続されているか？
3. CAN FD モードで初期化されているか？
4. CAN ID 設定は正しいか？（`openarm_constants.hpp` 参照）

---

## 6. Follower のステータスログの読み方

### 正常時
```
Status [R: ACTIVE, Loss: 0 | L: ACTIVE, Loss: 0]
```

### ステータス一覧

| ステータス | 意味 |
|-----------|------|
| `WAIT_EN` | Leader からの enable 信号待ち |
| `READY` | enable 信号受信済み、制御開始直前 |
| `ACTIVE` | 正常動作中 |
| `WARN` | パケット遅延検知（20ms超） |
| `HOLD` | パケット遅延大（50ms超）、現在位置で保持 |
| `FAULT` | Watchdog タイムアウト、モーター無効化 |
| `ESTOP` | 緊急停止 |

### Loss カウントの解釈
- `Loss: 0` → パケットロスなし（理想）
- `Loss: 数百以下` → Wi-Fi 環境では許容範囲
- `Loss: 急増` → ネットワーク不安定 or ソフトウェアバグ

---

## 7. デバッグ用コマンド集

### CAN 状態確認
```bash
# インターフェースの状態
ip link show can0

# 詳細統計
ip -details -statistics link show can0

# CANフレームのモニタリング
candump can0 -t a

# テストフレーム送信
cansend can0 001#DEADBEEF
```

### ネットワーク確認
```bash
# Wi-FiインターフェースのIP確認
ip addr show wlp46s0

# Followerへの疎通確認
ping <Follower_IP>

# UDPポートの使用確認
ss -ulnp | grep 5000
```

### プロセス管理
```bash
# 実行中のテレオペプロセスを終了
pkill -f wifi_bimanual

# 強制終了
pkill -9 -f wifi_bimanual
```
