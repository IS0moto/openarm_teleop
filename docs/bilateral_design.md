# バイラテラル制御 設計ドキュメント

対象: `openarm_wifi_bimanual_teleop` (leader PC1 / follower PC2 の 2 PC 構成)。
現行のユニラテラル (leader 位置 → follower 追従) を、follower の状態
(位置・速度・**トルク**) を leader に返して力覚提示するバイラテラルへ拡張する。

- Phase 1 (実装済み): 戻り経路の追加、トルクを含む feedback の伝送、
  有線・通信品質ゲート、運用コマンド。**leader 腕には何も反力を出さない。**
- Phase 2 (未実装): leader 側で feedback を制御に使う (係合条件・ゲインランプ・
  トルククランプ込み)。
- Phase 3 (未実装): GUI トグル、ゲインの追い込み、力チャネル。

## 1. 現状と再利用できるもの

| 項目 | 内容 |
| --- | --- |
| 制御則 | `Control::bilateral_step()` (`src/controller/control.cpp`) は既に位置対称型 (P-P) バイラテラル。両腕が相手位置を MIT PD で追従し、重力・摩擦を FF。単一 PC 版 `control/openarm_bilateral_control.cpp` が 1 kHz で使用 |
| follower | `unilateral_step()` follower 分岐 = PD 追従。既に「バイラテラルの半分」 |
| leader | `unilateral_step()` leader 分岐 = 重力補償のみ (Kp=Kd=0)。受信経路が無い |
| Arbiter | leader/AI/VR → follower の片方向。mode で転送元を選ぶ |
| リンク | PC1↔PC2 は eth0 有線 1000 Mbps。ping RTT 実測 1.6–2.5 ms |

## 2. 全体構成 (Phase 1 以降)

```text
leader  --TeleopPacket(mode=BILATERAL)-->  Arbiter 50100/50101  --mode==leader-->  follower 50000/50001
leader  <--FeedbackPacket(q,dq,tau)------  Arbiter 50500/50501  <--mode==leader--  follower -> Arbiter 50400/50401
```

- 戻り経路も Arbiter を経由する。**mode が `leader` のときだけ** feedback を
  leader へ転送するので、AI/VR モード中の follower の動きが leader 腕へ届く
  ことは構造的に無い (INSTRUCTION_FOR_ME の「MODE 切替は必ず Arbiter」を維持)。
- telemetry (100 Hz, float, recorder/AI 向け) は変更しない。feedback は
  500 Hz・double・トルク付きの別パケットで、用途を分ける。

## 3. プロトコル

### 3.1 TeleopPacket (既存, v1 のまま)

`mode` フィールドに `ControlMode::BILATERAL = 1` を追加。レイアウト・サイズ・
CRC は不変なので、AI controller (Python) と VR leader の送信コードは影響を受け
ない。leader は `bilateral_requested` の間だけ `mode=1` を送る。follower は
受け取った `mode` を feedback にエコーする (leader から「follower がこちらの
要求を見ている」ことが分かる)。

### 3.2 FeedbackPacket (新規, `include/openarm_wifi_teleop/net/feedback_packet.hpp`)

340 byte, little-endian, magic `'OAFB'` (0x4246414F), version 1。先頭 24 byte
(magic/version/packet_size/seq/send_time_ns) は TeleopPacket と同じ並びで、
Arbiter が同じヘッダパーサで検証できる。

| フィールド | 用途 |
| --- | --- |
| `arm_side`, `mode`, `enabled`, `estop` | follower の腕・エコーした mode・ACTIVE か・ESTOP か |
| `safety_state`, `link_ok` | follower の `SafetyState`、follower 側の有線チェック結果 |
| `echo_cmd_seq`, `echo_cmd_send_time_ns` | 適用中の最新 TeleopPacket の seq と leader 時刻 |
| `cmd_hold_ns` | その command を受信してから feedback 送信までの follower 側経過時間 |
| `arm_pos/vel/tau[8]`, `hand_pos/vel/tau[4]` | 計測関節位置・速度・**トルク** (DM モータの `get_torque()`) |

RTT は leader が `system_now_ns() - echo_cmd_send_time_ns` で自分の時計だけで
測る (ホスト間の時刻同期は不要)。`cmd_hold_ns` を引けば純粋なネットワーク
+ Arbiter 遅延が分かる。

トルクは関節変換 (`motor_to_joint`) を通した値。腕は 1:1、グリッパはモータ
トルクそのまま (力への換算は Phase 2 以降で必要なら追加)。

## 4. 安全ゲート (Phase 1 で実装)

`bilateral_enable` は以下が **全て** 成立したときだけ受理し、成立している間
だけ `bilateral_requested` を維持する。どれかが崩れると 100 ms 以内に
ユニラテラルへ戻す (`Bilateral dropped to unilateral: <理由>` をログ)。

### 4.1 有線チェック (`utils::query_link_to`)

対向 IP へ `connect()` した UDP ソケットの `getsockname()` で経路上の
ローカル IF を特定し、sysfs を読む。

| 条件 | 判定 |
| --- | --- |
| `/sys/class/net/<if>/wireless` または `phy80211` が存在、または `SIOCGIWNAME` 成功 | 無線 → 拒否 |
| `/sys/class/net/<if>/device` が無い | 仮想 IF (bridge, tun, usb gadget) → 拒否 |
| `carrier != 1` | リンクダウン → 拒否 |
| `speed < MinLinkSpeedMbps` | 低速 → 拒否 |
| `lo` 経由 | `AllowLoopback: true` のときだけ許可 (mock 試験用) |

- leader は `BilateralLink.PeerIp` (= follower PC の IP) への経路を見る。
  Arbiter は同一 PC (127.0.0.1) なので、Arbiter 宛てを見ても物理リンクは
  判定できないため。
- follower は feedback 送信先 (= Arbiter PC の IP) への経路を見て、結果を
  `link_ok` として毎パケットに載せる。leader は
  `RequireFollowerLinkOk: true` で **両端が有線** であることを要求する。
- 両側とも `RecheckIntervalS` ごとに再判定する (ケーブル抜けは carrier で検出)。

### 4.2 通信品質チェック (`safety::BilateralGate::check_feedback`)

直近 `EvalWindowS` (1 s) の feedback 統計で判定する。

| 条件 | 既定値 | 意味 |
| --- | --- | --- |
| `age_ms <= MaxFeedbackAgeMs` | 20 ms | 最新 feedback が新鮮 |
| `rate_hz >= MinFeedbackRateHz` | 400 Hz | 500 Hz 送信に対し 20% までの欠落 |
| `rtt_p95_ms <= MaxRttP95Ms` | 8 ms | command→feedback 往復 (follower 保持時間込み) |
| `loss_percent <= MaxLossPercent` | 1 % | seq 飛びの割合 |
| RTT サンプルがある | – | follower が command をエコーしている |
| `link_ok == 1` (follower) | – | follower 側も有線 |
| `estop == 0` (follower) | – | |

しきい値は `config/leader.yaml` の `BilateralLink` で変更する。
loopback mock で実測した値: rate 500 Hz, RTT p50/p95 1.8/1.9 ms
(うち follower 保持 ~1.6 ms)、loss 0 %。実機有線では ping RTT ~1.9 ms が
加わるので p95 で 4–5 ms 程度を見込む。

### 4.3 モード連動

- leader の `disable` / `estop` / `shutdown` は `bilateral_requested` も落とす。
  Session Manager が mode を `leader` 以外へ変えると leader に `disable` を
  送るので、AI/VR に切り替えた時点でバイラテラルは必ず解除される。
- Arbiter が mode≠leader で feedback を落とすため、二重に遮断される。

## 5. Phase 2 (leader 側の力覚提示) の設計方針

Phase 1 のゲートが `ok` の間だけ、leader を `unilateral_step()` から
バイラテラル制御へ切り替える。

1. **係合条件**: 全関節で `|q_L - q_F| < EngageMaxErrorRad` (初期値 0.1 rad) を
   満たすまで係合しない。AI モード後などの姿勢ずれで leader が跳ねるのを防ぐ。
2. **ゲインランプ**: 係合後 `EngageRampS` (0.5 s) かけて Kp/Kd を 0 から目標
   値へ線形に上げる。解除時は 100 ms で 0 へ落としてから `unilateral_step()` へ。
3. **leader は software PD + トルククランプ**: `tau = Kp(q_F - q_L) - Kd*dq_L
   + gravity + friction` を leader 側で計算し、`effort_limit_L`
   (`openarm_constants.hpp`) でクランプして MIT の `tau` として送る
   (Kp=Kd=0)。人が触る側なので、モータ内蔵 PD (クランプ不可) は使わない。
   follower は従来どおりモータ内蔵 PD。
4. **`dq_ref = 0`**: 遅延した相手速度を Kd の参照に使うとエネルギーを注入する
   ので、leader 側は純粋ダンピングにする。
5. **Kp は leader.yaml で follower と別値**: 遅延 4–6 ms を前提に、まず
   `LeaderArmParam.Kp` の 30–50 % から実機で追い込む。
6. **力チャネル (任意)**: `arm_tau` を使って `tau_L += -Kf * tau_F` を足すと
   4ch に近い提示になる。P-P だけで感触が足りない場合の追加項として扱う。
7. **フォールバック**: ゲート失敗・feedback 停止・follower ESTOP はすべて
   「leader が重力補償のみになる」方向に倒れる。follower 側の watchdog 動作は
   変更しない。

## 6. 設定・運用

### 6.1 設定ファイル

- `config/leader.yaml`: `BilateralFeedback` (受信 bind/port),
  `BilateralLink` (PeerIp・有線条件・品質しきい値)
- `config/follower.yaml`: `BilateralFeedback` (送信先 = Arbiter PC),
  `BilateralLink` (有線条件)
- `openarm_session_manager/config/arbiter.yaml`: `feedback.inputs` (50400/50401),
  `feedback.target` (leader 127.0.0.1:50500/50501)

### 6.2 コマンド

leader runtime control (53201):

| コマンド | 動作 |
| --- | --- |
| `bilateral_enable` | ゲート評価 → ok なら requested=true。拒否理由は `message` に列挙 |
| `bilateral_disable` | requested=false |
| `bilateral_status` | ゲート結果・リンク情報・腕ごとの rate/RTT/loss/follower 状態 |

Session Manager 経由: `bilateral-enable` / `bilateral-disable` / `bilateral-status`
(`openarm_session_manager.cli`)。

### 6.3 実機での Phase 1 確認手順

1. 通常どおり follower → Arbiter+Session Manager → leader を起動。
2. `bilateral-status` で `link` が `eth0 ... wired carrier=1 speed=1000Mbps`、
   `link_reasons` が空であること。
3. mode を `leader` にし、数秒後 `bilateral-status` で両腕 `rate_hz≈500`,
   `rtt_p95_ms`, `loss_percent`, `follower_link_ok=true` を確認。
4. `bilateral-enable` → ok。follower の `feedback.mode` が 1 になる。
5. LAN ケーブルを抜く / mode を `ai` にする → leader ログに
   `Bilateral dropped to unilateral` が出て `requested=false` になる。

Phase 1 では leader 腕の挙動は従来と同一 (重力補償のみ)。

## 7. 変更ファイル一覧 (Phase 1)

```text
include/openarm_wifi_teleop/net/feedback_packet.hpp        新規: FeedbackPacket
include/openarm_wifi_teleop/net/teleop_packet.hpp          ControlMode::BILATERAL
include/openarm_wifi_teleop/net/packet_codec.hpp, src/net/packet_codec.cpp
include/openarm_wifi_teleop/core/feedback_state_buffer.hpp, src/core/feedback_state_buffer.cpp
include/openarm_wifi_teleop/utils/link_check.hpp, src/utils/link_check.cpp
include/openarm_wifi_teleop/safety/bilateral_gate.hpp, src/safety/bilateral_gate.cpp
src/net/udp_receiver.cpp        1 ms sleep ポーリング → poll() (全受信経路の遅延を最大 1 ms 削減)
src/controller/control.cpp      response.effort に get_torque() を格納
src/apps/wifi_bimanual_follower.cpp   feedback 送信・有線チェック・status
src/apps/wifi_bimanual_leader.cpp     feedback 受信・ゲート・runtime コマンド・mode ビット
src/yamlloader.hpp              get_string/get_bool/get_int と *_or 版
config/leader.yaml, config/follower.yaml
tests/test_feedback_codec.cpp, tests/test_bilateral_gate.cpp  (-DBUILD_TESTING=ON)
openarm_session_manager/openarm_session_manager/arbiter.py, manager.py, cli.py
openarm_session_manager/config/arbiter.yaml
```
