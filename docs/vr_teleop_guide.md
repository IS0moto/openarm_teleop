# OpenArm 実機 VR テレオペレーションガイド

このガイドでは、Meta Quest 2 コントローラを使用して、実機の OpenArm を操作するためのセットアップ手順を説明します。

## システム構成

- **Follower PC**: ロボットと CAN 接続されている PC。ロボットの状態を送信し、指令を受け取ります。
- **Leader PC**: IK（逆運動学）計算を行う PC。VR コントローラの入力を受け、ロボットへの指令を生成します。
  - ※ Follower と Leader は同一の PC でも動作可能です。
- **Quest 2 / PC**: OpenXR Logger アプリを実行し、コントローラの変位を送信します。

---

## 1. Follower PC（ロボット側）の準備

ロボット側では、テレメトリ（状態フィードバック）を有効にして起動する必要があります。これが無効だと、Leader 側で安全装置が働き、操作がブロックされます。

### CAN の初期化
```bash
sudo openarm-can-configure-socketcan can0 -fd
sudo openarm-can-configure-socketcan can1 -fd
```

### Follower の起動
**重要**: `--publish-telemetry` を必ず追加してください。同一 PC 内でテストする場合は `lo` インターフェースを使用します。

```bash
./build/wifi_bimanual_follower \
  --interface lo \
  --right-can can0 \
  --left-can can1 \
  --publish-telemetry \
  --telemetry-ip 127.0.0.1
```

---

## 2. Leader PC（操作側）の準備

Leader PC では、VR 入力を受け取り、IK を解いて Follower へ送信します。

### 設定の確認
`config/vr_teleop_config.yaml` を編集して、マッピングや安全制限（レート制限）を調整します。

### Leader の起動
```bash
./build/wifi_vr_bimanual_leader --follower-ip 127.0.0.1
```

起動後、画面に以下が表示されることを確認してください：
- `VR: OK` (Quest アプリ起動時)
- `TEL: OK` (Follower からデータ受信時)

---

## 3. VR 側（Quest 2）の準備

### Quest アプリの起動
```bash
cd quest2_openxr_logger
./build/openxr_relative_sender --dest-ip <Leader_PC_IP>
```

---

## 4. 操作方法

1. **Grip ボタン**: 押している間だけロボットが動きます（クラッチ操作）。
   - ボタンを押した瞬間のロボット姿勢が「アンカー（基準）」となり、そこからのコントローラの移動量がロボットに反映されます。
2. **Estop (B/Y ボタン)**: 緊急停止。押すと Leader からの送信が停止します。
3. **リセンター**: コントローラの向きや位置をリセットしたい場合は、一度 Grip を離して持ち直し、再度 Grip を押してください。

---

## トラブルシューティング

### Q: `TEL: NO` と表示されて操作できない
- **原因**: Follower が状態を送信していません。
- **対策**: Follower 起動時に `--publish-telemetry` が付いているか確認してください。

### Q: `IK: FAIL` が頻発する
- **原因**: 特異点に近づいているか、移動量が大きすぎます。
- **対策**: 
  - 一度 Grip を離して、腕を自然な位置に戻してから再度操作してください。
  - `config/vr_teleop_config.yaml` の `damping` を少し大きくする（例: 0.1）と安定する場合があります。

### Q: ロボットの動きが逆、または軸がズレている
- **対策**: `config/vr_teleop_config.yaml` の `right_axis_matrix` / `left_axis_matrix` を調整してください。
  - `--vr-mapping-test` フラグを付けて Leader を起動すると、ロボットを動かさずに座標変換の結果だけを確認できます。
