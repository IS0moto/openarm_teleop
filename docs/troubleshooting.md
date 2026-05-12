# Troubleshooting

- CAN interfaceが見つからない: `setup_can.sh` を実行してインターフェースを立ち上げてください。
- UDPが届かない / firewall: `sudo ufw disable` 等でファイアウォールを無効にするかポートを開放してください。
- wrong IP: `--follower-ip` の設定が正しいか確認してください。
- packet loss / jitterが大きい: ネットワーク帯域が不足しているか、Wi-Fi環境が不安定です。有線LANへの切り替えを検討してください。
- followerが動かない:
  - enable忘れ: Leader側に `--enable` を付与してください。
  - watchdogで止まる: UDPが遅延しています。
