# Network Test

- ping: Check basic connectivity and round-trip time.
- iperf3: Used for throughput and jitter testing.
- 250 Hz test: `iperf3 -c IP -u -b 500K -l 250 -t 60`
- 500 Hz test: `iperf3 -c IP -u -b 1M -l 250 -t 60`
- 解釈方法: Jitterが1ms以下、Packet lossが0%であることを確認してください。
- 映像通信と同時に測る注意: RealSense等の映像通信と帯域を食い合わないか、同時にテストしてパケットロスが増加しないか確認が必要です。
