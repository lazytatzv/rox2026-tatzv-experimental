# robstride_driver

Robstride モータ（EL05等）を `ros2_control` から制御するためのハードウェアインターフェースパッケージです。

## 特徴

- **複数プロトコル対応**: AT（シリアル）、CAN、DDSM の各プロトコルを `ProtocolHandler` で抽象化。
- **動的切り替え**: URDF のパラメータまたは起動引数でプロトコルを切り替え可能。
- **診断機能**: 通信エラーの詳細なデバッグログ出力をサポート。

## 使い方

通常、`robot_bringup` パッケージから自動的にロードされます。

```bash
ros2 launch robot_bringup robot_bringup.launch.py actuator_type:=at protocol:=at
```

## パラメータ (URDF)

- `protocol`: `at`, `can`, `ddsm`
- `topic_tx_queue`: 送信用トピック名（デフォルト: `/communication/tx`）
- `topic_rx_queue`: 受信用トピック名（デフォルト: `/communication/rx`）
