# custom_interfaces

ROX2026 プロジェクトで使用される共通メッセージ定義パッケージです。

## メッセージ一覧

- `msg/CanFrame.msg`: CAN 通信の 1 フレームを表すメッセージ。
  - `id` (uint32)
  - `extended` (bool)
  - `remote` (bool)
  - `dlc` (uint8)
  - `data` (uint8[8])
