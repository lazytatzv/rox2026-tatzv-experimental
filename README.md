# ROX2026 Experimental

ROX2026プロジェクトの試験的なソフトウェアスタックです。ROS 2 Jazzy を基盤とし、シミュレーションと実機制御を統合しています。

## Architecture

本プロジェクトは以下の設計方針に基づいています。

- **Config-Driven**: 物理特性、ハードウェア設定、制御パラメータを `config/params/*.yaml` に集約し、コード変更なしにシステムの挙動を制御します。
- **Component-Based**: 制御ノード群を `rclcpp_components` として実装し、`ComposableNodeContainer` にロードすることでプロセス間通信のオーバーヘッドを削減しています。
- **Multi-Stage Docker**: 開発環境(Dev)と実行環境(Prod)を分離し、実機デプロイ時のイメージサイズと起動時間を最小化しています。

## Requirements

- ホスト OS: Linux または macOS
- Docker (推奨: Docker Compose V2 互換環境)
- ROS 2 Jazzy (コンテナ内で提供)

## Setup & Execution

### 1. 開発環境の起動

Makefile が OS (Linux/macOS) を判別し、最適なネットワークモードでコンテナを起動します。

```bash
# 開発用イメージのビルドとコンテナ起動
make image
make up

# コンテナ内でのビルド
make shell
make build
```

### 2. シミュレーションと実機

デフォルトの Launch 設定は実機動作を優先しています。

```bash
# シミュレーション (Gazebo) の起動
make sim-gui

# 実機制御の起動
ros2 launch robot_bringup robot_bringup.launch.py
```

### 3. ベンチテスト環境

特定のモーター1軸のみを評価するためのモードです。

```bash
# ID: 1, Port: /dev/ttyUSB0 のモーターをテスト
make bench ID=1 PORT=/dev/ttyUSB0
```

## Tools

### GUI アクセス (noVNC)

macOS やリモート環境から X11 アプリケーションにアクセスするためのブラウザベースの GUI を内包しています。

- URL: `http://localhost:6080/vnc.html`
- Password: `password`

### モーター ID 設定

RobStride モーターの CAN ID を変更するユーティリティです。

```bash
ros2 run robstride_driver set_motor_id --ros-args -p old:=127 -p new:=1 -p port:=/dev/ttyUSB0 -p protocol:=at
```

### 制御解析ワークフロー

`analysis_settings.yaml` で定義されたパラメータに基づき、周波数応答やステップ応答を計測します。

```bash
# 自動レポート生成 (sim を analysis_mode で起動した状態で実行)
just report

# 手動での信号注入
just analyze-control step
```

## Deployment

実機へのデプロイには、コンパイルツールを含まない実行専用の軽量イメージを使用します。

```bash
# 実機用イメージのビルドと起動
make prod-image
make prod-up
```
