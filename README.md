# ROX2026 Experimental - Professional Usage Guide

このプロジェクトを最強のパフォーマンスで使いこなすための詳細ガイドです。

---

## 1. 開発環境のセットアップ

### マルチOS対応 (Linux / macOS)
このプロジェクトは Linux (NVIDIA) と macOS (Apple Silicon / Colima) の両方で動作するように最適化されています。`Makefile` が OS を自動検知し、最適なネットワークモードを選択します。

- **Linux**: `network_mode: host` により ROS 2 の通信パフォーマンスが最大化されます。
- **macOS**: `bridge` モードとポート転送により、ブラウザからの接続性を確保します。

### 起動手順
```bash
# 1. Nix環境へ入る
make nix

# 2. コンテナを起動 (OS自動判定)
make up

# 3. コンテナ内シェルに入る
make shell
```

---

## 2. GUI 環境 (noVNC) の使い方

macOS やリモート環境からでも、ブラウザ一つで Gazebo や RViz2 を操作できます。

1. **アクセス**: [http://localhost:6080/vnc.html](http://localhost:6080/vnc.html)
2. **パスワード**: `password`
3. **ヒント**:
   - 画面の右側メニューから `Scaling Mode` を `Remote Resizing` に設定すると、ブラウザのサイズに合わせてデスクトップが自動調整されます。
   - 新しいウィンドウは常に中央に配置されるように設定済みです。

---

## 3. 実機モーター ID の設定ツール

RobStride モーター（EduLite / CyberGear 等）の ID を書き換えるための専用 CLI ツールです。工場出荷時（ID 127）のモーターをキッティングする際に使用します。

### 実行方法
コンテナ内の `main_ws` にて実行します。

```bash
# ID 127 を 1 に変更する場合
ros2 run robstride_driver set_motor_id --ros-args -p old:=127 -p new:=1 -p port:=/dev/ttyUSB0 -p protocol:=at

# 引数詳細:
#  old: 現在のID (デフォルト127)
#  new: 新しいID
#  port: シリアルポートパス (デフォルト /dev/ttyUSB0)
#  protocol: 'at' (推奨) または 'can'
```
**注意**: コマンド送信後、設定を恒久化するためにモーターの**電源を再投入（パワーサイクル）**してください。

---

## 4. 制御解析ワークフロー (Bode / Step 応答)

制御解析のパラメータは YAML ファイルで一元管理されています。

### 設定の変更
`main_ws/src/control_analysis/config/analysis_settings.yaml` をエディタで編集します。

```yaml
/signal_injector:
  ros__parameters:
    mode: "chirp"        # 'step', 'sine', 'chirp'
    amplitude: 1.0       # 振幅
    duration: 10.0       # 実験時間
    frequency_end: 15.0  # スイープ終了周波数
```

### 解析の実行
```bash
# A. 全自動レポート生成 (Gazebo Headless)
# 起動 -> 記録 -> 注入 -> 解析 -> 画像出力まで一気に行います
cd main_ws
make report

# B. 手動注入 (動作を見ながら確認したい場合)
make injector
```

---

## 5. シミュレーションと実機の切り替え

リファクタリングにより、デフォルトで**実機動作**を優先する構成になっています。

### シミュレーション (Gazebo)
```bash
cd main_ws
# 画面あり
make sim-gui
# 画面なし
ros2 launch robot_bringup robot_bringup.launch.py gazebo:=true headless:=true
```

### 実機 (Real Hardware)
```bash
cd main_ws
# デフォルト引数が gazebo:=false なので叩くだけでOK
ros2 launch robot_bringup robot_bringup.launch.py
```

---

## 6. プロフェッショナル・アーキテクチャ

開発者がコードを変更する際の重要ポイントです。

- **`imu_stabilizer`**: 制御ロジックは `HeadingStabilizerCore` ライブラリとして独立しています。変更を加えたら必ず `colcon test` で単体テストを回してください。
- **Launch分割**: 起動設定は `launch/include/` 配下に機能別（`sim`, `description`, `localization`）でモジュール化されています。
- **URDF分割**: 物理モデルは `urdf/include/` 配下でパーツ（`wheel`, `common`）ごとに管理されています。
