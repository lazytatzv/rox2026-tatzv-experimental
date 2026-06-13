# Control Analysis Toolset

このディレクトリには、ロボットの制御性能（ステップ応答、周波数特性）を解析するためのツールが含まれています。
本体のソースコードとは分離されており、必要時のみ実行する構成です。

## 構成
- `signal_injector`: ステップ入力や正弦波を `/mecanum_drive_controller/reference` に注入するノード。
- `analyze`: ROS bag を読み込み、グラフ作成や時定数算出を行うスクリプト。

## 使い方

### 1. ビルド
```bash
make build  # または colcon build --packages-select control_analysis
```

### 2. データの記録
シミュレーションまたは実機を起動した状態で、別ターミナルで bag を記録します。
```bash
ros2 bag record /mecanum_drive_controller/reference /joint_states
```

### 3. 信号の注入 (Step Response)
bag 記録中に、解析用信号を送ります。
```bash
# 1.0 rad/s のステップ入力を5秒間
ros2 run control_analysis signal_injector --ros-args -p mode:=step -p amplitude:=1.0 -p duration:=5.0
```

### 4. 解析の実行
記録が終わったらスクリプトで解析します。
```bash
ros2 run control_analysis analyze <bag_file_path>
```

## PlotJuggler でのリアルタイム確認
推奨されるワークフローは、PlotJuggler を併用することです。
```bash
ros2 run plotjuggler plotjuggler
```
`joint_states/velocity[0]` と `reference/twist/linear/x` を重ねて表示することで、即座に特性を確認できます。
