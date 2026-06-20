# Control Analysis Toolset

このディレクトリには、ロボットの制御性能（ステップ応答、周波数特性）を解析するためのツールが含まれています。
本体のソースコードとは分離されており、必要時のみ実行する構成です。

## 構成
- `signal_injector`: ステップ入力や正弦波を `/cmd_vel_ext` に注入するノード。
- `auto_analyzer`: Chirp → 休止 → Step を自動実行し、位相マーカーを `/control_analysis/phase` に記録。
- `analyze`: ROS bag を読み込み、FOPDT同定・Bode/Nyquist・JSONレポートを生成。

## プロ級ワークフロー

### 1. シミュレーションを解析モードで起動
teleop より外部信号を優先する mux 設定を使います。

```bash
ros2 launch robot_bringup robot_bringup.launch.py gazebo:=true use_sim_time:=true analysis_mode:=true
```

### 2. 自動計測 + レポート生成
```bash
just report
# または
just analyze-control auto control_analysis_bag full_analysis_report
```

生成物:
- `reports/full_analysis_report.png` — 6パネルダッシュボード
- `reports/full_analysis_report.json` — 数値メトリクス（履歴比較用）

### 3. 応答信号の優先順位
解析は以下の順で「プラント出力」を自動選択します。

1. `/odom/ground_truth` — シミュレーション真値
2. `/mecanum_drive_controller/odometry` — ホイールオドメ（推奨）
3. `/joint_states` からのメカナムFK
4. `/odometry/filtered` — EKF（フィルタ遅延あり、最終手段）

入力信号は `/mecanum_drive_controller/reference`（実際にモーターへ渡る指令）を優先します。

## 手動解析

```bash
ros2 bag record /cmd_vel_ext /mecanum_drive_controller/reference /mecanum_drive_controller/odometry /odometry/filtered /joint_states /control_analysis/phase
ros2 run control_analysis signal_injector --ros-args -p mode:=step -p amplitude:=1.0 -p duration:=5.0
ros2 run control_analysis analyze <bag_file_path> my_report reports/
```

## PlotJuggler でのリアルタイム確認
```bash
ros2 run plotjuggler plotjuggler
```
`joint_states/velocity[0]` と `mecanum_drive_controller/reference/twist/linear/x` を重ねて表示することで、即座に特性を確認できます。
