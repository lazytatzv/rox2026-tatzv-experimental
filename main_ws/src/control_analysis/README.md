# Control Analysis Toolset

このディレクトリには、ロボットの制御性能（ステップ応答、周波数特性）を解析するためのツールが含まれています。
本体のソースコードとは分離されており、必要時のみ実行する構成です。

## 構成

| ノード           | 役割                                                                 |
|-----------------|----------------------------------------------------------------------|
| `signal_injector` | ステップ / 正弦波 / Chirp / **PRBS** を `/cmd_vel_ext` に注入       |
| `auto_analyzer`   | Chirp → 休止 → Step（正） → 休止 → **Step（負）** → 休止 → PRBS を自動実行 |
| `analyze`         | ROS bag を読み込み FOPDT同定・Bode/Nyquist・**PID推奨値**・**安定余裕診断**・JSON/PNG を生成 |

## プロ級ワークフロー

### 1. シミュレーションを解析モードで起動

```bash
ros2 launch robot_bringup robot_bringup.launch.py \
  gazebo:=true use_sim_time:=true analysis_mode:=true
```

### 2. 自動計測 + レポート生成

```bash
just report
# または
just analyze-control auto control_analysis_bag full_analysis_report
```

生成物:

| ファイル | 内容 |
|---|---|
| `reports/full_analysis_report.png` | 10パネルダッシュボード |
| `reports/full_analysis_report.json` | 数値メトリクス（PID推奨値含む） |

### 3. 応答信号の優先順位

解析は以下の順で「プラント出力」を自動選択します。

1. `/odom/ground_truth` — シミュレーション真値
2. `/mecanum_drive_controller/odometry` — ホイールオドメ（推奨）
3. `/joint_states` からのメカナムFK
4. `/odometry/filtered` — EKF（フィルタ遅延あり、最終手段）

入力信号は `/mecanum_drive_controller/reference`（実際にモーターへ渡る指令）を優先します。

---

## 手動解析

```bash
ros2 bag record \
  /cmd_vel_ext \
  /mecanum_drive_controller/reference \
  /mecanum_drive_controller/odometry \
  /odometry/filtered \
  /joint_states \
  /control_analysis/phase

# ステップ注入
ros2 run control_analysis signal_injector --ros-args \
  -p mode:=step -p amplitude:=1.0 -p duration:=5.0

# PRBS 注入（毎回シードが変わる）
ros2 run control_analysis signal_injector --ros-args \
  -p mode:=prbs -p amplitude:=1.0 -p duration:=10.0 -p prbs_hold_time:=0.05

# 解析
ros2 run control_analysis analyze <bag_file_path> my_report reports/
```

---

## 解析結果の読み方

### FOPDT モデル `G(s) = K·exp(-Ls) / (τs + 1)`

| パラメータ | 意味 | 目安 |
|---|---|---|
| K | プロセスゲイン | 1に近いほど追従性が高い |
| τ | 時定数 | 小さいほど速い応答 |
| L | むだ時間 | 小さいほど制御しやすい |

### PID 推奨値（IMC / Lambda チューニング）

`analyze` が自動出力する `Kp / Ti / Td` は FOPDT 同定結果から導出したスタート地点です。
実機では安全のため最初は Kp を半分程度に下げてから調整してください。

### 安定余裕の診断基準

| マージン | 危険 | 余裕あり | 良好 |
|---|---|---|---|
| Phase Margin (PM) | < 30° | 30–45° | ≥ 45° |
| Gain Margin (GM) | < 6 dB | 6–10 dB | ≥ 10 dB |

---

## PlotJuggler でのリアルタイム確認

```bash
ros2 run plotjuggler plotjuggler
```

`joint_states/velocity[0]` と `mecanum_drive_controller/reference/twist/linear/x` を重ねて表示することで、即座に特性を確認できます。
