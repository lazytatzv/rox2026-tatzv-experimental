# el05_usb_can_driver

RobStride EL05 を Seeed Studio USB-CAN Analyzer 経由で動かす ROS 2 パッケージです。

このパッケージは EL05 の **Private Protocol** 専用です。`seeed_usb_can_analyzer_driver/msg/CanFrame` を使って、USB-CAN ノードの `/ssuca/transmit` に 29-bit extended CAN フレームを送信し、`/ssuca/receive` のフィードバックを解釈します。

## 前提

- EL05 Private Protocol を対象にしています。
- CAN bitrate は 1 Mbps に合わせてください。
- モータ ID 既定値は `0x7f` (`127`)、ホスト ID 既定値は `0xfd` (`253`) です。
- EL05 用の USB-CAN 設定 `config/usb_can_analyzer_el05.yaml` を同梱しています。

## ビルド

```bash
colcon build --packages-select el05_usb_can_driver
source install/setup.zsh
```

## 起動

USB-CAN ノードも一緒に起動:

```bash
ros2 launch el05_usb_can_driver el05_usb_can.launch.py
```

EL05 ノードだけ起動:

```bash
ros2 launch el05_usb_can_driver el05_usb_can.launch.py start_usb_can:=false
```

## 制御トピック

- `~/enable` (`std_msgs/Bool`): `true` で Type 3 enable、`false` で Type 4 stop
- `~/stop` (`std_msgs/Empty`): Type 4 stop
- `~/zero` (`std_msgs/Empty`): Type 6 mechanical zero
- `~/clear_faults` (`std_msgs/Empty`): Type 4 fault clear
- `~/active_reporting` (`std_msgs/Bool`): Type 24 active reporting on/off
- `~/save` (`std_msgs/Empty`): Type 22 motor data save
- `~/set_mode` (`std_msgs/UInt8`): Type 18 で `0x7005 run_mode` を変更
- `~/motion_command` (`std_msgs/Float64MultiArray`): Type 1 operation control
- `~/position_command` (`std_msgs/Float64MultiArray`): Type 18 で `loc_ref` / `limit_spd` を変更
- `~/velocity_command` (`std_msgs/Float64MultiArray`): Type 18 で `spd_ref` / `limit_cur` を変更
- `~/current_command` (`std_msgs/Float64`): Type 18 で `iq_ref` を変更
- `~/torque_command` (`std_msgs/Float64`): Type 1 で torque のみ指令
- `~/read_parameter` (`std_msgs/UInt16`): Type 17 parameter read
- `~/write_parameter` (`std_msgs/Float64MultiArray`): Type 18 float parameter write

`set_mode` の値:

```text
0 : operation control mode
1 : position mode (PP)
2 : velocity mode
3 : current mode
5 : position mode (CSP)
```

## set_mode ごとのコマンド例

モード切り替えは、動作中に直接切り替えず、基本は stop → set_mode → enable → command の順にします。

### 0: Operation Control Mode

Private Protocol の Type 1 `motion_command` を使うモードです。

```bash
ros2 topic pub --once /el05_motor_node/stop std_msgs/msg/Empty '{}'
ros2 topic pub --once /el05_motor_node/set_mode std_msgs/msg/UInt8 '{data: 0}'
ros2 topic pub --once /el05_motor_node/enable std_msgs/msg/Bool '{data: true}'
ros2 topic pub /el05_motor_node/motion_command std_msgs/msg/Float64MultiArray \
  '{data: [0.0, 10.0, 0.0, 0.5, 0.0]}'
```

`motion_command` の配列:

```text
[position_rad, velocity_rad_s, kp, kd, torque_nm]
```

### 1: Position Mode (PP)

`position_command` で目標位置と速度制限を書きます。

```bash
ros2 topic pub --once /el05_motor_node/stop std_msgs/msg/Empty '{}'
ros2 topic pub --once /el05_motor_node/set_mode std_msgs/msg/UInt8 '{data: 1}'
ros2 topic pub --once /el05_motor_node/enable std_msgs/msg/Bool '{data: true}'
ros2 topic pub /el05_motor_node/position_command std_msgs/msg/Float64MultiArray \
  '{data: [1.0, 1.0]}'
```

`position_command` の配列:

```text
[position_rad, speed_limit_rad_s]
```

### 2: Velocity Mode

`velocity_command` で速度指令と電流制限を書きます。

```bash
ros2 topic pub --once /el05_motor_node/stop std_msgs/msg/Empty '{}'
ros2 topic pub --once /el05_motor_node/set_mode std_msgs/msg/UInt8 '{data: 2}'
ros2 topic pub --once /el05_motor_node/enable std_msgs/msg/Bool '{data: true}'
ros2 topic pub /el05_motor_node/velocity_command std_msgs/msg/Float64MultiArray \
  '{data: [10.0, 2.0]}'
```

`velocity_command` の配列:

```text
[velocity_rad_s, current_limit_a]
```

### 3: Current Mode

`current_command` で Iq 電流指令を書きます。

```bash
ros2 topic pub --once /el05_motor_node/stop std_msgs/msg/Empty '{}'
ros2 topic pub --once /el05_motor_node/set_mode std_msgs/msg/UInt8 '{data: 3}'
ros2 topic pub --once /el05_motor_node/enable std_msgs/msg/Bool '{data: true}'
ros2 topic pub /el05_motor_node/current_command std_msgs/msg/Float64 '{data: 1.0}'
```

`current_command` の値:

```text
iq_ref_a
```

### 5: Position Mode (CSP)

CSP でも `position_command` を使います。PP と同じ topic ですが、run_mode が `5` です。

```bash
ros2 topic pub --once /el05_motor_node/stop std_msgs/msg/Empty '{}'
ros2 topic pub --once /el05_motor_node/set_mode std_msgs/msg/UInt8 '{data: 5}'
ros2 topic pub --once /el05_motor_node/enable std_msgs/msg/Bool '{data: true}'
ros2 topic pub /el05_motor_node/position_command std_msgs/msg/Float64MultiArray \
  '{data: [1.0, 1.0]}'
```

## フィードバック

- `~/status` (`std_msgs/String`): JSON 形式の position/velocity/torque/temperature/fault/mode_status
- `/joint_states` (`sensor_msgs/JointState`): ROS 標準 joint state

## 例

Enable:

```bash
ros2 topic pub --once /el05_motor_node/enable std_msgs/msg/Bool '{data: true}'
```

Operation control mode にする:

```bash
ros2 topic pub --once /el05_motor_node/set_mode std_msgs/msg/UInt8 '{data: 0}'
```

10 rad/s で回す:

```bash
ros2 topic pub /el05_motor_node/motion_command std_msgs/msg/Float64MultiArray \
  '{data: [0.0, 10.0, 0.0, 0.5, 0.0]}'
```

速度モードで 10 rad/s:

```bash
ros2 topic pub --once /el05_motor_node/stop std_msgs/msg/Empty '{}'
ros2 topic pub --once /el05_motor_node/set_mode std_msgs/msg/UInt8 '{data: 2}'
ros2 topic pub --once /el05_motor_node/enable std_msgs/msg/Bool '{data: true}'
ros2 topic pub /el05_motor_node/velocity_command std_msgs/msg/Float64MultiArray \
  '{data: [10.0, 2.0]}'
```

Stop:

```bash
ros2 topic pub --once /el05_motor_node/stop std_msgs/msg/Empty '{}'
```

Status:

```bash
ros2 topic echo /el05_motor_node/status
```
