# Seeed Studio USB-CAN Analyzer Driver

Seeed Studio USB-CAN Analyzer 向けの ROS 2 ドライバです。  
USB-CANプロトコルのフレーム生成/解析を行い、ROS 2 トピックを介して CAN 通信を提供します。

## 構成

- `src/usb_can_analyzer_node.cpp`: ROS 2 ライフサイクルノード実装。
- `src/serial_protocol.cpp`: USB-CANプロトコルの実装。
- `include/`: ヘッダーファイル。

## Topic I/O

- **Subscribe**: `/communication/tx` ([custom_interfaces/msg/CanFrame](../../logic/custom_interfaces/msg/CanFrame.msg))
- **Publish**: `/communication/rx` ([custom_interfaces/msg/CanFrame](../../logic/custom_interfaces/msg/CanFrame.msg))

## 使い方

通常、`robot_bringup` パッケージから自動的に起動されます。

```bash
ros2 launch robot_bringup robot_bringup.launch.py actuator_type:=at protocol:=can
```

単体で実行する場合:
```bash
ros2 run seeed_usb_can_analyzer_driver usb_can_analyzer_node --ros-args -p usb_path:=/dev/ttyUSB0
```
