# virtual_actuator

シミュレーション用の仮想モータドライバです。`ros2_control` の `SystemInterface` として動作します。

## 特徴

- **慣性モデル**: 1次遅れ系による簡易的な慣性モデルを内蔵。
- **軽量**: 物理エンジンを使わずにモータの挙動を模擬可能。

## 使い方

```bash
ros2 launch robot_bringup robot_bringup.launch.py actuator_type:=virtual
```
