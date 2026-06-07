# 回路

## Battery

Makitaの40V. 降圧は多分しない.

## Motor

### Robstrideの基盤(via AT command)を使う

[Robstride05] <--CAN--> [提供された基盤] <--USB Typec--> PC(or rdk)

- CAN_H ==> 細い赤
- CAN_L ==> 細い黒

#### 回す

コンテナ中前提

```bash
$ cd main_ws

# In physical.yaml
# protocolを"at"に変える

$ make build
$ source install/setup.bash
$ ros2 launch robot_bringup robot_bringup.launch.py
```

### Usb-to-can analyzer(speedstudio)を使う

配線的には純正基盤とほぼ変わらない

[Robstride05] <--CAN--> [CanAnalyzer] <--Usb Typec--> PC(or rdk)

#### 回す

`physical.yaml`を`"can"`に変えるところ以外は同じ



