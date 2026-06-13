# 回路

## Battery

Makitaの40V. 降圧は多分しない.

## Motor

### Robstrideの基盤(via AT command)を使う

[Robstride05] <--CAN--> [提供された基盤] <--USB Type-C--> PC(or rdk)

#### 回す

1.  **コンテナ起動**: `make up`
2.  **ビルド**: `make build`
3.  **起動**: 
    ```bash
    ros2 launch robot_bringup robot_bringup.launch.py actuator_type:=at
    ```

### Usb-to-can analyzer(Seeed Studio)を使う

[Robstride05] <--CAN--> [CanAnalyzer] <--USB Type-C--> PC(or rdk)

#### 回す

1.  **URDFの設定変更**: `robot.urdf.xacro` 内の `RobstrideSystemHardware` パラメータで `protocol` を `can` に設定する（または起動引数での対応を検討中）。
2.  **起動**:
    ```bash
    ros2 launch robot_bringup robot_bringup.launch.py actuator_type:=at
    ```
    ※ `actuator_type:=at` は `ros2_control` を使うためのフラグです。内部のプロトコル切り替えは現在URDFで行います。



