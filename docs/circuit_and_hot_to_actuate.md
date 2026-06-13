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
    ros2 launch robot_bringup robot_bringup.launch.py actuator_type:=at protocol:=at
    ```

### Usb-to-can analyzer(Seeed Studio)を使う

[Robstride05] <--CAN--> [CanAnalyzer] <--USB Type-C--> PC(or rdk)

#### 回す

1.  **コンテナ起動**: `make up`
2.  **ビルド**: `make build`
3.  **起動**:
    ```bash
    ros2 launch robot_bringup robot_bringup.launch.py actuator_type:=at protocol:=can
    ```
    ※ `protocol:=can` を指定することで、内部で自動的に USB-CAN ブリッジが立ち上がり、プロトコルが CAN に切り替わります。



