# ROX2026 Experimental

rox2026の個人的かつ試験的なプロジェクト. AI(主にgemini)を使っています.

## Features

- ros2_controlを使用したsim/実機用ソフトウェア
- 物理シミュレーションを利用した足回りテスト
- step, 周波数解析&画像出力

## Environment

`Docker`を使うことを想定しています.

`ros2 jazzy`以外では動きません. (特にros2_controlの設定の為)

- Ubuntu24.04 (ROS:jazzy)


## description

- nodeは基本C++固定
- joystickはDualSense想定
- 対応は基本jazzy

## Usage

### Development

ホストは`nix`が入っているLinux想定です. `docker`は`nix`経由で入るのでホスト側に必須ではありません. GUI設定も`flake.nix`と`compose.yaml`で完結しているので他にコマンドを叩く必要性はありません.

```bash
# nix環境に入る
$ make nix

# Open VScode
$ code .

# devcontainerでビルド
# or
# docker cli経由でビルド
$ docker compose up -d
$ docker compose exec ros2_rox2026 bash
```

`nix`や`docker`環境に入っているか確認したい時は、

```bash
# 出力が/nix/store/..ならnix環境です
$ which docker

# ホストにros2が入っていない場合、
# /opt/ros/jazzy/bin/ros2と出ればdockerコンテナ内です.
$ ros2

```


### Visualization

- **Foxglove**: 推奨。ブラウザで [studio.foxglove.dev](https://studio.foxglove.dev/) を開き、`ws://localhost:8765` に接続。
- **noVNC (Browser GUI)**: macOS や Linux ホストで X11 設定なしに Gazebo/RViz を使いたい場合。
  - ブラウザで `http://localhost:6080/vnc.html` にアクセス。
  - Password: `password`
  - コンテナ内の X11 デスクトップが表示されます。

### Simulation

Foxgloveで足回りのkinematicsを確認したい場合(物理simなし)

```bash
$ cd main_ws
$ make virtual
```

物理シミュレーション

```bash
$ cd main_ws
$ make sim-gui
```



## Caution
