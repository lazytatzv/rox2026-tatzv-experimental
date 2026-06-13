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

ホストは`nix`が入っているLinux想定です. `docker`は`nix`経由で入るのでホスト側に必須ではありません.

```bash
# nix環境に入る
$ make nix

$ code .

# devcontainerでビルド

# or
$ docker compose up -d
$ docker compose exec ros2_rox2026 bash
```

### Visualization

Foxgloveを使うこと推奨です.

Foxglove-studioをいれるか、ブラウザでfoxgloveを開いて、`websocket`で接続できます.

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
