# ROX2026 Experimental

rox2026の個人的かつ試験的なプロジェクト. AI(主にgemini)を使っています.

## Env

Docker(compose)を使っています.

- Ubuntu24.04 (ROS:jazzy)

- Ubuntu22.04 (ROS:humble) <== CIでチェックしてる程度


## description

- nodeは基本C++固定
- joystickはDualSense想定
- 対応は基本jazzy

## Usage

### Dev

基本的に`Docker`の使用を想定しています.

```bash
$ git clone <this repo>
$ cd <REPO>

# build & up
$ docker compose up -d

# enter
$ docker compose exec lazy_container bash
```

.devcontainerを使う場合は、`git clone`して、vscodeを開いてそのまま使えます.

以下推奨の方法

```bash
$ nix develop
$ code .

# ==> devcontainer使う
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
$ make sim-gui
```



## Caution
