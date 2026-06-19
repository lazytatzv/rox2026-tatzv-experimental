# 開発ワークフローと作業手順

このプロジェクトでは、Nix によるホスト環境管理、Docker Compose によるコンテナ実行、そして `Justfile` を使ったタスク自動化を採用しています.

---

### Nix環境のsetup

`nix`環境に入ると自動でxhostなどのホスト側に必要な設定が読み込まれます. 毎回`nix developを`実行するか`direnv`をセットアップしてください.

`nix`のconfig

```bash
$ mkdir -p ~/.config/nix
$ vim ~/.config/nix/nix.conf
```

`nix.conf`の中身は以下を記述する.

```ini
experimental-features = nix-command flakes
```

起動方法

```bash
# project rootで
$ nix develop
```

---

### Dockerコンテナ

`nix`環境化で実行してください

```bash
# ビルド
$ just build

# コンテナの起動
$ just up

# コンテナに入る(ros2rox)
$ just shell

# rdk用コンテナに入る
$ just vision-shell

# コンテナの停止
$ just down
```

---

---

### ビルド

```bash
# 移動
$ cd main_ws

# ros2wsのビルド
$ just build

# シミュレーション実行
$ just sim-gui
```


