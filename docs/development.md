# 開発ワークフローと作業手順

このプロジェクトでは、Nix によるホスト環境管理、Docker Compose によるコンテナ実行、そして `Justfile` を使ったタスク自動化を採用しています。基本的な作業手順を以下にまとめます。

---

### 1. Nix / 環境のセットアップ
ホスト環境に Nix および `direnv` を導入している場合、自動的に開発環境（xhost 設定や必要なホスト側ツール）がロードされます。

```bash
# 1. Nix 開発環境への手動ログイン（direnv未導入の場合）
nix develop

# 2. direnv 連携の設定（初回のみ推奨）
just setup-env
```

---

### 2. Docker コンテナの管理（ホスト側から実行）
コンテナイメージのビルド、起動、停止を行います。

```bash
# コンテナイメージのビルド (host ネットワークを利用してDNSエラーを回避)
just build

# コンテナの起動 (バックグラウンドで起動、GUI転送設定なども自動処理)
just up

# コンテナの停止
just down
```

---

### 3. コンテナへの侵入
開発作業やロボット/ビジョンの個別実行のために、稼働中のコンテナに入ります。

```bash
# メインの開発コンテナ (ros2_rox2026) の bash に入る
just shell

# ビジョン処理用コンテナ (ros2_vision) の bash に入る
just vision-shell
```

---

### 4. ROS 2 ワークスペースのビルドとテスト
ROS 2 パッケージのビルドやテスト、シミュレーション起動はホスト側の `Justfile` ショートカット、またはコンテナ内から実行可能です。

```bash
# [ホストから] ROS 2 ワークスペースのビルド
just build-ws

# [ホストから] ヘッドシミュレーションの起動
just sim

# [ホストから] GUI付きGazeboシミュレーションの起動
just sim-gui

# [ホストから] ワークスペース内のテスト(colcon test)の一括実行
docker compose exec ros2_rox2026 just -f main_ws/Justfile test
```

*（※直接コンテナに入っている場合は、`/root/lazytatzv_ws/main_ws` ディレクトリにて `just build` や `just test` が使用できます。）*

---

### 5. Foxglove を用いた可視化
コンテナ起動時に `foxglove_bridge` が自動的にバックグラウンドで立ち上がります。
- Foxglove Studio（ブラウザ版またはアプリ版）を開き、`ws://localhost:8765` に接続することで、ロボットのステータスやトピック、URDF 視覚化が可能です。
