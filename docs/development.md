# 開発時のTips

## setup

```bash
# clone
$ git clone <repo>
$ cd <repo>

# nixenv
# direnv入れてたらしなくていい.
$ nix develop

# vscode(devcontainer)
$ code .

# devcontainerでビルド
```

## devcontainer

`F1`->`Dev Container: Rebuild Container`で再ビルド. `Dockerfile`を書き換えたら定期的にやる.

`devcontainer.json`に拡張とかかける.

## Docker

Imageが意外と容量食うので定期的に確認する. 特にSDカードとか使ってる場合.

```bash
# 容量
$ docker system df

# prune (あってるか不明)
$ docker prune -a
```

## compose.yaml

networkとデバイス周り、guiの設定は注意.

GUIに関してはそもそも`rviz2`とか使わずに`foxglove`をブラウザから使えば基本的に問題ない.

`xhost`は依然として必要だが、`nix develop`しておくとそのへん勝手にやってくれるので便利. `direnv`入れてると更に便利(自動でやってくれる).


## shell

とりあえず`bash`か`zsh`を大人しくつかっておく. `fish`はpluginで`bass`なんかを入れればなんとかなるが、変数の扱いとか微妙なのでおすすめしない.

## foxglove

`foxglove_bridge`が立ち上がるようになっているので、`websocket`で`8765`に繋げば動くはず.

joystickっぽいUIが出せたり、urdf書いとくと視覚化が簡単にできたりと便利. ブラウザから使うほうが個人的に好み.





